/*
 *   groupby.c: proves the FPGA GROUP BY user core, driven through DMA Iuventus, against real
 *   drives -- holds the environment open for apps/iuventus/sw/iuventus_groupby.py to drive runs
 *   over MI, and independently reads a finished run's result table back and checks it.
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) Vladislav Válek
 */

#include "spdk/stdinc.h"

#include "spdk/env.h"
#include "spdk/file.h"
#include "spdk/iuventus.h"
#include "spdk/json.h"
#include "spdk/nvme.h"
#include "spdk/util.h"

/*
 * GROUP BY result table layout. NUM_GROUPS matches SPLITMIX_KEY_MOD in
 * apps/iuventus/sw/iuventus_groupby.py -- the shipping group count is fixed at build time, so
 * this hardcodes it exactly as that script does rather than reading it from the FPGA over MI,
 * which is out of scope here (MI run control belongs entirely to that script).
 */
#define GROUPBY_NUM_GROUPS		16384u
#define GROUPBY_REC_BYTES		16u
#define GROUPBY_SECT_BYTES		512u
#define GROUPBY_RECS_PER_SECT		(GROUPBY_SECT_BYTES / GROUPBY_REC_BYTES)
#define GROUPBY_RESULT_SECTORS		(GROUPBY_NUM_GROUPS * GROUPBY_REC_BYTES / GROUPBY_SECT_BYTES)
#define GROUPBY_RESULT_BYTES		(GROUPBY_NUM_GROUPS * GROUPBY_REC_BYTES)

/* One drive per DMA Iuventus card queue, same bound as the environment library. */
#define GROUPBY_MAX_SIDECARS		IUVENTUS_MAX_PEERS

/* Cap on how many individual mismatches --verify prints; the total count is never capped. */
#define VERIFY_MAX_MISMATCH_REPORT	20u

/* Byte-wise so an odd group offset (never happens here, GROUPBY_REC_BYTES is 16) still decodes
 * correctly rather than relying on the buffer's alignment for a direct uint64_t cast. */
static uint64_t
le64_from_buf(const uint8_t *p)
{
	uint64_t v = 0;
	int i;

	for (i = 7; i >= 0; i--) {
		v = (v << 8) | p[i];
	}
	return v;
}

/* ================================================================================================
 * Reference sidecars, format "kv_fill.sidecar.v1" as written by examples/nvme/kv_fill/kv_fill.c's
 * write_sidecar() -- read directly from that source, not guessed, since kv_fill now exists in
 * this tree. Each sidecar already carries its drive's fully reduced per-key sum table
 * (group_sum[key_mod]), computed by kv_fill as it generated that drive's records, so a
 * multi-drive expected table is just the elementwise, wrapped sum of every sidecar's group_sum
 * array -- the same way the core itself sums every enabled queue into one table. This file only
 * reads that schema; examples/nvme/kv_fill owns writing it.
 *
 * Required: "key_mod" (u32, must equal GROUPBY_NUM_GROUPS -- it is also the length of
 * "group_sum") and "group_sum" (an array of exactly GROUPBY_NUM_GROUPS u64). "trid", "seed",
 * "record_count" and "error" are accepted and used only for log output and the error gate below;
 * their absence does not fail the load.
 * ================================================================================================
 */

/* Schema tag kv_fill stamps into every sidecar; mismatches are a warning; the length checks
 * inside decode_group_sum() are what actually gate this reader against an incompatible file. */
#define GROUPBY_SIDECAR_FORMAT		"kv_fill.sidecar.v1"

struct sidecar {
	char		*format;
	char		*trid;
	uint64_t	seed;
	uint32_t	key_mod;
	uint64_t	record_count;
	bool		error;
	uint64_t	*group_sum;	/* [GROUPBY_NUM_GROUPS], heap-allocated by decode_group_sum() */
};

static void
sidecar_free(struct sidecar *sc)
{
	free(sc->format);
	free(sc->trid);
	free(sc->group_sum);
}

/* Decodes the "group_sum" JSON array straight into a heap buffer sized for the fixed table this
 * app checks, failing the whole sidecar decode if the array is not exactly that long -- a wrapper
 * around spdk_json_decode_array() because the generic per-field decoder callback only takes a
 * (val, out) pair, not the extra max_size/stride spdk_json_decode_array() needs. */
static int
decode_group_sum(const struct spdk_json_val *val, void *out)
{
	uint64_t	**dst = out;
	uint64_t	*arr;
	size_t		count = 0;

	arr = calloc(GROUPBY_NUM_GROUPS, sizeof(*arr));
	if (arr == NULL) {
		return -1;
	}

	if (spdk_json_decode_array(val, spdk_json_decode_uint64, arr, GROUPBY_NUM_GROUPS, &count,
				   sizeof(*arr)) != 0 || count != GROUPBY_NUM_GROUPS) {
		free(arr);
		return -1;
	}

	free(*dst);
	*dst = arr;
	return 0;
}

static const struct spdk_json_object_decoder sidecar_decoders[] = {
	{"format", offsetof(struct sidecar, format), spdk_json_decode_string, true},
	{"trid", offsetof(struct sidecar, trid), spdk_json_decode_string, true},
	{"seed", offsetof(struct sidecar, seed), spdk_json_decode_uint64, true},
	{"key_mod", offsetof(struct sidecar, key_mod), spdk_json_decode_uint32, false},
	{"record_count", offsetof(struct sidecar, record_count), spdk_json_decode_uint64, true},
	{"error", offsetof(struct sidecar, error), spdk_json_decode_bool, true},
	{"group_sum", offsetof(struct sidecar, group_sum), decode_group_sum, false},
};

/*
 * Loads and decodes one sidecar file. *sc is zeroed first, so every field left undecoded (e.g.
 * sc->trid when "trid" is absent) is a well defined 0/NULL. On any failure *sc is freed and left
 * zeroed too, so a caller only ever needs to clean up sidecars this call reported as loaded.
 */
static int
sidecar_load(const char *path, struct sidecar *sc)
{
	void		*buf, *end;
	size_t		size, num_values;
	ssize_t		rc;
	struct spdk_json_val	*values = NULL;
	int		ret = -1;

	memset(sc, 0, sizeof(*sc));

	buf = spdk_posix_file_load_from_name(path, &size);
	if (buf == NULL) {
		fprintf(stderr, "ERROR: cannot read sidecar '%s'\n", path);
		return -1;
	}

	rc = spdk_json_parse(buf, size, NULL, 0, NULL, 0);
	if (rc <= 0) {
		fprintf(stderr, "ERROR: sidecar '%s' is not valid JSON (rc=%zd)\n", path, rc);
		goto out;
	}

	num_values = (size_t)rc;
	values = calloc(num_values, sizeof(*values));
	if (values == NULL) {
		fprintf(stderr, "ERROR: out of memory parsing sidecar '%s'\n", path);
		goto out;
	}

	rc = spdk_json_parse(buf, size, values, num_values, &end,
			     SPDK_JSON_PARSE_FLAG_DECODE_IN_PLACE);
	if (rc <= 0) {
		fprintf(stderr, "ERROR: sidecar '%s' failed to parse (rc=%zd)\n", path, rc);
		goto out;
	}

	if (spdk_json_decode_object_relaxed(values, sidecar_decoders,
					    SPDK_COUNTOF(sidecar_decoders), sc)) {
		fprintf(stderr, "ERROR: sidecar '%s' failed to decode -- needs at least "
			"'key_mod' and a %u-entry 'group_sum' array (%s format)\n", path,
			GROUPBY_NUM_GROUPS, GROUPBY_SIDECAR_FORMAT);
		goto out;
	}

	if (sc->format != NULL && strcmp(sc->format, GROUPBY_SIDECAR_FORMAT) != 0) {
		fprintf(stderr, "WARNING: sidecar '%s' declares format '%s', this reader was "
			"written against '%s' -- proceeding, but a mismatch here can mean the "
			"schema moved on\n", path, sc->format, GROUPBY_SIDECAR_FORMAT);
	}

	if (sc->key_mod != GROUPBY_NUM_GROUPS) {
		fprintf(stderr, "ERROR: sidecar '%s' has key_mod=%u, this result table is %u "
			"groups wide\n", path, sc->key_mod, GROUPBY_NUM_GROUPS);
		goto out;
	}

	if (sc->error) {
		fprintf(stderr, "ERROR: sidecar '%s' marks its own write as failed "
			"(\"error\": true) -- its group_sum reflects records kv_fill only "
			"attempted, not what is actually persisted on the drive, so it cannot "
			"be trusted as a reference\n", path);
		goto out;
	}

	ret = 0;
out:
	free(values);
	free(buf);
	if (ret != 0) {
		sidecar_free(sc);
		memset(sc, 0, sizeof(*sc));
	}
	return ret;
}

/* Elementwise, wrapped sum of one sidecar's group_sum into the shared reference table -- see the
 * section comment above for why this is the entire reduction needed. */
static void
sidecar_accumulate(const struct sidecar *sc, uint64_t *sums)
{
	uint32_t k;

	for (k = 0; k < GROUPBY_NUM_GROUPS; k++) {
		sums[k] += sc->group_sum[k];
	}
}

/* ================================================================================================
 * Result read-back: an ordinary SPDK read against a normal host-managed I/O queue pair, entirely
 * separate from the FPGA's own P2P queue. No FPGA/nfb access on this path.
 * ================================================================================================
 */

struct read_ctx {
	bool	done;
	int	status;
};

static void
read_cb(void *arg, const struct spdk_nvme_cpl *cpl)
{
	struct read_ctx *ctx = arg;

	ctx->done = true;
	ctx->status = spdk_nvme_cpl_is_error(cpl) ? -1 : 0;
}

/*
 * One command at a time: this is a correctness check, not a throughput test (see main task
 * README -- rates come from the FPGA's own counters on the Python side), so there is no reason
 * to pipeline reads. Chunked by the namespace's own MDTS so a single command never exceeds what
 * the controller allows; reassembled into buf in ascending LBA order, which is the GROUP BY
 * core's own dense monotonic key order.
 */
static int
read_result_table(struct spdk_nvme_ns *ns, struct spdk_nvme_qpair *qpair, uint64_t result_lba,
		  uint8_t *buf)
{
	uint32_t	sector_size = spdk_nvme_ns_get_sector_size(ns);
	uint32_t	max_xfer = spdk_nvme_ns_get_max_io_xfer_size(ns);
	uint32_t	chunk_sectors = max_xfer ? max_xfer / sector_size : 128;
	uint64_t	sectors_left = GROUPBY_RESULT_SECTORS;
	uint64_t	lba = result_lba;
	uint8_t		*dst = buf;
	uint32_t	this_chunk;
	struct read_ctx	ctx;
	int		rc;

	if (chunk_sectors == 0) {
		chunk_sectors = 1;
	}

	while (sectors_left > 0) {
		this_chunk = (uint32_t)spdk_min(sectors_left, (uint64_t)chunk_sectors);
		ctx.done = false;
		ctx.status = 0;

		rc = spdk_nvme_ns_cmd_read(ns, qpair, dst, lba, this_chunk, read_cb, &ctx, 0);
		if (rc != 0) {
			fprintf(stderr, "ERROR: read at LBA %lu (%u sectors) failed to submit: "
				"%d\n", lba, this_chunk, rc);
			return -1;
		}

		while (!ctx.done) {
			spdk_nvme_qpair_process_completions(qpair, 0);
		}
		if (ctx.status != 0) {
			fprintf(stderr, "ERROR: read at LBA %lu (%u sectors) completed with an "
				"error\n", lba, this_chunk);
			return -1;
		}

		dst += (uint64_t)this_chunk * sector_size;
		lba += this_chunk;
		sectors_left -= this_chunk;
	}

	return 0;
}

/* ================================================================================================
 * --verify: standalone check, independent of the environment holder below. Does not touch the
 * FPGA card -- only the drive the result table was written to.
 * ================================================================================================
 */

static void
verify_usage(void)
{
	printf("groupby --verify <result_lba> -t <trid> <sidecar.json> [<sidecar.json> ...]\n");
	printf("    <result_lba>   first LBA of the result table (decimal, or 0x-prefixed hex)\n");
	printf("    -t <trid>      transport ID of the drive holding the result table, in the\n");
	printf("                   same 'key:value ...' format the environment's own -t takes\n");
	printf("                   (e.g. \"trtype:PCIe traddr:0000:61:00.0\")\n");
	printf("    <sidecar...>   one \"%s\" reference sidecar (examples/nvme/kv_fill's\n",
	       GROUPBY_SIDECAR_FORMAT);
	printf("                   output) per drive that fed the run, each holding a\n");
	printf("                   %u-entry \"group_sum\" array plus \"key_mod\"\n", GROUPBY_NUM_GROUPS);
	printf("\n");
	printf("Reads the result table back with an ordinary SPDK read and compares it against\n");
	printf("the wrapped elementwise sum of every sidecar's group_sum array. Exits 0 when\n");
	printf("every group matches, non-zero on any mismatch or setup failure.\n");
}

static int
run_verify(int argc, char **argv)
{
	struct spdk_env_opts		env_opts = {0};
	struct spdk_nvme_transport_id	trid;
	struct sidecar			sidecars[GROUPBY_MAX_SIDECARS];
	const char			*sidecar_paths[GROUPBY_MAX_SIDECARS];
	uint32_t			num_sidecars = 0;
	uint64_t			result_lba = 0;
	bool				have_lba = false;
	bool				have_trid = false;
	uint64_t			*expected = NULL;
	uint8_t				*result_buf = NULL;
	struct spdk_nvme_ctrlr		*ctrlr = NULL;
	struct spdk_nvme_ns		*ns = NULL;
	struct spdk_nvme_qpair		*qpair = NULL;
	uint32_t			nsid;
	uint32_t			mismatches = 0;
	uint32_t			reported = 0;
	uint64_t			t0, t1;
	double				hz;
	int				argi;
	uint32_t			s, k, g;
	char				*endptr;
	int				exit_code = EXIT_FAILURE;
	int				rc;

	memset(&trid, 0, sizeof(trid));
	trid.trtype = SPDK_NVME_TRANSPORT_PCIE;

	for (argi = 2; argi < argc; argi++) {
		if (strcmp(argv[argi], "-t") == 0) {
			if (argi + 1 >= argc) {
				fprintf(stderr, "ERROR: -t needs an argument\n");
				verify_usage();
				return EXIT_FAILURE;
			}
			if (have_trid) {
				fprintf(stderr, "ERROR: --verify reads from exactly one drive; "
					"-t was given more than once\n");
				return EXIT_FAILURE;
			}
			argi++;
			if (spdk_nvme_transport_id_parse(&trid, argv[argi]) != 0) {
				fprintf(stderr, "ERROR: invalid transport ID '%s'\n", argv[argi]);
				return EXIT_FAILURE;
			}
			have_trid = true;
		} else if (!have_lba) {
			result_lba = strtoull(argv[argi], &endptr, 0);
			if (endptr == argv[argi] || *endptr != '\0') {
				fprintf(stderr, "ERROR: '%s' is not a valid result LBA\n",
					argv[argi]);
				verify_usage();
				return EXIT_FAILURE;
			}
			have_lba = true;
		} else {
			if (num_sidecars >= GROUPBY_MAX_SIDECARS) {
				fprintf(stderr, "ERROR: at most %u sidecars are supported\n",
					GROUPBY_MAX_SIDECARS);
				return EXIT_FAILURE;
			}
			sidecar_paths[num_sidecars++] = argv[argi];
		}
	}

	if (!have_lba || !have_trid || num_sidecars == 0) {
		fprintf(stderr, "ERROR: --verify needs a result LBA, -t <trid>, and at least one "
			"sidecar\n");
		verify_usage();
		return EXIT_FAILURE;
	}

	for (s = 0; s < num_sidecars; s++) {
		if (sidecar_load(sidecar_paths[s], &sidecars[s])) {
			for (k = 0; k < s; k++) {
				sidecar_free(&sidecars[k]);
			}
			return EXIT_FAILURE;
		}
	}

	/* Two sidecars for the same drive would double-count that drive's contribution against
	 * a result table it only fed once -- the same mistake kv_fill itself refuses for
	 * duplicate --seed values, checked here because kv_fill has no way to see this list. */
	for (s = 0; s < num_sidecars; s++) {
		for (k = s + 1; k < num_sidecars; k++) {
			if (sidecars[s].trid != NULL && sidecars[k].trid != NULL &&
			    strcmp(sidecars[s].trid, sidecars[k].trid) == 0) {
				fprintf(stderr, "ERROR: sidecars '%s' and '%s' both declare "
					"trid %s -- pass one sidecar per participating drive\n",
					sidecar_paths[s], sidecar_paths[k], sidecars[s].trid);
				for (g = 0; g < num_sidecars; g++) {
					sidecar_free(&sidecars[g]);
				}
				return EXIT_FAILURE;
			}
		}
	}

	expected = calloc(GROUPBY_NUM_GROUPS, sizeof(*expected));
	if (expected == NULL) {
		fprintf(stderr, "ERROR: out of memory allocating the reference table\n");
		goto free_sidecars;
	}

	printf("GROUP BY verify: result LBA %#lx (%s), %u sidecar(s)\n", result_lba,
	       trid.traddr, num_sidecars);
	for (s = 0; s < num_sidecars; s++) {
		printf("  [%u] %s: trid=%s seed=%#lx record_count=%lu key_mod=%u\n", s,
		       sidecar_paths[s], sidecars[s].trid ? sidecars[s].trid : "?",
		       sidecars[s].seed, sidecars[s].record_count, sidecars[s].key_mod);
		sidecar_accumulate(&sidecars[s], expected);
	}

	env_opts.opts_size = sizeof(env_opts);
	spdk_env_opts_init(&env_opts);
	env_opts.name = "groupby_verify";
	if (spdk_env_init(&env_opts) < 0) {
		fprintf(stderr, "ERROR: unable to initialise the SPDK env\n");
		goto free_expected;
	}

	ctrlr = spdk_nvme_connect(&trid, NULL, 0);
	if (ctrlr == NULL) {
		fprintf(stderr, "ERROR: unable to connect to %s\n", trid.traddr);
		goto env_fini;
	}

	nsid = spdk_nvme_ctrlr_get_first_active_ns(ctrlr);
	ns = spdk_nvme_ctrlr_get_ns(ctrlr, nsid);
	if (ns == NULL) {
		fprintf(stderr, "ERROR: %s has no active namespace\n", trid.traddr);
		goto detach;
	}

	if (spdk_nvme_ns_get_sector_size(ns) != GROUPBY_SECT_BYTES) {
		fprintf(stderr, "ERROR: %s reports %u B sectors, the result table needs %u B\n",
			trid.traddr, spdk_nvme_ns_get_sector_size(ns), GROUPBY_SECT_BYTES);
		goto detach;
	}

	if (result_lba + GROUPBY_RESULT_SECTORS > spdk_nvme_ns_get_num_sectors(ns)) {
		fprintf(stderr, "ERROR: result table at LBA %#lx..%#lx runs past %s's %lu "
			"sectors\n", result_lba,
			result_lba + GROUPBY_RESULT_SECTORS - 1, trid.traddr,
			spdk_nvme_ns_get_num_sectors(ns));
		goto detach;
	}

	qpair = spdk_nvme_ctrlr_alloc_io_qpair(ctrlr, NULL, 0);
	if (qpair == NULL) {
		fprintf(stderr, "ERROR: unable to allocate an I/O queue pair on %s\n",
			trid.traddr);
		goto detach;
	}

	result_buf = spdk_zmalloc(GROUPBY_RESULT_BYTES, 0x1000, NULL, SPDK_ENV_NUMA_ID_ANY,
				  SPDK_MALLOC_DMA);
	if (result_buf == NULL) {
		fprintf(stderr, "ERROR: unable to allocate the %u B read-back buffer\n",
			GROUPBY_RESULT_BYTES);
		goto free_qpair;
	}

	t0 = spdk_get_ticks();
	rc = read_result_table(ns, qpair, result_lba, result_buf);
	if (rc) {
		goto free_buf;
	}
	t1 = spdk_get_ticks();
	hz = (double)spdk_get_ticks_hz();
	printf("read back %u B in %.3f ms (host wall time, read-back only)\n",
	       GROUPBY_RESULT_BYTES, (double)(t1 - t0) / hz * 1e3);

	for (g = 0; g < GROUPBY_NUM_GROUPS; g++) {
		const uint8_t *rec = result_buf + (size_t)g * GROUPBY_REC_BYTES;
		uint64_t actual_key = le64_from_buf(rec);
		uint64_t actual_sum = le64_from_buf(rec + 8);

		if (actual_key != (uint64_t)g) {
			mismatches++;
			if (reported < VERIFY_MAX_MISMATCH_REPORT) {
				printf("  MISMATCH key %u: key field corrupt, holds %#lx\n", g,
				       actual_key);
				reported++;
			}
			continue;
		}
		if (actual_sum != expected[g]) {
			mismatches++;
			if (reported < VERIFY_MAX_MISMATCH_REPORT) {
				printf("  MISMATCH key %u: expected %lu, actual %lu\n", g,
				       expected[g], actual_sum);
				reported++;
			}
		}
	}

	if (mismatches > reported) {
		printf("  ... %u more mismatch(es) not shown\n", mismatches - reported);
	}

	printf("%u groups compared, %u mismatch(es)\n", GROUPBY_NUM_GROUPS, mismatches);
	if (mismatches == 0) {
		printf("PASS\n");
	} else {
		printf("FAIL\n");
	}
	exit_code = mismatches == 0 ? EXIT_SUCCESS : EXIT_FAILURE;

free_buf:
	spdk_free(result_buf);
free_qpair:
	spdk_nvme_ctrlr_free_io_qpair(qpair);
detach:
	spdk_nvme_detach(ctrlr);
env_fini:
	spdk_env_fini();
free_expected:
	free(expected);
free_sidecars:
	for (s = 0; s < num_sidecars; s++) {
		sidecar_free(&sidecars[s]);
	}
	return exit_code;
}

/* ================================================================================================
 * Environment holder: brings the DMA Iuventus environment up and keeps it up while
 * apps/iuventus/sw/iuventus_groupby.py drives GROUP BY runs over MI. Mirrors the bring-up and
 * teardown sequencing of fpga_zero_copy.c, without that tool's host-filesystem (ublk) bridge --
 * this app's only job while enabled is to keep the drives' admin queues serviced.
 * ================================================================================================
 */

static volatile sig_atomic_t g_stop;

static void
sig_handler(int signo)
{
	if (signo == SIGINT || signo == SIGTERM) {
		g_stop = 1;
	}
}

static void
usage(void)
{
	printf("groupby [options]\n");
	printf("Brings up the DMA Iuventus environment and holds it open so\n");
	printf("apps/iuventus/sw/iuventus_groupby.py can drive GROUP BY runs over MI while this\n");
	printf("process owns the card's queues and the drives behind them. SIGINT tears down\n");
	printf("cleanly: iuventus_disable() -> iuventus_drain() -> iuventus_fini(). Never SIGKILL\n");
	printf("this process while a run is in flight -- the drain barrier has to complete before\n");
	printf("the NVMe queues it is waiting on are deleted.\n");
	printf("\n");
	verify_usage();
}

static int
run_hold_open(int argc, char **argv)
{
	struct spdk_env_opts	env_opts = {0};
	struct iuventus_opts	iu_opts;
	struct iuventus_ctx	iu_ctx = {0};
	uint32_t		i;
	int			rc;

	env_opts.opts_size = sizeof(env_opts);
	spdk_env_opts_init(&env_opts);
	env_opts.name = "groupby";

	iuventus_opts_init(&iu_opts, sizeof(iu_opts));

	rc = iuventus_parse_args(argc, argv, &iu_opts, NULL, NULL, usage);
	if (rc == IUVENTUS_PARSE_ARGS_HELP) {
		return EXIT_SUCCESS;
	}
	if (rc != IUVENTUS_PARSE_ARGS_SUCCESS) {
		return EXIT_FAILURE;
	}

	if (spdk_env_init(&env_opts) < 0) {
		fprintf(stderr, "Unable to initialize SPDK env\n");
		return EXIT_FAILURE;
	}

	rc = iuventus_init(&iu_ctx, &iu_opts);
	if (rc) {
		fprintf(stderr, "Unable to bring up the DMA Iuventus environment: %d\n", rc);
		goto env_fini;
	}

	rc = iuventus_enable(&iu_ctx);
	if (rc) {
		fprintf(stderr, "Unable to enable the card: %d\n", rc);
		goto teardown;
	}

	signal(SIGINT, sig_handler);
	signal(SIGTERM, sig_handler);

	printf("DMA Iuventus environment up: %u drive(s)\n", iuventus_num_peers(&iu_ctx));
	for (i = 0; i < iuventus_num_peers(&iu_ctx); i++) {
		const struct iuventus_peer *peer = iuventus_peer(&iu_ctx, i);

		printf("  queue %u -> %s (%s)\n", i, peer->trid.traddr, peer->name);
	}
	printf("Holding the environment open -- drive the GROUP BY core over MI now. "
	       "SIGINT to tear down.\n");
	fflush(stdout);

	while (!g_stop) {
		for (i = 0; i < iuventus_num_peers(&iu_ctx); i++) {
			spdk_nvme_ctrlr_process_admin_completions(iuventus_peer(&iu_ctx, i)->ctrlr);
		}
		usleep(1000);
	}

	printf("SIGINT received, tearing down...\n");

	/*
	 * Explicit and in order, even though iuventus_fini() would do the same internally: the
	 * drain barrier must complete before the queues it is waiting on are deleted, which is
	 * the sequence this app was asked to prove out rather than delegate silently.
	 */
	iuventus_disable(&iu_ctx);
	if (iuventus_drain(&iu_ctx, IUVENTUS_DRAIN_TIMEOUT_MS)) {
		fprintf(stderr, "WARNING: tearing down a design that has not gone idle\n");
	}

teardown:
	iuventus_fini(&iu_ctx);
env_fini:
	spdk_env_fini();
	printf("DMA Iuventus environment torn down\n");

	return rc ? EXIT_FAILURE : EXIT_SUCCESS;
}

int
main(int argc, char **argv)
{
	if (argc >= 2 && strcmp(argv[1], "--verify") == 0) {
		return run_verify(argc, argv);
	}

	return run_hold_open(argc, argv);
}
