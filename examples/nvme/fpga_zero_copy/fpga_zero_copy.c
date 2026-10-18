/*
 *   fpga_zero_copy.c: Establishes peer-to-peer chain between an FPGA card and an NVMe device
 *   with FPGA as a master.
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) Vladislav Válek
 */

#include "spdk/stdinc.h"

#include "spdk/bdev.h"
#include "spdk/env.h"
#include "spdk/event.h"
#include "spdk/iuventus.h"
#include "spdk/json.h"
#include "spdk/log.h"
#include "spdk/memory.h"
#include "spdk/nvme.h"
#include "spdk/nvme_spec.h"
#include "spdk/rpc.h"
#include "spdk/string.h"
#include "spdk/thread.h"

/* Run scripts find fzc by this process name. */
#define APP_NAME "fpga_zero_copy"
/* bdev_nvme names drive q's controller <prefix><q> and its namespace bdev <prefix><q>n1. */
#define HOST_CTRLR_PREFIX "fpga_nvme_host"
/* Admin queues of the drives the library attached are polled at this period. */
#define ADMIN_POLL_PERIOD_US 1000

/* SIGUSR1 asks for the drives' log pages; the admin poller serves the request. */
static volatile sig_atomic_t    g_dumpreq = 0;

/* -r / -s: which reset to issue, whether it was reached, and how it went. */
/* Path for the PRP dump, or NULL to skip it entirely. */
static const char              *g_prp_dump_path = NULL;
static bool                     g_do_ctrl_rst = false;
static bool                     g_do_subs_rst = false;
static bool                     g_reset_issued = false;
static int                      g_reset_rc = 0;

/* App options are filled in from the option callback, so they live here rather than in main(). */
static struct spdk_app_opts     g_app_opts;

/* -U: drives (bit q = queue q's drive) whose namespace is also exposed to the host via ublk. */
static uint32_t                 g_host_fs_mask = 0;

static struct iuventus_opts     g_iu_opts;
static struct iuventus_ctx      g_iu_ctx;
static bool                     g_iu_up = false;
static struct spdk_poller      *g_admin_poller = NULL;

static void
sig_usr(int signo)
{
	g_dumpreq = 1;
}

/* Log NVMe Asynchronous Event Notifications as they arrive (wedge diagnostics). */
static void
aer_cb(void *arg, const struct spdk_nvme_cpl *cpl)
{
	uint32_t r = cpl->cdw0;
	uint8_t aet = r & 0x7;
	uint8_t aei = (r >> 8) & 0xff;
	uint8_t lid = (r >> 16) & 0xff;
	const char *t = aet == 0 ? "Error" : aet == 1 ? "SMART/Health" :
		aet == 2 ? "Notice" : aet == 3 ? "Immediate" : aet == 6 ? "IoCmdSet" :
		aet == 7 ? "Vendor" : "?";
	printf(">>> AEN cdw0=0x%08x type=%u(%s) info=0x%02x logpage=0x%02x sct=%u sc=%u <<<\n",
		r, aet, t, aei, lid, cpl->status.sct, cpl->status.sc);
	fflush(stdout);
}

/* P2P test: disable APST (needs 256B APST-table buffer) and force PS0 on the
 * live controller (volatile; Samsung rejects the SAVE bit). Verify via GET. */
static uint32_t g_feat_val;
static bool     g_feat_done;
static int      g_feat_status;
static void
feat_get_cb(void *ctx, const struct spdk_nvme_cpl *cpl)
{
	g_feat_done = true;
	if (spdk_nvme_cpl_is_error(cpl)) { g_feat_status = -1; return; }
	g_feat_val = cpl->cdw0;
}
static uint32_t
get_feature_val(struct spdk_nvme_ctrlr *ctrlr, uint8_t fid)
{
	void *buf = spdk_dma_zmalloc(4096, 0x1000, NULL);
	g_feat_done = false; g_feat_status = 0; g_feat_val = 0xffffffff;
	if (spdk_nvme_ctrlr_cmd_get_feature(ctrlr, fid, 0, buf, 4096,
					    feat_get_cb, NULL)) {
		if (buf) spdk_dma_free(buf);
		return 0xffffffff;
	}
	while (!g_feat_done)
		spdk_nvme_ctrlr_process_admin_completions(ctrlr);
	if (buf) spdk_dma_free(buf);
	return g_feat_val;
}
static bool g_set_done;
static int  g_set_status;
static void
feat_set_cb(void *ctx, const struct spdk_nvme_cpl *cpl)
{
	g_set_done = true;
	g_set_status = spdk_nvme_cpl_is_error(cpl) ? -1 : 0;
}
static int
set_feature_buf(struct spdk_nvme_ctrlr *ctrlr, uint8_t fid, uint32_t cdw11, void *buf, uint32_t len)
{
	g_set_done = false; g_set_status = 0;
	if (spdk_nvme_ctrlr_cmd_set_feature(ctrlr, fid, cdw11, 0,
					    buf, len, feat_set_cb, NULL))
		return -1;
	while (!g_set_done)
		spdk_nvme_ctrlr_process_admin_completions(ctrlr);
	return g_set_status;
}
static void
force_active_power_state(struct spdk_nvme_ctrlr *ctrlr)
{
	void *apst_buf;
	uint32_t a, pm;

	printf("P2P: forcing APST off + PS0 on live controller...\n");
	apst_buf = spdk_dma_zmalloc(256, 0x1000, NULL);
	if (set_feature_buf(ctrlr, 0x0c, 0x0, apst_buf, 256))
		fprintf(stderr, "WARN: SET APST-disable failed\n");
	if (set_feature_buf(ctrlr, 0x02, 0x0, NULL, 0))
		fprintf(stderr, "WARN: SET PS0 failed\n");
	if (apst_buf)
		spdk_dma_free(apst_buf);

	a = get_feature_val(ctrlr, 0x0c);
	pm = get_feature_val(ctrlr, 0x02);
	printf("P2P VERIFY: APST(0x0c) cdw0=0x%08x (APSTE bit0=%u); PWRMGMT(0x02) cdw0=0x%08x (PS=%u)\n",
	       a, a & 0x1, pm, pm & 0x1f);
}


/* P2P diagnostic: dump the drive's own NVMe log pages (Error Info 0x01,
 * SMART 0x02) and CSTS, decoded. Called at baseline and on SIGUSR1. */
static bool g_log_done;
static int  g_log_status;
static void
log_cb(void *ctx, const struct spdk_nvme_cpl *cpl)
{
	g_log_done = true;
	g_log_status = spdk_nvme_cpl_is_error(cpl) ? -1 : 0;
}
static int
get_log(struct spdk_nvme_ctrlr *ctrlr, uint8_t lid, void *buf, uint32_t sz)
{
	g_log_done = false; g_log_status = 0;
	if (spdk_nvme_ctrlr_cmd_get_log_page(ctrlr, lid, 0xffffffff,
					     buf, sz, 0, log_cb, NULL))
		return -1;
	while (!g_log_done)
		spdk_nvme_ctrlr_process_admin_completions(ctrlr);
	return g_log_status;
}
static void
dump_nvme_logs(struct spdk_nvme_ctrlr *ctrlr, const char *tag)
{
	void *buf = spdk_dma_zmalloc(4096, 0x1000, NULL);
	union spdk_nvme_csts_register csts;
	int i, shown;

	printf("==== NVME LOGS [%s] ====\n", tag);
	csts = spdk_nvme_ctrlr_get_regs_csts(ctrlr);
	printf("[%s] CSTS: rdy=%u cfs=%u shst=%u\n", tag,
	       csts.bits.rdy, csts.bits.cfs, csts.bits.shst);

	if (buf && get_log(ctrlr, 0x02, buf, 512) == 0) {
		struct spdk_nvme_health_information_page *h = buf;
		printf("[%s] SMART: critical_warning=0x%02x temperature=%uK media_errors=%lu num_err_log_entries=%lu ctrl_busy_time=%lu\n",
		       tag, h->critical_warning.raw, h->temperature,
		       (unsigned long)h->media_errors[0],
		       (unsigned long)h->num_error_info_log_entries[0],
		       (unsigned long)h->controller_busy_time[0]);
	} else {
		printf("[%s] SMART(0x02): READ FAILED\n", tag);
	}

	if (buf && get_log(ctrlr, 0x01, buf, 64 * sizeof(struct spdk_nvme_error_information_entry)) == 0) {
		struct spdk_nvme_error_information_entry *e = buf;
		shown = 0;
		printf("[%s] ERROR-INFO(0x01): entries with error_count>0:\n", tag);
		for (i = 0; i < 64; i++) {
			uint16_t st;
			if (e[i].error_count == 0)
				continue;
			st = *(uint16_t *)&e[i].status;
			printf("  [%s] ent%d: error_count=%lu sqid=%u cid=%u status=0x%04x(sct=%u sc=0x%02x) err_loc=0x%04x lba=%lu nsid=0x%x vs=0x%02x\n",
			       tag, i, (unsigned long)e[i].error_count, e[i].sqid, e[i].cid,
			       st, (st >> 9) & 0x7, (st >> 1) & 0xff,
			       e[i].error_location, (unsigned long)e[i].lba, e[i].nsid,
			       e[i].vendor_specific);
			if (++shown >= 8) break;
		}
		if (shown == 0)
			printf("  [%s] (no entries with error_count>0)\n", tag);
	} else {
		printf("[%s] ERROR-INFO(0x01): READ FAILED\n", tag);
	}
	if (buf) spdk_dma_free(buf);
	fflush(stdout);
}

static void
rd_ctrl_regs(struct spdk_nvme_ctrlr *ctrlr)
{
	union spdk_nvme_csts_register csts = spdk_nvme_ctrlr_get_regs_csts(ctrlr);
	printf("Controller CSTS register:\n");
	printf("\tCSTS.RDY: %d\n", csts.bits.rdy);
	printf("\tCSTS.CFS: %d\n", csts.bits.cfs);
	printf("\tCSTS.SHST: %d\n", csts.bits.shst);
	printf("\tCSTS.NSSRO: %d\n", csts.bits.nssro);
	printf("\tCSTS.PS: %d\n", csts.bits.pp);
}

/* Dump the page tables published to the drives, so a bad pool address is visible from the host.
 * Written to a file and only when -P names one: it is one line per managed page per buffer, and on
 * a terminal that is slow enough at large pool sizes to delay bring-up past what a caller waiting
 * on the design reasonably expects. */
static void
prp_list_dump(const struct iuventus_ctx *ctx, const char *path)
{
	uint32_t e;
	uint32_t i;
	FILE *f;

	if (path == NULL) {
		return;
	}

	f = fopen(path, "w");
	if (f == NULL) {
		fprintf(stderr, "cannot open PRP dump file %s: %s\n", path, strerror(errno));
		return;
	}

	for (e = 0; e < ctx->num_eps; e++) {
		const struct iuventus_ep *ep = &ctx->ep[e];

		fprintf(f, "Endpoint %u Write PRP List (vaddr: %p, paddr: 0x%lx, entries: %u):\n",
			e, ep->wrbuff_prp_list.vaddr, ep->wrbuff_prp_list.paddr,
			ep->wrbuff_prp_list.entries);
		for (i = 0; i < ep->wrbuff_prp_list.entries; i++) {
			fprintf(f, "\tEntry %u: 0x%lx\n", i, ((uint64_t *)ep->wrbuff_prp_list.vaddr)[i]);
		}

		fprintf(f, "Endpoint %u Read PRP List (vaddr: %p, paddr: 0x%lx, entries: %u):\n",
			e, ep->rdbuff_prp_list.vaddr, ep->rdbuff_prp_list.paddr,
			ep->rdbuff_prp_list.entries);
		for (i = 0; i < ep->rdbuff_prp_list.entries; i++) {
			fprintf(f, "\tEntry %u: 0x%lx\n", i, ((uint64_t *)ep->rdbuff_prp_list.vaddr)[i]);
		}
	}

	fclose(f);
	printf("PRP lists written to %s (%u endpoint(s))\n", path, ctx->num_eps);
}

static void
usage(void)
{
	printf("fpga_zero_copy [options]\n");
	printf("SPDK options:\n");
	printf("\t[-m DPDK huge memory size in MB]\n");
	printf("\t[-g use single file descriptor for DPDK memory segments]\n");
	printf("\t[-i shared memory group ID]\n");
#ifdef DEBUG
	printf("\t[-L enable debug logging]\n");
#else
	printf("\t[-L enable debug logging (flag disabled, must reconfigure with --enable-debug)]\n");
#endif
	printf("Application options:\n");
	printf("\t[-s do a subsystem reset (i.e. hard reset, the PCIe device can disappear from the system)]\n");
	printf("\t[-r do a controller reset (i.e. soft reset)]\n");
	printf("\t[-P file to dump the PRP lists into]\n");
	printf("\t[-U peer mask: bit q exposes queue q's drive to the host as /dev/ublkb<q> through\n");
	printf("\t     bdev_nvme, next to the card's queue pair (default 0 = none)]\n");
	printf("\t     Requires kernel >= 6.0 with CONFIG_BLK_DEV_UBLK (modprobe ublk_drv).\n");
}

/* Options this application owns; everything else belongs to the environment library. */
#define APP_GETOPT_STRING "si:gm:L:rP:U:"

static int
app_parse(int ch, char *arg)
{
	switch (ch) {
	case 's':
		g_do_subs_rst = true;
		break;
	case 'r':
		g_do_ctrl_rst = true;
		break;
	case 'P':
		g_prp_dump_path = arg;
		break;
	case 'U': {
		char *end = NULL;
		unsigned long mask;

		errno = 0;
		mask = strtoul(arg, &end, 0);
		if (errno != 0 || end == arg || *end != '\0' || mask == 0 ||
		    mask >= (1UL << IUVENTUS_MAX_PEERS)) {
			fprintf(stderr, "Invalid -U peer mask '%s'\n", arg);
			return -1;
		}
		g_host_fs_mask = (uint32_t)mask;
		break;
	}
	case 'i':
		g_app_opts.shm_id = spdk_strtol(arg, 10);
		if (g_app_opts.shm_id < 0) {
			fprintf(stderr, "Invalid shared memory ID\n");
			return -1;
		}
		break;
	case 'g':
		g_app_opts.hugepage_single_segments = true;
		break;
	case 'm':
		g_app_opts.mem_size = spdk_strtol(arg, 10);
		if (g_app_opts.mem_size < 0) {
			fprintf(stderr, "Invalid DPDK memory size\n");
			return -1;
		}
		break;
	case 'L':
		if (spdk_log_set_flag(arg) < 0) {
			fprintf(stderr, "unknown flag: %s\n", arg);
			return -1;
		}
#ifdef DEBUG
		spdk_log_set_print_level(SPDK_LOG_DEBUG);
#endif
		break;
	default:
		return -1;
	}

	return 0;
}

/*
 * A reset has to land on a controller that has attached but carries no I/O queue pair yet and no
 * card configuration behind it, which is where this hook runs. Returning non-zero stops the
 * bring-up, so iuventus_init() unwinds and detaches on the way out.
 */
static int
reset_peers_cb(struct iuventus_ctx *ctx, void *cb_arg)
{
	const struct iuventus_peer *peer = iuventus_peer(ctx, 0);

	if (g_do_ctrl_rst) {
		printf("Resetting controller...\n");
		if (spdk_nvme_ctrlr_reset(peer->ctrlr)) {
			fprintf(stderr, "Failed to reset controller!\n");
			g_reset_rc = -11;
		}
	} else if (g_do_subs_rst) {
		printf("Resetting subsystem...\n");
		if (spdk_nvme_ctrlr_reset_subsystem(peer->ctrlr)) {
			fprintf(stderr, "Failed to reset subsystem!\n");
			g_reset_rc = -12;
		}
	}

	/* A reset run does nothing else, so stop here whatever the reset returned. */
	g_reset_issued = true;
	return 1;
}

/*
 * The data plane runs on the card, so the host only services admin completions and asynchronous
 * events of the drives the library attached; bdev_nvme polls the admin queues of its own.
 */
static int
admin_poll(void *arg)
{
	uint32_t i;

	for (i = 0; i < iuventus_num_peers(&g_iu_ctx); i++) {
		const struct iuventus_peer *peer = iuventus_peer(&g_iu_ctx, i);

		if (!peer->caller_owned) {
			spdk_nvme_ctrlr_process_admin_completions(peer->ctrlr);
		}
	}
	if (g_dumpreq) {
		g_dumpreq = 0;
		for (i = 0; i < iuventus_num_peers(&g_iu_ctx); i++) {
			const struct iuventus_peer *peer = iuventus_peer(&g_iu_ctx, i);

			printf("--- drive %u (%s) ---\n", i, peer->trid.traddr);
			dump_nvme_logs(peer->ctrlr, "POST-WEDGE");
		}
	}
	return SPDK_POLLER_IDLE;
}

/* Growing, NUL-terminated buffer the JSON writer emits into. */
struct json_buf {
	char	*data;
	size_t	len;
};

static int
json_buf_write(void *cb_ctx, const void *data, size_t size)
{
	struct json_buf *buf = cb_ctx;
	char *grown = realloc(buf->data, buf->len + size + 1);

	if (grown == NULL) {
		return -ENOMEM;
	}
	memcpy(grown + buf->len, data, size);
	buf->len += size;
	grown[buf->len] = '\0';
	buf->data = grown;
	return 0;
}

static void
json_method_begin(struct spdk_json_write_ctx *w, const char *method)
{
	spdk_json_write_object_begin(w);
	spdk_json_write_named_string(w, "method", method);
	spdk_json_write_named_object_begin(w, "params");
}

static void
json_method_end(struct spdk_json_write_ctx *w)
{
	spdk_json_write_object_end(w);
	spdk_json_write_object_end(w);
}

/* JSON config that attaches every -U drive through bdev_nvme and exports its namespace via ublk. */
static char *
host_fs_config(void)
{
	struct json_buf buf = {};
	struct spdk_json_write_ctx *w;
	char name[32];
	uint32_t q;

	w = spdk_json_write_begin(json_buf_write, &buf, 0);
	if (w == NULL) {
		return NULL;
	}
	spdk_json_write_object_begin(w);
	spdk_json_write_named_array_begin(w, "subsystems");

	spdk_json_write_object_begin(w);
	spdk_json_write_named_string(w, "subsystem", "bdev");
	spdk_json_write_named_array_begin(w, "config");
	for (q = 0; q < g_iu_opts.num_peers; q++) {
		if (g_host_fs_mask & (1U << q)) {
			snprintf(name, sizeof(name), "%s%u", HOST_CTRLR_PREFIX, q);
			json_method_begin(w, "bdev_nvme_attach_controller");
			spdk_json_write_named_string(w, "name", name);
			spdk_json_write_named_string(w, "trtype", "PCIe");
			spdk_json_write_named_string(w, "traddr", g_iu_opts.peer_trid[q].traddr);
			json_method_end(w);
		}
	}
	spdk_json_write_array_end(w);
	spdk_json_write_object_end(w);

	spdk_json_write_object_begin(w);
	spdk_json_write_named_string(w, "subsystem", "ublk");
	spdk_json_write_named_array_begin(w, "config");
	json_method_begin(w, "ublk_create_target");
	spdk_json_write_named_string(w, "cpumask", "0x1");
	json_method_end(w);
	for (q = 0; q < g_iu_opts.num_peers; q++) {
		if (g_host_fs_mask & (1U << q)) {
			snprintf(name, sizeof(name), "%s%un1", HOST_CTRLR_PREFIX, q);
			json_method_begin(w, "ublk_start_disk");
			spdk_json_write_named_string(w, "bdev_name", name);
			spdk_json_write_named_uint32(w, "ublk_id", q);
			json_method_end(w);
		}
	}
	spdk_json_write_array_end(w);
	spdk_json_write_object_end(w);

	spdk_json_write_array_end(w);
	spdk_json_write_object_end(w);
	if (spdk_json_write_end(w) != 0) {
		free(buf.data);
		return NULL;
	}
	return buf.data;
}

static void
host_bdev_event_cb(enum spdk_bdev_event_type type, struct spdk_bdev *bdev, void *event_ctx)
{
}

/* Controller bdev_nvme attached for drive q, or NULL when its namespace bdev is missing. */
static struct spdk_nvme_ctrlr *
host_fs_ctrlr(uint32_t q)
{
	struct spdk_bdev_desc *desc;
	struct spdk_nvme_ns *ns = NULL;
	char name[32];

	snprintf(name, sizeof(name), "%s%un1", HOST_CTRLR_PREFIX, q);
	if (spdk_bdev_open_ext(name, false, host_bdev_event_cb, NULL, &desc) != 0) {
		return NULL;
	}
	if (strcmp(spdk_bdev_get_module_name(spdk_bdev_desc_get_bdev(desc)), "nvme") == 0) {
		ns = spdk_bdev_get_module_ctx(desc);
	}
	spdk_bdev_close(desc);

	return ns != NULL ? spdk_nvme_ns_get_ctrlr(ns) : NULL;
}

/* Resets every drive's controller in the order NVMe Base 2.3 lays out; see iuventus_reset_peers(). */
static void
rpc_iuventus_reset_controllers(struct spdk_jsonrpc_request *request,
			       const struct spdk_json_val *params)
{
	struct spdk_json_write_ctx *w;
	uint32_t i;
	int rc;

	if (params != NULL) {
		spdk_jsonrpc_send_error_response(request, SPDK_JSONRPC_ERROR_INVALID_PARAMS,
						 "iuventus_reset_controllers takes no parameters");
		return;
	}
	if (!g_iu_up) {
		spdk_jsonrpc_send_error_response(request, -ENODEV, "the card is not up");
		return;
	}

	rc = iuventus_reset_peers(&g_iu_ctx);
	if (rc != 0) {
		spdk_jsonrpc_send_error_response_fmt(request, rc, "controller reset failed: %s",
						     spdk_strerror(-rc));
		return;
	}

	w = spdk_jsonrpc_begin_result(request);
	spdk_json_write_array_begin(w);
	for (i = 0; i < iuventus_num_peers(&g_iu_ctx); i++) {
		const struct iuventus_peer *peer = iuventus_peer(&g_iu_ctx, i);
		union spdk_nvme_cap_register cap = spdk_nvme_ctrlr_get_regs_cap(peer->ctrlr);

		spdk_json_write_object_begin(w);
		spdk_json_write_named_string(w, "traddr", peer->trid.traddr);
		spdk_json_write_named_int32(w, "qid", peer->hw_qid);
		spdk_json_write_named_uint32(w, "cap_to_ms", cap.bits.to * 500);
		spdk_json_write_named_uint64(w, "disable_us", peer->reset_disable_us);
		spdk_json_write_named_uint64(w, "enable_us", peer->reset_enable_us);
		spdk_json_write_object_end(w);
	}
	spdk_json_write_array_end(w);
	spdk_jsonrpc_end_result(request, w);
}
SPDK_RPC_REGISTER("iuventus_reset_controllers", rpc_iuventus_reset_controllers, SPDK_RPC_RUNTIME)

/* SIGINT/SIGTERM land here through the app framework: stop the card before its drives go away. */
static void
fzc_shutdown(void)
{
	uint32_t i;

	spdk_poller_unregister(&g_admin_poller);
	if (g_iu_up) {
		iuventus_disable(&g_iu_ctx);
		if (iuventus_drain(&g_iu_ctx, IUVENTUS_DRAIN_TIMEOUT_MS)) {
			fprintf(stderr, "WARNING: tearing down a design that has not gone idle\n");
		}
		for (i = 0; i < iuventus_num_peers(&g_iu_ctx); i++) {
			rd_ctrl_regs(iuventus_peer(&g_iu_ctx, i)->ctrlr);
		}
		iuventus_fini(&g_iu_ctx);
		g_iu_up = false;
		printf("DMA Iuventus environment torn down\n");
	}
	spdk_app_stop(0);
}

/* Runs once the subsystems are up and, with -U, bdev_nvme and ublk hold the selected drives. */
static void
fzc_start(void *arg)
{
	uint32_t i;
	int rc;

	/* The reactor renamed the main thread, which carries the process name. */
	pthread_setname_np(pthread_self(), APP_NAME);

	for (i = 0; i < g_iu_opts.num_peers; i++) {
		if (!(g_host_fs_mask & (1U << i))) {
			continue;
		}
		g_iu_opts.peer_ctrlr[i] = host_fs_ctrlr(i);
		if (g_iu_opts.peer_ctrlr[i] == NULL) {
			fprintf(stderr, "bdev_nvme holds no controller for drive %u (%s)\n", i,
				g_iu_opts.peer_trid[i].traddr);
			spdk_app_stop(-ENODEV);
			return;
		}
		printf("Host filesystem bdev %s%un1 registered as: /dev/ublkb%u\n",
		       HOST_CTRLR_PREFIX, i, i);
	}

	if (g_do_ctrl_rst || g_do_subs_rst) {
		g_iu_opts.on_peers_ready = reset_peers_cb;
	}

	rc = iuventus_init(&g_iu_ctx, &g_iu_opts);
	if (g_reset_issued) {
		/* The hook stopped the bring-up on purpose once the reset had been issued, so the
		 * reset's own result is what this run reports. */
		spdk_app_stop(g_reset_rc);
		return;
	}
	if (rc) {
		fprintf(stderr, "Unable to bring up the DMA Iuventus environment: %d\n", rc);
		spdk_app_stop(rc);
		return;
	}
	g_iu_up = true;

	for (i = 0; i < iuventus_num_peers(&g_iu_ctx); i++) {
		const struct iuventus_peer *peer = iuventus_peer(&g_iu_ctx, i);

		printf("--- drive %u (%s) ---\n", i, peer->trid.traddr);
		/* bdev_nvme owns the event callback of the drives it attached. */
		if (!peer->caller_owned) {
			spdk_nvme_ctrlr_register_aer_callback(peer->ctrlr, aer_cb, NULL);
		}
		rd_ctrl_regs(peer->ctrlr);
		force_active_power_state(peer->ctrlr);
		dump_nvme_logs(peer->ctrlr, "BASELINE");
	}

	prp_list_dump(&g_iu_ctx, g_prp_dump_path);

	rc = iuventus_enable(&g_iu_ctx);
	if (rc) {
		fprintf(stderr, "Unable to enable the card: %d\n", rc);
		iuventus_fini(&g_iu_ctx);
		g_iu_up = false;
		spdk_app_stop(rc);
		return;
	}

	g_admin_poller = SPDK_POLLER_REGISTER(admin_poll, NULL, ADMIN_POLL_PERIOD_US);
	signal(SIGUSR1, sig_usr);

	printf("Initialization complete. Starting main loop.\n");
	fflush(stdout);
}

int
main(int argc, char **argv)
{
	char *json = NULL;
	int rc;

	spdk_app_opts_init(&g_app_opts, sizeof(g_app_opts));
	g_app_opts.name = APP_NAME;
	g_app_opts.shutdown_cb = fzc_shutdown;

	iuventus_opts_init(&g_iu_opts, sizeof(g_iu_opts));

	rc = iuventus_parse_args(argc, argv, &g_iu_opts, APP_GETOPT_STRING, app_parse, usage);
	if (rc == IUVENTUS_PARSE_ARGS_HELP) {
		return EXIT_SUCCESS;
	}
	if (rc != IUVENTUS_PARSE_ARGS_SUCCESS) {
		return EXIT_FAILURE;
	}
	if (g_host_fs_mask >> g_iu_opts.num_peers) {
		fprintf(stderr, "-U 0x%x names a drive past the %u given with -t\n",
			g_host_fs_mask, g_iu_opts.num_peers);
		return EXIT_FAILURE;
	}
	if (g_host_fs_mask != 0 && (g_do_ctrl_rst || g_do_subs_rst)) {
		fprintf(stderr, "-U cannot be combined with -r or -s: bdev_nvme would lose its drive\n");
		return EXIT_FAILURE;
	}

	if (g_host_fs_mask != 0) {
		json = host_fs_config();
		if (json == NULL) {
			fprintf(stderr, "Unable to build the host filesystem configuration\n");
			return EXIT_FAILURE;
		}
		g_app_opts.json_data = json;
		g_app_opts.json_data_size = strlen(json);
	}
	/* Without -U only timed pollers run, so the reactor may sleep between them. bdev_nvme gets no
	 * interrupts from a drive on uio_pci_generic, so -U keeps the reactor polling. */
	g_app_opts.interrupt_mode = (g_host_fs_mask == 0);

	rc = spdk_app_start(&g_app_opts, fzc_start, NULL);
	spdk_app_fini();
	free(json);
	printf("Env finished\n");
	return rc;
}
