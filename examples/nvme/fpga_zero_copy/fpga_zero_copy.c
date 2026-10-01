/*
 *   fpga_zero_copy.c: Establishes peer-to-peer chain between an FPGA card and an NVMe device
 *   with FPGA as a master.
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) Vladislav Válek
 */

#include <stdint.h>
#include <stdlib.h>
#include <sys/queue.h>
#include <signal.h>
#include <pthread.h>

#include "spdk/stdinc.h"

#include "spdk/nvme.h"
#include "spdk/nvme_spec.h"
#include "spdk/vmd.h"
#include "spdk/env.h"
#include "spdk/string.h"
#include "spdk/log.h"
#include "spdk/memory.h"
#include "spdk/accel.h"
#include "spdk/bdev.h"
#include "spdk/bdev_module.h"
#include "spdk/thread.h"
#include "spdk/ublk.h"
#include "spdk/iuventus.h"
#include "ublk_internal.h"

/*
 * ============================================================
 * Host Filesystem Bdev Module
 *
 * This module wraps host-side NVMe software queue pairs as
 * an SPDK bdev so that the host can access the NVMe namespace
 * through the standard SPDK block-device layer and, from there,
 * through ublk as a regular Linux block device (/dev/ublkb<q> for
 * every drive q selected with -U).
 *
 * The FPGA uses its own queue pair (hw_qid, managed via raw
 * admin commands) while each host I/O channel owns its own
 * software queue pair,
 * so no cross-device locking is required.
 * ============================================================
 */

/* Bdev name prefix; drive q's bdev is fpga_nvme_host<q> and its kernel disk /dev/ublkb<q>. */
#define NVME_HOST_BDEV_PREFIX "fpga_nvme_host"
/* Host I/O queue pair per drive: deeper than one ublk queue (128) so a full ublk queue fits. */
#define NVME_HOST_QPAIR_SIZE  256
/* Upper bound on any one host-fs control step; a step that misses it is reported, not waited on. */
#define HOST_FS_STEP_TIMEOUT_MS 15000

/* Opaque per-bdev context. */
struct nvme_host_bdev {
	struct spdk_bdev        bdev;
	struct spdk_nvme_ns    *ns;
};

/* Per-I/O-channel context: one poller drives NVMe completions. */
struct nvme_host_io_channel {
	struct nvme_host_bdev *bdev_ctx;
	struct spdk_nvme_qpair *qpair;
	struct spdk_poller    *poller;
};

/*
 * SGL iterator state used by readv/writev path.
 * Stored per submitted bdev_io and freed in completion callback.
 */
struct nvme_host_sgl_ctx {
	struct spdk_bdev_io *bdev_io;
	struct iovec        *iovs;
	int                  iovcnt;
	int                  iovpos;
	uint32_t             iov_offset;
};

/* Forward declarations. */
static int  nvme_host_module_init(void);
static void nvme_host_bdev_submit_request(struct spdk_io_channel *ch,
					  struct spdk_bdev_io *bdev_io);
static void nvme_host_read_get_buf_cb(struct spdk_io_channel *ch,
				      struct spdk_bdev_io *bdev_io,
				      bool success);
static bool nvme_host_bdev_io_type_supported(void *ctx,
					     enum spdk_bdev_io_type io_type);
static struct spdk_io_channel *nvme_host_bdev_get_io_channel(void *ctx);
static int  nvme_host_bdev_destruct(void *ctx);

static struct spdk_bdev_module g_nvme_host_module = {
	.name        = "nvme_host",
	.module_init = nvme_host_module_init,
};

SPDK_BDEV_MODULE_REGISTER(nvme_host, &g_nvme_host_module)

static int
nvme_host_module_init(void)
{
	return 0;
}

/* Called when an NVMe read/write command issued through the bdev layer completes. */
static void
nvme_host_io_cb(void *cb_arg, const struct spdk_nvme_cpl *cpl)
{
	struct spdk_bdev_io *bdev_io = cb_arg;

	spdk_bdev_io_complete(bdev_io,
			      spdk_nvme_cpl_is_error(cpl)
			      ? SPDK_BDEV_IO_STATUS_FAILED
			      : SPDK_BDEV_IO_STATUS_SUCCESS);
}

static void
nvme_host_sgl_reset(void *cb_arg, uint32_t offset)
{
	struct nvme_host_sgl_ctx *sgl_ctx = cb_arg;
	struct iovec *iov;

	/* Position iterator to the SGL element containing byte offset. */
	sgl_ctx->iov_offset = offset;
	for (sgl_ctx->iovpos = 0; sgl_ctx->iovpos < sgl_ctx->iovcnt; sgl_ctx->iovpos++) {
		iov = &sgl_ctx->iovs[sgl_ctx->iovpos];
		if (sgl_ctx->iov_offset < iov->iov_len) {
			break;
		}

		sgl_ctx->iov_offset -= iov->iov_len;
	}
}

static int
nvme_host_sgl_next(void *cb_arg, void **address, uint32_t *length)
{
	struct nvme_host_sgl_ctx *sgl_ctx = cb_arg;
	struct iovec *iov;

	/* Signal malformed request if iterator is out of bounds. */
	if (sgl_ctx->iovpos >= sgl_ctx->iovcnt) {
		return -EINVAL;
	}

	/* Return current segment and apply intra-segment offset if needed. */
	iov = &sgl_ctx->iovs[sgl_ctx->iovpos];
	*address = iov->iov_base;
	*length = iov->iov_len;

	if (sgl_ctx->iov_offset) {
		if (sgl_ctx->iov_offset > iov->iov_len) {
			return -EINVAL;
		}
		*address += sgl_ctx->iov_offset;
		*length -= sgl_ctx->iov_offset;
	}

	sgl_ctx->iov_offset += *length;
	if (sgl_ctx->iov_offset == iov->iov_len) {
		sgl_ctx->iovpos++;
		sgl_ctx->iov_offset = 0;
	}

	/* Continue until all segments are consumed. */
	return 0;
}

static void
nvme_host_io_sgl_cb(void *cb_arg, const struct spdk_nvme_cpl *cpl)
{
	struct nvme_host_sgl_ctx *sgl_ctx = cb_arg;
	struct spdk_bdev_io *bdev_io = sgl_ctx->bdev_io;

	/* SGL context lifetime ends when NVMe command completes. */
	free(sgl_ctx);

	spdk_bdev_io_complete(bdev_io,
			      spdk_nvme_cpl_is_error(cpl)
			      ? SPDK_BDEV_IO_STATUS_FAILED
			      : SPDK_BDEV_IO_STATUS_SUCCESS);
}

static void
nvme_host_read_get_buf_cb(struct spdk_io_channel *ch,
			  struct spdk_bdev_io *bdev_io,
			  bool success)
{
	if (!success) {
		spdk_bdev_io_complete(bdev_io, SPDK_BDEV_IO_STATUS_FAILED);
		return;
	}

	nvme_host_bdev_submit_request(ch, bdev_io);
}

static void
nvme_host_bdev_submit_request(struct spdk_io_channel *ch, struct spdk_bdev_io *bdev_io)
{
	struct nvme_host_io_channel *host_ch = spdk_io_channel_get_ctx(ch);
	struct nvme_host_bdev *bdev_ctx = host_ch->bdev_ctx;
	struct nvme_host_sgl_ctx *sgl_ctx = NULL;
	int rc = 0;

	switch (bdev_io->type) {
	case SPDK_BDEV_IO_TYPE_READ:
		/* Acquire a data buffer for requests that arrived without iovecs. */
		if (bdev_io->u.bdev.iovcnt <= 0) {
			spdk_bdev_io_get_buf(bdev_io, nvme_host_read_get_buf_cb,
					     bdev_io->u.bdev.num_blocks * bdev_io->bdev->blocklen);
			return;
		}

		if (bdev_io->u.bdev.iovcnt == 1) {
			/* Fast path for contiguous payload. */
			rc = spdk_nvme_ns_cmd_read(bdev_ctx->ns, host_ch->qpair,
						   bdev_io->u.bdev.iovs[0].iov_base,
						   bdev_io->u.bdev.offset_blocks,
						   bdev_io->u.bdev.num_blocks,
						   nvme_host_io_cb, bdev_io, 0);
		} else {
			/* Multi-iov path uses readv + SGL callbacks. */
			sgl_ctx = calloc(1, sizeof(*sgl_ctx));
			if (sgl_ctx == NULL) {
				rc = -ENOMEM;
				break;
			}

			/* Capture request scatter-gather array for callback iteration. */
			sgl_ctx->bdev_io = bdev_io;
			sgl_ctx->iovs = bdev_io->u.bdev.iovs;
			sgl_ctx->iovcnt = bdev_io->u.bdev.iovcnt;

			rc = spdk_nvme_ns_cmd_readv(bdev_ctx->ns, host_ch->qpair,
						    bdev_io->u.bdev.offset_blocks,
						    bdev_io->u.bdev.num_blocks,
						    nvme_host_io_sgl_cb, sgl_ctx, 0,
						    nvme_host_sgl_reset, nvme_host_sgl_next);
		}
		break;
	case SPDK_BDEV_IO_TYPE_WRITE:
		/* Defensive guard: bdev request must contain at least one iovec. */
		if (bdev_io->u.bdev.iovcnt <= 0) {
			rc = -EINVAL;
			break;
		}

		if (bdev_io->u.bdev.iovcnt == 1) {
			/* Fast path for contiguous payload. */
			rc = spdk_nvme_ns_cmd_write(bdev_ctx->ns, host_ch->qpair,
						    bdev_io->u.bdev.iovs[0].iov_base,
						    bdev_io->u.bdev.offset_blocks,
						    bdev_io->u.bdev.num_blocks,
						    nvme_host_io_cb, bdev_io, 0);
		} else {
			/* Multi-iov path uses writev + SGL callbacks. */
			sgl_ctx = calloc(1, sizeof(*sgl_ctx));
			if (sgl_ctx == NULL) {
				rc = -ENOMEM;
				break;
			}

			/* Capture request scatter-gather array for callback iteration. */
			sgl_ctx->bdev_io = bdev_io;
			sgl_ctx->iovs = bdev_io->u.bdev.iovs;
			sgl_ctx->iovcnt = bdev_io->u.bdev.iovcnt;

			rc = spdk_nvme_ns_cmd_writev(bdev_ctx->ns, host_ch->qpair,
						     bdev_io->u.bdev.offset_blocks,
						     bdev_io->u.bdev.num_blocks,
						     nvme_host_io_sgl_cb, sgl_ctx, 0,
						     nvme_host_sgl_reset, nvme_host_sgl_next);
		}
		break;
	case SPDK_BDEV_IO_TYPE_FLUSH:
		rc = spdk_nvme_ns_cmd_flush(bdev_ctx->ns, host_ch->qpair,
						nvme_host_io_cb, bdev_io);
		break;

	default:
		spdk_bdev_io_complete(bdev_io, SPDK_BDEV_IO_STATUS_FAILED);
		return;
	}

	if (rc == -ENOMEM) {
		if (sgl_ctx != NULL) {
			free(sgl_ctx);
		}

		spdk_bdev_io_complete(bdev_io, SPDK_BDEV_IO_STATUS_NOMEM);
		return;
	}

	if (rc) {
		spdk_bdev_io_complete(bdev_io, SPDK_BDEV_IO_STATUS_FAILED);
	}

	if (rc && sgl_ctx != NULL) {
		/* Submission failed before completion callback could own the context. */
		free(sgl_ctx);
	}
}

static bool
nvme_host_bdev_io_type_supported(void *ctx, enum spdk_bdev_io_type io_type)
{
	switch (io_type) {
	case SPDK_BDEV_IO_TYPE_READ:
	case SPDK_BDEV_IO_TYPE_WRITE:
	case SPDK_BDEV_IO_TYPE_FLUSH:
		return true;
	default:
		return false;
	}
}

static struct spdk_io_channel *
nvme_host_bdev_get_io_channel(void *ctx)
{
	return spdk_get_io_channel(ctx);
}

static int
nvme_host_bdev_destruct(void *ctx)
{
	struct nvme_host_bdev *bdev_ctx = ctx;

	spdk_io_device_unregister(bdev_ctx, NULL);
	free(bdev_ctx->bdev.name);
	free(bdev_ctx);
	return 0;
}

/* Poller: drains the host SW queue pair's completion queue. */
static int
nvme_host_io_poll(void *arg)
{
	struct nvme_host_io_channel *ch = arg;
	int32_t completions;

	completions = spdk_nvme_qpair_process_completions(ch->qpair, 0);
	return completions > 0 ? SPDK_POLLER_BUSY : SPDK_POLLER_IDLE;
}

static int
nvme_host_create_ch(void *io_device, void *ctx_buf)
{
	struct nvme_host_bdev *bdev_ctx = io_device;
	struct nvme_host_io_channel *ch = ctx_buf;

	struct spdk_nvme_ctrlr *ctrlr = spdk_nvme_ns_get_ctrlr(bdev_ctx->ns);
	struct spdk_nvme_io_qpair_opts qopts;

	/* Explicit size: the controller default is the card's -q depth, shallower than a ublk queue. */
	spdk_nvme_ctrlr_get_default_io_qpair_opts(ctrlr, &qopts, sizeof(qopts));
	qopts.io_queue_size     = NVME_HOST_QPAIR_SIZE;
	qopts.io_queue_requests = NVME_HOST_QPAIR_SIZE;

	ch->bdev_ctx = bdev_ctx;
	ch->qpair = spdk_nvme_ctrlr_alloc_io_qpair(ctrlr, &qopts, sizeof(qopts));
	if (ch->qpair == NULL) {
		return -ENOMEM;
	}

	ch->poller = SPDK_POLLER_REGISTER(nvme_host_io_poll, ch, 0);
	if (ch->poller == NULL) {
		spdk_nvme_ctrlr_free_io_qpair(ch->qpair);
		ch->qpair = NULL;
		return -ENOMEM;
	}

	return 0;
}

static void
nvme_host_destroy_ch(void *io_device, void *ctx_buf)
{
	struct nvme_host_io_channel *ch = ctx_buf;

	spdk_poller_unregister(&ch->poller);
	if (ch->qpair != NULL) {
		spdk_nvme_ctrlr_free_io_qpair(ch->qpair);
		ch->qpair = NULL;
	}
}

static const struct spdk_bdev_fn_table g_nvme_host_fn_table = {
	.destruct          = nvme_host_bdev_destruct,
	.submit_request    = nvme_host_bdev_submit_request,
	.io_type_supported = nvme_host_bdev_io_type_supported,
	.get_io_channel    = nvme_host_bdev_get_io_channel,
};

static struct nvme_host_bdev *g_nvme_host_bdev[IUVENTUS_MAX_PEERS];
static bool                   g_ublk_started[IUVENTUS_MAX_PEERS];
static struct spdk_thread    *g_host_thread = NULL;
/* How far host_fs_lib_init() got, so host_fs_fini() undoes exactly that much. */
enum host_fs_stage { HOST_FS_NONE, HOST_FS_ACCEL, HOST_FS_IOBUF, HOST_FS_BDEV, HOST_FS_UBLK };
static enum host_fs_stage     g_host_fs_stage = HOST_FS_NONE;

/*
 * Thread registry: ublk_create_target() spawns one SPDK thread per
 * requested CPU core.  We collect them all here so that the main loop
 * can drive every thread without the reactor framework.
 *
 * 64 is far larger than any practical CPU count; if the limit is ever
 * hit a warning is emitted so the value can be adjusted.
 */
#define HOST_FS_MAX_THREADS 64
static struct spdk_thread *g_all_threads[HOST_FS_MAX_THREADS];
static int                 g_num_threads = 0;

/*
 * SPDK thread-library operation callback: called whenever a new SPDK
 * thread is created (e.g. by ublk_create_target).  We register the
 * thread so the main loop can poll it.
 */
static int
host_thread_op_fn(struct spdk_thread *thread)
{
	if (g_num_threads < HOST_FS_MAX_THREADS) {
		g_all_threads[g_num_threads++] = thread;
		return 0;
	} else {
		fprintf(stderr, "WARNING: HOST_FS_MAX_THREADS (%d) exceeded; "
			"new SPDK thread will not be polled by the main loop.\n",
			HOST_FS_MAX_THREADS);
		return -1;
	}
	return 0;
}

static void
host_thread_exit_msg(void *cb_arg)
{
	spdk_thread_exit(spdk_get_thread());
}

/* Poll every tracked SPDK thread once; returns non-zero when any of them did work. */
static int
host_fs_poll_all(void)
{
	int busy = 0;
	int i;

	for (i = 0; i < g_num_threads; i++) {
		if (g_all_threads[i] != NULL && spdk_thread_poll(g_all_threads[i], 0, 0) > 0) {
			busy = 1;
		}
	}
	return busy;
}

static uint64_t
host_fs_now_ms(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

/* Polls every tracked thread until *done is set or the step deadline passes (-ETIMEDOUT). */
static int
host_fs_wait(volatile bool *done, const char *what)
{
	uint64_t deadline = host_fs_now_ms() + HOST_FS_STEP_TIMEOUT_MS;

	while (!*done) {
		host_fs_poll_all();
		if (host_fs_now_ms() > deadline) {
			fprintf(stderr, "ERROR: host fs: '%s' did not finish within %d ms\n",
				what, HOST_FS_STEP_TIMEOUT_MS);
			return -ETIMEDOUT;
		}
	}
	return 0;
}

/*
 * Register a thin bdev backed by the host SW queue pair so that the
 * NVMe namespace can be exposed as a standard Linux block device.
 */
static int
nvme_host_bdev_create(uint32_t q, const char *name, struct spdk_nvme_ns *ns)
{
	struct nvme_host_bdev *bdev_ctx;
	int rc;

	bdev_ctx = calloc(1, sizeof(*bdev_ctx));
	if (!bdev_ctx) {
		return -ENOMEM;
	}

	bdev_ctx->ns = ns;

	bdev_ctx->bdev.ctxt         = bdev_ctx;
	bdev_ctx->bdev.name         = strdup(name);
	bdev_ctx->bdev.product_name = "FPGA NVMe Host Bdev";
	bdev_ctx->bdev.fn_table     = &g_nvme_host_fn_table;
	bdev_ctx->bdev.module       = &g_nvme_host_module;
	bdev_ctx->bdev.write_cache  = 1;
	bdev_ctx->bdev.blocklen     = spdk_nvme_ns_get_sector_size(ns);
	bdev_ctx->bdev.blockcnt     = spdk_nvme_ns_get_num_sectors(ns);

	spdk_io_device_register(bdev_ctx,
				nvme_host_create_ch, nvme_host_destroy_ch,
				sizeof(struct nvme_host_io_channel),
				name);

	rc = spdk_bdev_register(&bdev_ctx->bdev);
	if (rc) {
		spdk_io_device_unregister(bdev_ctx, NULL);
		free(bdev_ctx->bdev.name);
		free(bdev_ctx);
		return rc;
	}

	g_nvme_host_bdev[q] = bdev_ctx;
	return 0;
}

/* Callbacks used while initialising / finalising the bdev subsystem. */
struct host_fs_sync_ctx {
	int           rc;
	volatile bool done;
	uint32_t      id;
};

static void
bdev_init_done_cb(void *cb_arg, int rc)
{
	struct host_fs_sync_ctx *ctx = cb_arg;

	ctx->rc   = rc;
	ctx->done = true;
}

static void
bdev_fini_done_cb(void *cb_arg)
{
	struct host_fs_sync_ctx *ctx = cb_arg;

	ctx->done = true;
}

/* Called once ublk_start_disk() reports success or failure. */
static void
ublk_start_cb(void *cb_arg, int result)
{
	struct host_fs_sync_ctx *ctx = cb_arg;

	ctx->rc   = result;
	ctx->done = true;

	if (result) {
		fprintf(stderr, "ERROR: Failed to start ublk disk (id=%u): %d\n", ctx->id, result);
	} else {
		printf("Host filesystem bdev %s%u registered as: /dev/ublkb%u\n",
		       NVME_HOST_BDEV_PREFIX, ctx->id, ctx->id);
	}
	fflush(stdout);
}

/* Called once ublk_stop_disk() completes. */
static void
ublk_stop_cb(void *cb_arg, int result)
{
	struct host_fs_sync_ctx *ctx = cb_arg;

	ctx->rc   = result;
	ctx->done = true;
}

static void host_fs_fini(void);

/*
 * Start the host-filesystem libraries once, before any drive is added:
 *  1. The SPDK thread library (with a thread-tracking hook so that the
 *     extra threads spawned by ublk_create_target are polled by the main loop).
 *  2. accel, iobuf and the bdev subsystem.
 *  3. The ublk target. Requires kernel >= 6.0 with CONFIG_BLK_DEV_UBLK.
 *
 * Returns 0 on success, negative errno on failure (everything started is undone).
 */
static int
host_fs_lib_init(void)
{
	struct host_fs_sync_ctx ctx = {0};
	int rc;

	rc = spdk_thread_lib_init(host_thread_op_fn, 0);
	if (rc) {
		fprintf(stderr, "ERROR: spdk_thread_lib_init() failed: %d\n", rc);
		return rc;
	}

	/* Create the primary host-filesystem SPDK thread (the app thread ublk control runs on). */
	g_host_thread = spdk_thread_create("host_fs", NULL);
	if (!g_host_thread) {
		fprintf(stderr, "ERROR: Failed to create host_fs SPDK thread\n");
		spdk_thread_lib_fini();
		return -ENOMEM;
	}
	spdk_set_thread(g_host_thread);

	/*
	 * The bdev layer acquires an accel I/O channel for each bdev channel.
	 * When this standalone example bypasses the normal SPDK app startup
	 * path, the accel framework must be initialized explicitly.
	 */
	rc = spdk_accel_initialize();
	if (rc) {
		fprintf(stderr, "ERROR: spdk_accel_initialize() failed: %d\n", rc);
		goto fail;
	}
	g_host_fs_stage = HOST_FS_ACCEL;

	/* Both the bdev I/O-channel layer and the ublk driver need the iobuf pool. */
	rc = spdk_iobuf_initialize();
	if (rc) {
		fprintf(stderr, "ERROR: spdk_iobuf_initialize() failed: %d\n", rc);
		goto fail;
	}
	g_host_fs_stage = HOST_FS_IOBUF;

	/* Initialise the bdev subsystem; our custom module_init() is a no-op. */
	spdk_bdev_initialize(bdev_init_done_cb, &ctx);
	rc = host_fs_wait(&ctx.done, "bdev initialize");
	if (rc == 0 && ctx.rc) {
		fprintf(stderr, "ERROR: spdk_bdev_initialize() failed: %d\n", ctx.rc);
		rc = ctx.rc;
	}
	if (rc) {
		goto fail;
	}
	g_host_fs_stage = HOST_FS_BDEV;

	/*
	 * ublk uses io_uring for the kernel<->user path and spawns one poll-group
	 * thread per CPU in the mask; those are captured by host_thread_op_fn()
	 * and polled by the main loop. One CPU (core 0) keeps the thread count
	 * small in this non-reactor environment.
	 */
	spdk_ublk_init();
	rc = ublk_create_target("0x1", NULL);
	if (rc) {
		fprintf(stderr, "ERROR: ublk_create_target() failed: %d\n", rc);
		goto fail;
	}
	g_host_fs_stage = HOST_FS_UBLK;

	return 0;

fail:
	host_fs_fini();
	return rc;
}

/* Expose drive q's namespace through its own bdev and ublk disk /dev/ublkb<q>. */
static int
host_fs_add(uint32_t q, struct spdk_nvme_ns *ns)
{
	struct host_fs_sync_ctx ctx = {0};
	char name[32];
	int rc;

	snprintf(name, sizeof(name), "%s%u", NVME_HOST_BDEV_PREFIX, q);
	rc = nvme_host_bdev_create(q, name, ns);
	if (rc) {
		fprintf(stderr, "ERROR: nvme_host_bdev_create(%s) failed: %d\n", name, rc);
		return rc;
	}

	ctx.id = q;
	rc = ublk_start_disk(name, q, UBLK_DEV_NUM_QUEUE, UBLK_DEV_QUEUE_DEPTH,
			     ublk_start_cb, &ctx);
	if (rc) {
		fprintf(stderr, "ERROR: ublk_start_disk(%s) failed: %d\n", name, rc);
		return rc;
	}
	rc = host_fs_wait(&ctx.done, "ublk start disk");
	if (rc) {
		return rc;
	}
	if (ctx.rc) {
		return ctx.rc;
	}
	g_ublk_started[q] = true;
	return 0;
}

/*
 * Tear down the host filesystem path in reverse initialisation order. Safe on a partial
 * bring-up; a step that misses its deadline is reported and the teardown carries on.
 */
static void
host_fs_fini(void)
{
	struct host_fs_sync_ctx ctx;
	uint64_t deadline;
	uint32_t q;
	int i;

	if (!g_host_thread) {
		return;
	}
	spdk_set_thread(g_host_thread);

	/* Stop every ublk disk first, then destroy the target. */
	for (q = 0; q < IUVENTUS_MAX_PEERS; q++) {
		if (!g_ublk_started[q]) {
			continue;
		}
		ctx = (struct host_fs_sync_ctx){0};
		if (ublk_stop_disk(q, ublk_stop_cb, &ctx) == 0) {
			if (host_fs_wait(&ctx.done, "ublk stop disk") == 0) {
				printf("/dev/ublkb%u stopped (%d)\n", q, ctx.rc);
			}
		}
		g_ublk_started[q] = false;
	}

	/* Destroy the ublk target (tears down io_uring rings and threads). */
	if (g_host_fs_stage >= HOST_FS_UBLK) {
		ctx = (struct host_fs_sync_ctx){0};
		if (ublk_destroy_target(bdev_fini_done_cb, &ctx) == 0) {
			host_fs_wait(&ctx.done, "ublk destroy target");
		}
	}

	/* Finalise the bdev subsystem (unregisters all bdevs). */
	if (g_host_fs_stage >= HOST_FS_BDEV) {
		ctx = (struct host_fs_sync_ctx){0};
		spdk_bdev_finish(bdev_fini_done_cb, &ctx);
		host_fs_wait(&ctx.done, "bdev finish");
		memset(g_nvme_host_bdev, 0, sizeof(g_nvme_host_bdev));
	}

	if (g_host_fs_stage >= HOST_FS_IOBUF) {
		ctx = (struct host_fs_sync_ctx){0};
		spdk_iobuf_finish(bdev_fini_done_cb, &ctx);
		host_fs_wait(&ctx.done, "iobuf finish");
	}

	/* Finalise accel after all bdev channels have been torn down. */
	if (g_host_fs_stage >= HOST_FS_ACCEL) {
		ctx = (struct host_fs_sync_ctx){0};
		spdk_accel_finish(bdev_fini_done_cb, &ctx);
		host_fs_wait(&ctx.done, "accel finish");
	}
	g_host_fs_stage = HOST_FS_NONE;

	/*
	 * ublk_destroy_target() already requests shutdown of ublk worker
	 * threads on their own SPDK threads.  Only the current thread may call
	 * spdk_thread_exit() directly, so request exit for any remaining
	 * non-host threads via a message and destroy them only after they have
	 * actually exited. A thread that does not exit in time is leaked, not waited on.
	 */
	for (i = 0; i < g_num_threads; i++) {
		if (g_all_threads[i] != NULL && g_all_threads[i] != g_host_thread &&
		    !spdk_thread_is_exited(g_all_threads[i])) {
			spdk_thread_send_msg(g_all_threads[i], host_thread_exit_msg, NULL);
		}
	}

	deadline = host_fs_now_ms() + HOST_FS_STEP_TIMEOUT_MS;
	for (i = 0; i < g_num_threads; i++) {
		if (g_all_threads[i] == NULL || g_all_threads[i] == g_host_thread) {
			continue;
		}

		while (!spdk_thread_is_exited(g_all_threads[i]) && host_fs_now_ms() <= deadline) {
			spdk_thread_poll(g_all_threads[i], 0, 0);
		}
		if (!spdk_thread_is_exited(g_all_threads[i])) {
			fprintf(stderr, "ERROR: host fs: thread %s did not exit, leaking it\n",
				spdk_thread_get_name(g_all_threads[i]));
			g_all_threads[i] = NULL;
			continue;
		}
		spdk_thread_destroy(g_all_threads[i]);
		g_all_threads[i] = NULL;
	}

	spdk_thread_exit(g_host_thread);
	deadline = host_fs_now_ms() + HOST_FS_STEP_TIMEOUT_MS;
	while (!spdk_thread_is_exited(g_host_thread) && host_fs_now_ms() <= deadline) {
		spdk_thread_poll(g_host_thread, 0, 0);
	}
	if (!spdk_thread_is_exited(g_host_thread)) {
		/* spdk_thread_lib_fini() would assert on a live thread, so stop here. */
		fprintf(stderr, "ERROR: host fs: host_fs thread did not exit; skipping lib fini\n");
		g_host_thread = NULL;
		g_num_threads = 0;
		return;
	}
	spdk_thread_destroy(g_host_thread);

	g_host_thread = NULL;
	g_num_threads = 0;

	spdk_thread_lib_fini();
	printf("Host filesystem path torn down\n");
}

/* ============================================================
 * End of Host Filesystem Bdev Module
 * ============================================================ */

/* Set from the signal handler; the main loop and the admin-poll threads watch it. */
static volatile int             stop = 0;
static volatile sig_atomic_t    g_dumpreq = 0;

/* -r / -s: which reset to issue, whether it was reached, and how it went. */
/* Path for the PRP dump, or NULL to skip it entirely. */
static const char              *g_prp_dump_path = NULL;
static bool                     g_do_ctrl_rst = false;
static bool                     g_do_subs_rst = false;
static bool                     g_reset_issued = false;
static int                      g_reset_rc = 0;

/* Env options are filled in from the option callback, so they live here rather than in main(). */
static struct spdk_env_opts     g_env_opts;

/* One admin-poll thread per drive. */
static pthread_t                g_poll_thread[IUVENTUS_MAX_PEERS];
static bool                     g_thread_started[IUVENTUS_MAX_PEERS];

/* -U: drives (bit q = queue q's drive) whose namespace is also exposed to the host via ublk. */
static uint32_t                 g_host_fs_mask = 0;

static void
sig_usr(int signo)
{
	if (signo == SIGINT || signo == SIGTERM) {
		stop = 1;
	} else if (signo == SIGUSR1) {
		g_dumpreq = 1;
	}
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
	printf("\t     its own host queue pair next to the card's (default 0 = none)]\n");
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
		g_env_opts.shm_id = spdk_strtol(arg, 10);
		if (g_env_opts.shm_id < 0) {
			fprintf(stderr, "Invalid shared memory ID\n");
			return -1;
		}
		break;
	case 'g':
		g_env_opts.hugepage_single_segments = true;
		break;
	case 'm':
		g_env_opts.mem_size = spdk_strtol(arg, 10);
		if (g_env_opts.mem_size < 0) {
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
 * The data plane runs on the card, so this only services admin completions and asynchronous
 * events. One thread per drive means a drive that stops answering cannot hold up the others.
 */
static void *
drive_poll_fn(void *arg)
{
	struct spdk_nvme_ctrlr *ctrlr = arg;

	while (!stop) {
		spdk_nvme_ctrlr_process_admin_completions(ctrlr);
		usleep(1000);
	}
	return NULL;
}

int
main(int argc, char **argv)
{
	struct iuventus_opts iu_opts;
	struct iuventus_ctx iu_ctx = {0};
	uint32_t i;
	int rc;

	g_env_opts.opts_size = sizeof(g_env_opts);
	spdk_env_opts_init(&g_env_opts);
	g_env_opts.name = "fpga_zero_copy";

	iuventus_opts_init(&iu_opts, sizeof(iu_opts));

	rc = iuventus_parse_args(argc, argv, &iu_opts, APP_GETOPT_STRING, app_parse, usage);
	if (rc == IUVENTUS_PARSE_ARGS_HELP) {
		return EXIT_SUCCESS;
	}
	if (rc != IUVENTUS_PARSE_ARGS_SUCCESS) {
		return EXIT_FAILURE;
	}
	if (g_host_fs_mask >> iu_opts.num_peers) {
		fprintf(stderr, "-U 0x%x names a drive past the %u given with -t\n",
			g_host_fs_mask, iu_opts.num_peers);
		return EXIT_FAILURE;
	}

	if (spdk_env_init(&g_env_opts) < 0) {
		fprintf(stderr, "Unable to initialize SPDK env\n");
		return -1;
	}

	if (g_do_ctrl_rst || g_do_subs_rst) {
		iu_opts.on_peers_ready = reset_peers_cb;
	}

	rc = iuventus_init(&iu_ctx, &iu_opts);
	if (g_reset_issued) {
		/* The hook stopped the bring-up on purpose once the reset had been issued, so the
		 * reset's own result is what this run reports. A bring-up that failed before the
		 * hook ran falls through to the check below instead. */
		rc = g_reset_rc;
		goto env_fini;
	}
	if (rc) {
		fprintf(stderr, "Unable to bring up the DMA Iuventus environment: %d\n", rc);
		goto env_fini;
	}

	for (i = 0; i < iuventus_num_peers(&iu_ctx); i++) {
		const struct iuventus_peer *peer = iuventus_peer(&iu_ctx, i);

		printf("--- drive %u (%s) ---\n", i, peer->trid.traddr);
		spdk_nvme_ctrlr_register_aer_callback(peer->ctrlr, aer_cb, NULL);
		rd_ctrl_regs(peer->ctrlr);
		force_active_power_state(peer->ctrlr);
		dump_nvme_logs(peer->ctrlr, "BASELINE");
	}

	prp_list_dump(&iu_ctx, g_prp_dump_path);

	/*
	 * Host-side filesystem path (-U): every selected drive gets a thin bdev backed by its own
	 * host software queue pair, exposed via ublk as /dev/ublkb<q>. The card keeps its own
	 * queue pair on the same drive, so the two paths need no synchronisation here.
	 */
	if (g_host_fs_mask != 0) {
		rc = host_fs_lib_init();
		for (i = 0; rc == 0 && i < iuventus_num_peers(&iu_ctx); i++) {
			if (g_host_fs_mask & (1U << i)) {
				rc = host_fs_add(i, iuventus_peer(&iu_ctx, i)->ns);
			}
		}
		if (rc) {
			fprintf(stderr, "Warning: host filesystem (ublk) initialisation failed (%d).\n"
				"The FPGA path will continue to operate normally.\n", rc);
			/* Non-fatal: proceed without the host filesystem. */
			host_fs_fini();
			rc = 0;
		}
	}

	rc = iuventus_enable(&iu_ctx);
	if (rc) {
		fprintf(stderr, "Unable to enable the card: %d\n", rc);
		goto teardown;
	}

	for (i = 0; i < iuventus_num_peers(&iu_ctx); i++) {
		const struct iuventus_peer *peer = iuventus_peer(&iu_ctx, i);

		if (pthread_create(&g_poll_thread[i], NULL, drive_poll_fn, peer->ctrlr) != 0) {
			fprintf(stderr, "WARNING: could not start poll thread for %s; its admin "
				"queue will be polled by the main loop instead\n",
				peer->trid.traddr);
			continue;
		}
		g_thread_started[i] = true;
	}

	signal(SIGINT, sig_usr);
	signal(SIGTERM, sig_usr);
	signal(SIGUSR1, sig_usr);

	printf("Initialization complete. Starting main loop.\n");

	usleep(1000);

	/*
	 * Main loop: poll every tracked SPDK thread so that ublk requests
	 * are processed and NVMe completions on the SW queue pair are
	 * returned to the kernel in a timely fashion.
	 * The card operates independently through its own hardware queue pair.
	 *
	 * ublk_create_target() creates one poll-group thread per requested
	 * CPU core; all of them are tracked in g_all_threads[].
	 */
	while (!stop) {
		int busy = host_fs_poll_all();

		/* Drives whose poll thread could not start are serviced here. */
		for (i = 0; i < iuventus_num_peers(&iu_ctx); i++) {
			if (!g_thread_started[i]) {
				spdk_nvme_ctrlr_process_admin_completions(
					iuventus_peer(&iu_ctx, i)->ctrlr);
			}
		}
		if (g_dumpreq) {
			g_dumpreq = 0;
			for (i = 0; i < iuventus_num_peers(&iu_ctx); i++) {
				const struct iuventus_peer *peer = iuventus_peer(&iu_ctx, i);

				printf("--- drive %u (%s) ---\n", i, peer->trid.traddr);
				dump_nvme_logs(peer->ctrlr, "POST-WEDGE");
			}
		}
		/* Back off only while every poller is idle; a short nap keeps host I/O latency low. */
		if (!busy) {
			usleep(g_num_threads > 0 ? 50 : 1000);
		}
	}

	/* Stop the pollers before touching any controller from this thread again. */
	for (i = 0; i < iuventus_num_peers(&iu_ctx); i++) {
		if (g_thread_started[i]) {
			pthread_join(g_poll_thread[i], NULL);
			g_thread_started[i] = false;
		}
	}

	iuventus_disable(&iu_ctx);
	if (iuventus_drain(&iu_ctx, IUVENTUS_DRAIN_TIMEOUT_MS)) {
		fprintf(stderr, "WARNING: tearing down a design that has not gone idle\n");
	}

	/* Tear down the host filesystem before any lower-level cleanup. */
	host_fs_fini();

	for (i = 0; i < iuventus_num_peers(&iu_ctx); i++) {
		rd_ctrl_regs(iuventus_peer(&iu_ctx, i)->ctrlr);
	}

teardown:
	iuventus_fini(&iu_ctx);
	printf("DMA Iuventus environment torn down\n");
env_fini:
	spdk_env_fini();
	printf("Env finished\n");
	return rc;
}
