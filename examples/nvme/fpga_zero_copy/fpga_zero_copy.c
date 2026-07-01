/*
 *   fpga_zero_copy.c: Establishes peer-to-peer chain between an FPGA card and an NVMe device
 *   with FPGA as a master.
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) Vladislav Válek
 */

#include <stdint.h>
#include <stdlib.h>
#include <sys/queue.h>
#include <libfdt.h>
#include <signal.h>

#include <nfb/nfb.h>

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
#include "ublk_internal.h"

#define DATA_BUFFER_STRING "Dan Kriz je best!"

#define REG_CONTROL             0x00
#define REG_STATUS              0x04
#define REG_SQTDBL              0x08
#define REG_SQHDBL              0x0C
#define REG_CQHDBL              0x10
#define REG_DBL_MASK            0x14
#define REG_SQTDBL_BADDR        0x18
#define REG_CQHDBL_BADDR        0x20
#define REG_RDBUFF_BADDR        0x28
#define REG_RDBUFF_PRP_LIST_PTR 0x30
#define REG_WRBUFF_BADDR        0x38
#define REG_WRBUFF_PRP_LIST_PTR 0x40
#define REG_LBA_NUM_MASK        0x98
#define REG_LBA_SPACE_SIZE      0xD4
#define REG_META_PTR 			0x10C

#define CTRL_ENABLE       (1 << 0)
#define CTRL_RPT_UPD_EN   (1 << 4)

#define STAT_READY         		 (1 << 0)
#define STAT_RST_DONE       	 (1 << 1)
#define STAT_TAG_FIFO_INIT_DONE  (1 << 2)

#define SQ_BAR 0
#define CQ_BAR 1
#define WRBUFF_BAR 2
#define RDBUFF_BAR 3

struct ctrlr_entry {
	struct spdk_nvme_ctrlr	*ctrlr;
	char			name[1024];
} g_controller;

struct ns_entry {
	struct spdk_nvme_ctrlr	*ctrlr;
	struct spdk_nvme_ns	*ns;
	struct spdk_nvme_qpair	*hw_qpair;
	int32_t hw_qid;
} g_namespace;

/*
 * ============================================================
 * Host Filesystem Bdev Module
 *
 * This module wraps host-side NVMe software queue pairs as
 * an SPDK bdev so that the host can access the NVMe namespace
 * through the standard SPDK block-device layer and, from there,
 * through ublk as a regular Linux block device (/dev/ublkb0).
 *
 * The FPGA uses its own queue pair (hw_qid, managed via raw
 * admin commands) while each host I/O channel owns its own
 * software queue pair,
 * so no cross-device locking is required.
 * ============================================================
 */

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

	ch->bdev_ctx = bdev_ctx;
	ch->qpair = spdk_nvme_ctrlr_alloc_io_qpair(spdk_nvme_ns_get_ctrlr(bdev_ctx->ns), NULL, 0);
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

/* Name used to identify the bdev within the SPDK bdev layer. */
#define NVME_HOST_BDEV_NAME "fpga_nvme_host"
/*
 * ublk device ID.  The kernel block device will appear as
 * /dev/ublkb<NVME_HOST_UBLK_ID>.
 */
#define NVME_HOST_UBLK_ID   0

static struct nvme_host_bdev *g_nvme_host_bdev = NULL;
static struct spdk_thread    *g_host_thread = NULL;

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

/*
 * Register a thin bdev backed by the host SW queue pair so that the
 * NVMe namespace can be exposed as a standard Linux block device.
 */
static int
nvme_host_bdev_create(struct spdk_nvme_ns *ns)
{
	struct nvme_host_bdev *bdev_ctx;
	int rc;

	bdev_ctx = calloc(1, sizeof(*bdev_ctx));
	if (!bdev_ctx) {
		return -ENOMEM;
	}

	bdev_ctx->ns = ns;

	bdev_ctx->bdev.ctxt         = bdev_ctx;
	bdev_ctx->bdev.name         = strdup(NVME_HOST_BDEV_NAME);
	bdev_ctx->bdev.product_name = "FPGA NVMe Host Bdev";
	bdev_ctx->bdev.fn_table     = &g_nvme_host_fn_table;
	bdev_ctx->bdev.module       = &g_nvme_host_module;
	bdev_ctx->bdev.write_cache  = 1;
	bdev_ctx->bdev.blocklen     = spdk_nvme_ns_get_sector_size(ns);
	bdev_ctx->bdev.blockcnt     = spdk_nvme_ns_get_num_sectors(ns);

	spdk_io_device_register(bdev_ctx,
				nvme_host_create_ch, nvme_host_destroy_ch,
				sizeof(struct nvme_host_io_channel),
				"nvme_host");

	rc = spdk_bdev_register(&bdev_ctx->bdev);
	if (rc) {
		spdk_io_device_unregister(bdev_ctx, NULL);
		free(bdev_ctx->bdev.name);
		free(bdev_ctx);
		return rc;
	}

	g_nvme_host_bdev = bdev_ctx;
	return 0;
}

/* Callbacks used while initialising / finalising the bdev subsystem. */
struct host_fs_sync_ctx {
	int  rc;
	bool done;
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
		fprintf(stderr, "ERROR: Failed to start ublk disk (id=%u): %d\n",
			NVME_HOST_UBLK_ID, result);
	} else {
		printf("Host filesystem bdev registered as: /dev/ublkb%u\n",
		       NVME_HOST_UBLK_ID);
		printf("  To format: mkfs.ext4 /dev/ublkb%u\n", NVME_HOST_UBLK_ID);
		printf("  To mount:  mount /dev/ublkb%u /mnt/nvme\n", NVME_HOST_UBLK_ID);
	}
}

/* Called once ublk_stop_disk() completes. */
static void
ublk_stop_cb(void *cb_arg, int result)
{
	struct host_fs_sync_ctx *ctx = cb_arg;

	ctx->rc   = result;
	ctx->done = true;
}

/*
 * Initialise the host-filesystem path:
 *  1. Start the SPDK thread library (with a thread-tracking hook so that
 *     the extra threads spawned by ublk_create_target are polled by the
 *     main loop).
 *  2. Initialise the iobuf pool and bdev subsystem.
 *  3. Register the custom NVMe bdev backed by the SW queue pair.
 *  4. Expose the bdev via ublk as /dev/ublkb<NVME_HOST_UBLK_ID>.
 *     Requires kernel >= 6.0 with CONFIG_BLK_DEV_UBLK=y.
 *
 * Returns 0 on success, negative errno on failure.
 */
static int
host_fs_init(struct spdk_nvme_ns *ns)
{
	struct host_fs_sync_ctx ctx = {0};
	int rc;

	/*
	 * Initialise the SPDK thread library with the tracking callback so
	 * that any additional SPDK threads created later (e.g. by
	 * ublk_create_target) are collected in g_all_threads[].
	 */
	rc = spdk_thread_lib_init(host_thread_op_fn, 0);
	if (rc) {
		fprintf(stderr, "ERROR: spdk_thread_lib_init_ext() failed: %d\n", rc);
		return rc;
	}

	/* Create the primary host-filesystem SPDK thread. */
	g_host_thread = spdk_thread_create("host_fs", NULL);
	if (!g_host_thread) {
		fprintf(stderr, "ERROR: Failed to create host_fs SPDK thread\n");
		rc = -ENOMEM;
		goto thread_create_fail;
	}
	/* g_host_thread was tracked via host_thread_op_fn above. */
	spdk_set_thread(g_host_thread);

	/*
	 * The bdev layer acquires an accel I/O channel for each bdev channel.
	 * When this standalone example bypasses the normal SPDK app startup
	 * path, the accel framework must be initialized explicitly.
	 */
	rc = spdk_accel_initialize();
	if (rc) {
		fprintf(stderr, "ERROR: spdk_accel_initialize() failed: %d\n", rc);
		goto accel_init_fail;
	}

	/*
	 * Initialise the iobuf memory pool.  Both the bdev I/O-channel layer
	 * and the ublk driver require this before any I/O channels can be
	 * created.
	 */
	rc = spdk_iobuf_initialize();
	if (rc) {
		fprintf(stderr, "ERROR: spdk_iobuf_initialize() failed: %d\n", rc);
		goto iobuf_init_fail;
	}

	/* Initialise the bdev subsystem; our custom module_init() is a no-op. */
	ctx = (struct host_fs_sync_ctx){0};
	spdk_bdev_initialize(bdev_init_done_cb, &ctx);
	while (!ctx.done) {
		spdk_thread_poll(g_host_thread, 0, 0);
	}
	if (ctx.rc) {
		fprintf(stderr, "ERROR: spdk_bdev_initialize() failed: %d\n", ctx.rc);
		rc = ctx.rc;
		goto bdev_init_fail;
	}

	/* Register the custom bdev that wraps per-channel host SW queue pairs. */
	rc = nvme_host_bdev_create(ns);
	if (rc) {
		fprintf(stderr, "ERROR: nvme_host_bdev_create() failed: %d\n", rc);
		goto bdev_create_fail;
	}

	/*
	 * ublk path: exposes the NVMe namespace as /dev/ublkb<id>.
	 *
	 * ublk uses io_uring for the kernel↔user communication path.
	 * ublk_create_target() spawns one additional SPDK thread per
	 * active CPU core; those threads are captured by
	 * host_thread_op_fn() above and polled in the main loop.
	 *
	 * Restrict ublk to a single CPU (core 0) to avoid creating
	 * too many poll-group threads in this non-reactor environment.
	 */
	spdk_ublk_init();

	rc = ublk_create_target("0x1", NULL);
	if (rc) {
		fprintf(stderr, "ERROR: ublk_create_target() failed: %d\n", rc);
		goto frontend_fail;
	}

	ctx = (struct host_fs_sync_ctx){0};
	rc = ublk_start_disk(NVME_HOST_BDEV_NAME, NVME_HOST_UBLK_ID,
			     UBLK_DEV_NUM_QUEUE, UBLK_DEV_QUEUE_DEPTH,
			     ublk_start_cb, &ctx);
	if (rc) {
		fprintf(stderr, "ERROR: ublk_start_disk() failed: %d\n", rc);
		goto frontend_fail;
	}
	while (!ctx.done) {
		int i;
		for (i = 0; i < g_num_threads; i++) {
			spdk_thread_poll(g_all_threads[i], 0, 0);
		}
	}
	if (ctx.rc) {
		fprintf(stderr, "ERROR: ublk disk start failed: %d\n", ctx.rc);
		rc = ctx.rc;
		goto frontend_fail;
	}

	return 0;

frontend_fail:
	{
		struct host_fs_sync_ctx fini_ctx = {0};
		int i;
		ublk_destroy_target(bdev_fini_done_cb, &fini_ctx);
		while (!fini_ctx.done) {
			for (i = 0; i < g_num_threads; i++) {
				spdk_thread_poll(g_all_threads[i], 0, 0);
			}
		}
	}
bdev_create_fail:
	{
		struct host_fs_sync_ctx fini_ctx = {0};
		spdk_bdev_finish(bdev_fini_done_cb, &fini_ctx);
		while (!fini_ctx.done) {
			spdk_thread_poll(g_host_thread, 0, 0);
		}
	}
bdev_init_fail:
	{
		struct host_fs_sync_ctx fini_ctx = {0};
		spdk_iobuf_finish(bdev_fini_done_cb, &fini_ctx);
		while (!fini_ctx.done) {
			spdk_thread_poll(g_host_thread, 0, 0);
		}
	}
iobuf_init_fail:
	{
		struct host_fs_sync_ctx fini_ctx = {0};
		spdk_accel_finish(bdev_fini_done_cb, &fini_ctx);
		while (!fini_ctx.done) {
			spdk_thread_poll(g_host_thread, 0, 0);
		}
	}
accel_init_fail:
	spdk_thread_exit(g_host_thread);
	while (!spdk_thread_is_exited(g_host_thread)) {
		spdk_thread_poll(g_host_thread, 0, 0);
	}
	spdk_thread_destroy(g_host_thread);
	g_host_thread = NULL;
thread_create_fail:
	spdk_thread_lib_fini();
	return rc;
}

/*
 * Tear down the host filesystem path in reverse initialisation order.
 */
static void
host_fs_fini(void)
{
	struct host_fs_sync_ctx ctx = {0};
	int i;

	if (!g_host_thread) {
		return;
	}

	/* Stop the ublk disk first, then destroy the target. */
	ctx = (struct host_fs_sync_ctx){0};
	ublk_stop_disk(NVME_HOST_UBLK_ID, ublk_stop_cb, &ctx);
	while (!ctx.done) {
		for (i = 0; i < g_num_threads; i++) {
			spdk_thread_poll(g_all_threads[i], 0, 0);
		}
	}

	/* Destroy the ublk target (tears down io_uring rings and threads). */
	ctx = (struct host_fs_sync_ctx){0};
	ublk_destroy_target(bdev_fini_done_cb, &ctx);
	while (!ctx.done) {
		for (i = 0; i < g_num_threads; i++) {
			spdk_thread_poll(g_all_threads[i], 0, 0);
		}
	}

	/* Finalise the bdev subsystem (unregisters all bdevs). */
	ctx = (struct host_fs_sync_ctx){0};
	spdk_bdev_finish(bdev_fini_done_cb, &ctx);
	while (!ctx.done) {
		spdk_thread_poll(g_host_thread, 0, 0);
	}

	/* Finalise the iobuf pool. */
	ctx = (struct host_fs_sync_ctx){0};
	spdk_iobuf_finish(bdev_fini_done_cb, &ctx);
	while (!ctx.done) {
		spdk_thread_poll(g_host_thread, 0, 0);
	}

	/* Finalise accel after all bdev channels have been torn down. */
	ctx = (struct host_fs_sync_ctx){0};
	spdk_accel_finish(bdev_fini_done_cb, &ctx);
	while (!ctx.done) {
		spdk_thread_poll(g_host_thread, 0, 0);
	}

	/*
	 * ublk_destroy_target() already requests shutdown of ublk worker
	 * threads on their own SPDK threads.  Only the current thread may call
	 * spdk_thread_exit() directly, so request exit for any remaining
	 * non-host threads via a message and destroy them only after they have
	 * actually exited.
	 */
	for (i = 0; i < g_num_threads; i++) {
		if (g_all_threads[i] != NULL && g_all_threads[i] != g_host_thread &&
		    !spdk_thread_is_exited(g_all_threads[i])) {
			spdk_thread_send_msg(g_all_threads[i], host_thread_exit_msg, NULL);
		}
	}

	for (i = 0; i < g_num_threads; i++) {
		if (g_all_threads[i] == NULL || g_all_threads[i] == g_host_thread) {
			continue;
		}

		while (!spdk_thread_is_exited(g_all_threads[i])) {
			spdk_thread_poll(g_all_threads[i], 0, 0);
		}
		spdk_thread_destroy(g_all_threads[i]);
		g_all_threads[i] = NULL;
	}

	spdk_thread_exit(g_host_thread);
	while (!spdk_thread_is_exited(g_host_thread)) {
		spdk_thread_poll(g_host_thread, 0, 0);
	}
	spdk_thread_destroy(g_host_thread);

	g_host_thread = NULL;
	g_num_threads = 0;

	spdk_thread_lib_fini();
}

/* ============================================================
 * End of Host Filesystem Bdev Module
 * ============================================================ */

struct fpga_bar_ctx {
	void *vaddr;
	uint64_t paddr;
	uint64_t size;
};

struct fpga_prp_list_ctx {
	void *vaddr;
	uint64_t paddr;
};

struct dma_device_ctx {
	struct nfb_device *dev;
	struct nfb_comp *comp;
	const char *pcie_bdf;
};

struct admin_cmd_ctx {
	const char *cmd_name;
	bool done;
	int status;
};

struct fpga_hw_ctx {
	struct spdk_pci_device *pci_dev;
	uint32_t nsid;
	bool qid_allocated;
	bool cq_created;
	bool sq_created;
	bool prp_lists_allocated;

	struct fpga_bar_ctx sq;
	struct fpga_bar_ctx cq;
	struct fpga_bar_ctx wrbuff;
	struct fpga_bar_ctx rdbuff;

	struct fpga_prp_list_ctx wrbuff_prp_list;
	struct fpga_prp_list_ctx rdbuff_prp_list;

	uint64_t doorbell_base;
	uint32_t doorbell_stride;

	/* Values programmed into the FPGA command-dispatch registers. */
	uint16_t doorbell_mask;
	uint64_t sqtdbl_paddr;
	uint64_t cqhdbl_paddr;
	uint16_t lba_num_mask;
	uint64_t lba_space_size;
};

struct app_ctx {
	const char *select_dev;
	uint16_t qsize;
	struct spdk_nvme_transport_id *trid;
	struct fpga_hw_ctx hw;
};

volatile int stop = 0;

static void sig_usr(int signo)
{
	if (signo == SIGINT || signo == SIGTERM) {
		stop = 1;
	}
}

static struct spdk_pci_id ncd_pci_driver_id[] = {
	{
		SPDK_PCI_DEVICE(0x18ec, 0xc020)
	},
	{
		SPDK_PCI_DEVICE(0x1c2c, 0xc020)
	},
};

SPDK_PCI_DRIVER_REGISTER(ncd, ncd_pci_driver_id, SPDK_PCI_DRIVER_NEED_MAPPING)

static void
qop_complete_cb(void *ctx, const struct spdk_nvme_cpl *cpl)
{
	struct admin_cmd_ctx *cmd_ctx = ctx;

	cmd_ctx->done = true;
	if (spdk_nvme_cpl_is_error(cpl)) {
		spdk_nvme_print_completion(0, (struct spdk_nvme_cpl *)cpl);
		fprintf(stderr, "CPL error status: %s\n", spdk_nvme_cpl_get_status_string(&cpl->status));
		fprintf(stderr, "%s failed, aborting run\n", cmd_ctx->cmd_name);
		cmd_ctx->status = -1;
	}
}

static int
submit_admin_request(struct spdk_nvme_cmd *cmd, const char *cmd_name)
{
	int rc = 0;
	struct admin_cmd_ctx cmd_ctx = {
		.cmd_name = cmd_name,
	};

	rc = spdk_nvme_ctrlr_cmd_admin_raw(g_namespace.ctrlr, cmd, NULL, 0, qop_complete_cb, &cmd_ctx);
	if (rc) {
		printf("Failed to submit the command %s!\n", cmd_ctx.cmd_name);
		return -1;
	}

	while (!cmd_ctx.done)
		spdk_nvme_ctrlr_process_admin_completions(g_namespace.ctrlr);

	if (cmd_ctx.status != 0)
		return -2;

	return 0;
}

static int
queues_alloc(struct app_ctx *app_ctx)
{
	int			       rc = 0;
	struct spdk_nvme_io_qpair_opts qopts;
	struct spdk_nvme_cmd           cmd = {0};
	struct fpga_hw_ctx *hw = &app_ctx->hw;

	spdk_nvme_ctrlr_get_default_io_qpair_opts(g_namespace.ctrlr, &qopts, sizeof(struct spdk_nvme_io_qpair_opts));

	printf("Default queue pair options:\n");
	printf("IO queue size: %d\n", qopts.io_queue_size);
	printf("IO queue requests: %d\n", qopts.io_queue_requests);

	if (app_ctx->qsize == 0) {
		qopts.io_queue_requests = qopts.io_queue_size;
	} else {
		qopts.io_queue_requests = app_ctx->qsize;
		qopts.io_queue_size = app_ctx->qsize;
	}

	hw->doorbell_mask = qopts.io_queue_size - 1;

	qopts.sq.vaddr = hw->sq.vaddr;
	qopts.sq.paddr = hw->sq.paddr;
	qopts.sq.buffer_size = qopts.io_queue_size * sizeof(struct spdk_nvme_cmd);

	qopts.cq.vaddr = hw->cq.vaddr;
	qopts.cq.paddr = hw->cq.paddr;
	qopts.cq.buffer_size = qopts.io_queue_size * sizeof(struct spdk_nvme_cpl);

	// Reset the Completion Queue in the Hardware, otherwise previous completion entries get
	// detected. This means return Phase Tags to value 0 (i.e. default value)
	uint8_t *cpl_buff = hw->cq.vaddr;
	for (uint32_t i = 14; i < qopts.cq.buffer_size; i+=sizeof(struct spdk_nvme_cpl)) {
		cpl_buff[i] = 0;
	}

	g_namespace.hw_qid = spdk_nvme_ctrlr_alloc_qid(g_namespace.ctrlr);
	if (g_namespace.hw_qid < 0) {
		printf("ERROR: Failed to allocated QID for the HW queues\n");
		rc = -13;
		goto qid_alloc_fail;

	}
	hw->qid_allocated = true;

	hw->sqtdbl_paddr = hw->doorbell_base + (2 * g_namespace.hw_qid) * (4 << hw->doorbell_stride);
	hw->cqhdbl_paddr = hw->doorbell_base + (2 * g_namespace.hw_qid + 1) * (4 << hw->doorbell_stride);

	printf("Allocate qid %d\n", g_namespace.hw_qid);

	cmd.opc = SPDK_NVME_OPC_CREATE_IO_CQ;
	cmd.nsid = 0;
	cmd.cdw10_bits.create_io_q.qid = g_namespace.hw_qid;
	cmd.cdw10_bits.create_io_q.qsize = qopts.io_queue_size - 1;
	cmd.cdw11_bits.create_io_cq.pc = 1;
	cmd.dptr.prp.prp1 = hw->cq.paddr;

	rc = submit_admin_request(&cmd, "CQ_CREATE");
	if (rc) {
		printf("Failed to submit the Admin command!\n");
		goto cq_create_fail;
	}
	hw->cq_created = true;

	printf("HW CQ allocated!\n");

	cmd.opc = SPDK_NVME_OPC_CREATE_IO_SQ;
	cmd.nsid = 0;
	cmd.cdw10_bits.create_io_q.qid = g_namespace.hw_qid;
	cmd.cdw10_bits.create_io_q.qsize = qopts.io_queue_size - 1;
	cmd.cdw11_bits.create_io_sq.pc = 1;
	cmd.cdw11_bits.create_io_sq.qprio = 2;
	cmd.cdw11_bits.create_io_sq.cqid = g_namespace.hw_qid;
	cmd.dptr.prp.prp1 = hw->sq.paddr;

	rc = submit_admin_request(&cmd, "SQ_CREATE");
	if (rc) {
		printf("Failed to submit the Admin command!\n");
		goto sq_create_fail;
	}
	hw->sq_created = true;

	printf("HW SQ allocated!\n");

	return 0;

sq_create_fail:
	if (hw->cq_created) {
		cmd.opc = SPDK_NVME_OPC_DELETE_IO_CQ;
		cmd.cdw10_bits.delete_io_q.qid = g_namespace.hw_qid;
		rc = submit_admin_request(&cmd, "CQ_DELETE");
		if (rc) {
			printf("Failed to submit the Admin command!\n");
		}
		hw->cq_created = false;
	}

cq_create_fail:
	if (hw->qid_allocated) {
		spdk_nvme_ctrlr_free_qid(g_namespace.ctrlr, g_namespace.hw_qid);
		hw->qid_allocated = false;
		g_namespace.hw_qid = -1;
	}
qid_alloc_fail:
	return rc;
}

static void queues_delete(struct fpga_hw_ctx *hw)
{
	int rc = 0;
	struct spdk_nvme_cmd cmd = {0};

	if (hw->sq_created) {
		cmd.opc = SPDK_NVME_OPC_DELETE_IO_SQ;
		cmd.cdw10_bits.delete_io_q.qid = g_namespace.hw_qid;
		rc = submit_admin_request(&cmd, "SQ_DELETE");
		if (rc) {
			printf("Failed to submit Admin command!\n");
		}
		hw->sq_created = false;
	}

	if (hw->cq_created) {
		cmd.opc = SPDK_NVME_OPC_DELETE_IO_CQ;
		cmd.cdw10_bits.delete_io_q.qid = g_namespace.hw_qid;
		rc = submit_admin_request(&cmd, "CQ_DELETE");
		if (rc) {
			printf("Failed to submit the Admin command!\n");
		}
		hw->cq_created = false;
	}
}

static void
queues_dealloc(struct fpga_hw_ctx *hw)
{
	queues_delete(hw);
	if (hw->qid_allocated) {
		spdk_nvme_ctrlr_free_qid(g_namespace.ctrlr, g_namespace.hw_qid);
		hw->qid_allocated = false;
		g_namespace.hw_qid = -1;
	}
}

static bool
probe_cb(void *cb_ctx, const struct spdk_nvme_transport_id *trid,
	 struct spdk_nvme_ctrlr_opts *opts)
{
	struct app_ctx *app_ctx = cb_ctx;

	printf("Probing %s ...\n", trid->traddr);

	if (app_ctx->qsize != 0) {
		opts->io_queue_size = app_ctx->qsize;
		opts->io_queue_requests = app_ctx->qsize;
	} else {
		opts->io_queue_requests = opts->io_queue_size;
	}
	opts->arb_mechanism = SPDK_NVME_CC_AMS_RR;
	opts->enable_interrupts = false;

	if (!strcmp(trid->traddr, app_ctx->trid->traddr))
		return true;

	return false;
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

static void
attach_cb(void *cb_ctx, const struct spdk_nvme_transport_id *trid,
	  struct spdk_nvme_ctrlr *ctrlr, const struct spdk_nvme_ctrlr_opts *opts)
{
	int nsid;
	struct spdk_nvme_ns *ns;
	const struct spdk_nvme_ctrlr_data *cdata;
	struct spdk_pci_device *pci_dev;
	struct spdk_pci_addr pci_addr;
	char bdf[32];
	char sysfs_path[128];
	uint64_t bar_start, bar_end, bar_flags;
	FILE *fp;
	union spdk_nvme_cap_register cap;
	struct app_ctx *app_ctx = cb_ctx;
	struct fpga_hw_ctx *hw = &app_ctx->hw;
	uint32_t sect_size;

	printf("Attaching to %s ...\n", trid->traddr);

	/*
	 * spdk_nvme_ctrlr is the logical abstraction in SPDK for an NVMe
	 *  controller.  During initialization, the IDENTIFY data for the
	 *  controller is read using an NVMe admin command, and that data
	 *  can be retrieved using spdk_nvme_ctrlr_get_data() to get
	 *  detailed information on the controller.  Refer to the NVMe
	 *  specification for more details on IDENTIFY for NVMe controllers.
	 */
	cdata = spdk_nvme_ctrlr_get_data(ctrlr);
	// Retrieve capability registers
	cap = spdk_nvme_ctrlr_get_regs_cap(ctrlr);

	snprintf(g_controller.name, sizeof(g_controller.name), "%-20.20s (%-20.20s)", cdata->mn, cdata->sn);

	g_controller.ctrlr = ctrlr;

	nsid = spdk_nvme_ctrlr_get_first_active_ns(ctrlr);
	ns = spdk_nvme_ctrlr_get_ns(ctrlr, nsid);
	if (ns == NULL) {
		fprintf(stderr, "ERROR: Invalid namespace!\n");
		return;
	}

	if (!spdk_nvme_ns_is_active(ns)) {
		return;
	}

	g_namespace.ctrlr = ctrlr;
	g_namespace.ns = ns;
	sect_size = spdk_nvme_ns_get_sector_size(ns);

	// Unlimited Max Data transfer size
	if (cdata->mdts == 0) {
		hw->lba_num_mask = 0xFFFF;
	} else {
		hw->lba_num_mask = (uint16_t)((1 << (12 + cap.bits.mpsmin + cdata->mdts)) / sect_size) - 1;
	}

	printf("Controller options:\n");

	printf("\tNumber of IO queues:   %d\n", opts->num_io_queues);
	printf("\tSize of IO queues:     %d\n", opts->io_queue_size);
	printf("\tIO queue requests:     %d\n", opts->io_queue_requests);
	printf("\tNS %d size:             %juGB\n", nsid, spdk_nvme_ns_get_size(ns) / 1000000000);
	printf("\tNS number of sectors:  %ld\n", spdk_nvme_ns_get_num_sectors(ns));
	printf("\tNS sector size:        %dB\n", sect_size);
	printf("\tLBA Mask:              x%x (%d)\n", hw->lba_num_mask, hw->lba_num_mask);
	hw->lba_space_size = spdk_nvme_ns_get_num_sectors(ns);

	rd_ctrl_regs(ctrlr);

	pci_dev = spdk_nvme_ctrlr_get_pci_device(ctrlr);
	if (!pci_dev) {
		fprintf(stderr, "Device is not PCI-backed, can't retrieve BDF.\n");
		return;
	}
	pci_addr = spdk_pci_device_get_addr(pci_dev);

	snprintf(bdf, sizeof(bdf), "%04x:%02x:%02x.%x", pci_addr.domain, pci_addr.bus, pci_addr.dev, pci_addr.func);

	// Construct sysfs path for BAR0
	snprintf(sysfs_path, sizeof(sysfs_path), "/sys/bus/pci/devices/%s/resource", bdf);

	// Read the resource file
	fp = fopen(sysfs_path, "r");
	if (!fp) {
		fprintf(stderr, "Failed to open resource file %s\n", sysfs_path);
		return;
	}

	if (fscanf(fp, "%lx %lx %lx", &bar_start, &bar_end, &bar_flags) != 3) {
		fprintf(stderr, "Failed to parse resource file\n");
		fclose(fp);
		return;
	}

	fclose(fp);

	printf("Physical address of NVME BAR0: 0x%lx\n", bar_start);

	// Doorbell registers start at offset 0x1000 from BAR0 but they start from the Admin Queues!
	hw->doorbell_base = bar_start + 0x1000;
	hw->doorbell_stride = cap.bits.dstrd;
	printf("Doorbell stride: %d\n", hw->doorbell_stride);
	hw->nsid = nsid;
}

static int dma_dev_init(const char *select_dev, struct dma_device_ctx *dma_ctx)
{
	int rc = 0;
	const void *fdt;
	int fdt_offset;
	int len;

	dma_ctx->dev = nfb_open(select_dev);
	if(!dma_ctx->dev) {
		fprintf(stderr, "ERROR: Failed to open NFB device");
		rc = -1;
		goto dev_open_fail;
	}

	fdt = nfb_get_fdt(dma_ctx->dev);
	fdt_offset = fdt_path_offset(fdt, "/system/device/endpoint0");
	dma_ctx->pcie_bdf = fdt_getprop(fdt, fdt_offset, "pci-slot", &len);
	if (len < 0) {
		fprintf(stderr, "ERROR: Failed to get pci-slot from device's Device Tree!\n");
		rc = -2;
		goto fdt_get_fail;

	}

	return 0;

fdt_get_fail:
	nfb_close(dma_ctx->dev);
	dma_ctx->dev = NULL;
dev_open_fail:
	return rc;
}

static int prp_list_alloc(struct fpga_hw_ctx *hw)
{
	uint64_t size = VALUE_4KB;
	hw->wrbuff_prp_list.vaddr = spdk_dma_zmalloc(VALUE_4KB, VALUE_4KB, NULL);
	if (hw->wrbuff_prp_list.vaddr == NULL) {
		fprintf(stderr, "ERROR: Write PRP list allocation failed\n");
		return -1;
	}

	hw->rdbuff_prp_list.vaddr = spdk_dma_zmalloc(VALUE_4KB, VALUE_4KB, NULL);
	if (hw->rdbuff_prp_list.vaddr == NULL) {
		fprintf(stderr, "ERROR: Read PRP list allocation failed\n");
		spdk_dma_free(hw->wrbuff_prp_list.vaddr);
		hw->wrbuff_prp_list.vaddr = NULL;
		return -2;
	}

	hw->wrbuff_prp_list.paddr = spdk_vtophys(hw->wrbuff_prp_list.vaddr, &size);
	if (hw->wrbuff_prp_list.paddr == SPDK_VTOPHYS_ERROR) {
		fprintf(stderr, "ERROR: Failed to get physical address of the Write PRP list buffer\n");
		spdk_dma_free(hw->wrbuff_prp_list.vaddr);
		spdk_dma_free(hw->rdbuff_prp_list.vaddr);
		hw->wrbuff_prp_list.vaddr = NULL;
		hw->rdbuff_prp_list.vaddr = NULL;
		return -3;
	}

	if (size != VALUE_4KB) {
		fprintf(stderr, "ERROR: Write PRP list buffer size is not 4096 bytes (detected size: %lu)\n", size);
		spdk_dma_free(hw->wrbuff_prp_list.vaddr);
		spdk_dma_free(hw->rdbuff_prp_list.vaddr);
		hw->wrbuff_prp_list.vaddr = NULL;
		hw->rdbuff_prp_list.vaddr = NULL;
		return -4;
	}

	hw->rdbuff_prp_list.paddr = spdk_vtophys(hw->rdbuff_prp_list.vaddr, &size);
	if (hw->rdbuff_prp_list.paddr == SPDK_VTOPHYS_ERROR) {
		fprintf(stderr, "ERROR: Failed to get physical address of the Read PRP list buffer\n");
		spdk_dma_free(hw->wrbuff_prp_list.vaddr);
		spdk_dma_free(hw->rdbuff_prp_list.vaddr);
		hw->wrbuff_prp_list.vaddr = NULL;
		hw->rdbuff_prp_list.vaddr = NULL;
		return -5;
	}

	if (size != VALUE_4KB) {
		fprintf(stderr, "ERROR: Read PRP list buffer size is not 4096 bytes (detected size: %lu)\n", size);
		spdk_dma_free(hw->wrbuff_prp_list.vaddr);
		spdk_dma_free(hw->rdbuff_prp_list.vaddr);
		hw->wrbuff_prp_list.vaddr = NULL;
		hw->rdbuff_prp_list.vaddr = NULL;
		return -6;
	}

	for (int i = 1; i < (int)(hw->wrbuff.size / VALUE_4KB); i++) {
		((uint64_t *)hw->wrbuff_prp_list.vaddr)[i - 1] = hw->wrbuff.paddr + (i * VALUE_4KB);
		((uint64_t *)hw->rdbuff_prp_list.vaddr)[i - 1] = hw->rdbuff.paddr + (i * VALUE_4KB);
	}
	hw->prp_lists_allocated = true;

	return 0;
}

static void prp_list_free(struct fpga_hw_ctx *hw)
{
	if (!hw->prp_lists_allocated) {
		return;
	}

	spdk_dma_free(hw->wrbuff_prp_list.vaddr);
	spdk_dma_free(hw->rdbuff_prp_list.vaddr);
	hw->wrbuff_prp_list.vaddr = NULL;
	hw->rdbuff_prp_list.vaddr = NULL;
	hw->wrbuff_prp_list.paddr = 0;
	hw->rdbuff_prp_list.paddr = 0;
	hw->prp_lists_allocated = false;
}

static void prp_list_print(const struct fpga_hw_ctx *hw)
{
	printf("Write PRP List (vaddr: %p, paddr: 0x%lx):\n", hw->wrbuff_prp_list.vaddr, hw->wrbuff_prp_list.paddr);
	for (int i = 0; i < (int)(hw->wrbuff.size / VALUE_4KB) - 1; i++) {
		printf("\tEntry %d: 0x%lx\n", i, ((uint64_t *)hw->wrbuff_prp_list.vaddr)[i]);
	}

	printf("Read PRP List (vaddr: %p, paddr: 0x%lx):\n", hw->rdbuff_prp_list.vaddr, hw->rdbuff_prp_list.paddr);
	for (int i = 0; i < (int)(hw->rdbuff.size / VALUE_4KB) - 1; i++) {
		printf("\tEntry %d: 0x%lx\n", i, ((uint64_t *)hw->rdbuff_prp_list.vaddr)[i]);
	}
}

static int dma_ctrl_init(struct fpga_hw_ctx *hw, struct dma_device_ctx *dma_ctx)
{
	int rc = 0;
	int node;

	node = nfb_comp_find(dma_ctx->dev, "ziti,dma_iuventus", 0);
	dma_ctx->comp = nfb_comp_open(dma_ctx->dev, node);
	if (dma_ctx->comp == NULL) {
		fprintf(stderr, "ERROR: Failed to open DMA control registers as nfb_comp!\n");
		rc = -2;
		goto dma_open_fail;
	}

	rc = prp_list_alloc(hw);
	if (rc) {
		fprintf(stderr, "ERROR: PRP list allocation failed\n");
		goto prp_alloc_fail;
	}

	nfb_comp_write16(dma_ctx->comp, REG_DBL_MASK, hw->doorbell_mask);
	nfb_comp_write64(dma_ctx->comp, REG_SQTDBL_BADDR, hw->sqtdbl_paddr);
	nfb_comp_write64(dma_ctx->comp, REG_CQHDBL_BADDR, hw->cqhdbl_paddr);
	nfb_comp_write64(dma_ctx->comp, REG_RDBUFF_BADDR, hw->rdbuff.paddr);
	nfb_comp_write64(dma_ctx->comp, REG_RDBUFF_PRP_LIST_PTR, hw->rdbuff_prp_list.paddr);
	nfb_comp_write64(dma_ctx->comp, REG_WRBUFF_BADDR, hw->wrbuff.paddr);
	nfb_comp_write64(dma_ctx->comp, REG_WRBUFF_PRP_LIST_PTR, hw->wrbuff_prp_list.paddr);
	nfb_comp_write16(dma_ctx->comp, REG_LBA_NUM_MASK, hw->lba_num_mask);
	nfb_comp_write64(dma_ctx->comp, REG_LBA_SPACE_SIZE, hw->lba_space_size);
	nfb_comp_write64(dma_ctx->comp, REG_META_PTR, 0);
	return 0;

prp_alloc_fail:
	nfb_comp_close(dma_ctx->comp);
	dma_ctx->comp = NULL;
dma_open_fail:
	return rc;
}

static void dma_ctrl_deinit(struct fpga_hw_ctx *hw, struct dma_device_ctx *dma_ctx)
{
	prp_list_free(hw);
	if (dma_ctx->comp != NULL) {
		nfb_comp_close(dma_ctx->comp);
		dma_ctx->comp = NULL;
	}
}

static void
usage(const char *program_name)
{
	printf("%s [options]", program_name);
	printf("\t\n");
	printf("\t[-h show this help]\n");
	printf("SPDK options:\n");
	printf("\t[-m DPDK huge memory size in MB]\n");
	printf("\t[-g use single file descriptor for DPDK memory segments]\n");
	printf("\t[-i shared memory group ID]\n");
#ifdef DEBUG
	printf("\t[-L enable debug logging]\n");
#else
	printf("\t[-L enable debug logging (flag disabled, must reconfigure with --enable-debug)]\n");
#endif
	printf("DMA Iuventus options:\n");
	printf("\t[-d selected nfb device (default 0)]\n");
	printf("\t[-t <fmt> Transport ID for local PCIe NVMe]\n");
	printf("\t\t Format: 'key:value [key:value] ...'\n");
	printf("\t\t Keys:\n");
	printf("\t\t  trtype      Transport type (e.g. PCIe, RDMA)\n");
	printf("\t\t  adrfam      Address family (e.g. IPv4, IPv6)\n");
	printf("\t\t  traddr      Transport address (e.g. 0000:04:00.0 for PCIe or 192.168.100.8 for RDMA)\n");
	printf("\t\t  trsvcid     Transport service identifier (e.g. 4420)\n");
	printf("\t\t  subnqn      Subsystem NQN (default: %s)\n", SPDK_NVMF_DISCOVERY_NQN);
	printf("\t\t  ns          NVMe namespace ID (all active namespaces are used by default)\n");
	printf("\t\t  hostnqn     Host NQN\n");
	printf("\t\t Example: -t 'trtype:PCIe traddr:0000:04:00.0' for PCIe\n");
	printf("\t\t Note: Currently, only PCIe transfer are supported for one device only\n");
	printf("\t[-s do a subsystem reset (i.e. hard reset, the PCIe device can disappear from the system)]\n");
	printf("\t[-r do a controller reset (i.e. soft reset)]\n");
	printf("\t[-q <num> size of the NVMe queues in items]\n");
	printf("\t     Note: the NVMe namespace is exposed as /dev/ublkb%u via ublk.\n", NVME_HOST_UBLK_ID);
	printf("\t           Requires kernel >= 6.0 with CONFIG_BLK_DEV_UBLK=y.\n");
}

bool do_ctrl_rst = false;
bool do_subs_rst = false;

static int
parse_args(int argc, char **argv, struct spdk_env_opts *env_opts, struct app_ctx *ctx)
{
	int op, rc;

	while ((op = getopt(argc, argv, "s:i:gm:L:hrt:cq:p:d:")) != -1) {
		switch (op) {
		case 'd':
			ctx->select_dev = optarg;
			break;
		case 'q':
			ctx->qsize = spdk_strtol(optarg, 10);
			if (ctx->qsize < 4) {
				fprintf(stderr, "Invalid size of a queue\n");
				exit(EXIT_FAILURE);
			}
			break;
		case 's':
			do_subs_rst = true;
			break;
		case 'i':
			env_opts->shm_id = spdk_strtol(optarg, 10);
			if (env_opts->shm_id < 0) {
				fprintf(stderr, "Invalid shared memory ID\n");
				return env_opts->shm_id;
			}
			break;
		case 'g':
			env_opts->hugepage_single_segments = true;
			break;
		case 'm':
			env_opts->mem_size = spdk_strtol(optarg, 10);
			if (env_opts->mem_size < 0) {
				fprintf(stderr, "Invalid DPDK memory size\n");
				return env_opts->mem_size;
			}
			break;
		case 'L':
			rc = spdk_log_set_flag(optarg);
			if (rc < 0) {
				fprintf(stderr, "unknown flag: %s\n", optarg);
				usage(argv[0]);
				exit(EXIT_FAILURE);
			}
#ifdef DEBUG
			spdk_log_set_print_level(SPDK_LOG_DEBUG);
#endif
			break;
		case 't':
			if (spdk_nvme_transport_id_parse(ctx->trid, optarg) != 0) {
				fprintf(stderr, "Invalid transport ID format '%s'\n", optarg);
				usage(argv[0]);
				return -1;
			}
			break;
		case 'h':
			usage(argv[0]);
			exit(EXIT_SUCCESS);
		case 'r':
			do_ctrl_rst = true;
			break;
		default:
			usage(argv[0]);
			return 1;
		}
	}

	return 0;
}

static int ncd_drv_attach_cb(void *ctx, struct spdk_pci_device *pci_dev)
{
	int rc;
	uint16_t cmd_reg;
	struct app_ctx *app_ctx = ctx;
	struct fpga_hw_ctx *hw = &app_ctx->hw;

	hw->pci_dev = pci_dev;

	/* Enable Memory accesses, PCI busmaster and disable INTx */
	spdk_pci_device_cfg_read16(pci_dev, &cmd_reg, 4);
	cmd_reg |= 0x404;
	spdk_pci_device_cfg_write16(pci_dev, cmd_reg, 4);

	// 1. SKIP Enable device (Apparently, it is enabled by the dpdk-devbind)
	// 2. map BARs for Submission Queue, Completion Queue, and the Data Transmission

	struct fpga_bar_ctx bar0 = {0}, bar2 = {0};

	rc = spdk_pci_device_map_bar(pci_dev, SQ_BAR /* physical BAR0 */, &bar0.vaddr, &bar0.paddr, &bar0.size);
	if (rc) {
		fprintf(stderr, "Unable to map physical BAR 0\n");
		return rc;
	}

	rc = spdk_pci_device_map_bar(pci_dev, WRBUFF_BAR /* physical BAR2 */, &bar2.vaddr, &bar2.paddr, &bar2.size);
	if (rc) {
		fprintf(stderr, "Unable to map physical BAR 2\n");
		return rc;
	}

	/* Two-BAR PF1 layout: BAR0 = {SQ lower half, CQ upper half},
	 * BAR2 = {WrBuf lower half, RdBuf upper half}. Each region is 128 KiB. */
	uint64_t bar0_half = bar0.size / 2;
	uint64_t bar2_half = bar2.size / 2;

	hw->sq.vaddr = bar0.vaddr;
	hw->sq.paddr = bar0.paddr;
	hw->sq.size  = bar0_half;

	hw->cq.vaddr = (uint8_t *)bar0.vaddr + bar0_half;
	hw->cq.paddr = bar0.paddr + bar0_half;
	hw->cq.size  = bar0_half;

	hw->wrbuff.vaddr = bar2.vaddr;
	hw->wrbuff.paddr = bar2.paddr;
	hw->wrbuff.size  = bar2_half;

	hw->rdbuff.vaddr = (uint8_t *)bar2.vaddr + bar2_half;
	hw->rdbuff.paddr = bar2.paddr + bar2_half;
	hw->rdbuff.size  = bar2_half;

	if (hw->cq.vaddr == NULL || hw->sq.vaddr == NULL || hw->rdbuff.vaddr == NULL || hw->wrbuff.vaddr == NULL) {
		fprintf(stderr, "Virtual BAR adresses invalid!\n");
		return -1;
	}
	if (hw->cq.paddr == 0 || hw->sq.paddr == 0 || hw->rdbuff.paddr == 0 || hw->wrbuff.paddr == 0) {
		fprintf(stderr, "Physical BAR adresses invalid!\n");
		return -2;
	}
	if (hw->cq.size == 0 || hw->sq.size == 0 || hw->rdbuff.size == 0 || hw->wrbuff.size == 0) {
		fprintf(stderr, "BAR sizes invalid!\n");
		return -3;
	}

	return 0;
 }

int
main(int argc, char **argv)
{
	int rc;
	struct spdk_env_opts opts;
	struct spdk_pci_driver *ncd_driver;
	struct spdk_nvme_transport_id trid = {0};
	struct spdk_pci_addr pcie_addr;
	struct app_ctx app_ctx = {0};
	struct dma_device_ctx dma_ctx = {0};
	struct fpga_hw_ctx *hw = &app_ctx.hw;

	// Assign default attributes
	trid.trtype = SPDK_NVME_TRANSPORT_PCIE;
	app_ctx.trid = &trid;
	app_ctx.qsize = 0;
	app_ctx.select_dev = "0";
	g_namespace.hw_qid = -1;

	opts.opts_size = sizeof(opts);
	spdk_env_opts_init(&opts);
	rc = parse_args(argc, argv, &opts, &app_ctx);
	if (rc != 0) {
		return rc;
	}

	opts.name = "fpga_zero_copy";
	if (spdk_env_init(&opts) < 0) {
		fprintf(stderr, "Unable to initialize SPDK env\n");
		return -1;
	}

	printf("Initializing NVMe Controller for device %s\n", trid.traddr);
	rc = spdk_nvme_probe(NULL, &app_ctx, probe_cb, attach_cb, NULL);
	if (rc != 0) {
		fprintf(stderr, "ERROR: spdk_nvme_probe() failed\n");
		goto nvme_probe_fail;
	}

	if (g_controller.ctrlr == NULL) {
		fprintf(stderr, "ERROR: ctrlr structure uninitialized! Specify the PCIe address.\n");
		rc = -10;
		goto nvme_probe_fail;
	}

	if (do_ctrl_rst) {
		printf("Resetting controller...");
		if (spdk_nvme_ctrlr_reset(g_controller.ctrlr)) {
			fprintf(stderr, "Failed to reset controller!");
			rc = -11;
		}
		goto ctrlr_reset_fail;
	}

	if (do_subs_rst) {
		if (spdk_nvme_ctrlr_reset_subsystem(g_controller.ctrlr)) {
			fprintf(stderr, "Failed to reset subsystem!");
			rc = -12;
		}
		goto ctrlr_reset_fail;
	}

	ncd_driver = spdk_pci_get_driver("ncd");
	if (ncd_driver == NULL) {
		fprintf(stderr, "Unable to get NCD driver!\n");
		rc = -13;
		goto ctrlr_reset_fail;
	}

	rc = dma_dev_init(app_ctx.select_dev, &dma_ctx);
	if (rc) {
		fprintf(stderr, "Error initializing DMA NFB Device\n");
		goto dma_dev_init_fail;
	}

	rc = spdk_pci_addr_parse(&pcie_addr, dma_ctx.pcie_bdf);
	if (rc) {
		fprintf(stderr, "Unable to parse PCIE address!\n");
		goto dma_dev_init_fail;
	}

	pcie_addr.func += 1;

	rc = spdk_pci_device_attach(ncd_driver, ncd_drv_attach_cb, &app_ctx, &pcie_addr);
	if (rc) {
		fprintf(stderr, "Unable to attach PCIE device!\n");
		goto dma_dev_init_fail;
	}

	printf("PCIE domain initialization complete\n");
	printf("NCD PCIe device context:\n");
	printf("\tSQ VADDR: %p\n", hw->sq.vaddr);
	printf("\tSQ PADDR: %lx\n", hw->sq.paddr);
	printf("\tSQ size:  %ld bytes\n", hw->sq.size);
	printf("\tCQ VADDR: %p\n", hw->cq.vaddr);
	printf("\tCQ PADDR: %lx\n", hw->cq.paddr);
	printf("\tCQ size:  %ld bytes\n", hw->cq.size);
	printf("\tWRBUFF VADDR: %p\n", hw->wrbuff.vaddr);
	printf("\tWRBUFF PADDR: %lx\n", hw->wrbuff.paddr);
	printf("\tWRBUFF size:  %ld bytes\n", hw->wrbuff.size);
	printf("\tRDBUFF VADDR: %p\n", hw->rdbuff.vaddr);
	printf("\tRDBUFF PADDR: %lx\n", hw->rdbuff.paddr);
	printf("\tRDBUFF size:  %ld bytes\n", hw->rdbuff.size);

	rc = queues_alloc(&app_ctx);
	if (rc) {
		fprintf(stderr, "Unable to allocate queues!\n");
		goto queue_alloc_fail;
	}
	printf("\tSQTDBL physical address: 0x%lx\n", hw->sqtdbl_paddr);
	printf("\tCQHDBL physical address: 0x%lx\n", hw->cqhdbl_paddr);
	printf("Queues allocated.\n");

	rc = dma_ctrl_init(hw, &dma_ctx);
	if (rc) {
		fprintf(stderr, "Error configuring the DMA Iuventus controller structure\n");
		goto dma_ctrl_alloc_fail;
	}

	prp_list_print(hw);

	/*
	 * Initialise the host-side filesystem path.  This registers a thin
	 * NVMe bdev backed by per-channel host software queue pairs and exposes
	 * it via ublk so the host can format and mount a standard Linux
	 * filesystem:
	 *
	 *   mkfs.ext4 /dev/ublkb0
	 *   mount /dev/ublkb0 /mnt/nvme
	 *
	 * The FPGA continues to use its own dedicated queue pair (hw_qid)
	 * managed entirely by the FPGA DMA logic – no additional
	 * synchronisation between the two paths is required here.
	 */
	rc = host_fs_init(g_namespace.ns);
	if (rc) {
		fprintf(stderr, "Warning: host filesystem (ublk) initialisation failed (%d).\n"
			"The FPGA path will continue to operate normally.\n", rc);
		/* Non-fatal: proceed without the host filesystem. */
		rc = 0;
	}

	nfb_comp_write16(dma_ctx.comp, REG_CONTROL, CTRL_RPT_UPD_EN | CTRL_ENABLE);
	usleep(1);

	signal(SIGINT, sig_usr);
	signal(SIGTERM, sig_usr);

	printf("Initialization complete. Starting main loop.\n");

	usleep(1000);

	/*
	 * Main loop: poll every tracked SPDK thread so that ublk requests
	 * are processed and NVMe completions on the SW queue pair are
	 * returned to the kernel in a timely fashion.
	 * The FPGA operates independently through its own hardware queue pair.
	 *
	 * ublk_create_target() creates one poll-group thread per requested
	 * CPU core; all of them are tracked in g_all_threads[].
	 */
	while (!stop) {
		int i;
		for (i = 0; i < g_num_threads; i++) {
			spdk_thread_poll(g_all_threads[i], 0, 0);
		}
		usleep(1000);
	}
	nfb_comp_write16(dma_ctx.comp, REG_CONTROL, 0);

	while (nfb_comp_read16(dma_ctx.comp, REG_SQTDBL) != nfb_comp_read16(dma_ctx.comp, REG_SQHDBL) &&
		(nfb_comp_read8(dma_ctx.comp, REG_STATUS) & 0x1) == 0) {
		usleep(1000000);
	}

	/* Tear down the host filesystem before any lower-level cleanup. */
	host_fs_fini();

	dma_ctrl_deinit(hw, &dma_ctx);

	rd_ctrl_regs(g_namespace.ctrlr);

dma_ctrl_alloc_fail:
	queues_dealloc(hw);
	printf("Qeues dealloced\n");
queue_alloc_fail:
	if (hw->pci_dev != NULL) {
		spdk_pci_device_detach(hw->pci_dev);
		hw->pci_dev = NULL;
	}
	printf("Pcie dev detached\n");
dma_dev_init_fail:
	if (dma_ctx.dev != NULL) {
		nfb_close(dma_ctx.dev);
		dma_ctx.dev = NULL;
	}
ctrlr_reset_fail:
	if (g_controller.ctrlr != NULL) {
		spdk_nvme_detach(g_controller.ctrlr);
		g_controller.ctrlr = NULL;
	}
	printf("NVME Controller detached\n");
nvme_probe_fail:
	spdk_env_fini();
	printf("Env finished\n");
	return rc;
}
