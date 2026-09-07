/*
 *   iuventus.c: host-side environment for the DMA Iuventus FPGA-to-NVMe peer-to-peer engine
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) Vladislav Válek
 */

#include "spdk/stdinc.h"

#include <libfdt.h>
#include <nfb/nfb.h>

#include "spdk/iuventus.h"

#include "spdk/env.h"
#include "spdk/log.h"
#include "spdk/memory.h"
#include "spdk/nvme.h"
#include "spdk/nvme_spec.h"
#include "spdk/string.h"

/*
 * Card control-register map, reached through the nfb component "ziti,dma_iuventus".
 */
#define REG_CONTROL             0x000
#define REG_STATUS              0x004
/* COMMON block: single, shared across queues (data pool + metadata). */
#define REG_RDBUFF_BADDR        0x008
#define REG_RDBUFF_PRP_LIST_PTR 0x010
#define REG_WRBUFF_BADDR        0x018
#define REG_WRBUFF_PRP_LIST_PTR 0x020
#define REG_META_PTR            0x028
#define REG_RD_PAGES_FREE       0x0F4
/* The queues live in their own BARs rather than at the head of the data pool, so page 0 is data
 * page 0. The card does hold back one page per PRP list page group, but those sit at the top of
 * each group, not at index 0, and the list must still carry an entry for every index below them. */
#define FIRST_DATA_PAGE         0
/* Must match the page count the card is built with. The card indexes its PRP list as
 * base + page_idx*8 over this whole range, so the list has to be this many entries or it
 * reads past the allocation -- silently, since the drive just gets whatever follows in the
 * hugepage. */
#define MANAGED_PAGES           1024
/* NVMe 2.0e 4.1.2 confines a PRP List to ONE memory page and reaches any further page only
 * through a chain pointer in that page's last entry, which the card does not emit. The list may
 * still span pages: the card's allocator holds back the last page of every group, so no granted
 * run straddles a boundary and no walk reaches a slot where a chain pointer would have to be.
 * That only holds while the groups divide the pool evenly. */
#define PRP_LIST_ENTRIES_PER_PAGE (VALUE_4KB / 8)
#define PRP_LIST_BYTES          (MANAGED_PAGES * 8)
_Static_assert(MANAGED_PAGES % PRP_LIST_ENTRIES_PER_PAGE == 0,
	       "MANAGED_PAGES must be a multiple of the entries one PRP list page holds, or the "
	       "card's reserved pages do not line up with the list page boundaries");
/* PER-QUEUE 2D block: queue q's registers at PER_Q_BASE + q*PER_Q_STRIDE + field offset. */
#define PER_Q_BASE              0x200
#define PER_Q_STRIDE            0x040
#define PQ_SQTDBL               0x00
#define PQ_SQHDBL               0x04
#define PQ_CQHDBL               0x08
#define PQ_DBL_MASK             0x0C
#define PQ_SQTDBL_BADDR         0x10
#define PQ_CQHDBL_BADDR         0x18
#define PQ_LBA_SPACE_SIZE       0x20
#define PQ_NAMESPACE_ID         0x28
#define PQ_LBA_NUM_MASK         0x2C
#define PQ(q, off) (PER_Q_BASE + (uint32_t)(q) * PER_Q_STRIDE + (off))

#define CTRL_ENABLE       (1 << 0)
#define CTRL_RPT_UPD_EN   (1 << 4)

#define STAT_READY                (1 << 0)
#define STAT_RST_DONE             (1 << 1)
#define STAT_TAG_FIFO_INIT_DONE   (1 << 2)

#define SQ_BAR 0
#define CQ_BAR 1
#define WRBUFF_BAR 2
/* BAR2 is a 64-bit BAR, so it consumes config-space registers 2 AND 3; RDBUFF is the next
 * usable index, 4 (also 64-bit, consuming 4 and 5). */
#define RDBUFF_BAR 4

/* Options this library takes off the command line before the caller sees it. */
#define IUVENTUS_GETOPT_STRING "hd:t:q:RQ:N:"

static struct spdk_pci_id ncd_pci_driver_id[] = {
	{
		SPDK_PCI_DEVICE(0x18ec, 0xc020)
	},
	{
		SPDK_PCI_DEVICE(0x1c2c, 0xc020)
	},
};

SPDK_PCI_DRIVER_REGISTER(ncd, ncd_pci_driver_id, SPDK_PCI_DRIVER_NEED_MAPPING)

struct admin_cmd_ctx {
	const char	*cmd_name;
	bool		done;
	int		status;
};

/* Index of the peer with this transport address, or -1. */
static int
peer_by_traddr(const struct iuventus_ctx *ctx, const char *traddr)
{
	uint32_t i;

	for (i = 0; i < ctx->num_peers; i++) {
		if (!strcmp(ctx->peers[i].trid.traddr, traddr)) {
			return (int)i;
		}
	}
	return -1;
}

static void
qop_complete_cb(void *cb_ctx, const struct spdk_nvme_cpl *cpl)
{
	struct admin_cmd_ctx *cmd_ctx = cb_ctx;

	cmd_ctx->done = true;
	if (spdk_nvme_cpl_is_error(cpl)) {
		spdk_nvme_print_completion(0, (struct spdk_nvme_cpl *)cpl);
		SPDK_ERRLOG("CPL error status: %s\n",
			    spdk_nvme_cpl_get_status_string(&cpl->status));
		SPDK_ERRLOG("%s failed, aborting run\n", cmd_ctx->cmd_name);
		cmd_ctx->status = -1;
	}
}

static int
submit_admin_request_on(struct spdk_nvme_ctrlr *ctrlr, struct spdk_nvme_cmd *cmd,
			const char *cmd_name)
{
	int rc = 0;
	struct admin_cmd_ctx cmd_ctx = {
		.cmd_name = cmd_name,
	};

	rc = spdk_nvme_ctrlr_cmd_admin_raw(ctrlr, cmd, NULL, 0, qop_complete_cb, &cmd_ctx);
	if (rc) {
		SPDK_ERRLOG("Failed to submit the command %s!\n", cmd_ctx.cmd_name);
		return -EIO;
	}

	while (!cmd_ctx.done) {
		spdk_nvme_ctrlr_process_admin_completions(ctrlr);
	}

	if (cmd_ctx.status != 0) {
		return -EIO;
	}

	return 0;
}

static bool
probe_cb(void *cb_ctx, const struct spdk_nvme_transport_id *trid,
	 struct spdk_nvme_ctrlr_opts *opts)
{
	struct iuventus_ctx *ctx = cb_ctx;

	SPDK_NOTICELOG("Probing %s ...\n", trid->traddr);

	if (ctx->qsize != 0) {
		opts->io_queue_size = ctx->qsize;
		opts->io_queue_requests = ctx->qsize;
	} else {
		opts->io_queue_requests = opts->io_queue_size;
	}
	opts->arb_mechanism = SPDK_NVME_CC_AMS_RR;
	opts->enable_interrupts = false;

	return peer_by_traddr(ctx, trid->traddr) >= 0;
}

static void
attach_cb(void *cb_ctx, const struct spdk_nvme_transport_id *trid,
	  struct spdk_nvme_ctrlr *ctrlr, const struct spdk_nvme_ctrlr_opts *opts)
{
	struct iuventus_ctx *ctx = cb_ctx;
	struct iuventus_peer *peer;
	const struct spdk_nvme_ctrlr_data *cdata;
	union spdk_nvme_cap_register cap;
	struct spdk_nvme_ns *ns;
	struct spdk_pci_device *pci_dev;
	struct spdk_pci_addr pci_addr;
	char bdf[32];
	char sysfs_path[128];
	uint64_t bar_start, bar_end, bar_flags;
	uint32_t sect_size;
	FILE *fp;
	int nsid;
	int idx;

	SPDK_NOTICELOG("Attaching to %s ...\n", trid->traddr);

	/*
	 * spdk_nvme_ctrlr is the logical abstraction in SPDK for an NVMe controller. During
	 * initialization, the IDENTIFY data for the controller is read using an NVMe admin
	 * command, and that data can be retrieved using spdk_nvme_ctrlr_get_data().
	 */
	cdata = spdk_nvme_ctrlr_get_data(ctrlr);
	cap = spdk_nvme_ctrlr_get_regs_cap(ctrlr);

	idx = peer_by_traddr(ctx, trid->traddr);
	if (idx < 0) {
		SPDK_ERRLOG("attached %s was never requested\n", trid->traddr);
		return;
	}
	peer = &ctx->peers[idx];

	snprintf(peer->name, sizeof(peer->name), "%-20.20s", cdata->mn);
	peer->ctrlr = ctrlr;

	nsid = spdk_nvme_ctrlr_get_first_active_ns(ctrlr);
	ns = spdk_nvme_ctrlr_get_ns(ctrlr, nsid);
	if (ns == NULL) {
		SPDK_ERRLOG("Invalid namespace!\n");
		return;
	}

	if (!spdk_nvme_ns_is_active(ns)) {
		return;
	}

	peer->ns = ns;
	peer->attached = true;
	sect_size = spdk_nvme_ns_get_sector_size(ns);

	/* An MDTS of zero means the controller places no bound on a transfer. */
	if (cdata->mdts == 0) {
		peer->lba_num_mask = 0xFFFF;
	} else {
		peer->lba_num_mask =
			(uint16_t)((1 << (12 + cap.bits.mpsmin + cdata->mdts)) / sect_size) - 1;
	}

	peer->lba_space_size = spdk_nvme_ns_get_num_sectors(ns);

	SPDK_NOTICELOG("Controller options for %s:\n", trid->traddr);
	SPDK_NOTICELOG("\tNumber of IO queues:   %d\n", opts->num_io_queues);
	SPDK_NOTICELOG("\tSize of IO queues:     %d\n", opts->io_queue_size);
	SPDK_NOTICELOG("\tIO queue requests:     %d\n", opts->io_queue_requests);
	SPDK_NOTICELOG("\tNS %d size:            %juGB\n", nsid,
		       spdk_nvme_ns_get_size(ns) / 1000000000);
	SPDK_NOTICELOG("\tNS number of sectors:  %ld\n", spdk_nvme_ns_get_num_sectors(ns));
	SPDK_NOTICELOG("\tNS sector size:        %dB\n", sect_size);
	SPDK_NOTICELOG("\tLBA Mask:              x%x (%d)\n", peer->lba_num_mask,
		       peer->lba_num_mask);

	pci_dev = spdk_nvme_ctrlr_get_pci_device(ctrlr);
	if (!pci_dev) {
		SPDK_ERRLOG("Device is not PCI-backed, can't retrieve BDF.\n");
		return;
	}
	pci_addr = spdk_pci_device_get_addr(pci_dev);

	snprintf(bdf, sizeof(bdf), "%04x:%02x:%02x.%x", pci_addr.domain, pci_addr.bus,
		 pci_addr.dev, pci_addr.func);
	snprintf(sysfs_path, sizeof(sysfs_path), "/sys/bus/pci/devices/%s/resource", bdf);

	fp = fopen(sysfs_path, "r");
	if (!fp) {
		SPDK_ERRLOG("Failed to open resource file %s\n", sysfs_path);
		return;
	}

	if (fscanf(fp, "%lx %lx %lx", &bar_start, &bar_end, &bar_flags) != 3) {
		SPDK_ERRLOG("Failed to parse resource file %s\n", sysfs_path);
		fclose(fp);
		return;
	}

	fclose(fp);

	SPDK_NOTICELOG("Physical address of NVMe BAR0: 0x%lx\n", bar_start);

	/* Doorbells start 0x1000 into BAR0, counted from the admin queue pair. Every drive has its
	 * own BAR, hence its own doorbell page. */
	peer->doorbell_base = bar_start + 0x1000;
	peer->doorbell_stride = cap.bits.dstrd;
	peer->nsid = nsid;
	SPDK_NOTICELOG("Doorbell stride: %d\n", peer->doorbell_stride);
}

static void
queues_delete(struct iuventus_ctx *ctx)
{
	struct spdk_nvme_cmd cmd;
	uint32_t q;

	for (q = 0; q < ctx->num_peers; q++) {
		struct iuventus_peer *peer = &ctx->peers[q];

		if (peer->ctrlr == NULL) {
			continue;
		}
		if (peer->sq_created) {
			memset(&cmd, 0, sizeof(cmd));
			cmd.opc = SPDK_NVME_OPC_DELETE_IO_SQ;
			cmd.cdw10_bits.delete_io_q.qid = peer->hw_qid;
			if (submit_admin_request_on(peer->ctrlr, &cmd, "SQ_DELETE")) {
				SPDK_ERRLOG("Failed SQ_DELETE (q%u)!\n", q);
			}
			peer->sq_created = false;
		}
		if (peer->cq_created) {
			memset(&cmd, 0, sizeof(cmd));
			cmd.opc = SPDK_NVME_OPC_DELETE_IO_CQ;
			cmd.cdw10_bits.delete_io_q.qid = peer->hw_qid;
			if (submit_admin_request_on(peer->ctrlr, &cmd, "CQ_DELETE")) {
				SPDK_ERRLOG("Failed CQ_DELETE (q%u)!\n", q);
			}
			peer->cq_created = false;
		}
	}
}

static void
queues_dealloc(struct iuventus_ctx *ctx)
{
	uint32_t q;

	queues_delete(ctx);
	for (q = 0; q < ctx->num_peers; q++) {
		struct iuventus_peer *peer = &ctx->peers[q];

		if (peer->qid_alloc && peer->ctrlr != NULL) {
			spdk_nvme_ctrlr_free_qid(peer->ctrlr, peer->hw_qid);
			peer->qid_alloc = false;
			peer->hw_qid = -1;
		}
	}
}

/*
 * One NVMe I/O queue pair per drive, each on its own controller, with its submission and
 * completion rings at flat page q of the card's queue BARs. All of them share the one data pool.
 */
static int
queues_alloc(struct iuventus_ctx *ctx)
{
	struct spdk_nvme_io_qpair_opts qopts;
	struct spdk_nvme_cmd cmd = {0};
	uint32_t q;
	int rc = 0;

	spdk_nvme_ctrlr_get_default_io_qpair_opts(ctx->peers[0].ctrlr, &qopts, sizeof(qopts));

	SPDK_NOTICELOG("Default queue pair options: size %d, requests %d\n",
		       qopts.io_queue_size, qopts.io_queue_requests);

	if (ctx->qsize == 0) {
		qopts.io_queue_requests = qopts.io_queue_size;
	} else {
		qopts.io_queue_requests = ctx->qsize;
		qopts.io_queue_size = ctx->qsize;
	}

	for (q = 0; q < ctx->num_peers; q++) {
		struct iuventus_peer *peer = &ctx->peers[q];
		void *sq_vaddr = (uint8_t *)ctx->sq.vaddr + (uint64_t)q * VALUE_4KB;
		uint64_t sq_paddr = ctx->sq.paddr + (uint64_t)q * VALUE_4KB;
		void *cq_vaddr = (uint8_t *)ctx->cq.vaddr + (uint64_t)q * VALUE_4KB;
		uint64_t cq_paddr = ctx->cq.paddr + (uint64_t)q * VALUE_4KB;
		uint8_t *cpl_buff;
		int32_t qid;
		uint32_t i;

		peer->doorbell_mask = qopts.io_queue_size - 1;

		qopts.sq.vaddr = sq_vaddr;
		qopts.sq.paddr = sq_paddr;
		qopts.sq.buffer_size = qopts.io_queue_size * sizeof(struct spdk_nvme_cmd);
		qopts.cq.vaddr = cq_vaddr;
		qopts.cq.paddr = cq_paddr;
		qopts.cq.buffer_size = qopts.io_queue_size * sizeof(struct spdk_nvme_cpl);

		/* Clear this completion ring before the drive can post into it. Every byte, not
		 * just the phase tag at offset 14: a sparse byte write is a read-modify-write and
		 * this BAR does not support reads, so the narrow form never lands. */
		cpl_buff = cq_vaddr;
		for (i = 0; i < qopts.cq.buffer_size; i++) {
			cpl_buff[i] = 0;
		}

		qid = spdk_nvme_ctrlr_alloc_qid(peer->ctrlr);
		if (qid < 0) {
			SPDK_ERRLOG("Failed to allocate QID for card queue %u\n", q);
			rc = -ENOSPC;
			goto fail;
		}
		peer->hw_qid = qid;
		peer->qid_alloc = true;
		peer->sqtdbl_paddr = peer->doorbell_base +
				     (2 * qid) * (4 << peer->doorbell_stride);
		peer->cqhdbl_paddr = peer->doorbell_base +
				     (2 * qid + 1) * (4 << peer->doorbell_stride);
		SPDK_NOTICELOG("Card queue %u -> %s qid %d, flat page %u\n", q,
			       peer->trid.traddr, qid, q);
		SPDK_NOTICELOG("\tSQTDBL physical address: 0x%lx\n", peer->sqtdbl_paddr);
		SPDK_NOTICELOG("\tCQHDBL physical address: 0x%lx\n", peer->cqhdbl_paddr);

		memset(&cmd, 0, sizeof(cmd));
		cmd.opc = SPDK_NVME_OPC_CREATE_IO_CQ;
		cmd.nsid = 0;
		cmd.cdw10_bits.create_io_q.qid = qid;
		cmd.cdw10_bits.create_io_q.qsize = qopts.io_queue_size - 1;
		cmd.cdw11_bits.create_io_cq.pc = 1;
		cmd.dptr.prp.prp1 = cq_paddr;
		rc = submit_admin_request_on(peer->ctrlr, &cmd, "CQ_CREATE");
		if (rc) {
			SPDK_ERRLOG("Failed CQ_CREATE (q%u)!\n", q);
			goto fail;
		}
		peer->cq_created = true;

		memset(&cmd, 0, sizeof(cmd));
		cmd.opc = SPDK_NVME_OPC_CREATE_IO_SQ;
		cmd.nsid = 0;
		cmd.cdw10_bits.create_io_q.qid = qid;
		cmd.cdw10_bits.create_io_q.qsize = qopts.io_queue_size - 1;
		cmd.cdw11_bits.create_io_sq.pc = 1;
		cmd.cdw11_bits.create_io_sq.qprio = 2;
		cmd.cdw11_bits.create_io_sq.cqid = qid;
		cmd.dptr.prp.prp1 = sq_paddr;
		rc = submit_admin_request_on(peer->ctrlr, &cmd, "SQ_CREATE");
		if (rc) {
			SPDK_ERRLOG("Failed SQ_CREATE (q%u)!\n", q);
			goto fail;
		}
		peer->sq_created = true;
	}

	SPDK_NOTICELOG("%u card queue(s) allocated, one per drive.\n", ctx->num_peers);
	return 0;

fail:
	queues_dealloc(ctx);
	return rc;
}

static int
dma_dev_init(struct iuventus_ctx *ctx, const char *nfb_dev)
{
	const void *fdt;
	int fdt_offset;
	int len;
	int rc;

	ctx->nfb_dev = nfb_open(nfb_dev);
	if (!ctx->nfb_dev) {
		SPDK_ERRLOG("Failed to open NFB device %s\n", nfb_dev);
		rc = -ENODEV;
		goto dev_open_fail;
	}

	fdt = nfb_get_fdt(ctx->nfb_dev);
	fdt_offset = fdt_path_offset(fdt, "/system/device/endpoint0");
	ctx->pcie_bdf = fdt_getprop(fdt, fdt_offset, "pci-slot", &len);
	if (len < 0) {
		SPDK_ERRLOG("Failed to get pci-slot from the device tree!\n");
		rc = -ENOENT;
		goto fdt_get_fail;
	}

	return 0;

fdt_get_fail:
	nfb_close(ctx->nfb_dev);
	ctx->nfb_dev = NULL;
	ctx->pcie_bdf = NULL;
dev_open_fail:
	return rc;
}

static void
dma_dev_fini(struct iuventus_ctx *ctx)
{
	if (ctx->nfb_dev != NULL) {
		nfb_close(ctx->nfb_dev);
		ctx->nfb_dev = NULL;
	}
	ctx->pcie_bdf = NULL;
}

static void
prp_list_free(struct iuventus_ctx *ctx)
{
	if (!ctx->prp_lists_allocated) {
		return;
	}

	spdk_dma_free(ctx->wrbuff_prp_list.vaddr);
	spdk_dma_free(ctx->rdbuff_prp_list.vaddr);
	memset(&ctx->wrbuff_prp_list, 0, sizeof(ctx->wrbuff_prp_list));
	memset(&ctx->rdbuff_prp_list, 0, sizeof(ctx->rdbuff_prp_list));
	ctx->prp_lists_allocated = false;
}

static int
prp_list_alloc(struct iuventus_ctx *ctx)
{
	uint64_t size = PRP_LIST_BYTES;
	int i;
	int rc;

	ctx->wrbuff_prp_list.vaddr = spdk_dma_zmalloc(PRP_LIST_BYTES, VALUE_4KB, NULL);
	if (ctx->wrbuff_prp_list.vaddr == NULL) {
		SPDK_ERRLOG("Write PRP list allocation failed\n");
		return -ENOMEM;
	}

	ctx->rdbuff_prp_list.vaddr = spdk_dma_zmalloc(PRP_LIST_BYTES, VALUE_4KB, NULL);
	if (ctx->rdbuff_prp_list.vaddr == NULL) {
		SPDK_ERRLOG("Read PRP list allocation failed\n");
		rc = -ENOMEM;
		goto fail;
	}

	ctx->wrbuff_prp_list.paddr = spdk_vtophys(ctx->wrbuff_prp_list.vaddr, &size);
	if (ctx->wrbuff_prp_list.paddr == SPDK_VTOPHYS_ERROR) {
		SPDK_ERRLOG("Failed to get physical address of the Write PRP list buffer\n");
		rc = -EFAULT;
		goto fail;
	}

	if (size != PRP_LIST_BYTES) {
		SPDK_ERRLOG("Write PRP list buffer is not physically contiguous over %d B "
			    "(detected size: %lu)\n", PRP_LIST_BYTES, size);
		rc = -EFAULT;
		goto fail;
	}

	ctx->rdbuff_prp_list.paddr = spdk_vtophys(ctx->rdbuff_prp_list.vaddr, &size);
	if (ctx->rdbuff_prp_list.paddr == SPDK_VTOPHYS_ERROR) {
		SPDK_ERRLOG("Failed to get physical address of the Read PRP list buffer\n");
		rc = -EFAULT;
		goto fail;
	}

	if (size != PRP_LIST_BYTES) {
		SPDK_ERRLOG("Read PRP list buffer is not physically contiguous over %d B "
			    "(detected size: %lu)\n", PRP_LIST_BYTES, size);
		rc = -EFAULT;
		goto fail;
	}

	/* The card can hand out any page in [0, MANAGED_PAGES), so every entry must be valid. A
	 * data BAR smaller than that would let it address past the window -- refuse rather than
	 * leave zeroed entries pointing at physical address 0. */
	if (ctx->wrbuff.size < (uint64_t)MANAGED_PAGES * VALUE_4KB ||
	    ctx->rdbuff.size < (uint64_t)MANAGED_PAGES * VALUE_4KB) {
		SPDK_ERRLOG("data BARs too small for %d managed pages (wrbuff %llu B, "
			    "rdbuff %llu B, need %llu B each)\n",
			    MANAGED_PAGES, (unsigned long long)ctx->wrbuff.size,
			    (unsigned long long)ctx->rdbuff.size,
			    (unsigned long long)MANAGED_PAGES * VALUE_4KB);
		rc = -ENOSPC;
		goto fail;
	}

	for (i = FIRST_DATA_PAGE; i < MANAGED_PAGES; i++) {
		((uint64_t *)ctx->wrbuff_prp_list.vaddr)[i - FIRST_DATA_PAGE] =
			ctx->wrbuff.paddr + ((uint64_t)i * VALUE_4KB);
		((uint64_t *)ctx->rdbuff_prp_list.vaddr)[i - FIRST_DATA_PAGE] =
			ctx->rdbuff.paddr + ((uint64_t)i * VALUE_4KB);
	}
	ctx->wrbuff_prp_list.entries = MANAGED_PAGES - FIRST_DATA_PAGE;
	ctx->rdbuff_prp_list.entries = MANAGED_PAGES - FIRST_DATA_PAGE;
	ctx->prp_lists_allocated = true;

	return 0;

fail:
	spdk_dma_free(ctx->wrbuff_prp_list.vaddr);
	spdk_dma_free(ctx->rdbuff_prp_list.vaddr);
	memset(&ctx->wrbuff_prp_list, 0, sizeof(ctx->wrbuff_prp_list));
	memset(&ctx->rdbuff_prp_list, 0, sizeof(ctx->rdbuff_prp_list));
	return rc;
}

static void
dma_ctrl_deinit(struct iuventus_ctx *ctx)
{
	prp_list_free(ctx);
	if (ctx->nfb_comp != NULL) {
		nfb_comp_close(ctx->nfb_comp);
		ctx->nfb_comp = NULL;
	}
}

static int
dma_ctrl_init(struct iuventus_ctx *ctx)
{
	uint32_t q;
	int node;
	int rc;

	node = nfb_comp_find(ctx->nfb_dev, "ziti,dma_iuventus", 0);
	ctx->nfb_comp = nfb_comp_open(ctx->nfb_dev, node);
	if (ctx->nfb_comp == NULL) {
		SPDK_ERRLOG("Failed to open the control registers as an nfb component!\n");
		return -ENODEV;
	}

	rc = prp_list_alloc(ctx);
	if (rc) {
		SPDK_ERRLOG("PRP list allocation failed\n");
		nfb_comp_close(ctx->nfb_comp);
		ctx->nfb_comp = NULL;
		return rc;
	}

	/* COMMON: the shared data pool and its metadata. */
	nfb_comp_write64(ctx->nfb_comp, REG_RDBUFF_BADDR, ctx->rdbuff.paddr);
	nfb_comp_write64(ctx->nfb_comp, REG_RDBUFF_PRP_LIST_PTR, ctx->rdbuff_prp_list.paddr);
	nfb_comp_write64(ctx->nfb_comp, REG_WRBUFF_BADDR, ctx->wrbuff.paddr);
	nfb_comp_write64(ctx->nfb_comp, REG_WRBUFF_PRP_LIST_PTR, ctx->wrbuff_prp_list.paddr);
	nfb_comp_write64(ctx->nfb_comp, REG_META_PTR, 0);

	/* PER-QUEUE: one block per card queue, each pointing at its own drive's doorbells. */
	for (q = 0; q < ctx->num_peers; q++) {
		const struct iuventus_peer *peer = &ctx->peers[q];

		nfb_comp_write16(ctx->nfb_comp, PQ(q, PQ_DBL_MASK), peer->doorbell_mask);
		nfb_comp_write64(ctx->nfb_comp, PQ(q, PQ_SQTDBL_BADDR), peer->sqtdbl_paddr);
		nfb_comp_write64(ctx->nfb_comp, PQ(q, PQ_CQHDBL_BADDR), peer->cqhdbl_paddr);
		nfb_comp_write64(ctx->nfb_comp, PQ(q, PQ_LBA_SPACE_SIZE), peer->lba_space_size);
		nfb_comp_write16(ctx->nfb_comp, PQ(q, PQ_LBA_NUM_MASK), peer->lba_num_mask);
		nfb_comp_write32(ctx->nfb_comp, PQ(q, PQ_NAMESPACE_ID), 1);
	}

	return 0;
}

static int
ncd_drv_attach_cb(void *cb_ctx, struct spdk_pci_device *pci_dev)
{
	struct iuventus_ctx *ctx = cb_ctx;
	uint16_t cmd_reg;
	int rc;

	ctx->pci_dev = pci_dev;

	/* Enable memory accesses and PCI bus mastering, and disable INTx. The device itself is
	 * already enabled by the uio bind, so only the command register needs touching. */
	spdk_pci_device_cfg_read16(pci_dev, &cmd_reg, 4);
	cmd_reg |= 0x404;
	spdk_pci_device_cfg_write16(pci_dev, cmd_reg, 4);

	rc = spdk_pci_device_map_bar(pci_dev, SQ_BAR, &ctx->sq.vaddr, &ctx->sq.paddr,
				     &ctx->sq.size);
	if (rc) {
		SPDK_ERRLOG("Unable to map BAR %d (SQ)\n", SQ_BAR);
		return rc;
	}

	rc = spdk_pci_device_map_bar(pci_dev, CQ_BAR, &ctx->cq.vaddr, &ctx->cq.paddr,
				     &ctx->cq.size);
	if (rc) {
		SPDK_ERRLOG("Unable to map BAR %d (CQ)\n", CQ_BAR);
		return rc;
	}

	/* Four separate regions, one per logical BAR: the queues in their own small BARs and the
	 * data pools in their own large 64-bit BARs. They are not aliases of the queue BARs, and
	 * mapping them separately is what makes the card's page index and this host's addresses
	 * agree. */
	rc = spdk_pci_device_map_bar(pci_dev, WRBUFF_BAR, &ctx->wrbuff.vaddr, &ctx->wrbuff.paddr,
				     &ctx->wrbuff.size);
	if (rc) {
		SPDK_ERRLOG("Unable to map BAR %d (WRBUFF)\n", WRBUFF_BAR);
		return rc;
	}

	rc = spdk_pci_device_map_bar(pci_dev, RDBUFF_BAR, &ctx->rdbuff.vaddr, &ctx->rdbuff.paddr,
				     &ctx->rdbuff.size);
	if (rc) {
		SPDK_ERRLOG("Unable to map BAR %d (RDBUFF)\n", RDBUFF_BAR);
		return rc;
	}

	if (ctx->sq.vaddr == NULL || ctx->cq.vaddr == NULL ||
	    ctx->wrbuff.vaddr == NULL || ctx->rdbuff.vaddr == NULL) {
		SPDK_ERRLOG("Virtual BAR addresses invalid!\n");
		return -EFAULT;
	}
	if (ctx->sq.paddr == 0 || ctx->cq.paddr == 0 ||
	    ctx->wrbuff.paddr == 0 || ctx->rdbuff.paddr == 0) {
		SPDK_ERRLOG("Physical BAR addresses invalid!\n");
		return -EFAULT;
	}
	if (ctx->sq.size == 0 || ctx->cq.size == 0 ||
	    ctx->wrbuff.size == 0 || ctx->rdbuff.size == 0) {
		SPDK_ERRLOG("BAR sizes invalid!\n");
		return -EFAULT;
	}

	return 0;
}

static void
bars_print(const struct iuventus_ctx *ctx)
{
	SPDK_NOTICELOG("PCIe domain initialization complete\n");
	SPDK_NOTICELOG("\tSQ     VADDR %p PADDR 0x%lx size %lu B\n",
		       ctx->sq.vaddr, ctx->sq.paddr, ctx->sq.size);
	SPDK_NOTICELOG("\tCQ     VADDR %p PADDR 0x%lx size %lu B\n",
		       ctx->cq.vaddr, ctx->cq.paddr, ctx->cq.size);
	SPDK_NOTICELOG("\tWRBUFF VADDR %p PADDR 0x%lx size %lu B\n",
		       ctx->wrbuff.vaddr, ctx->wrbuff.paddr, ctx->wrbuff.size);
	SPDK_NOTICELOG("\tRDBUFF VADDR %p PADDR 0x%lx size %lu B\n",
		       ctx->rdbuff.vaddr, ctx->rdbuff.paddr, ctx->rdbuff.size);
}

void
iuventus_opts_init(struct iuventus_opts *opts, size_t opts_size)
{
	if (opts == NULL) {
		SPDK_ERRLOG("opts should not be NULL\n");
		return;
	}

	if (opts_size == 0) {
		SPDK_ERRLOG("opts_size should not be zero\n");
		return;
	}

	memset(opts, 0, opts_size);
	opts->opts_size = opts_size;

#define SET_FIELD(field, value) \
	if (offsetof(struct iuventus_opts, field) + sizeof(opts->field) <= opts_size) { \
		opts->field = value; \
	}

	SET_FIELD(nfb_dev, "0");
	SET_FIELD(qsize, 0);
	SET_FIELD(rpt_upd_en, false);
	SET_FIELD(num_peers, 0);

#undef SET_FIELD
}

void
iuventus_usage(void)
{
	printf("DMA Iuventus options:\n");
	printf("\t[-d selected nfb device (default 0)]\n");
	printf("\t[-t <fmt> Transport ID for local PCIe NVMe]\n");
	printf("\t\t Format: 'key:value [key:value] ...'\n");
	printf("\t\t Keys:\n");
	printf("\t\t  trtype      Transport type (e.g. PCIe, RDMA)\n");
	printf("\t\t  adrfam      Address family (e.g. IPv4, IPv6)\n");
	printf("\t\t  traddr      Transport address (e.g. 0000:04:00.0 for PCIe)\n");
	printf("\t\t  trsvcid     Transport service identifier (e.g. 4420)\n");
	printf("\t\t  subnqn      Subsystem NQN (default: %s)\n", SPDK_NVMF_DISCOVERY_NQN);
	printf("\t\t  ns          NVMe namespace ID (all active namespaces are used by default)\n");
	printf("\t\t  hostnqn     Host NQN\n");
	printf("\t\t Example: -t 'trtype:PCIe traddr:0000:04:00.0' for PCIe\n");
	printf("\t\t Note: PCIe only. Repeat -t once per SSD -- the number given IS the number of\n");
	printf("\t\t       card queues, and queue q is bound to the q-th -t drive.\n");
	printf("\t[-q <num> size of the NVMe queues in items]\n");
	printf("\t[-R let the card re-write an unchanged doorbell value (off by default)]\n");
	printf("\t[-h show this help]\n");
}

int
iuventus_parse_args(int argc, char **argv, struct iuventus_opts *opts,
		    const char *app_getopt_str, int (*app_parse)(int ch, char *arg),
		    void (*app_usage)(void))
{
	char optstr[128];
	long qsize;
	int len;
	int op;

	if (opts == NULL || opts->opts_size == 0) {
		SPDK_ERRLOG("opts must be initialised with iuventus_opts_init() first\n");
		return IUVENTUS_PARSE_ARGS_FAIL;
	}

	if (app_getopt_str != NULL && app_parse == NULL) {
		SPDK_ERRLOG("app_parse is required when app_getopt_str is given\n");
		return IUVENTUS_PARSE_ARGS_FAIL;
	}

	len = snprintf(optstr, sizeof(optstr), "%s%s", IUVENTUS_GETOPT_STRING,
		       app_getopt_str != NULL ? app_getopt_str : "");
	if (len < 0 || (size_t)len >= sizeof(optstr)) {
		SPDK_ERRLOG("getopt string is too long\n");
		return IUVENTUS_PARSE_ARGS_FAIL;
	}

	while ((op = getopt(argc, argv, optstr)) != -1) {
		switch (op) {
		case 'd':
			opts->nfb_dev = optarg;
			break;
		case 't':
			/* One -t per drive; the card queue count IS the number given. */
			if (opts->num_peers >= IUVENTUS_MAX_PEERS) {
				SPDK_ERRLOG("At most %d drives are supported\n",
					    IUVENTUS_MAX_PEERS);
				goto fail;
			}
			opts->peer_trid[opts->num_peers].trtype = SPDK_NVME_TRANSPORT_PCIE;
			if (spdk_nvme_transport_id_parse(&opts->peer_trid[opts->num_peers],
							 optarg) != 0) {
				SPDK_ERRLOG("Invalid transport ID format '%s'\n", optarg);
				goto fail;
			}
			opts->num_peers++;
			break;
		case 'q':
			qsize = spdk_strtol(optarg, 10);
			if (qsize < 4 || qsize > UINT16_MAX) {
				SPDK_ERRLOG("Invalid size of a queue\n");
				goto fail;
			}
			opts->qsize = (uint16_t)qsize;
			break;
		case 'R':
			opts->rpt_upd_en = true;
			break;
		case 'Q':
			SPDK_ERRLOG("-Q is gone: one instance now drives every queue, and a "
				    "queue's index is its position in the -t list.\n");
			goto fail;
		case 'N':
			SPDK_ERRLOG("-N is gone: exactly one queue pair per SSD, and the queue "
				    "count is the number of -t options given.\n");
			goto fail;
		case 'h':
			if (app_usage != NULL) {
				app_usage();
			}
			iuventus_usage();
			return IUVENTUS_PARSE_ARGS_HELP;
		default:
			if (app_parse != NULL && app_parse(op, optarg) == 0) {
				break;
			}
			goto fail;
		}
	}

	return IUVENTUS_PARSE_ARGS_SUCCESS;

fail:
	if (app_usage != NULL) {
		app_usage();
	}
	iuventus_usage();
	return IUVENTUS_PARSE_ARGS_FAIL;
}

int
iuventus_init(struct iuventus_ctx *ctx, const struct iuventus_opts *opts)
{
	struct spdk_pci_driver *ncd_driver;
	struct spdk_pci_addr pcie_addr;
	uint32_t q;
	int rc;

	if (ctx == NULL || opts == NULL) {
		SPDK_ERRLOG("ctx and opts are both required\n");
		return -EINVAL;
	}

	if (opts->opts_size == 0) {
		SPDK_ERRLOG("opts must be initialised with iuventus_opts_init() first\n");
		return -EINVAL;
	}

	if (opts->num_peers < 1 || opts->num_peers > IUVENTUS_MAX_PEERS) {
		SPDK_ERRLOG("no drive given, or too many. Pass -t once per SSD, at most %d.\n",
			    IUVENTUS_MAX_PEERS);
		return -EINVAL;
	}

	memset(ctx, 0, sizeof(*ctx));
	ctx->num_peers = opts->num_peers;
	ctx->qsize = opts->qsize;
	ctx->rpt_upd_en = opts->rpt_upd_en;

	for (q = 0; q < ctx->num_peers; q++) {
		ctx->peers[q].trid = opts->peer_trid[q];
		ctx->peers[q].hw_qid = -1;
	}

	SPDK_NOTICELOG("Initializing %u NVMe controller(s):\n", ctx->num_peers);
	for (q = 0; q < ctx->num_peers; q++) {
		SPDK_NOTICELOG("\tcard queue %u -> %s\n", q, ctx->peers[q].trid.traddr);
	}

	rc = spdk_nvme_probe(NULL, ctx, probe_cb, attach_cb, NULL);
	if (rc != 0) {
		SPDK_ERRLOG("spdk_nvme_probe() failed\n");
		rc = -ENODEV;
		goto probe_fail;
	}

	for (q = 0; q < ctx->num_peers; q++) {
		if (!ctx->peers[q].attached || ctx->peers[q].ctrlr == NULL) {
			SPDK_ERRLOG("%s did not attach -- is it bound to uio_pci_generic?\n",
				    ctx->peers[q].trid.traddr);
			rc = -ENODEV;
			goto probe_fail;
		}
	}

	if (opts->on_peers_ready != NULL) {
		rc = opts->on_peers_ready(ctx, opts->cb_arg);
		if (rc != 0) {
			goto probe_fail;
		}
	}

	ncd_driver = spdk_pci_get_driver("ncd");
	if (ncd_driver == NULL) {
		SPDK_ERRLOG("Unable to get the NCD driver!\n");
		rc = -ENODEV;
		goto probe_fail;
	}

	rc = dma_dev_init(ctx, opts->nfb_dev != NULL ? opts->nfb_dev : "0");
	if (rc) {
		SPDK_ERRLOG("Error initializing the NFB device\n");
		goto probe_fail;
	}

	rc = spdk_pci_addr_parse(&pcie_addr, ctx->pcie_bdf);
	if (rc) {
		SPDK_ERRLOG("Unable to parse the PCIe address!\n");
		rc = -EINVAL;
		goto pci_attach_fail;
	}

	/* The management function is what the device tree names; the peer BARs live on the next
	 * function of the same device. */
	pcie_addr.func += 1;

	rc = spdk_pci_device_attach(ncd_driver, ncd_drv_attach_cb, ctx, &pcie_addr);
	if (rc) {
		SPDK_ERRLOG("Unable to attach the PCIe device!\n");
		rc = -ENODEV;
		goto pci_attach_fail;
	}

	bars_print(ctx);

	rc = queues_alloc(ctx);
	if (rc) {
		SPDK_ERRLOG("Unable to allocate queues!\n");
		goto queues_fail;
	}

	rc = dma_ctrl_init(ctx);
	if (rc) {
		SPDK_ERRLOG("Error configuring the card's control registers\n");
		goto ctrl_fail;
	}

	return 0;

ctrl_fail:
	queues_dealloc(ctx);
queues_fail:
	if (ctx->pci_dev != NULL) {
		spdk_pci_device_detach(ctx->pci_dev);
		ctx->pci_dev = NULL;
	}
pci_attach_fail:
	dma_dev_fini(ctx);
probe_fail:
	for (q = 0; q < ctx->num_peers; q++) {
		if (ctx->peers[q].ctrlr != NULL) {
			spdk_nvme_detach(ctx->peers[q].ctrlr);
			ctx->peers[q].ctrlr = NULL;
		}
	}
	memset(ctx, 0, sizeof(*ctx));
	return rc;
}

int
iuventus_enable(struct iuventus_ctx *ctx)
{
	uint8_t *cqbuf;
	uint32_t free_pages;
	size_t i;

	if (ctx == NULL || ctx->nfb_comp == NULL || ctx->cq.vaddr == NULL) {
		SPDK_ERRLOG("environment is not initialised\n");
		return -EINVAL;
	}

	/* Bounded by the BAR size rather than by a page count so that queues this run did not
	 * create are cleared too; those are exactly the ones that carry stale entries. */
	cqbuf = ctx->cq.vaddr;
	for (i = 0; i < ctx->cq.size; i++) {
		cqbuf[i] = 0;
	}

	/* MANAGED_PAGES is duplicated from the card's iuventus_sizing_pkg, and nothing links the
	 * two: if the card manages more pages than this list describes, it indexes past the
	 * allocation and the drive fetches whatever follows in the hugepage -- silently, as wrong
	 * data pointers rather than an error. Idle free pages is the card's own count, so a list
	 * that cannot cover it is caught here instead of as corrupt traffic later. */
	free_pages = nfb_comp_read32(ctx->nfb_comp, REG_RD_PAGES_FREE) & 0xFFFF;
	if (free_pages > MANAGED_PAGES) {
		SPDK_ERRLOG("card manages at least %u pages but this build's PRP list describes only "
			    "%u; rebuild with MANAGED_PAGES matching iuventus_sizing_pkg.vhd\n",
			    free_pages, MANAGED_PAGES);
		return -EINVAL;
	}

	nfb_comp_write16(ctx->nfb_comp, REG_CONTROL,
			 (ctx->rpt_upd_en ? CTRL_RPT_UPD_EN : 0) | CTRL_ENABLE);
	ctx->enabled = true;
	usleep(1);

	return 0;
}

void
iuventus_disable(struct iuventus_ctx *ctx)
{
	if (ctx == NULL || ctx->nfb_comp == NULL) {
		return;
	}

	nfb_comp_write16(ctx->nfb_comp, REG_CONTROL, 0);
	ctx->enabled = false;
}

int
iuventus_drain(struct iuventus_ctx *ctx, uint32_t timeout_ms)
{
	uint32_t waited;

	if (ctx == NULL || ctx->nfb_comp == NULL) {
		return -EINVAL;
	}

	for (waited = 0; waited < timeout_ms; waited++) {
		if ((nfb_comp_read8(ctx->nfb_comp, REG_STATUS) & STAT_READY) == 0) {
			return 0;
		}
		usleep(1000);
	}

	if ((nfb_comp_read8(ctx->nfb_comp, REG_STATUS) & STAT_READY) != 0) {
		SPDK_WARNLOG("design did not drain within %u ms\n", timeout_ms);
		return -ETIMEDOUT;
	}

	return 0;
}

void
iuventus_fini(struct iuventus_ctx *ctx)
{
	uint32_t q;

	if (ctx == NULL) {
		return;
	}

	/* The drain must finish before the NVMe queues go away. Unconditional rather than gated on
	 * ctx->enabled: a design an earlier run left going is the case that most needs stopping,
	 * and it is the one a caller cannot know about. Repeating a stop that already happened
	 * costs one register write and one status read. */
	if (ctx->nfb_comp != NULL) {
		iuventus_disable(ctx);
		iuventus_drain(ctx, IUVENTUS_DRAIN_TIMEOUT_MS);
	}

	dma_ctrl_deinit(ctx);
	queues_dealloc(ctx);

	if (ctx->pci_dev != NULL) {
		spdk_pci_device_detach(ctx->pci_dev);
		ctx->pci_dev = NULL;
	}

	dma_dev_fini(ctx);

	for (q = 0; q < ctx->num_peers; q++) {
		if (ctx->peers[q].ctrlr != NULL) {
			spdk_nvme_detach(ctx->peers[q].ctrlr);
			ctx->peers[q].ctrlr = NULL;
		}
	}

	memset(ctx, 0, sizeof(*ctx));
}

uint32_t
iuventus_num_peers(const struct iuventus_ctx *ctx)
{
	return ctx != NULL ? ctx->num_peers : 0;
}

const struct iuventus_peer *
iuventus_peer(const struct iuventus_ctx *ctx, uint32_t i)
{
	if (ctx == NULL || i >= ctx->num_peers) {
		return NULL;
	}

	return &ctx->peers[i];
}
