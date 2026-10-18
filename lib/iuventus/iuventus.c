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
/* EP1 block: mirrors the COMMON block above, one-for-one, for the second bifurcated endpoint's
 * data pool. Reached through the same nfb component as everything else -- MI registers are
 * EP0-only, even though EP1's BARs and PCIe device are not. Absent on single-endpoint firmware. */
#define REG_EP1_RDBUFF_BADDR        0x180
#define REG_EP1_RDBUFF_PRP_LIST_PTR 0x188
#define REG_EP1_WRBUFF_BADDR        0x190
#define REG_EP1_WRBUFF_PRP_LIST_PTR 0x198
#define REG_EP1_RD_PAGES_FREE       0x1A0
#define REG_EP1_WR_PAGES_FREE       0x1A4
/* The queues live in their own BARs rather than at the head of the data pool, so page 0 is data
 * page 0. The card does hold back one page per PRP list page group, but those sit at the top of
 * each group, not at index 0, and the list must still carry an entry for every index below them. */
#define FIRST_DATA_PAGE         0
/* Must match the page count the card is built with. The card indexes its PRP list as
 * base + page_idx*8 over this whole range, so the list has to be this many entries or it
 * reads past the allocation -- silently, since the drive just gets whatever follows in the
 * hugepage. */
#define MANAGED_PAGES           2048
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
/* True when a caller built against an older, shorter iuventus_opts still carries field. */
#define OPTS_HAS(opts, field) \
	(offsetof(struct iuventus_opts, field) + sizeof((opts)->field) <= (opts)->opts_size)
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

/* Device ids of the peer-BAR function on each endpoint, used to tell EP0 and EP1 apart on attach
 * rather than trusting the order they were found in. */
#define NCD_EP0_DEVICE_ID 0xc020
#define NCD_EP1_DEVICE_ID 0xc022

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

/* Separate driver, id-scoped to EP1's peer-BAR device id alone: spdk_pci_enumerate() attaches
 * whatever its driver's table matches, so a table shared with "ncd" would risk grabbing an
 * unrelated card's EP0 during the EP1 PCI scan in dma_dev_ep1_locate(). */
static struct spdk_pci_id ncd_ep1_pci_driver_id[] = {
	{
		SPDK_PCI_DEVICE(0x18ec, 0xc022)
	},
	{
		SPDK_PCI_DEVICE(0x1c2c, 0xc022)
	},
};

SPDK_PCI_DRIVER_REGISTER(ncd_ep1, ncd_ep1_pci_driver_id, SPDK_PCI_DRIVER_NEED_MAPPING)

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

static bool
probe_cb(void *cb_ctx, const struct spdk_nvme_transport_id *trid,
	 struct spdk_nvme_ctrlr_opts *opts)
{
	struct iuventus_ctx *ctx = cb_ctx;
	int idx = peer_by_traddr(ctx, trid->traddr);

	if (idx < 0 || ctx->peers[idx].caller_owned) {
		return false;
	}

	SPDK_NOTICELOG("Probing %s ...\n", trid->traddr);

	if (ctx->qsize != 0) {
		opts->io_queue_size = ctx->qsize;
		opts->io_queue_requests = ctx->qsize;
	} else {
		opts->io_queue_requests = opts->io_queue_size;
	}
	opts->arb_mechanism = SPDK_NVME_CC_AMS_RR;
	opts->enable_interrupts = false;

	return true;
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

/* Detaches every controller the library attached itself; the caller detaches its own. */
static void
peers_detach(struct iuventus_ctx *ctx)
{
	uint32_t q;

	for (q = 0; q < ctx->num_peers; q++) {
		if (ctx->peers[q].ctrlr != NULL && !ctx->peers[q].caller_owned) {
			spdk_nvme_detach(ctx->peers[q].ctrlr);
		}
		ctx->peers[q].ctrlr = NULL;
	}
}

static void
queues_dealloc(struct iuventus_ctx *ctx)
{
	uint32_t q;

	for (q = 0; q < ctx->num_peers; q++) {
		struct iuventus_peer *peer = &ctx->peers[q];

		if (peer->qpair != NULL) {
			spdk_nvme_ctrlr_free_io_qpair(peer->qpair);
			peer->qpair = NULL;
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
	qopts.peer_owned_cq = true;

	for (q = 0; q < ctx->num_peers; q++) {
		struct iuventus_peer *peer = &ctx->peers[q];
		const struct iuventus_ep *ep = &ctx->ep[iuventus_queue_ep(ctx, q)];
		void *sq_vaddr = (uint8_t *)ep->sq.vaddr + (uint64_t)q * VALUE_4KB;
		uint64_t sq_paddr = ep->sq.paddr + (uint64_t)q * VALUE_4KB;
		void *cq_vaddr = (uint8_t *)ep->cq.vaddr + (uint64_t)q * VALUE_4KB;
		uint64_t cq_paddr = ep->cq.paddr + (uint64_t)q * VALUE_4KB;
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

		peer->qpair = spdk_nvme_ctrlr_alloc_io_qpair(peer->ctrlr, &qopts, sizeof(qopts));
		if (peer->qpair == NULL) {
			SPDK_ERRLOG("Failed to create the queue pair of card queue %u\n", q);
			rc = -EIO;
			goto fail;
		}
		qid = spdk_nvme_qpair_get_id(peer->qpair);
		peer->hw_qid = qid;
		peer->sqtdbl_paddr = peer->doorbell_base +
				     (2 * qid) * (4 << peer->doorbell_stride);
		peer->cqhdbl_paddr = peer->doorbell_base +
				     (2 * qid + 1) * (4 << peer->doorbell_stride);
		SPDK_NOTICELOG("Card queue %u -> %s qid %d, flat page %u\n", q,
			       peer->trid.traddr, qid, q);
		SPDK_NOTICELOG("\tSQTDBL physical address: 0x%lx\n", peer->sqtdbl_paddr);
		SPDK_NOTICELOG("\tCQHDBL physical address: 0x%lx\n", peer->cqhdbl_paddr);
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

/* PCIe slot of the card's second endpoint, as named by the device tree. The nfb driver only
 * fills "pci-slot" into "/system/device/endpointN" for a function it probes, and EP1's PF0 is
 * deliberately device id 0xc021 -- a dummy the nfb driver never touches -- so this misses on
 * every firmware built so far. Kept for forward compatibility only; dma_dev_ep1_locate() falls
 * back to a PCI scan when it does. */
static bool
dma_dev_ep1_fdt_bdf(struct iuventus_ctx *ctx, const char **bdf)
{
	const void *fdt;
	int fdt_offset;
	int len;

	fdt = nfb_get_fdt(ctx->nfb_dev);
	fdt_offset = fdt_path_offset(fdt, "/system/device/endpoint1");
	*bdf = fdt_getprop(fdt, fdt_offset, "pci-slot", &len);

	return len >= 0 && *bdf != NULL;
}

/* Collects every PCIe address spdk_pci_enumerate() offers while scanning for EP1's peer-BAR
 * device id -- never attaches on its own. Only the count and the first couple of addresses are
 * kept, just enough to fail loudly and name the offending BDFs if more than one card is present. */
#define EP1_SCAN_LOG_MAX 2
struct ep1_scan_ctx {
	struct spdk_pci_addr	addr[EP1_SCAN_LOG_MAX];
	uint32_t		count;
};

static int
ep1_scan_cb(void *cb_ctx, struct spdk_pci_device *pci_dev)
{
	struct ep1_scan_ctx *sctx = cb_ctx;

	if (sctx->count < EP1_SCAN_LOG_MAX) {
		sctx->addr[sctx->count] = spdk_pci_device_get_addr(pci_dev);
	}
	sctx->count++;

	/* Report only -- the caller attaches by address afterwards, once it knows there is
	 * exactly one match. */
	return 1;
}

/*
 * Locates the card's second endpoint's peer-BAR function, without attaching it.
 *
 * Tries the device tree first (see dma_dev_ep1_fdt_bdf()); on a miss, falls back to a PCI scan
 * for NCD_EP1_DEVICE_ID. Zero matches means single-endpoint firmware, which is not an error.
 * More than one match is refused rather than guessed at: a second bifurcated card would need
 * serial matching, which this library does not support.
 *
 * \param ep1_driver Driver whose id table is scoped to NCD_EP1_DEVICE_ID alone.
 * \param addr Filled in with EP1's peer-BAR address when *found comes back true.
 * \param found Set to whether an endpoint 1 was located.
 * \return 0 on success (including the "not found" case), negative errno on a real error.
 */
/* The bridge above a function, as sysfs names it ("0000:00:01.2"); false when sysfs has no
 * such device. The two endpoints of one bifurcated slot hang off root ports that differ only
 * in function, which is what tells one card's endpoint 1 from another's. */
static bool
pci_parent_bdf(const struct spdk_pci_addr *a, char *out, size_t len)
{
	char path[128];
	char real[PATH_MAX];
	const char *base;

	snprintf(path, sizeof(path), "/sys/bus/pci/devices/%04x:%02x:%02x.%x/..", a->domain, a->bus,
		 a->dev, a->func);
	if (realpath(path, real) == NULL) {
		return false;
	}
	base = strrchr(real, '/');
	if (base == NULL) {
		return false;
	}
	snprintf(out, len, "%s", base + 1);
	return true;
}

static int
dma_dev_ep1_locate(struct iuventus_ctx *ctx, struct spdk_pci_driver *ep1_driver,
		   struct spdk_pci_addr *addr, bool *found)
{
	const char *bdf;
	struct ep1_scan_ctx sctx = {0};
	int rc;

	*found = false;

	if (dma_dev_ep1_fdt_bdf(ctx, &bdf)) {
		rc = spdk_pci_addr_parse(addr, bdf);
		if (rc) {
			SPDK_ERRLOG("endpoint 1: device tree pci-slot '%s' does not parse\n", bdf);
			return -EINVAL;
		}
		/* Mirrors endpoint 0: the device tree names the management function, the peer BAR
		 * is the next one over. */
		addr->func += 1;
		SPDK_NOTICELOG("endpoint 1: found via device tree at %04x:%02x:%02x.%x\n",
			       addr->domain, addr->bus, addr->dev, addr->func);
		*found = true;
		return 0;
	}

	rc = spdk_pci_enumerate(ep1_driver, ep1_scan_cb, &sctx);
	if (rc != 0) {
		SPDK_ERRLOG("endpoint 1: PCI scan for device id 0x%04x failed\n", NCD_EP1_DEVICE_ID);
		return -ENODEV;
	}

	if (sctx.count == 0) {
		return 0;
	}

	if (sctx.count > 1) {
		/* Several bifurcated cards: keep the candidate whose root port is the sibling of
		 * endpoint 0's (same domain, bus and device, other function). *addr still holds
		 * endpoint 0's peer-BAR address on entry. */
		char ep0_parent[32];
		char parent[32];
		uint32_t i;
		uint32_t hits = 0;
		uint32_t hit = 0;

		if (sctx.count > EP1_SCAN_LOG_MAX || !pci_parent_bdf(addr, ep0_parent, sizeof(ep0_parent))) {
			SPDK_ERRLOG("endpoint 1: %u devices with id 0x%04x found and endpoint 0's root "
				    "port is unknown -- cannot tell the cards apart\n",
				    sctx.count, NCD_EP1_DEVICE_ID);
			return -ENODEV;
		}
		for (i = 0; i < sctx.count; i++) {
			if (pci_parent_bdf(&sctx.addr[i], parent, sizeof(parent)) &&
			    strncmp(parent, ep0_parent, 10) == 0 && strcmp(parent, ep0_parent) != 0) {
				hits++;
				hit = i;
			}
		}
		if (hits != 1) {
			SPDK_ERRLOG("endpoint 1: %u devices with id 0x%04x found, %u of them beside "
				    "endpoint 0's root port %s -- cannot tell the cards apart\n",
				    sctx.count, NCD_EP1_DEVICE_ID, hits, ep0_parent);
			return -ENODEV;
		}
		*addr = sctx.addr[hit];
		SPDK_NOTICELOG("endpoint 1: %u candidates, took %04x:%02x:%02x.%x beside endpoint 0's "
			       "root port %s\n", sctx.count, addr->domain, addr->bus, addr->dev,
			       addr->func, ep0_parent);
		*found = true;
		return 0;
	}

	*addr = sctx.addr[0];
	SPDK_NOTICELOG("endpoint 1: found via PCI scan at %04x:%02x:%02x.%x\n", addr->domain,
		       addr->bus, addr->dev, addr->func);
	*found = true;
	return 0;
}

static void
prp_list_free(struct iuventus_ep *ep)
{
	if (!ep->prp_lists_allocated) {
		return;
	}

	spdk_dma_free(ep->wrbuff_prp_list.vaddr);
	spdk_dma_free(ep->rdbuff_prp_list.vaddr);
	memset(&ep->wrbuff_prp_list, 0, sizeof(ep->wrbuff_prp_list));
	memset(&ep->rdbuff_prp_list, 0, sizeof(ep->rdbuff_prp_list));
	ep->prp_lists_allocated = false;
}

static int
prp_list_alloc(struct iuventus_ep *ep)
{
	uint64_t size = PRP_LIST_BYTES;
	int i;
	int rc;

	ep->wrbuff_prp_list.vaddr = spdk_dma_zmalloc(PRP_LIST_BYTES, VALUE_4KB, NULL);
	if (ep->wrbuff_prp_list.vaddr == NULL) {
		SPDK_ERRLOG("Write PRP list allocation failed\n");
		return -ENOMEM;
	}

	ep->rdbuff_prp_list.vaddr = spdk_dma_zmalloc(PRP_LIST_BYTES, VALUE_4KB, NULL);
	if (ep->rdbuff_prp_list.vaddr == NULL) {
		SPDK_ERRLOG("Read PRP list allocation failed\n");
		rc = -ENOMEM;
		goto fail;
	}

	ep->wrbuff_prp_list.paddr = spdk_vtophys(ep->wrbuff_prp_list.vaddr, &size);
	if (ep->wrbuff_prp_list.paddr == SPDK_VTOPHYS_ERROR) {
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

	ep->rdbuff_prp_list.paddr = spdk_vtophys(ep->rdbuff_prp_list.vaddr, &size);
	if (ep->rdbuff_prp_list.paddr == SPDK_VTOPHYS_ERROR) {
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
	if (ep->wrbuff.size < (uint64_t)MANAGED_PAGES * VALUE_4KB ||
	    ep->rdbuff.size < (uint64_t)MANAGED_PAGES * VALUE_4KB) {
		SPDK_ERRLOG("data BARs too small for %d managed pages (wrbuff %llu B, "
			    "rdbuff %llu B, need %llu B each)\n",
			    MANAGED_PAGES, (unsigned long long)ep->wrbuff.size,
			    (unsigned long long)ep->rdbuff.size,
			    (unsigned long long)MANAGED_PAGES * VALUE_4KB);
		rc = -ENOSPC;
		goto fail;
	}

	for (i = FIRST_DATA_PAGE; i < MANAGED_PAGES; i++) {
		((uint64_t *)ep->wrbuff_prp_list.vaddr)[i - FIRST_DATA_PAGE] =
			ep->wrbuff.paddr + ((uint64_t)i * VALUE_4KB);
		((uint64_t *)ep->rdbuff_prp_list.vaddr)[i - FIRST_DATA_PAGE] =
			ep->rdbuff.paddr + ((uint64_t)i * VALUE_4KB);
	}
	ep->wrbuff_prp_list.entries = MANAGED_PAGES - FIRST_DATA_PAGE;
	ep->rdbuff_prp_list.entries = MANAGED_PAGES - FIRST_DATA_PAGE;
	ep->prp_lists_allocated = true;

	return 0;

fail:
	spdk_dma_free(ep->wrbuff_prp_list.vaddr);
	spdk_dma_free(ep->rdbuff_prp_list.vaddr);
	memset(&ep->wrbuff_prp_list, 0, sizeof(ep->wrbuff_prp_list));
	memset(&ep->rdbuff_prp_list, 0, sizeof(ep->rdbuff_prp_list));
	return rc;
}

static void
dma_ctrl_deinit(struct iuventus_ctx *ctx)
{
	uint32_t e;

	for (e = 0; e < ctx->num_eps; e++) {
		prp_list_free(&ctx->ep[e]);
	}
	if (ctx->nfb_comp != NULL) {
		nfb_comp_close(ctx->nfb_comp);
		ctx->nfb_comp = NULL;
	}
}

static int
dma_ctrl_init(struct iuventus_ctx *ctx)
{
	uint32_t q;
	uint32_t e;
	int node;
	int rc;

	node = nfb_comp_find(ctx->nfb_dev, "ziti,dma_iuventus", 0);
	ctx->nfb_comp = nfb_comp_open(ctx->nfb_dev, node);
	if (ctx->nfb_comp == NULL) {
		SPDK_ERRLOG("Failed to open the control registers as an nfb component!\n");
		return -ENODEV;
	}

	for (e = 0; e < ctx->num_eps; e++) {
		rc = prp_list_alloc(&ctx->ep[e]);
		if (rc) {
			SPDK_ERRLOG("PRP list allocation failed for endpoint %u\n", e);
			while (e-- > 0) {
				prp_list_free(&ctx->ep[e]);
			}
			nfb_comp_close(ctx->nfb_comp);
			ctx->nfb_comp = NULL;
			return rc;
		}
	}

	/* COMMON: EP0's data pool and its metadata. */
	nfb_comp_write64(ctx->nfb_comp, REG_RDBUFF_BADDR, ctx->ep[0].rdbuff.paddr);
	nfb_comp_write64(ctx->nfb_comp, REG_RDBUFF_PRP_LIST_PTR, ctx->ep[0].rdbuff_prp_list.paddr);
	nfb_comp_write64(ctx->nfb_comp, REG_WRBUFF_BADDR, ctx->ep[0].wrbuff.paddr);
	nfb_comp_write64(ctx->nfb_comp, REG_WRBUFF_PRP_LIST_PTR, ctx->ep[0].wrbuff_prp_list.paddr);
	nfb_comp_write64(ctx->nfb_comp, REG_META_PTR, 0);

	/* EP1: same fields as COMMON above, one endpoint over, present only on bifurcated firmware. */
	if (ctx->num_eps > 1) {
		nfb_comp_write64(ctx->nfb_comp, REG_EP1_RDBUFF_BADDR, ctx->ep[1].rdbuff.paddr);
		nfb_comp_write64(ctx->nfb_comp, REG_EP1_RDBUFF_PRP_LIST_PTR,
				 ctx->ep[1].rdbuff_prp_list.paddr);
		nfb_comp_write64(ctx->nfb_comp, REG_EP1_WRBUFF_BADDR, ctx->ep[1].wrbuff.paddr);
		nfb_comp_write64(ctx->nfb_comp, REG_EP1_WRBUFF_PRP_LIST_PTR,
				 ctx->ep[1].wrbuff_prp_list.paddr);
	}

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

/* Passed as the enum_ctx of spdk_pci_device_attach() for one endpoint's peer-BAR function: which
 * ctx->ep[] slot to fill in, and the device id that slot is expected to attach as -- checked
 * rather than assumed, so a BDF mix-up fails loudly instead of silently swapping the endpoints. */
struct ep_attach_ctx {
	struct iuventus_ctx	*ctx;
	uint32_t		ep_idx;
	uint16_t		expect_device_id;
};

static int
ncd_drv_attach_cb(void *cb_ctx, struct spdk_pci_device *pci_dev)
{
	struct ep_attach_ctx *actx = cb_ctx;
	struct iuventus_ep *ep = &actx->ctx->ep[actx->ep_idx];
	struct spdk_pci_id id;
	uint16_t cmd_reg;
	int rc;

	id = spdk_pci_device_get_id(pci_dev);
	if (id.device_id != actx->expect_device_id) {
		SPDK_ERRLOG("endpoint %u: attached device id 0x%04x, expected 0x%04x -- refusing to "
			    "guess which endpoint this is\n", actx->ep_idx, id.device_id,
			    actx->expect_device_id);
		return -1;
	}

	ep->pci_dev = pci_dev;

	/* Enable memory accesses and PCI bus mastering, and disable INTx. The device itself is
	 * already enabled by the uio bind, so only the command register needs touching. */
	spdk_pci_device_cfg_read16(pci_dev, &cmd_reg, 4);
	cmd_reg |= 0x404;
	spdk_pci_device_cfg_write16(pci_dev, cmd_reg, 4);

	rc = spdk_pci_device_map_bar(pci_dev, SQ_BAR, &ep->sq.vaddr, &ep->sq.paddr, &ep->sq.size);
	if (rc) {
		SPDK_ERRLOG("endpoint %u: unable to map BAR %d (SQ)\n", actx->ep_idx, SQ_BAR);
		return rc;
	}

	rc = spdk_pci_device_map_bar(pci_dev, CQ_BAR, &ep->cq.vaddr, &ep->cq.paddr, &ep->cq.size);
	if (rc) {
		SPDK_ERRLOG("endpoint %u: unable to map BAR %d (CQ)\n", actx->ep_idx, CQ_BAR);
		return rc;
	}

	/* Four separate regions, one per logical BAR: the queues in their own small BARs and the
	 * data pools in their own large 64-bit BARs. They are not aliases of the queue BARs, and
	 * mapping them separately is what makes the card's page index and this host's addresses
	 * agree. */
	rc = spdk_pci_device_map_bar(pci_dev, WRBUFF_BAR, &ep->wrbuff.vaddr, &ep->wrbuff.paddr,
				     &ep->wrbuff.size);
	if (rc) {
		SPDK_ERRLOG("endpoint %u: unable to map BAR %d (WRBUFF)\n", actx->ep_idx, WRBUFF_BAR);
		return rc;
	}

	rc = spdk_pci_device_map_bar(pci_dev, RDBUFF_BAR, &ep->rdbuff.vaddr, &ep->rdbuff.paddr,
				     &ep->rdbuff.size);
	if (rc) {
		SPDK_ERRLOG("endpoint %u: unable to map BAR %d (RDBUFF)\n", actx->ep_idx, RDBUFF_BAR);
		return rc;
	}

	if (ep->sq.vaddr == NULL || ep->cq.vaddr == NULL ||
	    ep->wrbuff.vaddr == NULL || ep->rdbuff.vaddr == NULL) {
		SPDK_ERRLOG("endpoint %u: virtual BAR addresses invalid!\n", actx->ep_idx);
		return -EFAULT;
	}
	if (ep->sq.paddr == 0 || ep->cq.paddr == 0 ||
	    ep->wrbuff.paddr == 0 || ep->rdbuff.paddr == 0) {
		SPDK_ERRLOG("endpoint %u: physical BAR addresses invalid!\n", actx->ep_idx);
		return -EFAULT;
	}
	if (ep->sq.size == 0 || ep->cq.size == 0 ||
	    ep->wrbuff.size == 0 || ep->rdbuff.size == 0) {
		SPDK_ERRLOG("endpoint %u: BAR sizes invalid!\n", actx->ep_idx);
		return -EFAULT;
	}

	return 0;
}

static void
bars_print(const struct iuventus_ctx *ctx)
{
	uint32_t e;

	SPDK_NOTICELOG("PCIe domain initialization complete, %u endpoint(s)\n", ctx->num_eps);
	for (e = 0; e < ctx->num_eps; e++) {
		const struct iuventus_ep *ep = &ctx->ep[e];
		struct spdk_pci_addr addr = spdk_pci_device_get_addr(ep->pci_dev);
		struct spdk_pci_id id = spdk_pci_device_get_id(ep->pci_dev);

		SPDK_NOTICELOG("Endpoint %u: %04x:%02x:%02x.%x, device id 0x%04x\n", e, addr.domain,
			       addr.bus, addr.dev, addr.func, id.device_id);
		SPDK_NOTICELOG("\tSQ     VADDR %p PADDR 0x%lx size %lu B\n",
			       ep->sq.vaddr, ep->sq.paddr, ep->sq.size);
		SPDK_NOTICELOG("\tCQ     VADDR %p PADDR 0x%lx size %lu B\n",
			       ep->cq.vaddr, ep->cq.paddr, ep->cq.size);
		SPDK_NOTICELOG("\tWRBUFF VADDR %p PADDR 0x%lx size %lu B\n",
			       ep->wrbuff.vaddr, ep->wrbuff.paddr, ep->wrbuff.size);
		SPDK_NOTICELOG("\tRDBUFF VADDR %p PADDR 0x%lx size %lu B\n",
			       ep->rdbuff.vaddr, ep->rdbuff.paddr, ep->rdbuff.size);
	}
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
	struct spdk_pci_driver *ncd_ep1_driver;
	struct spdk_pci_addr pcie_addr;
	struct ep_attach_ctx ep_actx;
	bool ep1_found;
	uint32_t attached_eps = 0;
	uint32_t to_probe = 0;
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
		if (OPTS_HAS(opts, peer_ctrlr) && opts->peer_ctrlr[q] != NULL) {
			ctx->peers[q].caller_owned = true;
		} else {
			to_probe++;
		}
	}

	SPDK_NOTICELOG("Initializing %u NVMe controller(s):\n", ctx->num_peers);
	for (q = 0; q < ctx->num_peers; q++) {
		SPDK_NOTICELOG("\tcard queue %u -> %s%s\n", q, ctx->peers[q].trid.traddr,
			       ctx->peers[q].caller_owned ? " (attached by the caller)" : "");
	}

	for (q = 0; q < ctx->num_peers; q++) {
		struct spdk_nvme_ctrlr *ctrlr;

		if (!ctx->peers[q].caller_owned) {
			continue;
		}
		ctrlr = opts->peer_ctrlr[q];
		if (spdk_nvme_transport_id_compare(spdk_nvme_ctrlr_get_transport_id(ctrlr),
						   &ctx->peers[q].trid) != 0) {
			SPDK_ERRLOG("the controller given for card queue %u is not %s\n", q,
				    ctx->peers[q].trid.traddr);
			rc = -EINVAL;
			goto probe_fail;
		}
		attach_cb(ctx, &ctx->peers[q].trid, ctrlr, spdk_nvme_ctrlr_get_opts(ctrlr));
	}

	if (to_probe > 0) {
		rc = spdk_nvme_probe(NULL, ctx, probe_cb, attach_cb, NULL);
		if (rc != 0) {
			SPDK_ERRLOG("spdk_nvme_probe() failed\n");
			rc = -ENODEV;
			goto probe_fail;
		}
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

	ncd_ep1_driver = spdk_pci_get_driver("ncd_ep1");
	if (ncd_ep1_driver == NULL) {
		SPDK_ERRLOG("Unable to get the NCD EP1 driver!\n");
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

	ep_actx.ctx = ctx;
	ep_actx.ep_idx = 0;
	ep_actx.expect_device_id = NCD_EP0_DEVICE_ID;
	rc = spdk_pci_device_attach(ncd_driver, ncd_drv_attach_cb, &ep_actx, &pcie_addr);
	if (rc) {
		SPDK_ERRLOG("Unable to attach endpoint 0's PCIe device!\n");
		rc = -ENODEV;
		goto pci_attach_fail;
	}
	attached_eps = 1;

	/* A second endpoint is optional: its absence is not an error -- every queue simply stays on
	 * endpoint 0. See dma_dev_ep1_locate() for how it is found. */
	rc = dma_dev_ep1_locate(ctx, ncd_ep1_driver, &pcie_addr, &ep1_found);
	if (rc) {
		rc = -ENODEV;
		goto eps_attach_fail;
	}
	if (ep1_found) {
		ep_actx.ctx = ctx;
		ep_actx.ep_idx = 1;
		ep_actx.expect_device_id = NCD_EP1_DEVICE_ID;
		rc = spdk_pci_device_attach(ncd_ep1_driver, ncd_drv_attach_cb, &ep_actx, &pcie_addr);
		if (rc) {
			SPDK_ERRLOG("Unable to attach endpoint 1's PCIe device!\n");
			rc = -ENODEV;
			goto eps_attach_fail;
		}
		attached_eps = 2;
	}
	ctx->num_eps = attached_eps;

	/* The card splits its IUVENTUS_HW_QUEUES queues into contiguous halves per endpoint, so a
	 * peer count is only bounded by the hardware queue count, not by the endpoint count. */
	if (ctx->num_peers > IUVENTUS_HW_QUEUES) {
		SPDK_ERRLOG("%u card queue(s) requested but the design has %u\n",
			    ctx->num_peers, IUVENTUS_HW_QUEUES);
		rc = -EINVAL;
		goto eps_attach_fail;
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
eps_attach_fail:
	while (attached_eps-- > 0) {
		if (ctx->ep[attached_eps].pci_dev != NULL) {
			spdk_pci_device_detach(ctx->ep[attached_eps].pci_dev);
			ctx->ep[attached_eps].pci_dev = NULL;
		}
	}
pci_attach_fail:
	dma_dev_fini(ctx);
probe_fail:
	peers_detach(ctx);
	memset(ctx, 0, sizeof(*ctx));
	return rc;
}

/* Bounded by the BAR size rather than by a page count so that queues this run did not create
 * are cleared too; those are exactly the ones that carry stale entries. */
static void
cq_rings_clear(struct iuventus_ctx *ctx)
{
	uint8_t *cqbuf;
	uint32_t e;
	size_t i;

	for (e = 0; e < ctx->num_eps; e++) {
		cqbuf = ctx->ep[e].cq.vaddr;
		for (i = 0; i < ctx->ep[e].cq.size; i++) {
			cqbuf[i] = 0;
		}
	}
}

int
iuventus_enable(struct iuventus_ctx *ctx)
{
	/* Per-endpoint free-page register, indexed by ctx->ep[]: EP0 reuses the register the
	 * single-endpoint card always had, EP1 is the mirrored block at REG_EP1_RD_PAGES_FREE. */
	static const uint32_t rd_pages_free_reg[IUVENTUS_MAX_EPS] = {
		REG_RD_PAGES_FREE, REG_EP1_RD_PAGES_FREE,
	};
	uint32_t free_pages;
	uint32_t e;

	if (ctx == NULL || ctx->nfb_comp == NULL || ctx->num_eps == 0 ||
	    ctx->ep[0].cq.vaddr == NULL) {
		SPDK_ERRLOG("environment is not initialised\n");
		return -EINVAL;
	}

	cq_rings_clear(ctx);
	for (e = 0; e < ctx->num_eps; e++) {
		/* MANAGED_PAGES is duplicated from the card's iuventus_sizing_pkg, and nothing
		 * links the two: if the card manages more pages than this list describes, it
		 * indexes past the allocation and the drive fetches whatever follows in the
		 * hugepage -- silently, as wrong data pointers rather than an error. Idle free
		 * pages is the card's own count, so a list that cannot cover it is caught here
		 * instead of as corrupt traffic later. */
		free_pages = nfb_comp_read32(ctx->nfb_comp, rd_pages_free_reg[e]) & 0xFFFF;
		if (free_pages > MANAGED_PAGES) {
			SPDK_ERRLOG("endpoint %u manages at least %u pages but this build's PRP "
				    "list describes only %u; rebuild with MANAGED_PAGES matching "
				    "iuventus_sizing_pkg.vhd\n", e, free_pages, MANAGED_PAGES);
			return -EINVAL;
		}
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

static uint64_t
ticks_to_us(uint64_t ticks)
{
	return ticks * 1000000 / spdk_get_ticks_hz();
}

/* Section 3.7.3: CC.EN 1->0, then CSTS.RDY 0 within CAP.TO, which deletes every I/O queue. */
static int
peer_reset_disable(struct iuventus_peer *peer)
{
	union spdk_nvme_cap_register cap = spdk_nvme_ctrlr_get_regs_cap(peer->ctrlr);
	uint64_t bound = (cap.bits.to * 500 + 1000) * spdk_get_ticks_hz() / 1000;
	uint64_t start = spdk_get_ticks();
	int rc;

	rc = spdk_nvme_ctrlr_disconnect(peer->ctrlr);
	if (rc != 0) {
		return rc;
	}
	while (spdk_nvme_ctrlr_process_admin_completions(peer->ctrlr) != -ENXIO) {
		if (spdk_get_ticks() - start > bound) {
			return -ETIMEDOUT;
		}
	}
	peer->reset_disable_us = ticks_to_us(spdk_get_ticks() - start);
	return 0;
}

/* Section 3.5.1 steps 5-11: CC.EN 1, CSTS.RDY 1, admin set-up, then each I/O CQ before its SQ. SPDK
 * bounds every step by CAP.TO itself and re-creates the card's queue pair with the rest. */
static int
peer_reset_enable(struct iuventus_peer *peer)
{
	uint64_t start = spdk_get_ticks();
	int rc;

	spdk_nvme_ctrlr_reconnect_async(peer->ctrlr);
	do {
		rc = spdk_nvme_ctrlr_reconnect_poll_async(peer->ctrlr);
	} while (rc == -EAGAIN);
	if (rc != 0) {
		return rc;
	}
	peer->reset_enable_us = ticks_to_us(spdk_get_ticks() - start);

	/* The card rings the doorbells of this queue identifier, so it must survive the reset. */
	if (spdk_nvme_qpair_get_id(peer->qpair) != peer->hw_qid) {
		SPDK_ERRLOG("%s came back with qid %u instead of %d\n", peer->trid.traddr,
			    spdk_nvme_qpair_get_id(peer->qpair), peer->hw_qid);
		return -EIO;
	}
	return 0;
}

int
iuventus_reset_peers(struct iuventus_ctx *ctx)
{
	bool was_enabled;
	uint32_t q;
	int rc;

	if (ctx == NULL || ctx->nfb_comp == NULL) {
		return -EINVAL;
	}

	/* The card restarts all of its queues at once, and the queues of a caller's drive are not
	 * re-created here, so that drive would come back out of step with the card. */
	for (q = 0; q < ctx->num_peers; q++) {
		if (ctx->peers[q].caller_owned) {
			SPDK_ERRLOG("%s belongs to the caller, which must reset it\n",
				    ctx->peers[q].trid.traddr);
			return -EPERM;
		}
	}

	/* Section 3.7.2: no command may still be outstanding when CC.EN drops. */
	was_enabled = ctx->enabled;
	iuventus_disable(ctx);
	rc = iuventus_drain(ctx, IUVENTUS_DRAIN_TIMEOUT_MS);
	if (rc != 0) {
		return rc;
	}

	for (q = 0; q < ctx->num_peers; q++) {
		struct iuventus_peer *peer = &ctx->peers[q];

		rc = peer_reset_disable(peer);
		if (rc == 0) {
			/* peer_owned_cq leaves clearing the ring before its re-creation to us. */
			cq_rings_clear(ctx);
			rc = peer_reset_enable(peer);
		}
		if (rc != 0) {
			SPDK_ERRLOG("controller reset of %s failed: %d\n", peer->trid.traddr, rc);
			return rc;
		}
		SPDK_NOTICELOG("%s reset: CSTS.RDY 0 after %" PRIu64 " us, queues back after %"
			       PRIu64 " us\n", peer->trid.traddr, peer->reset_disable_us,
			       peer->reset_enable_us);
	}

	/* The START of a re-enable is what rewinds every card queue to SQ tail 0, CQ head 0 and
	 * phase 1, which is where the re-created queues begin. */
	return was_enabled ? iuventus_enable(ctx) : 0;
}

void
iuventus_fini(struct iuventus_ctx *ctx)
{
	uint32_t e;

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

	for (e = 0; e < ctx->num_eps; e++) {
		if (ctx->ep[e].pci_dev != NULL) {
			spdk_pci_device_detach(ctx->ep[e].pci_dev);
			ctx->ep[e].pci_dev = NULL;
		}
	}

	dma_dev_fini(ctx);

	peers_detach(ctx);

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

uint32_t
iuventus_queue_ep(const struct iuventus_ctx *ctx, uint32_t q)
{
	if (ctx == NULL || ctx->num_eps == 0 || ctx->num_peers == 0) {
		return 0;
	}

	return q / (IUVENTUS_HW_QUEUES / ctx->num_eps);
}
