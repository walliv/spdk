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
	struct spdk_nvme_qpair	*sw_qpair;
} g_namespace;

struct dma_ctrl_ctx {
	struct nfb_device *dev;
	struct nfb_comp *comp;
	const char *pcie_bdf;
};

struct qop_cpl_ctx {
	char cmd_name[20];
	int qop_completed;
	int32_t qid;
};

struct ncd_probe_ctx {
	const char *select_dev;
	uint16_t qsize;
	struct spdk_nvme_transport_id *trid;
	uint32_t nsid;
	struct spdk_pci_device *dev;

	void *sq_vaddr;
	uint64_t sq_paddr;
	uint64_t sq_byte_size;
	void *cq_vaddr;
	uint64_t cq_paddr;
	uint64_t cq_byte_size;
	void *wrbuff_vaddr;
	uint64_t wrbuff_paddr;
	uint64_t wrbuff_byte_size;
	void *rdbuff_vaddr;
	uint64_t rdbuff_paddr;
	uint64_t rdbuff_byte_size;

	void *wrbuff_prp_list_vaddr;
	uint64_t wrbuff_prp_list_paddr;
	void *rdbuff_prp_list_vaddr;
	uint64_t rdbuff_prp_list_paddr;

	uint64_t doorbell_base;
	uint32_t doorbell_stride;

	// The values that need to be written into the C/S registers before
	// command gets to be dispatched.
	uint16_t dbl_mask;
	/* uint64_t sq_paddr; */
	uint64_t sqtdbl_paddr;
	uint64_t cqhdbl_paddr;
	uint16_t lba_num_mask;
	uint64_t lba_space_size;
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
	struct qop_cpl_ctx *qctx = ctx;

	qctx->qop_completed = 1;
	if (spdk_nvme_cpl_is_error(cpl)) {
		spdk_nvme_print_completion(qctx->qid, (struct spdk_nvme_cpl *)cpl);
		fprintf(stderr, "CPL error status: %s\n", spdk_nvme_cpl_get_status_string(&cpl->status));
		fprintf(stderr, "%s failed, aborting run\n", qctx->cmd_name);
		qctx->qop_completed = -1;
	}
}

static int
submit_admin_request(struct spdk_nvme_cmd* cmd, char *cmd_name)
{
	int rc = 0;
	struct qop_cpl_ctx qctx = {0};

	snprintf(qctx.cmd_name, sizeof(qctx.cmd_name), "%s", cmd_name);
	qctx.qop_completed = 0;
	qctx.qid = 0;
	rc = spdk_nvme_ctrlr_cmd_admin_raw(g_namespace.ctrlr, cmd, NULL, 0, qop_complete_cb, &qctx);
	if (rc) {
		printf("Failed to submit the command %s!\n", qctx.cmd_name);
		return -1;
	}

	while (!qctx.qop_completed)
		spdk_nvme_ctrlr_process_admin_completions(g_namespace.ctrlr);

	if (qctx.qop_completed == -1)
		return -2;

	return 0;
}

static int
submit_rw_request(uint8_t rw, struct spdk_nvme_qpair* qpair, void* buf)
{
	int rc = 0;
	struct qop_cpl_ctx qctx = {0};

	int (*rw_op)(struct spdk_nvme_ns *ns, struct spdk_nvme_qpair *qpair, void *payload,
			   uint64_t lba, uint32_t lba_count, spdk_nvme_cmd_cb cb_fn,
			   void *cb_arg, uint32_t io_flags) = NULL;

	// Read
	if (rw == 1) {
		rw_op = spdk_nvme_ns_cmd_read;
	} else if (rw == 0) {
		rw_op = spdk_nvme_ns_cmd_write;
	}

	snprintf(qctx.cmd_name, sizeof(qctx.cmd_name), "%s", (rw == 1) ? "RD" : "WR");
	qctx.qop_completed = 0;
	qctx.qid = spdk_nvme_qpair_get_id(qpair);
	rc = rw_op(g_namespace.ns, qpair, buf, 0, 1, qop_complete_cb, &qctx, 0);
	if (rc) {
		fprintf(stderr, "Initial write of the control string to NVMe failed\n");
		return -1;
	}

	while (!qctx.qop_completed)
		spdk_nvme_qpair_process_completions(g_namespace.sw_qpair, 0);

	if (qctx.qop_completed == -1)
		return -2;

	return 0;
}

static int
queues_alloc(struct ncd_probe_ctx *ncd_ctx)
{
	int			       rc = 0;
	struct spdk_nvme_io_qpair_opts qopts;
	struct spdk_nvme_cmd           cmd = {0};

	spdk_nvme_ctrlr_get_default_io_qpair_opts(g_namespace.ctrlr, &qopts, sizeof(struct spdk_nvme_io_qpair_opts));

	printf("Default queue pair options:\n");
	printf("IO queue size: %d\n", qopts.io_queue_size);
	printf("IO queue requests: %d\n", qopts.io_queue_requests);

	if (ncd_ctx->qsize == 0) {
		qopts.io_queue_requests = qopts.io_queue_size;
	} else {
		qopts.io_queue_requests = ncd_ctx->qsize;
		qopts.io_queue_size = ncd_ctx->qsize;
	}

	ncd_ctx->dbl_mask = qopts.io_queue_size -1;

	qopts.sq.vaddr = ncd_ctx->sq_vaddr;
	qopts.sq.paddr = ncd_ctx->sq_paddr;
	qopts.sq.buffer_size = qopts.io_queue_size*sizeof(struct spdk_nvme_cmd);

	qopts.cq.vaddr = ncd_ctx->cq_vaddr;
	qopts.cq.paddr = ncd_ctx->cq_paddr;
	qopts.cq.buffer_size = qopts.io_queue_size*sizeof(struct spdk_nvme_cpl);

	// Reset the Completion Queue in the Hardware, otherwise previous completion entries get
	// detected. This means return Phase Tags to value 0 (i.e. default value)
	uint8_t *cpl_buff = ncd_ctx->cq_vaddr;
	for (uint32_t i = 14; i < qopts.cq.buffer_size; i+=sizeof(struct spdk_nvme_cpl)) {
		cpl_buff[i] = 0;
	}

	g_namespace.hw_qid = spdk_nvme_ctrlr_alloc_qid(g_namespace.ctrlr);
	if (g_namespace.hw_qid < 0) {
		printf("ERROR: Failed to allocated QID for the HW queues\n");
		rc = -13;
		goto qid_alloc_fail;

	}

	ncd_ctx->sqtdbl_paddr = ncd_ctx->doorbell_base + (2*g_namespace.hw_qid) * (4 << ncd_ctx->doorbell_stride);
	ncd_ctx->cqhdbl_paddr = ncd_ctx->doorbell_base + (2*g_namespace.hw_qid+1) * (4 << ncd_ctx->doorbell_stride);

	printf("Allocate qid %d\n", g_namespace.hw_qid);

	cmd.opc = SPDK_NVME_OPC_CREATE_IO_CQ;
	cmd.nsid = 0;
	cmd.cdw10_bits.create_io_q.qid = g_namespace.hw_qid;
	cmd.cdw10_bits.create_io_q.qsize = qopts.io_queue_size-1;
	cmd.cdw11_bits.create_io_cq.pc = 1;
	cmd.dptr.prp.prp1 = ncd_ctx->cq_paddr;

	rc = submit_admin_request(&cmd, "CQ_CREATE");
	if (rc) {
		printf("Failed to submit the Admin command!\n");
		goto cq_create_fail;
	}

	printf("HW CQ allocated!\n");

	cmd.opc = SPDK_NVME_OPC_CREATE_IO_SQ;
	cmd.nsid = 0;
	cmd.cdw10_bits.create_io_q.qid = g_namespace.hw_qid;
	cmd.cdw10_bits.create_io_q.qsize = qopts.io_queue_size-1;
	cmd.cdw11_bits.create_io_sq.pc = 1;
	cmd.cdw11_bits.create_io_sq.qprio = 2;
	cmd.cdw11_bits.create_io_sq.cqid = g_namespace.hw_qid;
	cmd.dptr.prp.prp1 = ncd_ctx->sq_paddr;

	rc = submit_admin_request(&cmd, "SQ_CREATE");
	if (rc) {
		printf("Failed to submit the Admin command!\n");
		goto sq_create_fail;
	}

	printf("HW SQ allocated!\n");

	g_namespace.sw_qpair = spdk_nvme_ctrlr_alloc_io_qpair(g_namespace.ctrlr, NULL, sizeof (struct spdk_nvme_io_qpair_opts));
	if (g_namespace.sw_qpair == NULL) {
		printf("ERROR: Failed to alloc SW queues\n");
		rc = -4;
		goto swq_create_fail;
	}

	printf("SW queues allocated!\n");
	return 0;

swq_create_fail:
	cmd.opc = SPDK_NVME_OPC_DELETE_IO_SQ;
	cmd.cdw10_bits.delete_io_q.qid = g_namespace.hw_qid;
	rc = submit_admin_request(&cmd, "SQ_DELETE");
	if (rc) {
		printf("Failed to submit Admin command!\n");
	}

sq_create_fail:
	cmd.opc = SPDK_NVME_OPC_DELETE_IO_CQ;
	cmd.cdw10_bits.delete_io_q.qid = g_namespace.hw_qid;
	rc = submit_admin_request(&cmd, "CQ_DELETE");
	if (rc) {
		printf("Failed to submit the Admin command!\n");
	}

cq_create_fail:
	spdk_nvme_ctrlr_free_qid(g_namespace.ctrlr, g_namespace.hw_qid);
qid_alloc_fail:
	return rc;
}

static void queues_delete(struct ncd_probe_ctx *ncd_ctx)
{
	int rc = 0;
	struct spdk_nvme_cmd cmd = {0};

	cmd.opc = SPDK_NVME_OPC_DELETE_IO_SQ;
	cmd.cdw10_bits.delete_io_q.qid = g_namespace.hw_qid;
	rc = submit_admin_request(&cmd, "SQ_DELETE");
	if (rc) {
		printf("Failed to submit Admin command!\n");
	}

	cmd.opc = SPDK_NVME_OPC_DELETE_IO_CQ;
	cmd.cdw10_bits.delete_io_q.qid = g_namespace.hw_qid;
	rc = submit_admin_request(&cmd, "CQ_DELETE");
	if (rc) {
		printf("Failed to submit the Admin command!\n");
	}
}

static void
queues_dealloc(struct ncd_probe_ctx *ncd_ctx)
{
	queues_delete(ncd_ctx);
	/* spdk_nvme_ctrlr_free_io_qpair(g_namespace.sw_qpair); */
	spdk_nvme_ctrlr_free_qid(g_namespace.ctrlr, g_namespace.hw_qid);
}

static bool
probe_cb(void *cb_ctx, const struct spdk_nvme_transport_id *trid,
	 struct spdk_nvme_ctrlr_opts *opts)
{
	struct ncd_probe_ctx *probe_ctx = cb_ctx;

	printf("Probing %s ...\n", trid->traddr);

	if (probe_ctx->qsize != 0) {
		opts->io_queue_size = probe_ctx->qsize;
		opts->io_queue_requests = probe_ctx->qsize;
	} else {
		opts->io_queue_requests = opts->io_queue_size;
	}
	opts->arb_mechanism = SPDK_NVME_CC_AMS_RR;
	opts->enable_interrupts = false;

	if (!strcmp(trid->traddr, probe_ctx->trid->traddr))
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
	struct ncd_probe_ctx *probe_ctx = cb_ctx;
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
		probe_ctx->lba_num_mask = 0xFFFF;
	} else {
		probe_ctx->lba_num_mask = (uint16_t)((1 << (12 + cap.bits.mpsmin + cdata->mdts)) / sect_size) - 1;
	}

	printf("Controller options:\n");

	printf("\tNumber of IO queues:   %d\n", opts->num_io_queues);
	printf("\tSize of IO queues:     %d\n", opts->io_queue_size);
	printf("\tIO queue requests:     %d\n", opts->io_queue_requests);
	printf("\tNS %d size:             %juGB\n", nsid, spdk_nvme_ns_get_size(ns) / 1000000000);
	printf("\tNS number of sectors:  %ld\n", spdk_nvme_ns_get_num_sectors(ns));
	printf("\tNS sector size:        %dB\n", sect_size);
	printf("\tLBA Mask:              x%x (%d)\n", probe_ctx->lba_num_mask, probe_ctx->lba_num_mask);
	probe_ctx->lba_space_size = spdk_nvme_ns_get_num_sectors(ns);

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
	probe_ctx->doorbell_base = bar_start + 0x1000;
	probe_ctx->doorbell_stride = cap.bits.dstrd;
	printf("Doorbell stride: %d\n", probe_ctx->doorbell_stride);
	probe_ctx->nsid = nsid;
}

static int dma_dev_init(const char* select_dev, struct dma_ctrl_ctx *dma_ctx)
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
dev_open_fail:
	return rc;
}

static int prp_list_alloc(struct ncd_probe_ctx *ncd_ctx)
{
	uint64_t size = VALUE_4KB;
	ncd_ctx->wrbuff_prp_list_vaddr = spdk_dma_zmalloc(VALUE_4KB, VALUE_4KB, NULL);
	if (ncd_ctx->wrbuff_prp_list_vaddr == NULL) {
		fprintf(stderr, "ERROR: Write PRP list allocation failed\n");
		return -1;
	}

	ncd_ctx->rdbuff_prp_list_vaddr = spdk_dma_zmalloc(VALUE_4KB, VALUE_4KB, NULL);
	if (ncd_ctx->rdbuff_prp_list_vaddr == NULL) {
		fprintf(stderr, "ERROR: Read PRP list allocation failed\n");
		spdk_dma_free(ncd_ctx->wrbuff_prp_list_vaddr);
		return -2;
	}

	ncd_ctx->wrbuff_prp_list_paddr = spdk_vtophys(ncd_ctx->wrbuff_prp_list_vaddr, &size);
	if (ncd_ctx->wrbuff_prp_list_paddr == SPDK_VTOPHYS_ERROR) {
		fprintf(stderr, "ERROR: Failed to get physical address of the Write PRP list buffer\n");
		spdk_dma_free(ncd_ctx->wrbuff_prp_list_vaddr);
		spdk_dma_free(ncd_ctx->rdbuff_prp_list_vaddr);
		return -3;
	}

	if (size != VALUE_4KB) {
		fprintf(stderr, "ERROR: Write PRP list buffer size is not 4096 bytes (detected size: %lu)\n", size);
		spdk_dma_free(ncd_ctx->wrbuff_prp_list_vaddr);
		spdk_dma_free(ncd_ctx->rdbuff_prp_list_vaddr);
		return -4;
	}

	ncd_ctx->rdbuff_prp_list_paddr = spdk_vtophys(ncd_ctx->rdbuff_prp_list_vaddr, &size);
	if (ncd_ctx->rdbuff_prp_list_paddr == SPDK_VTOPHYS_ERROR) {
		fprintf(stderr, "ERROR: Failed to get physical address of the Read PRP list buffer\n");
		spdk_dma_free(ncd_ctx->wrbuff_prp_list_vaddr);
		spdk_dma_free(ncd_ctx->rdbuff_prp_list_vaddr);
		return -5;
	}

	if (size != VALUE_4KB) {
		fprintf(stderr, "ERROR: Read PRP list buffer size is not 4096 bytes (detected size: %lu)\n", size);
		spdk_dma_free(ncd_ctx->wrbuff_prp_list_vaddr);
		spdk_dma_free(ncd_ctx->rdbuff_prp_list_vaddr);
		return -6;
	}

	for (int i = 1; i < (int)(ncd_ctx->wrbuff_byte_size / VALUE_4KB); i++) {
		((uint64_t *)ncd_ctx->wrbuff_prp_list_vaddr)[i-1] = ncd_ctx->wrbuff_paddr + (i * VALUE_4KB);
		((uint64_t *)ncd_ctx->rdbuff_prp_list_vaddr)[i-1] = ncd_ctx->rdbuff_paddr + (i * VALUE_4KB);
	}

	return 0;
}

static void prp_list_free(struct ncd_probe_ctx *ncd_ctx)
{
	spdk_dma_free(ncd_ctx->wrbuff_prp_list_vaddr);
	spdk_dma_free(ncd_ctx->rdbuff_prp_list_vaddr);
}

static int dma_ctrl_init(struct ncd_probe_ctx *ncd_ctx, struct dma_ctrl_ctx *dma_ctx)
{
	int rc = 0;
	int node;
	uint64_t meta_buff_size = VALUE_4KB;
	uint64_t meta_buff_paddr;

	node = nfb_comp_find(dma_ctx->dev, "ziti,dma_iuventus", 0);
	dma_ctx->comp = nfb_comp_open(dma_ctx->dev, node);
	if (dma_ctx->comp == NULL) {
		fprintf(stderr, "ERROR: Failed to open DMA control registers as nfb_comp!\n");
		rc = -2;
		goto dma_open_fail;
	}

	rc = prp_list_alloc(ncd_ctx);
	if (rc) {
		fprintf(stderr, "ERROR: PRP list allocation failed\n");
		goto prp_alloc_fail;
	}

	nfb_comp_write16(dma_ctx->comp, REG_DBL_MASK, ncd_ctx->dbl_mask);
	nfb_comp_write64(dma_ctx->comp, REG_SQTDBL_BADDR, ncd_ctx->sqtdbl_paddr);
	nfb_comp_write64(dma_ctx->comp, REG_CQHDBL_BADDR, ncd_ctx->cqhdbl_paddr);
	nfb_comp_write64(dma_ctx->comp, REG_RDBUFF_BADDR, ncd_ctx->rdbuff_paddr);
	nfb_comp_write64(dma_ctx->comp, REG_RDBUFF_PRP_LIST_PTR, ncd_ctx->rdbuff_prp_list_paddr);
	nfb_comp_write64(dma_ctx->comp, REG_WRBUFF_BADDR, ncd_ctx->wrbuff_paddr);
	nfb_comp_write64(dma_ctx->comp, REG_WRBUFF_PRP_LIST_PTR, ncd_ctx->wrbuff_prp_list_paddr);
	nfb_comp_write16(dma_ctx->comp, REG_LBA_NUM_MASK, ncd_ctx->lba_num_mask);
	nfb_comp_write64(dma_ctx->comp, REG_LBA_SPACE_SIZE, ncd_ctx->lba_space_size);
	nfb_comp_write64(dma_ctx->comp, REG_META_PTR, 0);
	return 0;

prp_alloc_fail:
	nfb_comp_close(dma_ctx->comp);
dma_open_fail:
	return rc;
}

static void dma_ctrl_deinit(struct ncd_probe_ctx *ncd_ctx, struct dma_ctrl_ctx *dma_ctx)
{
	prp_list_free(ncd_ctx);
	nfb_comp_close(dma_ctx->comp);
}

static void
usage(const char *program_name)
{
	printf("%s [options]", program_name);
	printf("\t\n");
	printf("options:\n");
	printf("\t[-c dispatches commands continuously otherwise specify the amount with -p flag]\n");
	printf("\t[-d selected nfb device (default 0)]\n");
	printf("\t[-m DPDK huge memory size in MB]\n");
	printf("\t[-g use single file descriptor for DPDK memory segments]\n");
	printf("\t[-i shared memory group ID]\n");
#ifdef DEBUG
	printf("\t[-L enable debug logging]\n");
#else
	printf("\t[-L enable debug logging (flag disabled, must reconfigure with --enable-debug)]\n");
#endif
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
	printf("\t[-s <num> the amount of LBAs to copy within a single command]\n");
	printf("\t[-q <num> size of the queues in items (Commands for SQ or Completions for CQ)]\n");
	printf("\t[-p <num> the amount of commands to dispatch]\n");
}

bool ctrl_rst_done = true;

static int
parse_args(int argc, char **argv, struct spdk_env_opts *env_opts, struct ncd_probe_ctx *ctx)
{
	int op, rc;

	while ((op = getopt(argc, argv, "s:i:gm:L:hrt:cq:p:d:")) != -1) {
		switch (op) {
		case 'd':
			ctx->select_dev = optarg;
			break;
		// case 'p':
		// 	ctx->cmds_to_disp = spdk_strtol(optarg, 10);
		// 	if (ctx->cmds_to_disp < 1) {
		// 		fprintf(stderr, "Invalid amount of commands to dispatch (must be greater than 0)\n");
		// 		exit(EXIT_FAILURE);
		// 	}
		// 	break;
		case 'q':
			ctx->qsize = spdk_strtol(optarg, 10);
			if (ctx->qsize < 4) {
				fprintf(stderr, "Invalid size of a queue\n");
				exit(EXIT_FAILURE);
			}
			break;
		// case 'c':
		// 	ctx->contiguous_dispatch = true;
		// 	break;
		// case 's':
		// 	ctx->lba_num = spdk_strtol(optarg, 10);
		// 	if (ctx->lba_num < 1) {
		// 		fprintf(stderr, "Invalid amount of LBAs\n");
		// 		exit(EXIT_FAILURE);
		// 	}
		// 	break;
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
			ctrl_rst_done = false;
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
	struct ncd_probe_ctx *probe_ctx = ctx;

	probe_ctx->dev = pci_dev;

	/* Enable Memory accesses, PCI busmaster and disable INTx */
	spdk_pci_device_cfg_read16(pci_dev, &cmd_reg, 4);
	cmd_reg |= 0x404;
	spdk_pci_device_cfg_write16(pci_dev, cmd_reg, 4);

	// 1. SKIP Enable device (Apparently, it is enabled by the dpdk-devbind)
	// 2. map BARs for Submission Queue, Completion Queueu, and the Data Transmission

	rc = spdk_pci_device_map_bar(pci_dev, SQ_BAR, &probe_ctx->sq_vaddr, &probe_ctx->sq_paddr, &probe_ctx->sq_byte_size);
	if (rc) {
		fprintf(stderr, "Unable to map BAR %d (SQ)\n", SQ_BAR);
		return rc;
	}

	rc = spdk_pci_device_map_bar(pci_dev, CQ_BAR, &probe_ctx->cq_vaddr, &probe_ctx->cq_paddr, &probe_ctx->cq_byte_size);
	if (rc) {
		fprintf(stderr, "Unable to map BAR %d (CQ)\n", CQ_BAR);
		return rc;
	}

	rc = spdk_pci_device_map_bar(pci_dev, WRBUFF_BAR, &probe_ctx->wrbuff_vaddr, &probe_ctx->wrbuff_paddr, &probe_ctx->wrbuff_byte_size);
	if (rc) {
		fprintf(stderr, "Unable to map BAR %d (WR buffer)\n", WRBUFF_BAR);
		return rc;
	}

	rc = spdk_pci_device_map_bar(pci_dev, RDBUFF_BAR, &probe_ctx->rdbuff_vaddr, &probe_ctx->rdbuff_paddr, &probe_ctx->rdbuff_byte_size);
	if (rc) {
		fprintf(stderr, "Unable to map BAR %d (RD buffer)\n", RDBUFF_BAR);
		return rc;
	}

	if (probe_ctx->cq_vaddr == NULL || probe_ctx->sq_vaddr == NULL || probe_ctx->rdbuff_vaddr == NULL || probe_ctx->wrbuff_vaddr == NULL) {
	fprintf(stderr, "Virtual BAR adresses invalid!\n");
	return -1;
	}
	if (probe_ctx->cq_paddr == 0 || probe_ctx->sq_paddr == 0 || probe_ctx->rdbuff_paddr == 0 || probe_ctx->wrbuff_paddr == 0) {
	fprintf(stderr, "Physical BAR adresses invalid!\n");
	return -2;
	}
	if (probe_ctx->cq_byte_size <= 0 || probe_ctx->sq_byte_size <= 0 || probe_ctx->rdbuff_byte_size <= 0 || probe_ctx->wrbuff_byte_size <= 0) {
	fprintf(stderr, "BAR sizes invalid!\n");
	return -3;
	}

	/* printf("CQ BAR VADDR: %p\n", probe_ctx->cq_vaddr); */
	/* printf("CQ BAR PADDR: %lx\n", probe_ctx->cq_paddr); */
	/* printf("CQ BAR size:  %ld\n", probe_ctx->cq_byte_size); */
	/* printf("DATA BAR VADDR: %p\n", probe_ctx->wrbuff_vaddr); */
	/* printf("DATA BAR PADDR: %lx\n", probe_ctx->wrbuff_paddr); */
	/* printf("DATA BAR size:  %ld\n", probe_ctx->wrbuff_byte_size); */

	/* rc = spdk_pci_device_disable_interrupts(pci_dev); */
	/* if (rc) { */
	/* 	fprintf(stderr, "Unable to disable interrupts\n"); */
	/* 	return rc; */
	/* } */

	/* rc = spdk_pci_device_disable_interrupt(pci_dev); */
	/* if (rc) { */
	/* 	fprintf(stderr, "Unable to disable interrupt\n"); */
	/* 	return rc; */
	/* } */

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
	struct ncd_probe_ctx ctx = {0};
	struct dma_ctrl_ctx dma_ctx = {0};

	char *buf = NULL;

	// Assign default attributes
	trid.trtype = SPDK_NVME_TRANSPORT_PCIE;
	ctx.trid = &trid;
	ctx.qsize = 0;
	ctx.select_dev = "0";

	opts.opts_size = sizeof(opts);
	spdk_env_opts_init(&opts);
	rc = parse_args(argc, argv, &opts, &ctx);
	if (rc != 0) {
		return rc;
	}

	opts.name = "fpga_zero_copy";
	if (spdk_env_init(&opts) < 0) {
		fprintf(stderr, "Unable to initialize SPDK env\n");
		return -1;
	}

	printf("Initializing NVMe Controller for device %s\n", trid.traddr);
	rc = spdk_nvme_probe(NULL, &ctx, probe_cb, attach_cb, NULL);
	if (rc != 0) {
		fprintf(stderr, "ERROR: spdk_nvme_probe() failed\n");
		goto nvme_probe_fail;
	}

	if (g_controller.ctrlr == NULL) {
		fprintf(stderr, "ERROR: ctrlr structure uninitialized! Specify the PCIe address.\n");
		rc = -10;
		goto nvme_probe_fail;
	}

	if (!ctrl_rst_done) {
		printf("Resetting controller...");
		if (spdk_nvme_ctrlr_reset(g_controller.ctrlr)) {
			fprintf(stderr, "Failed to reset controller!");
			rc = -11;
			goto ctrlr_reset_fail;
		}
		if (spdk_nvme_ctrlr_reset_subsystem(g_controller.ctrlr)) {
			fprintf(stderr, "Failed to reset subsystem!");
			rc = -12;
			goto ctrlr_reset_fail;
		}
	}

	ncd_driver = spdk_pci_get_driver("ncd");
	if (ncd_driver == NULL) {
		fprintf(stderr, "Unable to get NCD driver!\n");
		rc = -13;
		goto ctrlr_reset_fail;
	}

	rc = dma_dev_init(ctx.select_dev, &dma_ctx);
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

	rc = spdk_pci_device_attach(ncd_driver, ncd_drv_attach_cb, &ctx, &pcie_addr);
	if (rc) {
		fprintf(stderr, "Unable to attach PCIE device!\n");
		goto dma_dev_init_fail;
	}

	printf("PCIE domain initialization complete\n");
	printf("NCD PCIe device context:\n");
	printf("\tSQ VADDR: %p\n", ctx.sq_vaddr);
	printf("\tSQ PADDR: %lx\n", ctx.sq_paddr);
	printf("\tSQ size:  %ld bytes\n", ctx.sq_byte_size);
	printf("\tCQ VADDR: %p\n", ctx.cq_vaddr);
	printf("\tCQ PADDR: %lx\n", ctx.cq_paddr);
	printf("\tCQ size:  %ld bytes\n", ctx.cq_byte_size);
	printf("\tWRBUFF VADDR: %p\n", ctx.wrbuff_vaddr);
	printf("\tWRBUFF PADDR: %lx\n", ctx.wrbuff_paddr);
	printf("\tWRBUFF size:  %ld bytes\n", ctx.wrbuff_byte_size);
	printf("\tRDBUFF VADDR: %p\n", ctx.rdbuff_vaddr);
	printf("\tRDBUFF PADDR: %lx\n", ctx.rdbuff_paddr);
	printf("\tRDBUFF size:  %ld bytes\n", ctx.rdbuff_byte_size);

	rc = queues_alloc(&ctx);
	if (rc) {
		fprintf(stderr, "Unable to allocate queues!\n");
		goto queue_alloc_fail;
	}
	printf("\tSQTDBL physical address: 0x%lx\n", ctx.sqtdbl_paddr);
	printf("\tCQHDBL physical address: 0x%lx\n", ctx.cqhdbl_paddr);
	printf("Queues allocated.\n");

	/* buf = spdk_dma_zmalloc(0x1000, 0x1000, NULL); */
	/* if (buf == NULL) { */
	/* 	fprintf(stderr, "ERROR: write buffer allocation failed\n"); */
	/* 	goto buf_alloc_fail; */
	/* } */

	/* snprintf(buf, 0x1000, "%s", DATA_BUFFER_STRING); */

	// Write test string to the NVMe
	/* rc = submit_rw_request(0, g_namespace.sw_qpair, buf); */
	/* if (rc) { */
	/* 	fprintf(stderr, "ERROR: Failed to submit RW request!\n"); */
	/* 	goto buf_alloc_fail; */
	/* } */

	/* Read test string from NVMe and write it to the FPGA */
	/* for (int it = 0; it < 1000; it++) { */
	/* rc = submit_rw_request(1, g_namespace.sw_qpair, ctx.data_bar_vaddr); */
	/* if (rc) { */
	/* 	fprintf(stderr, "ERROR: Failed to submit RW request!\n"); */
	/* 	goto buf_alloc_fail; */
	/* } */
	/* } */

	rc = dma_ctrl_init(&ctx, &dma_ctx);
	if (rc) {
		fprintf(stderr, "Error configuring the DMA Iuventus controller structure\n");
		goto dma_ctrl_alloc_fail;
	}

	// Can be commented out if we want to keep statistics between runs
	nfb_comp_write16(dma_ctx.comp, REG_CONTROL, CTRL_RPT_UPD_EN | CTRL_ENABLE);
	usleep(1);

	signal(SIGINT, sig_usr);
	signal(SIGTERM, sig_usr);

	usleep(1000);
	//spdk_nvme_print_command(g_namespace.hw_qid, ctx.sq_bar_vaddr);

	while (!stop) usleep(10000);
	nfb_comp_write16(dma_ctx.comp, REG_CONTROL, 0);

	while (nfb_comp_read16(dma_ctx.comp, REG_SQTDBL) != nfb_comp_read16(dma_ctx.comp, REG_SQHDBL) &&
		(nfb_comp_read8(dma_ctx.comp, REG_STATUS) & 0x1) == 0) {
		usleep(1000000);
	}

	dma_ctrl_deinit(&ctx, &dma_ctx);

	rd_ctrl_regs(g_namespace.ctrlr);

/* buf_alloc_fail: */
dma_ctrl_alloc_fail:
	queues_dealloc(&ctx);
	printf("Qeues dealloced\n");
queue_alloc_fail:
	spdk_pci_device_detach(ctx.dev);
	printf("Pcie dev detached\n");
	/* fflush(stdout); */
dma_dev_init_fail:
	nfb_close(dma_ctx.dev);
ctrlr_reset_fail:
	spdk_nvme_detach(g_controller.ctrlr);
	printf("NVME Controller detached\n");
nvme_probe_fail:
	spdk_env_fini();
	printf("Env finished\n");
	return rc;
}
