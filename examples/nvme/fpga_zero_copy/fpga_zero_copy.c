/*
 *   fpga_zero_copy.c: Establishes peer-to-peer chain between an FPGA card and an NVMe device
 *   with FPGA as a master.
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) Vladislav Válek
 */

#include "spdk/stdinc.h"

#include "spdk/nvme.h"
#include "spdk/vmd.h"
#include "spdk/env.h"
#include "spdk/string.h"
#include "spdk/log.h"
#include "spdk/memory.h"
#include <sys/queue.h>

#include <nfb/nfb.h>

#define DATA_BUFFER_STRING "Dan Kriz je best!"

#define REG_CONTROL             0x00
#define REG_STATUS              0x04
#define REG_SQTDBL              0x08
#define REG_SQHDBL              0x0C
#define REG_CQHDBL              0x10
#define REG_DBL_MASK            0x14
#define REG_SQTDBL_INIT_VAL     0x18
#define REG_SQ_BASE_ADDR        0x1C
#define REG_SQTDBL_BASE_ADDR    0x24
#define REG_CQHDBL_BASE_ADDR    0x2C
#define REG_PRP_ENTRY_1_ADDR    0x34
#define REG_PRP_ENTRY_2_ADDR    0x3C
#define REG_START_LBA_PTR       0x44
#define REG_LBA_AMOUNT          0x48
#define REG_LAST_CQ_ENTRY       0x4C
#define REG_SQES_DISPATCHED     0x5C
#define REG_CQES_PROCESSED      0x64
#define REG_RECV_PCIE_RDS       0x6C
#define REG_RECV_PCIE_RDS_BYTES 0x74
#define REG_RECV_PCIE_WRS       0x7C
#define REG_RECV_PCIE_WRS_BYTES 0x84
#define REG_PROC_RDS            0x8C
#define REG_PROC_RDS_BYTES      0x94

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
};

struct qop_cpl_ctx {
	char cmd_name[20];
	int qop_completed;
	int32_t qid;
};

struct ncd_probe_ctx {
	uint32_t nsid;
	struct spdk_pci_device *dev;
	struct spdk_nvme_cmd *sq_vaddr;
	void *cq_bar_vaddr;
	uint64_t cq_bar_paddr;
	uint64_t cq_bar_size;
	// Servers as PRP1 entry
	uint64_t data_bar_paddr;
	void *data_bar_vaddr;
	uint64_t data_bar_size;
	uint64_t doorbell_base;
	uint32_t doorbell_stride;

	// The values that need to be written into the C/S registers before
	// command gets to be dispatched.
	uint16_t dbl_mask;
	uint64_t sq_paddr;
	uint64_t sqtdbl_paddr;
	uint64_t cqhdbl_paddr;
};

static struct spdk_pci_id ncd_pci_driver_id[] = {
	{
		SPDK_PCI_DEVICE(0x18ec, 0xc020)
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

/* int spdk_nvme_ns_cmd_read(struct spdk_nvme_ns *ns, struct spdk_nvme_qpair *qpair, void *payload, */
/* 			  uint64_t lba, uint32_t lba_count, spdk_nvme_cmd_cb cb_fn, */
/* 			  void *cb_arg, uint32_t io_flags); */

/* int spdk_nvme_ns_cmd_write(struct spdk_nvme_ns *ns, struct spdk_nvme_qpair *qpair, void *payload, */
/* 			   uint64_t lba, uint32_t lba_count, spdk_nvme_cmd_cb cb_fn, */
/* 			   void *cb_arg, uint32_t io_flags); */

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
	int32_t			       numa_id;
	struct spdk_nvme_io_qpair_opts qopts;
	uint64_t                       buff_req_size;
	struct spdk_nvme_cmd           cmd = {0};
	size_t                         queue_align;

	spdk_nvme_ctrlr_get_default_io_qpair_opts(g_namespace.ctrlr, &qopts, sizeof(struct spdk_nvme_io_qpair_opts));

	printf("Queue pair options:\n");
	printf("IO queue size: %d\n", qopts.io_queue_size);
	printf("IO queue requests: %d\n", qopts.io_queue_requests);

	qopts.io_queue_requests = qopts.io_queue_size;
	ncd_ctx->dbl_mask = qopts.io_queue_size -1;

	buff_req_size = qopts.io_queue_size*sizeof(struct spdk_nvme_cmd);
	numa_id = spdk_nvme_ctrlr_get_numa_id(g_namespace.ctrlr);
	queue_align = spdk_max(spdk_align32pow2(buff_req_size), sysconf(_SC_PAGESIZE));
	// NOTE: The memory alignment needs to be aligned to the memory page size as specified in CC.MPS
	// register of the NVMe controller
	ncd_ctx->sq_vaddr = spdk_dma_zmalloc_socket(buff_req_size, queue_align, NULL, numa_id);
	if (ncd_ctx->sq_vaddr == NULL) {
		fprintf(stderr, "ERROR: Failed to alloc SQ buffer\n");
		rc = -1;
		goto sq_memalloc_fail;
	}

	qopts.sq.vaddr = ncd_ctx->sq_vaddr;
	qopts.sq.paddr = spdk_vtophys(ncd_ctx->sq_vaddr, &buff_req_size);
	if (qopts.sq.paddr == SPDK_VTOPHYS_ERROR) {
		fprintf(stderr, "ERROR: Unable to return physical address of an underlying buffer.");
		rc = -2;
		goto vtophys_fail;
	}
	ncd_ctx->sq_paddr = qopts.sq.paddr;
	printf("SQ VADD: %p, SQ PADDR: %lx\n", qopts.sq.vaddr, qopts.sq.paddr);
	qopts.sq.buffer_size = qopts.io_queue_size*sizeof(struct spdk_nvme_cmd);

	qopts.cq.vaddr = ncd_ctx->cq_bar_vaddr;
	qopts.cq.paddr = ncd_ctx->cq_bar_paddr;
	qopts.cq.buffer_size = qopts.io_queue_size*sizeof(struct spdk_nvme_cpl);


	// Reset the Completion Queue in the Hardware, otherwise previous completion entries get
	// detected
	struct spdk_nvme_cpl *cpl_buff = ncd_ctx->cq_bar_vaddr;
	for (uint32_t i = 0; i < qopts.io_queue_size; i++) {
		cpl_buff[i].status.p = 0;
	}

	g_namespace.hw_qid = spdk_nvme_ctrlr_alloc_qid(g_namespace.ctrlr);
	if (g_namespace.hw_qid < 0) {
		printf("ERROR: Failed to allocated QID for the HW queues\n");
		rc = -13;
		goto vtophys_fail;

	}

	ncd_ctx->sqtdbl_paddr = ncd_ctx->doorbell_base + (2*g_namespace.hw_qid) * (4 << ncd_ctx->doorbell_stride);
	ncd_ctx->cqhdbl_paddr = ncd_ctx->doorbell_base + (2*g_namespace.hw_qid+1) * (4 << ncd_ctx->doorbell_stride);

	printf("Allocate qid %d\n", g_namespace.hw_qid);

	cmd.opc = SPDK_NVME_OPC_CREATE_IO_CQ;
	cmd.nsid = 0;
	cmd.cdw10_bits.create_io_q.qid = g_namespace.hw_qid;
	cmd.cdw10_bits.create_io_q.qsize = qopts.io_queue_size-1;
	cmd.cdw11_bits.create_io_cq.pc = 1;
	cmd.dptr.prp.prp1 = ncd_ctx->cq_bar_paddr;

	rc = submit_admin_request(&cmd, "CQ_CREATE");
	if (rc) {
		printf("Failed to submit the Admin command!\n");
		goto vtophys_fail;
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

vtophys_fail:
	spdk_dma_free(ncd_ctx->sq_vaddr);
sq_memalloc_fail:
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
	spdk_dma_free(ncd_ctx->sq_vaddr);
}

static bool
probe_cb(void *cb_ctx, const struct spdk_nvme_transport_id *trid,
	 struct spdk_nvme_ctrlr_opts *opts)
{
	printf("Probing %s ...\n", trid->traddr);

	// Accepts all controllers that it finds
	return true;
}

static void
attach_cb(void *cb_ctx, const struct spdk_nvme_transport_id *trid,
	  struct spdk_nvme_ctrlr *ctrlr, const struct spdk_nvme_ctrlr_opts *opts)
{
	int nsid;
	struct spdk_nvme_ns *ns;
	const struct spdk_nvme_ctrlr_data *cdata;
	const struct spdk_nvme_ctrlr_opts *copts;
	struct spdk_pci_device *pci_dev;
	struct spdk_pci_addr pci_addr;
	char bdf[32];
	char sysfs_path[128];
	uint64_t bar_start, bar_end, bar_flags;
	FILE *fp;
	volatile struct spdk_nvme_registers *regs;
	struct ncd_probe_ctx *probe_ctx = cb_ctx;

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

	printf("  Namespace ID: %d size: %juGB\n", nsid,
	       spdk_nvme_ns_get_size(ns) / 1000000000);

	printf("Controller options:\n");
	copts = spdk_nvme_ctrlr_get_opts(ctrlr);
	if (copts == NULL) {
		fprintf(stderr, "No controller options found!");
		return;
	}

	printf("\tNumber of IO queues: %d\n", copts->num_io_queues);
	printf("\tSize of IO queues:   %d\n", copts->io_queue_size);
	printf("\tIO queue requests:   %d\n", copts->io_queue_requests);

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

	regs = spdk_nvme_ctrlr_get_registers(ctrlr);

	// Doorbell registers start at offset 0x1000 from BAR0 but they start from the Admin Queues!
	probe_ctx->doorbell_base = bar_start + 0x1000;
	probe_ctx->doorbell_stride = regs->cap.bits.dstrd;
	printf("Doorbell stride: %d\n", probe_ctx->doorbell_stride);
	probe_ctx->nsid = nsid;
}

static int dma_ctrl_init(struct ncd_probe_ctx *ncd_ctx, struct dma_ctrl_ctx *dma_ctx)
{
	int rc = 0;
	int node;

	dma_ctx->dev = nfb_open("/dev/nfb/by-pci-slot/0000:61:00.0");
	if(!dma_ctx->dev) {
		fprintf(stderr, "ERROR: Failed to open NFB device");
		rc = -1;
		goto dev_open_fail;
	}

	node = nfb_comp_find(dma_ctx->dev, "ziti,dma_iuventus", 0);
	dma_ctx->comp = nfb_comp_open(dma_ctx->dev, node);
	if (dma_ctx->comp == NULL) {
		fprintf(stderr, "ERROR: Failed to open NFB component");
		rc = -2;
		goto comp_open_fail;
	}

	// Send a reset and wait until its done
	nfb_comp_write8(dma_ctx->comp, REG_CONTROL, 2);
	while(!(nfb_comp_read8(dma_ctx->comp, REG_STATUS) & 2));
	printf("Reset of the Command Dispatcher done!\n");

	nfb_comp_write16(dma_ctx->comp, REG_DBL_MASK, ncd_ctx->dbl_mask);
	nfb_comp_write64(dma_ctx->comp, REG_SQ_BASE_ADDR, ncd_ctx->sq_paddr);
	nfb_comp_write64(dma_ctx->comp, REG_SQTDBL_BASE_ADDR, ncd_ctx->sqtdbl_paddr);
	nfb_comp_write64(dma_ctx->comp, REG_CQHDBL_BASE_ADDR, ncd_ctx->cqhdbl_paddr);
	nfb_comp_write64(dma_ctx->comp, REG_PRP_ENTRY_1_ADDR, ncd_ctx->data_bar_paddr);
	nfb_comp_write16(dma_ctx->comp, REG_LBA_AMOUNT, 0);

	return 0;

comp_open_fail:
	nfb_close(dma_ctx->dev);
dev_open_fail:
	return rc;
}

static void dma_ctrl_close(struct dma_ctrl_ctx *dma_ctx)
{

	nfb_comp_close(dma_ctx->comp);
	nfb_close(dma_ctx->dev);
}

static void
usage(const char *program_name)
{
	printf("%s [options]", program_name);
	printf("\t\n");
	printf("options:\n");
	printf("\t[-d DPDK huge memory size in MB]\n");
	printf("\t[-g use single file descriptor for DPDK memory segments]\n");
	printf("\t[-i shared memory group ID]\n");
#ifdef DEBUG
	printf("\t[-L enable debug logging]\n");
#else
	printf("\t[-L enable debug logging (flag disabled, must reconfigure with --enable-debug)]\n");
#endif
}

bool ctrl_rst_done = true;

static int
parse_args(int argc, char **argv, struct spdk_env_opts *env_opts)
{
	int op, rc;

	while ((op = getopt(argc, argv, "i:gd:L:hr")) != -1) {
		switch (op) {
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
		case 'd':
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
	uint64_t mem_register_start, mem_register_end;

	probe_ctx->dev = pci_dev;

	/* Enable Memory accesses, PCI busmaster and disable INTx */
	spdk_pci_device_cfg_read16(pci_dev, &cmd_reg, 4);
	cmd_reg |= 0x404;
	spdk_pci_device_cfg_write16(pci_dev, cmd_reg, 4);

	// 1. SKIP Enable device (Apparently, it is enabled by the dpdk-devbind)
	// 2. map BARs for both, the Completion Queueu, and the Data Transmission
	rc = spdk_pci_device_map_bar(pci_dev, 0, &probe_ctx->cq_bar_vaddr, &probe_ctx->cq_bar_paddr, &probe_ctx->cq_bar_size);
	if (rc) {
		fprintf(stderr, "Unable to map BAR 0\n");
		return rc;
	}

	rc = spdk_pci_device_map_bar(pci_dev, 2, &probe_ctx->data_bar_vaddr, &probe_ctx->data_bar_paddr, &probe_ctx->data_bar_size);
	if (rc) {
		fprintf(stderr, "Unable to map BAR 2\n");
		return rc;
	}

	printf("CQ BAR VADDR: %p\n", probe_ctx->cq_bar_vaddr);
	printf("CQ BAR PADDR: %lx\n", probe_ctx->cq_bar_paddr);
	printf("CQ BAR size:  %ld\n", probe_ctx->cq_bar_size);
	printf("DATA BAR VADDR: %p\n", probe_ctx->data_bar_vaddr);
	printf("DATA BAR PADDR: %lx\n", probe_ctx->data_bar_paddr);
	printf("DATA BAR size:  %ld\n", probe_ctx->data_bar_size);

	mem_register_start = _2MB_PAGE((uintptr_t)probe_ctx->cq_bar_vaddr);
	mem_register_end = CEIL_2MB((uintptr_t)probe_ctx->cq_bar_vaddr + probe_ctx->cq_bar_size);

	rc = spdk_mem_register((void *)mem_register_start, VALUE_2MB);
	if (rc) {
		SPDK_ERRLOG("spdk_mem_register() of CQ BAR failed\n");
		return rc;
	}

	mem_register_start = _2MB_PAGE((uintptr_t)probe_ctx->data_bar_vaddr);
	mem_register_end = CEIL_2MB((uintptr_t)probe_ctx->data_bar_vaddr + probe_ctx->data_bar_size);

	rc = spdk_mem_register((void *)mem_register_start, VALUE_2MB);
	if (rc) {
		SPDK_ERRLOG("spdk_mem_register() of DATA failed\n");
		rc = spdk_mem_unregister(probe_ctx->cq_bar_vaddr, VALUE_2MB);
		return rc;
	}

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
	struct spdk_pci_addr pcie_addr;
	struct ncd_probe_ctx ctx = {0};
	struct dma_ctrl_ctx dma_ctx = {0};

	char *buf = NULL;

	opts.opts_size = sizeof(opts);
	spdk_env_opts_init(&opts);
	rc = parse_args(argc, argv, &opts);
	if (rc != 0) {
		return rc;
	}

	opts.name = "fpga_zero_copy";
	if (spdk_env_init(&opts) < 0) {
		fprintf(stderr, "Unable to initialize SPDK env\n");
		return -1;
	}

	printf("Initializing NVMe Controller\n");
	rc = spdk_nvme_probe(NULL, &ctx, probe_cb, attach_cb, NULL);
	if (rc != 0) {
		fprintf(stderr, "ERROR: spdk_nvme_probe() failed\n");
		goto nvme_probe_fail;
	}

	if (g_controller.ctrlr == NULL) {
		fprintf(stderr, "ERROR: ctrlr structure invalid!\n");
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

	rc = spdk_pci_addr_parse(&pcie_addr, "0000:61:00.1");
	if (rc) {
		fprintf(stderr, "Unable to parse PCIE address!\n");
		goto ctrlr_reset_fail;
	}

	rc = spdk_pci_device_attach(ncd_driver, ncd_drv_attach_cb, &ctx, &pcie_addr);
	if (rc) {
		fprintf(stderr, "Unable to attach PCIE device!\n");
		goto ctrlr_reset_fail;
	}

	printf("PCIE domain initialization complete\n");
	printf("NCD PCIe device context:\n");
	printf("CQ BAR VADDR: %p\n", ctx.cq_bar_vaddr);
	printf("CQ BAR PADDR: %lx\n", ctx.cq_bar_paddr);
	printf("CQ BAR size:  %ld\n", ctx.cq_bar_size);
	printf("DATA BAR VADDR: %p\n", ctx.data_bar_vaddr);
	printf("DATA BAR PADDR: %lx\n", ctx.data_bar_paddr);
	printf("DATA BAR size:  %ld\n", ctx.data_bar_size);
	/* *(uint64_t *) ctx.cq_bar_vaddr = 0x1248; */

	rc = queues_alloc(&ctx);
	if (rc) {
		fprintf(stderr, "Unable to allocate queues!\n");
		goto queue_alloc_fail;
	}
	printf("Queues allocated.\n");

	printf("SQTDBL physical address: 0x%lx\n", ctx.sqtdbl_paddr);
	printf("CQHDBL physical address: 0x%lx\n", ctx.cqhdbl_paddr);

	buf = spdk_dma_zmalloc(0x1000, 0x1000, NULL);
	if (buf == NULL) {
		fprintf(stderr, "ERROR: write buffer allocation failed\n");
		goto buf_alloc_fail;
	}

	snprintf(buf, 0x1000, "%s", DATA_BUFFER_STRING);

	// Write test string to the NVMe
	rc = submit_rw_request(0, g_namespace.sw_qpair, buf);
	if (rc) {
		fprintf(stderr, "ERROR: Failed to submit RW request!\n");
		goto buf_alloc_fail;
	}

	/* Read test string from NVMe and write it to the FPGA */
	/* for (int it = 0; it < 1000; it++) { */
	rc = submit_rw_request(1, g_namespace.sw_qpair, ctx.data_bar_vaddr);
	if (rc) {
		fprintf(stderr, "ERROR: Failed to submit RW request!\n");
		goto buf_alloc_fail;
	}
	/* } */

	rc = dma_ctrl_init(&ctx, &dma_ctx);
	if (rc) {
		fprintf(stderr, "Error opening the DMA Iuventus controller structure\n");
		goto dma_ctrl_alloc_fail;
	}

	nfb_comp_write8(dma_ctx.comp, REG_CONTROL, 1);
	printf("NCD command written\n");

	usleep(1000);
	spdk_nvme_print_command(g_namespace.hw_qid, ctx.sq_vaddr);
	while (nfb_comp_read16(dma_ctx.comp, REG_SQTDBL) != nfb_comp_read16(dma_ctx.comp, REG_SQHDBL)) {
		usleep(1000);
	}

	dma_ctrl_close(&dma_ctx);

buf_alloc_fail:
dma_ctrl_alloc_fail:
	queues_dealloc(&ctx);
	printf("Qeues dealloced\n");
queue_alloc_fail:
	spdk_mem_unregister(ctx.data_bar_vaddr, VALUE_2MB);
	spdk_mem_unregister(ctx.cq_bar_vaddr, VALUE_2MB);
	spdk_pci_device_detach(ctx.dev);
	printf("Pcie dev detached\n");
	/* fflush(stdout); */
ctrlr_reset_fail:
	spdk_nvme_detach(g_controller.ctrlr);
	printf("NVME Controller detached\n");
nvme_probe_fail:
	spdk_env_fini();
	printf("Env finished\n");
	return rc;
}
