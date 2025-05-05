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
#include <sys/queue.h>

#define DATA_BUFFER_STRING "Dan Kriz je best!"

struct ctrlr_entry {
	struct spdk_nvme_ctrlr	*ctrlr;
	char			name[1024];
} g_controller;

struct ns_entry {
	struct spdk_nvme_ctrlr	*ctrlr;
	struct spdk_nvme_ns	*ns;
	struct spdk_nvme_qpair	*qpair;
} g_namespace;

struct ncd_probe_ctx {
	void *cq_bar_vaddr;
	uint64_t cq_bar_paddr;
	uint64_t cq_bar_size;
	void *data_bar_vaddr;
	uint64_t data_bar_paddr;
	uint64_t data_bar_size;
	struct spdk_pci_device *dev;
};

static struct spdk_pci_id ncd_pci_driver_id[] = {
	{
		SPDK_PCI_DEVICE(0x18ec, 0xc020)
	},
};

SPDK_PCI_DRIVER_REGISTER(ncd, ncd_pci_driver_id, SPDK_PCI_DRIVER_NEED_MAPPING)

static void
register_ns(struct spdk_nvme_ctrlr *ctrlr, struct spdk_nvme_ns *ns)
{
	if (!spdk_nvme_ns_is_active(ns)) {
		return;
	}

	g_namespace.ctrlr = ctrlr;
	g_namespace.ns = ns;

	printf("  Namespace ID: %d size: %juGB\n", spdk_nvme_ns_get_id(ns),
	       spdk_nvme_ns_get_size(ns) / 1000000000);
}

static int
hello_world(struct ncd_probe_ctx *ncd_ctx)
{
	int				rc = 0;
	struct spdk_nvme_io_qpair_opts  *qopts;
	struct spdk_nvme_cmd *sq_vaddr;
	uint64_t buff_req_size;
	uint64_t buff_size;

	printf("Entered hello world...\n");

	qopts = malloc(sizeof(struct spdk_nvme_io_qpair_opts));
	if (qopts == NULL) {
		printf("ERROR: unable to create queue options structure!");
		return -ENOMEM;
	}

	spdk_nvme_ctrlr_get_default_io_qpair_opts(g_namespace.ctrlr, qopts, sizeof(struct spdk_nvme_io_qpair_opts));

	qopts->io_queue_requests = qopts->io_queue_size;

	printf("Queue pair options:\n");
	printf("IO queue size: %d\n", qopts->io_queue_size);
	printf("IO queue requests: %d\n", qopts->io_queue_requests);


	buff_req_size = qopts->io_queue_requests*sizeof(struct spdk_nvme_cmd);
	buff_size = buff_req_size;
	// NOTE: The memory alignment needs to be aligned to the memory page size as specified in CC.MPS
	// register of the NVMe controller
	sq_vaddr = spdk_dma_zmalloc(buff_req_size, 0x1000, NULL);
	if (sq_vaddr == NULL) {
		fprintf(stderr, "ERROR: Failed to alloc SQ buffer\n");
		rc = -1;
		goto cleanup;
	}

	qopts->sq.vaddr = sq_vaddr;
	qopts->sq.paddr = spdk_vtophys(sq_vaddr, &buff_size);
	if (qopts->sq.paddr == SPDK_VTOPHYS_ERROR) {
		fprintf(stderr, "ERROR: Unable to return physical address of an underlying buffer.");
		rc = -1;
		goto cleanup;
	}
	printf("SQ VADD: %p, SQ PADDR: %lx\n", qopts->sq.vaddr, qopts->sq.paddr);
	qopts->sq.buffer_size = qopts->io_queue_requests*sizeof(struct spdk_nvme_cmd);

	qopts->cq.vaddr = ncd_ctx->cq_bar_vaddr;
	qopts->cq.paddr = ncd_ctx->cq_bar_paddr;
	qopts->cq.buffer_size = qopts->io_queue_requests*sizeof(struct spdk_nvme_cpl);

	g_namespace.qpair = spdk_nvme_ctrlr_alloc_io_qpair(g_namespace.ctrlr, qopts, sizeof (struct spdk_nvme_io_qpair_opts));
	if (g_namespace.qpair == NULL) {
		printf("ERROR: spdk_nvme_ctrlr_alloc_io_qpair() failed\n");
		rc = -1;
		goto cleanup;
	}
	printf("Successfully allocated Queue pair!\n");

	spdk_nvme_ctrlr_free_io_qpair(g_namespace.qpair);
	printf("Successfully freed Queue pair!\n");

cleanup:
	spdk_dma_free(sq_vaddr);
	spdk_free(qopts);
	return rc;
}

static bool
probe_cb(void *cb_ctx, const struct spdk_nvme_transport_id *trid,
	 struct spdk_nvme_ctrlr_opts *opts)
{
	printf("Probing %s ...\n", trid->traddr);

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
		printf("ERROR: Invalid namespace!\n");
		return;
	}
	register_ns(ctrlr, ns);

	printf("Controller options:\n");
	copts = spdk_nvme_ctrlr_get_opts(ctrlr);
	if (copts == NULL) {
		printf("No controller options found!");
		return;
	}

	printf("\tNumber of IO queues: %d\n", copts->num_io_queues);
	printf("\tSize of IO queues:   %d\n", copts->io_queue_size);
	printf("\tIO queue requests:   %d\n", copts->io_queue_requests);
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
	struct ncd_probe_ctx *probe_ctx = ctx;

	probe_ctx->dev = pci_dev;

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

	rc = spdk_pci_device_disable_interrupts(pci_dev);
	if (rc) {
		fprintf(stderr, "Unable to disable interrupts\n");
		return rc;
	}

	rc = spdk_pci_device_disable_interrupt(pci_dev);
	if (rc) {
		fprintf(stderr, "Unable to disable interrupt\n");
		return rc;
	}

	return 0;
}

int
main(int argc, char **argv)
{
	int rc;
	struct spdk_env_opts opts;
	//struct spdk_nvme_transport_id g_trid = {};
	struct spdk_pci_driver *ncd_driver;
	struct spdk_pci_addr pcie_addr;
	struct ncd_probe_ctx ctx = {};

	opts.opts_size = sizeof(opts);
	spdk_env_opts_init(&opts);
	rc = parse_args(argc, argv, &opts);
	if (rc != 0) {
		return rc;
	}

	opts.name = "fpga_zero_copy";
	if (spdk_env_init(&opts) < 0) {
		fprintf(stderr, "Unable to initialize SPDK env\n");
		return 1;
	}

	printf("Initializing NVMe Controller\n");

	//spdk_nvme_trid_populate_transport(&g_trid, SPDK_NVME_TRANSPORT_PCIE);
	rc = spdk_nvme_probe(NULL, NULL, probe_cb, attach_cb, NULL);
	if (rc != 0) {
		fprintf(stderr, "ERROR: spdk_nvme_probe() failed\n");
		return 1;
	}

	if (g_controller.ctrlr == NULL) {
		fprintf(stderr, "ERROR: ctrlr structure invalid!\n");
		return 1;
	}

	if (!ctrl_rst_done) {
		printf("Resetting controller...");
		if (spdk_nvme_ctrlr_reset(g_controller.ctrlr))
			printf("Failed to reset controller!");
		if (spdk_nvme_ctrlr_reset_subsystem(g_controller.ctrlr))
			printf("Failed to reset subsystem!");
	}

	ncd_driver = spdk_pci_get_driver("ncd");
	if (ncd_driver == NULL) {
		fprintf(stderr, "Unable to get NCD driver!\n");
		goto exit;
	}

	rc = spdk_pci_addr_parse(&pcie_addr, "0000:41:00.1");
	if (rc) {
		fprintf(stderr, "Unable to parse PCIE address!\n");
		goto exit;
	}

	rc = spdk_pci_device_attach(ncd_driver, ncd_drv_attach_cb, &ctx, &pcie_addr);
	if (rc) {
		fprintf(stderr, "Unable to attach PCIE device!\n");
		goto exit;
	}

	printf("Initialization complete.\n");
	printf("NCD PCIe device context:\n");
	printf("CQ BAR VADDR: %p\n", ctx.cq_bar_vaddr);
	printf("CQ BAR PADDR: %lx\n", ctx.cq_bar_paddr);
	printf("CQ BAR size:  %ld\n", ctx.cq_bar_size);
	printf("DATA BAR VADDR: %p\n", ctx.data_bar_vaddr);
	printf("DATA BAR PADDR: %lx\n", ctx.data_bar_paddr);
	printf("DATA BAR size:  %ld\n", ctx.data_bar_size);

	/* *(uint64_t *) ctx.cq_bar_vaddr = 0x1248; */

	rc = hello_world(&ctx);
exit:
	spdk_pci_device_detach(ctx.dev);
	fflush(stdout);
	spdk_env_fini();
	return rc;
}
