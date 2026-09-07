/*
 * p2p_dir_test.c: Test P2P data path from NVMe to FPGA BAR.
 * Copyright (C) 2025 Universitaet Heidelberg, Institut fuer Technische Informatik (ZITI)
 * Author(s): Vladislav Valek <vladislav.valek@stud.uni-heidelberg.de>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Test A — NVMe READ with FPGA WrBuf BAR as destination:
 *   Issue a host-driven NVMe READ of LBA 0 with the data buffer pointing at
 *   the FPGA BAR2 lower-half (WrBuf).  The drive DMA-writes data into the BAR.
 *   Success = completion OK and WrBuf contains non-zero / expected data.
 *
 * Usage: p2p_dir_test <nvme-pci-addr>
 *   Example: p2p_dir_test 0000:62:00.0   (Samsung 990 PRO)
 *            p2p_dir_test 0000:61:00.0   (SK Hynix PC611)
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

#include "spdk/stdinc.h"
#include "spdk/nvme.h"
#include "spdk/nvme_spec.h"
#include "spdk/env.h"
#include "spdk/log.h"
#include "spdk/memory.h"

/* FPGA PF1 PCIe address (always fixed for this setup) */
#define FPGA_PF1_BDF  "0000:41:00.1"

/* FPGA BAR indices (PF1 2-BAR layout):
 *   BAR0 = SQ (lower 128 KiB) + CQ (upper 128 KiB)  -- unused here
 *   BAR2 = WrBuf (lower 128 KiB) + RdBuf (upper 128 KiB)
 */
#define FPGA_BAR_DATA  2

/* How many seconds to wait for an NVMe completion before declaring timeout */
#define TIMEOUT_SEC  2

/* NVMe PCI IDs for the custom NCD driver (FPGA PF1) */
static struct spdk_pci_id g_ncd_pci_ids[] = {
    { SPDK_PCI_DEVICE(0x18ec, 0xc020) },
    { SPDK_PCI_DEVICE(0x1c2c, 0xc020) },
    { 0 }
};

SPDK_PCI_DRIVER_REGISTER(ncd, g_ncd_pci_ids, SPDK_PCI_DRIVER_NEED_MAPPING)

/* --------------------------------------------------------------------------
 * Global state
 * -------------------------------------------------------------------------- */

struct fpga_bar_info {
    void     *vaddr;
    uint64_t  paddr;
    uint64_t  size;
};

static struct fpga_bar_info g_wrbuf = {0};
static struct fpga_bar_info g_bar2  = {0};

static struct spdk_nvme_ctrlr *g_ctrlr     = NULL;
static struct spdk_nvme_ns    *g_ns        = NULL;
static struct spdk_nvme_qpair *g_qpair     = NULL;
static uint32_t                g_sect_size = 512;

/* Argv[1] — the NVMe BDF we want to test */
static const char *g_nvme_bdf = NULL;

/* Result flags */
static bool    g_use_vtophys_path = false;
static int     g_io_status        = 0;   /* 1 = ok, -1 = error, 0 = pending */

/* --------------------------------------------------------------------------
 * FPGA NCD driver attach callback
 * -------------------------------------------------------------------------- */

static int
ncd_drv_attach_cb(void *ctx, struct spdk_pci_device *pci_dev)
{
    int      rc;
    uint16_t cmd_reg;

    /* Enable Memory accesses and PCIe bus-master; disable INTx */
    spdk_pci_device_cfg_read16(pci_dev, &cmd_reg, 4);
    cmd_reg |= 0x0406;
    spdk_pci_device_cfg_write16(pci_dev, cmd_reg, 4);

    rc = spdk_pci_device_map_bar(pci_dev, FPGA_BAR_DATA,
                                 &g_bar2.vaddr, &g_bar2.paddr, &g_bar2.size);
    if (rc) {
        fprintf(stderr, "ERROR: spdk_pci_device_map_bar(%d) failed: %d\n",
                FPGA_BAR_DATA, rc);
        return rc;
    }

    printf("BAR%d mapped: vaddr=%p  paddr=0x%016lx  size=%lu bytes\n",
           FPGA_BAR_DATA, g_bar2.vaddr, g_bar2.paddr, g_bar2.size);

    /* Lower half of BAR2 is the write buffer (drive → FPGA) */
    g_wrbuf.vaddr = g_bar2.vaddr;
    g_wrbuf.paddr = g_bar2.paddr;
    g_wrbuf.size  = g_bar2.size / 2;

    printf("WrBuf region: vaddr=%p  paddr=0x%016lx  size=%lu bytes\n",
           g_wrbuf.vaddr, g_wrbuf.paddr, g_wrbuf.size);
    return 0;
}

/* --------------------------------------------------------------------------
 * NVMe probe / attach callbacks
 * -------------------------------------------------------------------------- */

static bool
probe_cb(void *cb_ctx, const struct spdk_nvme_transport_id *trid,
         struct spdk_nvme_ctrlr_opts *opts)
{
    (void)cb_ctx;
    (void)opts;

    printf("Probing NVMe at %s ...\n", trid->traddr);
    if (strcmp(trid->traddr, g_nvme_bdf) == 0) {
        printf("  -> accepting\n");
        return true;
    }
    return false;
}

static void
attach_cb(void *cb_ctx, const struct spdk_nvme_transport_id *trid,
          struct spdk_nvme_ctrlr *ctrlr,
          const struct spdk_nvme_ctrlr_opts *opts)
{
    int nsid;
    (void)cb_ctx;
    (void)trid;
    (void)opts;

    printf("Attached NVMe controller at %s\n", trid->traddr);

    g_ctrlr = ctrlr;

    nsid = spdk_nvme_ctrlr_get_first_active_ns(ctrlr);
    if (nsid <= 0) {
        fprintf(stderr, "ERROR: No active namespace found!\n");
        return;
    }
    g_ns = spdk_nvme_ctrlr_get_ns(ctrlr, nsid);
    if (g_ns == NULL) {
        fprintf(stderr, "ERROR: Failed to get namespace %d\n", nsid);
        return;
    }

    g_sect_size = spdk_nvme_ns_get_sector_size(g_ns);
    printf("Namespace %d: sector_size=%u  num_sectors=%lu\n",
           nsid, g_sect_size,
           (unsigned long)spdk_nvme_ns_get_num_sectors(g_ns));
}

/* --------------------------------------------------------------------------
 * Completion callbacks
 * -------------------------------------------------------------------------- */

static void
p2p_read_cb(void *cb_arg, const struct spdk_nvme_cpl *cpl)
{
    (void)cb_arg;

    if (spdk_nvme_cpl_is_error(cpl)) {
        fprintf(stderr, "NVMe READ completion ERROR: SC=%u SCT=%u\n",
                cpl->status.sc, cpl->status.sct);
        g_io_status = -1;
    } else {
        printf("NVMe READ completion: OK (SC=0 SCT=0)\n");
        g_io_status = 1;
    }
}

static void
ctrl_read_cb(void *cb_arg, const struct spdk_nvme_cpl *cpl)
{
    int *done = cb_arg;

    if (spdk_nvme_cpl_is_error(cpl)) {
        fprintf(stderr, "Control host-RAM READ ERROR: SC=%u SCT=%u\n",
                cpl->status.sc, cpl->status.sct);
        *done = -1;
    } else {
        *done = 1;
    }
}

/* --------------------------------------------------------------------------
 * Print 64 bytes of a buffer as hex
 * -------------------------------------------------------------------------- */

static void
print_hex64(const char *label, const uint8_t *buf, size_t avail)
{
    size_t  n  = avail < 64 ? avail : 64;
    size_t  i;
    bool    nonzero = false;

    printf("%s (first %zu bytes):\n  ", label, n);
    for (i = 0; i < n; i++) {
        printf("%02x ", buf[i]);
        if (buf[i]) {
            nonzero = true;
        }
        if ((i + 1) % 16 == 0) {
            printf("\n  ");
        }
    }
    printf("\n%s: region is %s\n", label, nonzero ? "NON-ZERO" : "ALL-ZERO");
}

/* --------------------------------------------------------------------------
 * main
 * -------------------------------------------------------------------------- */

int
main(int argc, char **argv)
{
    int                       rc;
    struct spdk_env_opts      opts;
    struct spdk_pci_driver   *ncd_driver;
    struct spdk_pci_addr      fpga_addr;
    uint64_t                  vtophys_result;
    uint64_t                  vtophys_n;
    struct timespec           ts_start, ts_now;
    double                    elapsed;
    uint8_t                  *ctrl_buf = NULL;
    int                       ctrl_done = 0;

    if (argc < 2) {
        fprintf(stderr, "Usage: %s <nvme-pci-addr>  (e.g. 0000:62:00.0)\n",
                argv[0]);
        return 1;
    }
    g_nvme_bdf = argv[1];

    /* ------------------------------------------------------------------ */
    printf("=== p2p_dir_test: NVMe=%s  FPGA=%s ===\n\n",
           g_nvme_bdf, FPGA_PF1_BDF);

    /* ------------------------------------------------------------------ */
    /* 1. SPDK env init */
    opts.opts_size = sizeof(opts);
    spdk_env_opts_init(&opts);
    opts.name = "p2p_dir_test";

    rc = spdk_env_init(&opts);
    if (rc < 0) {
        fprintf(stderr, "ERROR: spdk_env_init failed: %d\n", rc);
        return 1;
    }
    printf("SPDK env initialized\n");

    /* ------------------------------------------------------------------ */
    /* 2. Attach FPGA PF1 and map BAR2 */
    ncd_driver = spdk_pci_get_driver("ncd");
    if (ncd_driver == NULL) {
        fprintf(stderr, "ERROR: ncd PCI driver not found (SPDK_PCI_DRIVER_REGISTER failed?)\n");
        rc = -1;
        goto env_fini;
    }

    rc = spdk_pci_addr_parse(&fpga_addr, FPGA_PF1_BDF);
    if (rc) {
        fprintf(stderr, "ERROR: Failed to parse FPGA PCI addr '%s': %d\n",
                FPGA_PF1_BDF, rc);
        goto env_fini;
    }

    rc = spdk_pci_device_attach(ncd_driver, ncd_drv_attach_cb, NULL, &fpga_addr);
    if (rc) {
        fprintf(stderr, "ERROR: Failed to attach FPGA PCI device %s: %d\n",
                FPGA_PF1_BDF, rc);
        goto env_fini;
    }

    if (g_wrbuf.vaddr == NULL || g_wrbuf.paddr == 0) {
        fprintf(stderr, "ERROR: WrBuf BAR mapping is invalid after attach\n");
        rc = -1;
        goto env_fini;
    }

    /* ------------------------------------------------------------------ */
    /* 3. Try to register the BAR with SPDK memory map and check vtophys */
    printf("\n--- vtophys check ---\n");
    rc = spdk_mem_register(g_bar2.vaddr, g_bar2.size);
    if (rc) {
        printf("spdk_mem_register(bar2) returned %d — vtophys may not work for BAR\n", rc);
    } else {
        printf("spdk_mem_register(bar2): OK\n");
    }

    vtophys_n      = g_wrbuf.size;
    vtophys_result = spdk_vtophys(g_wrbuf.vaddr, &vtophys_n);

    printf("wrbuf_paddr (from map_bar) : 0x%016lx\n", g_wrbuf.paddr);
    printf("spdk_vtophys(wrbuf_vaddr)  : 0x%016lx\n", vtophys_result);

    if (vtophys_result != SPDK_VTOPHYS_ERROR &&
        vtophys_result == g_wrbuf.paddr) {
        printf("vtophys OK — using ns_cmd_read path\n");
        g_use_vtophys_path = true;
    } else if (vtophys_result != SPDK_VTOPHYS_ERROR) {
        printf("vtophys returned 0x%016lx but expected 0x%016lx — MISMATCH\n",
               vtophys_result, g_wrbuf.paddr);
        printf("Using io_raw with explicit PRP path\n");
        g_use_vtophys_path = false;
    } else {
        printf("vtophys returned SPDK_VTOPHYS_ERROR — using io_raw with explicit PRP path\n");
        g_use_vtophys_path = false;
    }

    /* ------------------------------------------------------------------ */
    /* 4. Probe and attach NVMe */
    printf("\n--- NVMe probe ---\n");
    rc = spdk_nvme_probe(NULL, NULL, probe_cb, attach_cb, NULL);
    if (rc) {
        fprintf(stderr, "ERROR: spdk_nvme_probe failed: %d\n", rc);
        goto env_fini;
    }
    if (g_ctrlr == NULL) {
        fprintf(stderr, "ERROR: NVMe controller at %s not found/attached\n", g_nvme_bdf);
        rc = -1;
        goto env_fini;
    }
    if (g_ns == NULL) {
        fprintf(stderr, "ERROR: No namespace attached\n");
        rc = -1;
        goto ctrlr_detach;
    }

    /* Allocate host IO qpair (SQ/CQ in host RAM — bypasses FPGA queue path) */
    g_qpair = spdk_nvme_ctrlr_alloc_io_qpair(g_ctrlr, NULL, 0);
    if (g_qpair == NULL) {
        fprintf(stderr, "ERROR: Failed to allocate host IO qpair\n");
        rc = -1;
        goto ctrlr_detach;
    }
    printf("Host IO qpair allocated\n");

    /* ------------------------------------------------------------------ */
    /* 5. Zero the WrBuf region via virtual address, then issue NVMe READ */
    printf("\n--- Test A: NVMe READ of LBA 0 -> FPGA WrBuf BAR ---\n");

    /* Baseline: read before any CPU writes */
    printf("WrBuf BEFORE memset (baseline FPGA state):\n");
    print_hex64("WrBuf[0..63] BASELINE", (const uint8_t *)g_wrbuf.vaddr, g_wrbuf.size);

    /* CPU write probe: write a sentinel pattern and read back */
    {
        uint32_t sentinel = 0xdeadbeef;
        volatile uint32_t *p = (volatile uint32_t *)g_wrbuf.vaddr;
        *p = sentinel;
        uint32_t readback = *p;
        printf("CPU write-probe: wrote 0x%08x, read back 0x%08x — BAR is %s\n",
               sentinel, readback,
               readback == sentinel ? "READ-WRITE (CPU reads reflect writes)" :
                                      "not directly readable by CPU (writes may still work for DMA)");
    }

    memset(g_wrbuf.vaddr, 0, g_wrbuf.size);
    printf("WrBuf after memset(0):\n");
    print_hex64("WrBuf[0..63] POST-ZERO", (const uint8_t *)g_wrbuf.vaddr, g_wrbuf.size);

    g_io_status = 0;

    if (g_use_vtophys_path) {
        /* ns_cmd_read: SPDK builds PRP from vtophys(wrbuf_vaddr) */
        rc = spdk_nvme_ns_cmd_read(g_ns, g_qpair,
                                   g_wrbuf.vaddr,
                                   0,   /* LBA start */
                                   1,   /* number of LBAs */
                                   p2p_read_cb, NULL, 0);
        if (rc) {
            fprintf(stderr, "ERROR: spdk_nvme_ns_cmd_read failed: %d\n", rc);
            goto qpair_free;
        }
        printf("Issued ns_cmd_read (vtophys path) to WrBuf BAR\n");
    } else {
        /* io_raw path: manually set PRP1 = wrbuf_paddr, pass buf=NULL */
        struct spdk_nvme_cmd cmd = {0};

        cmd.opc   = SPDK_NVME_OPC_READ;
        cmd.nsid  = spdk_nvme_ns_get_id(g_ns);
        /* cdw10 = starting LBA low 32 bits, cdw11 = starting LBA high 32 bits */
        cmd.cdw10 = 0;  /* LBA 0 low  */
        cmd.cdw11 = 0;  /* LBA 0 high */
        /* cdw12 bits[15:0] = NLB (0 = 1 block) */
        cmd.cdw12 = 0;  /* NLB=0 means 1 block */

        cmd.dptr.prp.prp1 = g_wrbuf.paddr;
        cmd.dptr.prp.prp2 = 0;

        rc = spdk_nvme_ctrlr_cmd_io_raw(g_ctrlr, g_qpair,
                                        &cmd, NULL, 0,
                                        p2p_read_cb, NULL);
        if (rc) {
            fprintf(stderr, "ERROR: spdk_nvme_ctrlr_cmd_io_raw failed: %d\n", rc);
            goto qpair_free;
        }
        printf("Issued io_raw READ (explicit PRP1=0x%016lx) to WrBuf BAR\n",
               g_wrbuf.paddr);
    }

    /* ------------------------------------------------------------------ */
    /* 6. Poll for completion with timeout */
    clock_gettime(CLOCK_MONOTONIC, &ts_start);
    while (g_io_status == 0) {
        spdk_nvme_qpair_process_completions(g_qpair, 0);

        clock_gettime(CLOCK_MONOTONIC, &ts_now);
        elapsed = (ts_now.tv_sec  - ts_start.tv_sec) +
                  (ts_now.tv_nsec - ts_start.tv_nsec) / 1e9;
        if (elapsed > TIMEOUT_SEC) {
            printf("TIMEOUT: no completion after %.1f s\n", elapsed);
            g_io_status = -2;
            break;
        }
    }

    /* ------------------------------------------------------------------ */
    /* 7. Read back WrBuf content and print results */
    printf("\n--- WrBuf readback (Test A result) ---\n");
    print_hex64("WrBuf[0..63]", (const uint8_t *)g_wrbuf.vaddr, g_wrbuf.size);

    if (g_io_status == 1) {
        /* Count non-zero bytes for a quick summary */
        size_t nz = 0, i;
        for (i = 0; i < (size_t)g_sect_size && i < g_wrbuf.size; i++) {
            if (((uint8_t *)g_wrbuf.vaddr)[i]) {
                nz++;
            }
        }
        printf("Non-zero bytes in first LBA's worth of WrBuf: %zu / %u\n",
               nz, g_sect_size);
    }

    /* ------------------------------------------------------------------ */
    /* 8. Control read: normal host-RAM read of LBA 0 */
    printf("\n--- Control: host-RAM NVMe READ of LBA 0 ---\n");
    ctrl_buf = spdk_zmalloc(g_sect_size, 0x1000, NULL,
                            SPDK_ENV_NUMA_ID_ANY, SPDK_MALLOC_DMA);
    if (ctrl_buf == NULL) {
        fprintf(stderr, "ERROR: Failed to allocate control DMA buffer\n");
        goto qpair_free;
    }

    ctrl_done = 0;
    rc = spdk_nvme_ns_cmd_read(g_ns, g_qpair, ctrl_buf,
                               0, 1, ctrl_read_cb, &ctrl_done, 0);
    if (rc) {
        fprintf(stderr, "ERROR: control ns_cmd_read failed: %d\n", rc);
        spdk_free(ctrl_buf);
        goto qpair_free;
    }

    clock_gettime(CLOCK_MONOTONIC, &ts_start);
    while (ctrl_done == 0) {
        spdk_nvme_qpair_process_completions(g_qpair, 0);
        clock_gettime(CLOCK_MONOTONIC, &ts_now);
        elapsed = (ts_now.tv_sec  - ts_start.tv_sec) +
                  (ts_now.tv_nsec - ts_start.tv_nsec) / 1e9;
        if (elapsed > TIMEOUT_SEC) {
            printf("TIMEOUT: control read no completion after %.1f s\n", elapsed);
            ctrl_done = -2;
            break;
        }
    }

    if (ctrl_done == 1) {
        print_hex64("CtrlBuf[0..63] (host-RAM LBA 0)", ctrl_buf, g_sect_size);
    } else {
        printf("Control read did not complete successfully (status=%d)\n", ctrl_done);
    }

    spdk_free(ctrl_buf);

    /* ------------------------------------------------------------------ */
    /* 9. Summary */
    printf("\n=== SUMMARY for NVMe %s ===\n", g_nvme_bdf);
    printf("vtophys path used : %s\n", g_use_vtophys_path ? "yes (ns_cmd_read)" : "no (io_raw+explicit PRP)");
    printf("Test A io_status  : %d (%s)\n", g_io_status,
           g_io_status == 1  ? "OK" :
           g_io_status == -1 ? "NVMe error" :
           g_io_status == -2 ? "TIMEOUT" : "pending/unknown");

    {
        bool nonzero = false;
        size_t i;
        for (i = 0; i < (size_t)g_sect_size && i < g_wrbuf.size; i++) {
            if (((uint8_t *)g_wrbuf.vaddr)[i]) {
                nonzero = true;
                break;
            }
        }
        printf("WrBuf after READ  : %s\n", nonzero ? "NON-ZERO (P2P data landed)" : "ALL-ZERO (P2P failed or data was all-zero)");
    }

qpair_free:
    if (g_qpair) {
        spdk_nvme_ctrlr_free_io_qpair(g_qpair);
        g_qpair = NULL;
    }
ctrlr_detach:
    if (g_ctrlr) {
        spdk_nvme_detach(g_ctrlr);
        g_ctrlr = NULL;
    }
env_fini:
    spdk_env_fini();
    printf("Done.\n");
    return (rc < 0) ? 1 : 0;
}
