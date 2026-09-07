/*
 * p2p_read_test.c: Test P2P non-posted MemRd: NVMe COMPARE with FPGA RdBuf BAR.
 * Copyright (C) 2025 Universitaet Heidelberg, Institut fuer Technische Informatik (ZITI)
 * Author(s): Vladislav Valek <vladislav.valek@stud.uni-heidelberg.de>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Test B -- NVMe COMPARE with FPGA RdBuf BAR as the data source.
 *   The drive must issue non-posted PCIe Memory Reads to the FPGA BAR to
 *   fetch the comparison buffer.  This is the decisive test for P2P-read
 *   capability: can the drive peer-read the FPGA BAR?
 *
 * Success: COMPARE completes SC=0x00 (match) or SC=0x85 (compare failure)
 *          -- either proves the drive successfully issued MemRd to the FPGA.
 * Failure: Timeout, SC=0x04 (Data Transfer Error), abort, AND/OR new
 *          UnsupReq / CplTimeout bits in AER UESta on any node.
 *
 * Usage:  sudo p2p_read_test <nvme-pci-addr>
 *   0000:62:00.0  Samsung 990 PRO
 *   0000:61:00.0  SK Hynix PC611
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdbool.h>

#include "spdk/stdinc.h"
#include "spdk/nvme.h"
#include "spdk/nvme_spec.h"
#include "spdk/env.h"
#include "spdk/log.h"
#include "spdk/memory.h"
#include "spdk/barrier.h"

/* ------------------------------------------------------------------ */
/* Fixed PCIe addresses                                                */
/* ------------------------------------------------------------------ */

#define FPGA_PF1_BDF    "0000:41:00.1"
#define FPGA_ROOT_PORT  "0000:40:01.1"

#define SAMSUNG_BDF     "0000:62:00.0"
#define SAMSUNG_RP_BDF  "0000:60:03.2"
#define HYNIX_BDF       "0000:61:00.0"
#define HYNIX_RP_BDF    "0000:60:03.1"

/* BAR2: WrBuf (lower 128 KiB, offset 0) + RdBuf (upper 128 KiB, offset 0x20000) */
#define FPGA_BAR_DATA   2

/* Poll timeout for NVMe completion */
#define TIMEOUT_SEC     2

/* Seed byte base value for RdBuf fill */
#define SEED_BASE       0xA5u

/* ------------------------------------------------------------------ */
/* NCD PCI driver registration for FPGA PF1                           */
/* ------------------------------------------------------------------ */

static struct spdk_pci_id g_ncd_pci_ids[] = {
    { SPDK_PCI_DEVICE(0x18ec, 0xc020) },
    { SPDK_PCI_DEVICE(0x1c2c, 0xc020) },
    { 0 }
};
SPDK_PCI_DRIVER_REGISTER(ncd, g_ncd_pci_ids, SPDK_PCI_DRIVER_NEED_MAPPING)

/* ------------------------------------------------------------------ */
/* Global state                                                        */
/* ------------------------------------------------------------------ */

static void     *g_bar2_vaddr  = NULL;
static uint64_t  g_bar2_paddr  = 0;
static uint64_t  g_bar2_size   = 0;

/* RdBuf: upper half of BAR2 */
static void     *g_rdbuf_vaddr = NULL;
static uint64_t  g_rdbuf_paddr = 0;

static struct spdk_nvme_ctrlr *g_ctrlr   = NULL;
static struct spdk_nvme_ns    *g_ns      = NULL;
static struct spdk_nvme_qpair *g_qpair   = NULL;
static uint32_t                g_sect_sz = 512;

static const char *g_nvme_bdf = NULL;

/*
 * g_io_status:
 *   0  = pending
 *   1  = done, SC=0x00 SCT=0x00 (success / data match)
 *   2  = done, SC=0x85 (compare failure, but drive read the BAR OK)
 *  -1  = done, NVMe error (transfer error, abort, etc.)
 *  -2  = timeout
 */
static int     g_io_status = 0;
static uint8_t g_cpl_sc    = 0;
static uint8_t g_cpl_sct   = 0;

/* ------------------------------------------------------------------ */
/* AER helpers via sysfs PCI config space                             */
/* ------------------------------------------------------------------ */

/*
 * AER Extended Capability register offsets (relative to cap base):
 *   +0x04  Uncorrectable Error Status (UESta)   RW1C
 *   +0x10  Correctable   Error Status (CESta)   RW1C
 */
#define AER_UESTA_OFF  0x04
#define AER_CESTA_OFF  0x10

/* Walk extended cap list (starts at 0x100), return AER cap offset or -1. */
static int
aer_find_cap(const char *bdf)
{
    char     path[256];
    int      fd;
    uint32_t hdr;
    int      off = 0x100;

    snprintf(path, sizeof(path), "/sys/bus/pci/devices/%s/config", bdf);
    fd = open(path, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "  [AER] open %s: %s\n", path, strerror(errno));
        return -1;
    }

    while (off >= 0x100 && off < 0x1000) {
        if (pread(fd, &hdr, 4, off) != 4) {
            break;
        }
        if (hdr == 0xFFFFFFFFu || hdr == 0u) {
            break;
        }
        if ((hdr & 0xFFFFu) == 0x0001u) {   /* AER extended cap ID = 0x0001 */
            close(fd);
            return off;
        }
        off = (int)((hdr >> 20) & 0xFFCu);
    }
    close(fd);
    return -1;
}

static uint32_t
aer_read32(const char *bdf, int cap, int reg_off)
{
    char     path[256];
    int      fd;
    uint32_t val = 0;

    snprintf(path, sizeof(path), "/sys/bus/pci/devices/%s/config", bdf);
    fd = open(path, O_RDWR);
    if (fd >= 0) {
        if (pread(fd, &val, 4, cap + reg_off) != 4) {
            val = 0;
        }
        close(fd);
    }
    return val;
}

static void
aer_write32(const char *bdf, int cap, int reg_off, uint32_t val)
{
    char path[256];
    int  fd;

    snprintf(path, sizeof(path), "/sys/bus/pci/devices/%s/config", bdf);
    fd = open(path, O_RDWR);
    if (fd >= 0) {
        pwrite(fd, &val, 4, cap + reg_off);
        close(fd);
    }
}

static const char *
uesta_bit_name(int bit)
{
    switch (bit) {
    case  4: return "DLP";
    case  5: return "SurpriseDown";
    case 12: return "PoisonedTLP";
    case 13: return "FlowCtrlProt";
    case 14: return "CplTimeout";
    case 15: return "CompleterAbort";
    case 16: return "UnexpectedCpl";
    case 17: return "RcvrOverflow";
    case 18: return "MalformedTLP";
    case 19: return "ECRC";
    case 20: return "UnsupReq";
    case 21: return "ACSViolation";
    case 22: return "UncorrIntErr";
    default: return "?";
    }
}

typedef struct {
    const char *bdf;
    const char *label;
    int         aer_cap;
    uint32_t    uesta_before;
    uint32_t    uesta_after;
    uint32_t    cesta_before;
    uint32_t    cesta_after;
} aer_dev_t;

static void
aer_snapshot_before(aer_dev_t *d)
{
    if (!d->bdf) {
        return;
    }
    d->aer_cap = aer_find_cap(d->bdf);
    if (d->aer_cap < 0) {
        printf("  [AER] %s (%s): no AER cap\n", d->label, d->bdf);
        return;
    }
    d->uesta_before = aer_read32(d->bdf, d->aer_cap, AER_UESTA_OFF);
    d->cesta_before = aer_read32(d->bdf, d->aer_cap, AER_CESTA_OFF);
    /* Clear both with 1s (RW1C) */
    aer_write32(d->bdf, d->aer_cap, AER_UESTA_OFF, 0xFFFFFFFFu);
    aer_write32(d->bdf, d->aer_cap, AER_CESTA_OFF, 0xFFFFFFFFu);
    printf("  [AER] %s (%s): cap=0x%03x  UESta_pre=0x%08x (cleared)  CESta_pre=0x%08x (cleared)\n",
           d->label, d->bdf, d->aer_cap, d->uesta_before, d->cesta_before);
}

static void
aer_snapshot_after(aer_dev_t *d)
{
    if (!d->bdf || d->aer_cap < 0) {
        return;
    }
    d->uesta_after = aer_read32(d->bdf, d->aer_cap, AER_UESTA_OFF);
    d->cesta_after = aer_read32(d->bdf, d->aer_cap, AER_CESTA_OFF);
}

static void
aer_print_delta(const aer_dev_t *d)
{
    int bit;

    if (!d->bdf) {
        return;
    }
    if (d->aer_cap < 0) {
        printf("  [AER] %s (%s): no cap -- skipped\n", d->label, d->bdf);
        return;
    }
    printf("  [AER] %s (%s): UESta=0x%08x  CESta=0x%08x",
           d->label, d->bdf, d->uesta_after, d->cesta_after);
    if (d->uesta_after == 0 && d->cesta_after == 0) {
        printf(" -- no new errors\n");
    } else {
        printf("\n");
        if (d->uesta_after) {
            printf("         -> NEW UESta:");
            for (bit = 0; bit < 32; bit++) {
                if (d->uesta_after & (1u << bit)) {
                    printf(" %s(b%d)", uesta_bit_name(bit), bit);
                }
            }
            printf("\n");
        }
        if (d->cesta_after) {
            printf("         -> NEW CESta: 0x%08x\n", d->cesta_after);
        }
    }
}

/* ------------------------------------------------------------------ */
/* FPGA NCD driver attach callback                                    */
/* ------------------------------------------------------------------ */

static int
ncd_drv_attach_cb(void *ctx, struct spdk_pci_device *pci_dev)
{
    int      rc;
    uint16_t cmd_reg;

    (void)ctx;

    spdk_pci_device_cfg_read16(pci_dev, &cmd_reg, 4);
    cmd_reg |= 0x0406u; /* BusMaster | MemEnable | INTxDisable */
    spdk_pci_device_cfg_write16(pci_dev, cmd_reg, 4);

    rc = spdk_pci_device_map_bar(pci_dev, FPGA_BAR_DATA,
                                 &g_bar2_vaddr, &g_bar2_paddr, &g_bar2_size);
    if (rc) {
        fprintf(stderr, "ERROR: spdk_pci_device_map_bar(%d) failed: %d\n",
                FPGA_BAR_DATA, rc);
        return rc;
    }

    /* RdBuf = upper half of BAR2 */
    g_rdbuf_vaddr = (uint8_t *)g_bar2_vaddr + g_bar2_size / 2;
    g_rdbuf_paddr = g_bar2_paddr + g_bar2_size / 2;

    printf("BAR%d mapped : vaddr=%p  paddr=0x%016lx  size=%lu bytes\n",
           FPGA_BAR_DATA, g_bar2_vaddr, g_bar2_paddr, g_bar2_size);
    printf("RdBuf region: vaddr=%p  paddr=0x%016lx  size=%lu bytes\n",
           g_rdbuf_vaddr, g_rdbuf_paddr, g_bar2_size / 2);
    return 0;
}

/* ------------------------------------------------------------------ */
/* NVMe probe / attach callbacks                                      */
/* ------------------------------------------------------------------ */

static bool
probe_cb(void *cb_ctx, const struct spdk_nvme_transport_id *trid,
         struct spdk_nvme_ctrlr_opts *opts)
{
    (void)cb_ctx;
    (void)opts;
    if (strcmp(trid->traddr, g_nvme_bdf) == 0) {
        printf("Probing NVMe at %s -> accepting\n", trid->traddr);
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
        fprintf(stderr, "ERROR: No active namespace\n");
        return;
    }
    g_ns = spdk_nvme_ctrlr_get_ns(ctrlr, nsid);
    if (!g_ns) {
        fprintf(stderr, "ERROR: Cannot get namespace %d\n", nsid);
        return;
    }
    g_sect_sz = spdk_nvme_ns_get_sector_size(g_ns);
    printf("Namespace %d: sector_size=%u  num_sectors=%lu\n",
           nsid, g_sect_sz,
           (unsigned long)spdk_nvme_ns_get_num_sectors(g_ns));
}

/* ------------------------------------------------------------------ */
/* COMPARE completion callback                                        */
/* ------------------------------------------------------------------ */

static void
compare_cb(void *cb_arg, const struct spdk_nvme_cpl *cpl)
{
    (void)cb_arg;

    g_cpl_sc  = cpl->status.sc;
    g_cpl_sct = cpl->status.sct;

    if (g_cpl_sc == 0 && g_cpl_sct == 0) {
        printf("COMPARE completion: SC=0x00 SCT=0x00 -- SUCCESS (data matched LBA 0)\n");
        g_io_status = 1;
    } else if (g_cpl_sc == SPDK_NVME_SC_COMPARE_FAILURE) {
        printf("COMPARE completion: SC=0x%02x SCT=0x%02x -- COMPARE FAILURE (mismatch: "
               "drive READ the FPGA BAR but data differed from LBA 0)\n",
               g_cpl_sc, g_cpl_sct);
        g_io_status = 2;
    } else {
        printf("COMPARE completion: SC=0x%02x SCT=0x%02x -- ERROR\n",
               g_cpl_sc, g_cpl_sct);
        g_io_status = -1;
    }
}

/* ------------------------------------------------------------------ */
/* main                                                               */
/* ------------------------------------------------------------------ */

int
main(int argc, char **argv)
{
    int                     rc;
    struct spdk_env_opts    opts;
    struct spdk_pci_driver *ncd_driver;
    struct spdk_pci_addr    fpga_addr;
    struct timespec         ts_start, ts_now;
    double                  elapsed;
    size_t                  seed_bytes;
    size_t                  i;

    /* AER-monitored devices (label, bdf, aer_cap initialised below) */
    aer_dev_t aer_fpga     = { FPGA_PF1_BDF,   "FPGA",     -1, 0, 0, 0, 0 };
    aer_dev_t aer_fpga_rp  = { FPGA_ROOT_PORT,  "FPGA-RP",  -1, 0, 0, 0, 0 };
    aer_dev_t aer_drive    = { NULL,            "Drive",    -1, 0, 0, 0, 0 };
    aer_dev_t aer_drive_rp = { NULL,            "Drive-RP", -1, 0, 0, 0, 0 };

    if (argc < 2) {
        fprintf(stderr,
                "Usage: %s <nvme-pci-addr>\n"
                "  sudo %s 0000:62:00.0   (Samsung 990 PRO)\n"
                "  sudo %s 0000:61:00.0   (SK Hynix PC611)\n",
                argv[0], argv[0], argv[0]);
        return 1;
    }
    g_nvme_bdf = argv[1];

    /* Resolve drive-side AER nodes */
    aer_drive.bdf = g_nvme_bdf;
    if (strcmp(g_nvme_bdf, SAMSUNG_BDF) == 0) {
        aer_drive.label    = "Samsung";
        aer_drive_rp.bdf   = SAMSUNG_RP_BDF;
        aer_drive_rp.label = "Samsung-RP";
    } else if (strcmp(g_nvme_bdf, HYNIX_BDF) == 0) {
        aer_drive.label    = "Hynix";
        aer_drive_rp.bdf   = HYNIX_RP_BDF;
        aer_drive_rp.label = "Hynix-RP";
    } else {
        aer_drive.label    = "Drive";
        aer_drive_rp.bdf   = NULL;   /* unknown RP, skip */
        fprintf(stderr, "WARNING: unknown NVMe BDF %s -- drive RP AER skipped\n", g_nvme_bdf);
    }

    printf("=== p2p_read_test: NVMe=%s  FPGA=%s ===\n\n", g_nvme_bdf, FPGA_PF1_BDF);

    /* ---------------------------------------------------------------- */
    /* 1. SPDK env init                                                 */
    opts.opts_size = sizeof(opts);
    spdk_env_opts_init(&opts);
    opts.name = "p2p_read_test";

    rc = spdk_env_init(&opts);
    if (rc < 0) {
        fprintf(stderr, "ERROR: spdk_env_init failed: %d\n", rc);
        return 1;
    }
    printf("SPDK env initialized\n");

    /* ---------------------------------------------------------------- */
    /* 2. Attach FPGA PF1 and map BAR2                                  */
    ncd_driver = spdk_pci_get_driver("ncd");
    if (!ncd_driver) {
        fprintf(stderr, "ERROR: ncd PCI driver not registered\n");
        rc = -1;
        goto env_fini;
    }

    rc = spdk_pci_addr_parse(&fpga_addr, FPGA_PF1_BDF);
    if (rc) {
        fprintf(stderr, "ERROR: cannot parse '%s': %d\n", FPGA_PF1_BDF, rc);
        goto env_fini;
    }

    rc = spdk_pci_device_attach(ncd_driver, ncd_drv_attach_cb, NULL, &fpga_addr);
    if (rc) {
        fprintf(stderr, "ERROR: attach FPGA %s failed: %d\n", FPGA_PF1_BDF, rc);
        goto env_fini;
    }

    if (!g_rdbuf_vaddr || !g_rdbuf_paddr) {
        fprintf(stderr, "ERROR: RdBuf BAR mapping invalid after attach\n");
        rc = -1;
        goto env_fini;
    }

    /* ---------------------------------------------------------------- */
    /* 3. Probe and attach NVMe (needed before seeding so we know       */
    /*    g_sect_sz for sizing the seed region)                         */
    printf("\n--- NVMe probe ---\n");
    rc = spdk_nvme_probe(NULL, NULL, probe_cb, attach_cb, NULL);
    if (rc || !g_ctrlr || !g_ns) {
        fprintf(stderr, "ERROR: NVMe probe/attach failed (rc=%d ctrlr=%p ns=%p)\n",
                rc, (void *)g_ctrlr, (void *)g_ns);
        rc = -1;
        goto env_fini;
    }

    g_qpair = spdk_nvme_ctrlr_alloc_io_qpair(g_ctrlr, NULL, 0);
    if (!g_qpair) {
        fprintf(stderr, "ERROR: failed to alloc host IO qpair\n");
        rc = -1;
        goto ctrlr_detach;
    }
    printf("Host IO qpair allocated\n");

    /* ---------------------------------------------------------------- */
    /* 4. Seed FPGA RdBuf with incrementing pattern via CPU MMIO writes */
    /*    (at least one sector's worth so COMPARE has valid data)        */
    seed_bytes = g_sect_sz;
    if (seed_bytes < 512) {
        seed_bytes = 512;
    }
    if (seed_bytes > g_bar2_size / 2) {
        seed_bytes = g_bar2_size / 2;
    }

    printf("\n--- Seeding FPGA RdBuf (%zu bytes @ paddr=0x%016lx) ---\n",
           seed_bytes, g_rdbuf_paddr);
    {
        volatile uint8_t *p = (volatile uint8_t *)g_rdbuf_vaddr;
        for (i = 0; i < seed_bytes; i++) {
            p[i] = (uint8_t)(SEED_BASE + (i & 0xFFu));
        }
    }
    /* Ensure all write-combined writes are flushed before the doorbell */
    spdk_wmb();

    /* Readback a few bytes (write-combined -- may show 0, that's normal) */
    printf("RdBuf[0..7] CPU readback (WC, may be 0): ");
    {
        volatile uint8_t *p = (volatile uint8_t *)g_rdbuf_vaddr;
        for (i = 0; i < 8 && i < seed_bytes; i++) {
            printf("%02x ", p[i]);
        }
    }
    printf("\n");

    /* ---------------------------------------------------------------- */
    /* 5. AER: read current state, then clear, right before the command */
    printf("\n--- AER snapshot BEFORE COMPARE ---\n");
    aer_snapshot_before(&aer_fpga);
    aer_snapshot_before(&aer_fpga_rp);
    aer_snapshot_before(&aer_drive);
    if (aer_drive_rp.bdf) {
        aer_snapshot_before(&aer_drive_rp);
    }

    /* ---------------------------------------------------------------- */
    /* 6. Issue NVMe COMPARE via io_raw with PRP1 = rdbuf_paddr         */
    /*    The drive must issue non-posted PCIe MemRd to the FPGA BAR    */
    /*    to obtain the comparison data.                                 */
    printf("\n--- Test B: NVMe COMPARE LBA 0 <- FPGA RdBuf (paddr=0x%016lx) ---\n",
           g_rdbuf_paddr);
    {
        struct spdk_nvme_cmd cmd;
        memset(&cmd, 0, sizeof(cmd));

        cmd.opc  = SPDK_NVME_OPC_COMPARE;          /* 0x05 */
        cmd.nsid = spdk_nvme_ns_get_id(g_ns);
        cmd.cdw10 = 0;  /* starting LBA bits[31:0]  = 0 */
        cmd.cdw11 = 0;  /* starting LBA bits[63:32] = 0 */
        cmd.cdw12 = 0;  /* NLB bits[15:0] = 0 -> 1 block */
        cmd.dptr.prp.prp1 = g_rdbuf_paddr;
        cmd.dptr.prp.prp2 = 0;

        g_io_status = 0;

        rc = spdk_nvme_ctrlr_cmd_io_raw(g_ctrlr, g_qpair,
                                        &cmd, NULL, 0,
                                        compare_cb, NULL);
        if (rc) {
            fprintf(stderr, "ERROR: spdk_nvme_ctrlr_cmd_io_raw failed: %d\n", rc);
            goto qpair_free;
        }
        printf("COMPARE submitted (opc=0x05, nsid=%u, LBA=0, NLB=1, PRP1=0x%016lx)\n",
               cmd.nsid, g_rdbuf_paddr);
    }

    /* ---------------------------------------------------------------- */
    /* 7. Poll for completion with timeout                               */
    clock_gettime(CLOCK_MONOTONIC, &ts_start);
    while (g_io_status == 0) {
        spdk_nvme_qpair_process_completions(g_qpair, 0);

        clock_gettime(CLOCK_MONOTONIC, &ts_now);
        elapsed = (double)(ts_now.tv_sec  - ts_start.tv_sec) +
                  (double)(ts_now.tv_nsec - ts_start.tv_nsec) / 1e9;
        if (elapsed > TIMEOUT_SEC) {
            printf("TIMEOUT: no COMPARE completion after %.1f s\n", elapsed);
            g_io_status = -2;
            break;
        }
    }

    /* ---------------------------------------------------------------- */
    /* 8. AER: snapshot AFTER the command                                */
    printf("\n--- AER snapshot AFTER COMPARE ---\n");
    aer_snapshot_after(&aer_fpga);
    aer_snapshot_after(&aer_fpga_rp);
    aer_snapshot_after(&aer_drive);
    if (aer_drive_rp.bdf) {
        aer_snapshot_after(&aer_drive_rp);
    }

    /* ---------------------------------------------------------------- */
    /* 9. Print results                                                   */
    printf("\n========================================\n");
    printf("=== RESULT: NVMe %s ===\n", g_nvme_bdf);
    printf("========================================\n");

    switch (g_io_status) {
    case 1:
        printf("COMPARE SC/SCT : 0x00/0x00 -- SUCCESS (data matched)\n");
        printf("PEER-READ      : YES (drive read FPGA RdBuf; coincidentally matched LBA 0)\n");
        break;
    case 2:
        printf("COMPARE SC/SCT : 0x%02x/0x%02x -- COMPARE FAILURE (data mismatch)\n",
               g_cpl_sc, g_cpl_sct);
        printf("PEER-READ      : YES (drive successfully read FPGA RdBuf; mismatch is expected)\n");
        break;
    case -1:
        printf("COMPARE SC/SCT : 0x%02x/0x%02x -- NVMe ERROR\n", g_cpl_sc, g_cpl_sct);
        printf("PEER-READ      : FAILED (NVMe error during COMPARE transfer)\n");
        break;
    case -2:
        printf("COMPARE SC/SCT : TIMEOUT (no completion in %d s)\n", TIMEOUT_SEC);
        printf("PEER-READ      : FAILED (command did not complete)\n");
        break;
    default:
        printf("COMPARE SC/SCT : unknown io_status=%d\n", g_io_status);
        break;
    }

    printf("\nAER delta (new errors set by the COMPARE command):\n");
    aer_print_delta(&aer_fpga);
    aer_print_delta(&aer_fpga_rp);
    aer_print_delta(&aer_drive);
    if (aer_drive_rp.bdf) {
        aer_print_delta(&aer_drive_rp);
    }

    /* Summary interpretation */
    {
        bool     peer_ok  = (g_io_status == 1 || g_io_status == 2);
        uint32_t ur_bit   = (1u << 20);
        uint32_t cto_bit  = (1u << 14);
        bool     any_ur   = ((aer_fpga.uesta_after  & ur_bit) != 0) ||
                            ((aer_fpga_rp.uesta_after & ur_bit) != 0) ||
                            ((aer_drive.uesta_after  & ur_bit) != 0) ||
                            (aer_drive_rp.bdf && ((aer_drive_rp.uesta_after & ur_bit) != 0));
        bool     any_cto  = ((aer_fpga.uesta_after  & cto_bit) != 0) ||
                            ((aer_fpga_rp.uesta_after & cto_bit) != 0) ||
                            ((aer_drive.uesta_after  & cto_bit) != 0) ||
                            (aer_drive_rp.bdf && ((aer_drive_rp.uesta_after & cto_bit) != 0));

        printf("\nConclusion:\n");
        if (peer_ok && !any_ur && !any_cto) {
            printf("  PEER-READ SUCCEEDED -- drive can issue non-posted MemRd to FPGA BAR, no AER errors\n");
        } else if (!peer_ok && (any_ur || any_cto)) {
            printf("  PEER-READ FAILED -- command timed out or errored; AER shows:\n");
            if (any_ur)  { printf("    UnsupReq set on at least one node\n"); }
            if (any_cto) { printf("    CplTimeout set on at least one node\n"); }
        } else if (peer_ok && (any_ur || any_cto)) {
            printf("  PEER-READ COMPLETED but AER errors also set -- investigate\n");
        } else {
            printf("  AMBIGUOUS -- no AER errors but command did not complete normally\n");
        }
    }
    printf("========================================\n");

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
    return (g_io_status > 0) ? 0 : 1;
}
