/*
 *   p2p_bw.c: raw P2P NVMe read bandwidth into the Alveo U55C FPGA PF1 BAR2 vs. host DRAM
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) Vladislav Válek
 */

/** \file
 * Measures raw peer-to-peer NVMe read bandwidth when the submission/completion queues live in
 * host DRAM (ordinary SPDK qpairs, host-rung doorbells) but a command's data PRPs point into the
 * card's PF1 BAR2, so the payload travels P2P into the card and the host never touches it.
 *
 * Two independent PCI drivers are registered for the card's two PF1 endpoints (EP0 18ec:c020,
 * EP1 18ec:c022); only the endpoint(s) a run actually targets get attached and mapped. Each
 * task's buffer is a CEIL(io_size, 4096)-aligned slice of a 2 MiB-aligned window registered with
 * spdk_mem_register() inside BAR2 so spdk_vtophys() resolves it to the BAR's physical range and
 * spdk_nvme_ns_cmd_read() builds PRPs that point straight at the card (see nvme_pcie_common.c
 * nvme_pcie_prp_list_append() and the CMB template in nvme_pcie.c nvme_pcie_ctrlr_map_io_cmb()).
 */

#include <unistd.h>
static char g_env_name[32];
#include "spdk/stdinc.h"
#include "spdk/nvme.h"
#include "spdk/nvme_spec.h"
#include "spdk/env.h"
#include "spdk/log.h"
#include "spdk/memory.h"
#include "spdk/barrier.h"

#include <getopt.h>
#include <signal.h>
#include <dirent.h>
#include <inttypes.h>

/* ------------------------------------------------------------------ */
/* Fixed platform facts                                                */
/* ------------------------------------------------------------------ */

#define P2PBW_OS_DRIVE_BDF     "0000:2c:00.0"
#define P2PBW_DEFAULT_EP0_BDF  "0000:41:00.1"
#define P2PBW_DEFAULT_EP1_BDF  "0000:42:00.1"
#define P2PBW_VENDOR_ID        0x18ecu
#define P2PBW_EP0_DEVICE_ID    0xc020u
#define P2PBW_EP1_DEVICE_ID    0xc022u
#define P2PBW_BAR_IDX           2

#define P2PBW_MAX_DRIVES        8
#define P2PBW_DEFAULT_QD        64
#define P2PBW_DEFAULT_IO_SIZE   (128u * 1024u)
#define P2PBW_DEFAULT_DURATION  20
#define P2PBW_DEFAULT_CORE_MASK "0x2"
#define P2PBW_GBPS_DIVISOR      1e9

#define CEIL_4KB(x)  (((uint64_t)(x) + VALUE_4KB - 1) & ~(uint64_t)MASK_4KB)

/* ------------------------------------------------------------------ */
/* Endpoints                                                           */
/* ------------------------------------------------------------------ */

enum p2pbw_dst {
    P2PBW_DST_DRAM = 0,
    P2PBW_DST_EP0  = 1,
    P2PBW_DST_EP1  = 2,
};

struct p2pbw_endpoint {
    const char              *bdf;
    uint16_t                 device_id;
    bool                     needed;
    bool                     attached;
    struct spdk_pci_device  *pci_dev;
    void                    *bar_va;
    uint64_t                 bar_pa;
    uint64_t                 bar_size;
    void                    *reg_va;
    uint64_t                 reg_pa;
    size_t                   reg_len;
    uint64_t                 rot_cursor;
};

static struct p2pbw_endpoint g_ep[2]; /* [0] = EP0, [1] = EP1 */

/* ------------------------------------------------------------------ */
/* Drives / tasks                                                      */
/* ------------------------------------------------------------------ */

struct p2pbw_drive;

struct p2pbw_task {
    struct p2pbw_drive *drive;
    void                *buf;
    uint64_t             submit_tsc;
};

struct p2pbw_drive {
    char                     traddr[SPDK_NVMF_TRADDR_MAX_LEN + 1];
    enum p2pbw_dst           dst;
    struct spdk_nvme_ctrlr  *ctrlr;
    struct spdk_nvme_ns     *ns;
    struct spdk_nvme_qpair  *qpair;
    uint32_t                 sector_size;
    uint32_t                 lba_count;
    uint64_t                 num_ios;    /* random granule index range, exclusive upper bound */
    uint64_t                 rng_state;
    struct p2pbw_task       *tasks;
    uint32_t                 qd;
    uint64_t                 outstanding;

    uint64_t                 completions;
    uint64_t                 bytes;
    uint64_t                 errors;
    bool                     has_error;
    uint8_t                  first_err_sct;
    uint8_t                  first_err_sc;
    uint64_t                 sum_tsc;
    uint64_t                 min_tsc;
    uint64_t                 max_tsc;

    bool                     draining;
    bool                     window_closed;
    uint64_t                 window_start_tsc;
    uint64_t                 window_end_tsc;
    uint64_t                 window_completions;
    uint64_t                 window_bytes;
};

static struct p2pbw_drive g_drives[P2PBW_MAX_DRIVES];
static uint32_t            g_num_drives;

/* ------------------------------------------------------------------ */
/* Options                                                             */
/* ------------------------------------------------------------------ */

static uint32_t     g_qd            = P2PBW_DEFAULT_QD;
static uint32_t     g_io_size       = P2PBW_DEFAULT_IO_SIZE;
static uint32_t     g_duration_s    = P2PBW_DEFAULT_DURATION;
static uint64_t     g_num_ios_opt   = 0; /* -n, 0 = time based */
static const char  *g_core_mask     = P2PBW_DEFAULT_CORE_MASK;
static uint64_t     g_offset_opt    = 0;
static bool         g_rotate        = false; /* --rotate: cycle every command over the whole window */
static const char  *g_ep_bdf[2]     = { P2PBW_DEFAULT_EP0_BDF, P2PBW_DEFAULT_EP1_BDF };
static const char  *g_dst_arg       = "dram";

static volatile sig_atomic_t g_stop;
static uint64_t g_empty_polls;
static uint64_t g_nonempty_polls;

static void
sig_handler(int signo)
{
    if (signo == SIGINT || signo == SIGTERM) {
        g_stop = 1;
    }
}

/* ------------------------------------------------------------------ */
/* PCI driver registration for the card's two PF1 endpoints            */
/* ------------------------------------------------------------------ */

static struct spdk_pci_id g_ep0_pci_ids[] = {
    { SPDK_PCI_DEVICE(P2PBW_VENDOR_ID, P2PBW_EP0_DEVICE_ID) },
    { 0 }
};
SPDK_PCI_DRIVER_REGISTER(p2pbw_ep0, g_ep0_pci_ids, SPDK_PCI_DRIVER_NEED_MAPPING)

static struct spdk_pci_id g_ep1_pci_ids[] = {
    { SPDK_PCI_DEVICE(P2PBW_VENDOR_ID, P2PBW_EP1_DEVICE_ID) },
    { 0 }
};
SPDK_PCI_DRIVER_REGISTER(p2pbw_ep1, g_ep1_pci_ids, SPDK_PCI_DRIVER_NEED_MAPPING)

static int
ep_attach_cb(void *ctx, struct spdk_pci_device *pci_dev)
{
    struct p2pbw_endpoint *ep = ctx;
    struct spdk_pci_id     id = spdk_pci_device_get_id(pci_dev);
    uint16_t                cmd_reg;
    int                     rc;

    if (id.vendor_id != P2PBW_VENDOR_ID || id.device_id != ep->device_id) {
        fprintf(stderr, "ERROR: %s: unexpected PCI id %04x:%04x (want %04x:%04x)\n",
                ep->bdf, id.vendor_id, id.device_id, P2PBW_VENDOR_ID, ep->device_id);
        return -1;
    }

    spdk_pci_device_cfg_read16(pci_dev, &cmd_reg, 4);
    /* BusMaster | MemEnable | INTxDisable: matches p2p_read_test.c and the platform's documented
     * "setpci COMMAND=0x406" bring-up value; a plain 0x404 leaves bit1 (Memory Space Enable)
     * unset and would only be safe if a prior devbind step already enabled it. */
    cmd_reg |= 0x0406u;
    spdk_pci_device_cfg_write16(pci_dev, cmd_reg, 4);

    rc = spdk_pci_device_map_bar(pci_dev, P2PBW_BAR_IDX, &ep->bar_va, &ep->bar_pa, &ep->bar_size);
    if (rc != 0) {
        fprintf(stderr, "ERROR: %s: spdk_pci_device_map_bar(%d) failed: %d\n",
                ep->bdf, P2PBW_BAR_IDX, rc);
        return -1;
    }
    if (ep->bar_va == NULL || ep->bar_pa == 0 || ep->bar_size == 0) {
        fprintf(stderr, "ERROR: %s: BAR%d mapping invalid (va=%p pa=0x%" PRIx64 " size=%" PRIu64 ")\n",
                ep->bdf, P2PBW_BAR_IDX, ep->bar_va, ep->bar_pa, ep->bar_size);
        return -1;
    }

    ep->pci_dev  = pci_dev;
    ep->attached = true;
    printf("Attached %s: BAR%d vaddr=%p paddr=0x%016" PRIx64 " size=%" PRIu64 " bytes\n",
           ep->bdf, P2PBW_BAR_IDX, ep->bar_va, ep->bar_pa, ep->bar_size);
    return 0;
}

/* ------------------------------------------------------------------ */
/* fpga_zero_copy guard                                                */
/* ------------------------------------------------------------------ */

static int
refuse_if_fzc_running(void)
{
    DIR           *dir;
    struct dirent *ent;
    pid_t          self = getpid();

    dir = opendir("/proc");
    if (dir == NULL) {
        return 0; /* can't check, don't block on it */
    }

    while ((ent = readdir(dir)) != NULL) {
        pid_t pid;
        char  comm_path[64];
        char  comm[64];
        FILE *f;

        if (ent->d_name[0] < '0' || ent->d_name[0] > '9') {
            continue;
        }
        pid = (pid_t)atoi(ent->d_name);
        if (pid == self) {
            continue;
        }
        snprintf(comm_path, sizeof(comm_path), "/proc/%d/comm", pid);
        f = fopen(comm_path, "r");
        if (f == NULL) {
            continue;
        }
        comm[0] = '\0';
        if (fgets(comm, sizeof(comm), f) != NULL) {
            size_t len = strlen(comm);
            if (len > 0 && comm[len - 1] == '\n') {
                comm[len - 1] = '\0';
            }
        }
        fclose(f);
        if (strcmp(comm, "fpga_zero_copy") == 0) {
            closedir(dir);
            fprintf(stderr,
                    "ERROR: fpga_zero_copy is running (pid %d); stop it with ~/temp/stop_fzc.sh first\n",
                    pid);
            return -1;
        }
    }
    closedir(dir);
    return 0;
}

/* ------------------------------------------------------------------ */
/* CLI                                                                  */
/* ------------------------------------------------------------------ */

static void
usage(const char *prog)
{
    printf("%s [options] -r <traddr> [-r <traddr> ...]\n", prog);
    printf("Raw P2P NVMe read bandwidth into an Alveo U55C PF1 BAR2 vs. host DRAM.\n\n");
    printf("  -r <traddr>       NVMe PCIe address, repeatable, order matters (required)\n");
    printf("  -q <n>            queue depth per drive (default: %u)\n", P2PBW_DEFAULT_QD);
    printf("  -o <bytes>        I/O size (default: %u)\n", P2PBW_DEFAULT_IO_SIZE);
    printf("  -t <seconds>      run duration, ignored if -n is given (default: %u)\n",
           P2PBW_DEFAULT_DURATION);
    printf("  -n <count>        completions per drive, 0 = time-based (default: 0)\n");
    printf("  -c <mask>         DPDK core mask (default: %s)\n", P2PBW_DEFAULT_CORE_MASK);
    printf("  --dst <spec>      dram|ep0|ep1|split|<comma list of 0/1 per drive> (default: dram)\n");
    printf("  --offset <bytes>  byte offset into BAR2 for the registered window, rounded up\n");
    printf("                    to 2 MiB (default: 0)\n");
    printf("  --ep0 <bdf>       EP0 PF1 BDF (default: %s)\n", P2PBW_DEFAULT_EP0_BDF);
    printf("  --ep1 <bdf>       EP1 PF1 BDF (default: %s)\n", P2PBW_DEFAULT_EP1_BDF);
    printf("  -h                show this help\n");
}

enum p2pbw_long_opt {
    P2PBW_OPT_DST = 256,
    P2PBW_OPT_OFFSET,
    P2PBW_OPT_EP0,
    P2PBW_OPT_EP1,
    P2PBW_OPT_ROTATE,
};

#define P2PBW_GETOPT_STRING "r:q:o:t:n:c:h"

static const struct option g_p2pbw_long_opts[] = {
    { "dst",    required_argument, NULL, P2PBW_OPT_DST },
    { "offset", required_argument, NULL, P2PBW_OPT_OFFSET },
    { "ep0",    required_argument, NULL, P2PBW_OPT_EP0 },
    { "ep1",    required_argument, NULL, P2PBW_OPT_EP1 },
    { "rotate", no_argument,       NULL, P2PBW_OPT_ROTATE },
    { "help",   no_argument,       NULL, 'h' },
    { 0, 0, 0, 0 }
};

static int
parse_args(int argc, char **argv)
{
    int op, opt_index;

    while ((op = getopt_long(argc, argv, P2PBW_GETOPT_STRING, g_p2pbw_long_opts, &opt_index)) != -1) {
        switch (op) {
        case 'r':
            if (strcmp(optarg, P2PBW_OS_DRIVE_BDF) == 0) {
                fprintf(stderr, "ERROR: refusing to touch the OS drive %s\n", P2PBW_OS_DRIVE_BDF);
                return -1;
            }
            if (g_num_drives >= P2PBW_MAX_DRIVES) {
                fprintf(stderr, "ERROR: at most %u drives are supported\n", P2PBW_MAX_DRIVES);
                return -1;
            }
            snprintf(g_drives[g_num_drives].traddr, sizeof(g_drives[g_num_drives].traddr),
                     "%s", optarg);
            g_num_drives++;
            break;
        case 'q':
            g_qd = (uint32_t)strtoul(optarg, NULL, 0);
            break;
        case 'o':
            g_io_size = (uint32_t)strtoul(optarg, NULL, 0);
            break;
        case 't':
            g_duration_s = (uint32_t)strtoul(optarg, NULL, 0);
            break;
        case 'n':
            g_num_ios_opt = strtoull(optarg, NULL, 0);
            break;
        case 'c':
            g_core_mask = optarg;
            break;
        case P2PBW_OPT_DST:
            g_dst_arg = optarg;
            break;
        case P2PBW_OPT_OFFSET:
            g_offset_opt = strtoull(optarg, NULL, 0);
            break;
        case P2PBW_OPT_EP0:
            g_ep_bdf[0] = optarg;
            break;
        case P2PBW_OPT_ROTATE:
            g_rotate = true;
            break;
        case P2PBW_OPT_EP1:
            g_ep_bdf[1] = optarg;
            break;
        case 'h':
            usage(argv[0]);
            exit(0);
        default:
            usage(argv[0]);
            return -1;
        }
    }

    if (g_num_drives == 0) {
        fprintf(stderr, "ERROR: at least one -r <traddr> is required\n");
        return -1;
    }
    if (g_qd == 0 || g_io_size == 0) {
        fprintf(stderr, "ERROR: -q and -o must be non-zero\n");
        return -1;
    }
    return 0;
}

/* Resolves g_dst_arg into per-drive g_drives[i].dst and marks the endpoints a run needs. */
static int
resolve_dst(void)
{
    uint32_t i;

    if (strcmp(g_dst_arg, "dram") == 0) {
        for (i = 0; i < g_num_drives; i++) {
            g_drives[i].dst = P2PBW_DST_DRAM;
        }
    } else if (strcmp(g_dst_arg, "ep0") == 0) {
        for (i = 0; i < g_num_drives; i++) {
            g_drives[i].dst = P2PBW_DST_EP0;
        }
        g_ep[0].needed = true;
    } else if (strcmp(g_dst_arg, "ep1") == 0) {
        for (i = 0; i < g_num_drives; i++) {
            g_drives[i].dst = P2PBW_DST_EP1;
        }
        g_ep[1].needed = true;
    } else if (strcmp(g_dst_arg, "split") == 0) {
        uint32_t n_ep0 = (g_num_drives + 1) / 2;

        for (i = 0; i < g_num_drives; i++) {
            g_drives[i].dst = (i < n_ep0) ? P2PBW_DST_EP0 : P2PBW_DST_EP1;
        }
        g_ep[0].needed = true;
        if (n_ep0 < g_num_drives) {
            g_ep[1].needed = true;
        }
    } else {
        /* comma-separated list of 0 (ep0) / 1 (ep1), one entry per -r drive, in order */
        char       buf[256];
        char      *tok, *save = NULL;

        snprintf(buf, sizeof(buf), "%s", g_dst_arg);
        i = 0;
        tok = strtok_r(buf, ",", &save);
        while (tok != NULL) {
            if (i >= g_num_drives) {
                fprintf(stderr, "ERROR: --dst list has more entries than -r drives\n");
                return -1;
            }
            if (strcmp(tok, "0") == 0) {
                g_drives[i].dst = P2PBW_DST_EP0;
                g_ep[0].needed = true;
            } else if (strcmp(tok, "1") == 0) {
                g_drives[i].dst = P2PBW_DST_EP1;
                g_ep[1].needed = true;
            } else {
                fprintf(stderr, "ERROR: bad --dst entry '%s' (want 0 or 1)\n", tok);
                return -1;
            }
            i++;
            tok = strtok_r(NULL, ",", &save);
        }
        if (i != g_num_drives) {
            fprintf(stderr, "ERROR: --dst list has %u entries, expected %u\n", i, g_num_drives);
            return -1;
        }
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* xorshift64 RNG for random LBA selection                             */
/* ------------------------------------------------------------------ */

static inline uint64_t
xorshift64(uint64_t *state)
{
    uint64_t x = *state;

    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    *state = x;
    return x;
}

/* ------------------------------------------------------------------ */
/* Completion handling                                                 */
/* ------------------------------------------------------------------ */

static int submit_task(struct p2pbw_task *task);

static void
io_complete(void *ctx, const struct spdk_nvme_cpl *cpl)
{
    struct p2pbw_task  *task  = ctx;
    struct p2pbw_drive *drive = task->drive;
    uint64_t             tsc_diff = spdk_get_ticks() - task->submit_tsc;

    drive->outstanding--;
    drive->completions++;
    drive->bytes += g_io_size;
    drive->sum_tsc += tsc_diff;
    if (drive->min_tsc == 0 || tsc_diff < drive->min_tsc) {
        drive->min_tsc = tsc_diff;
    }
    if (tsc_diff > drive->max_tsc) {
        drive->max_tsc = tsc_diff;
    }

    if (!drive->window_closed) {
        drive->window_completions++;
        drive->window_bytes += g_io_size;
    }

    if (spdk_nvme_cpl_is_error(cpl)) {
        drive->errors++;
        if (!drive->has_error) {
            drive->has_error      = true;
            drive->first_err_sct  = cpl->status.sct;
            drive->first_err_sc   = cpl->status.sc;
        }
    }

    if (!drive->draining) {
        submit_task(task);
    }
}

static int
submit_task(struct p2pbw_task *task)
{
    struct p2pbw_drive *drive = task->drive;
    uint64_t             idx  = xorshift64(&drive->rng_state) % drive->num_ios;
    uint64_t             lba  = idx * drive->lba_count;
    int                   rc;

    if (g_rotate && drive->dst != P2PBW_DST_DRAM) {
        struct p2pbw_endpoint *ep = &g_ep[drive->dst == P2PBW_DST_EP0 ? 0 : 1];
        uint64_t stride = CEIL_4KB(g_io_size);
        uint64_t nslots = ep->reg_len / stride;
        task->buf = (uint8_t *)ep->reg_va + (ep->rot_cursor++ % nslots) * stride;
    }
    task->submit_tsc = spdk_get_ticks();
    rc = spdk_nvme_ns_cmd_read(drive->ns, drive->qpair, task->buf, lba, drive->lba_count,
                               io_complete, task, 0);
    if (rc != 0) {
        return rc;
    }
    drive->outstanding++;
    return 0;
}

/* ------------------------------------------------------------------ */
/* BAR2 window setup                                                   */
/* ------------------------------------------------------------------ */

static int
setup_ep_window(struct p2pbw_endpoint *ep, uint32_t n_drives_on_ep)
{
    uint64_t stride = CEIL_4KB(g_io_size);
    uintptr_t start  = CEIL_2MB((uintptr_t)ep->bar_va + g_offset_opt);
    uint64_t  len    = CEIL_2MB((uint64_t)n_drives_on_ep * g_qd * stride) + VALUE_2MB;
    if (g_rotate) {
        /* the whole BAR below its untranslatable last 2 MiB */
        len = ((uintptr_t)ep->bar_va + ep->bar_size - VALUE_2MB - start) & ~(VALUE_2MB - 1);
    }
    int       rc;
    uint64_t  vtophys;

    if (start + len > (uintptr_t)ep->bar_va + ep->bar_size - VALUE_2MB) {
        fprintf(stderr,
                "ERROR: %s: registered window (start=%#" PRIxPTR " len=%" PRIu64 ") does not fit "
                "below the last 2 MiB of BAR%d (va=%p size=%" PRIu64 ")\n",
                ep->bdf, start, len, P2PBW_BAR_IDX, ep->bar_va, ep->bar_size);
        return -1;
    }

    rc = spdk_mem_register((void *)start, len);
    if (rc != 0) {
        fprintf(stderr, "ERROR: %s: spdk_mem_register(%#" PRIxPTR ", %" PRIu64 ") failed: %d\n",
                ep->bdf, start, len, rc);
        return -1;
    }

    vtophys = spdk_vtophys((void *)start, NULL);
    if (vtophys != ep->bar_pa + (start - (uintptr_t)ep->bar_va)) {
        fprintf(stderr,
                "ERROR: %s: spdk_vtophys self-check failed: got 0x%" PRIx64 ", want 0x%" PRIx64 "\n",
                ep->bdf, vtophys, ep->bar_pa + (start - (uintptr_t)ep->bar_va));
        spdk_mem_unregister((void *)start, len);
        return -1;
    }

    ep->reg_va  = (void *)start;
    ep->reg_pa  = vtophys;
    ep->reg_len = len;

    printf("%s: registered window vaddr=%p paddr=0x%016" PRIx64 " len=%" PRIu64
           " (effective offset=0x%" PRIx64 ")\n",
           ep->bdf, ep->reg_va, ep->reg_pa, ep->reg_len,
           (uint64_t)(start - (uintptr_t)ep->bar_va));
    return 0;
}

/* ------------------------------------------------------------------ */
/* Teardown -- idempotent, safe to call from any point after globals   */
/* have been zero-initialised.                                         */
/* ------------------------------------------------------------------ */

static void
drain_all(void)
{
    uint64_t deadline = spdk_get_ticks() + 5 * spdk_get_ticks_hz();
    uint32_t i;
    bool     pending;

    do {
        pending = false;
        for (i = 0; i < g_num_drives; i++) {
            struct p2pbw_drive *drive = &g_drives[i];

            if (drive->qpair == NULL) {
                continue;
            }
            drive->draining = true;
            spdk_nvme_qpair_process_completions(drive->qpair, 0);
            if (drive->outstanding > 0) {
                pending = true;
            }
        }
    } while (pending && spdk_get_ticks() < deadline);
}

static void
teardown(void)
{
    uint32_t i;

    drain_all();

    for (i = 0; i < g_num_drives; i++) {
        struct p2pbw_drive *drive = &g_drives[i];
        uint32_t             t;

        if (drive->qpair != NULL) {
            spdk_nvme_ctrlr_free_io_qpair(drive->qpair);
            drive->qpair = NULL;
        }
        if (drive->tasks != NULL) {
            if (drive->dst == P2PBW_DST_DRAM) {
                for (t = 0; t < drive->qd; t++) {
                    if (drive->tasks[t].buf != NULL) {
                        spdk_dma_free(drive->tasks[t].buf);
                    }
                }
            }
            free(drive->tasks);
            drive->tasks = NULL;
        }
        if (drive->ctrlr != NULL) {
            spdk_nvme_detach(drive->ctrlr);
            drive->ctrlr = NULL;
        }
    }

    for (i = 0; i < 2; i++) {
        if (g_ep[i].reg_va != NULL) {
            spdk_mem_unregister(g_ep[i].reg_va, g_ep[i].reg_len);
            g_ep[i].reg_va = NULL;
        }
        if (g_ep[i].pci_dev != NULL) {
            spdk_pci_device_detach(g_ep[i].pci_dev);
            g_ep[i].pci_dev = NULL;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Reporting                                                           */
/* ------------------------------------------------------------------ */

static const char *
dst_name(enum p2pbw_dst dst)
{
    switch (dst) {
    case P2PBW_DST_DRAM: return "dram";
    case P2PBW_DST_EP0:  return "ep0";
    case P2PBW_DST_EP1:  return "ep1";
    default:             return "?";
    }
}

static double
window_seconds(const struct p2pbw_drive *drive)
{
    uint64_t hz = spdk_get_ticks_hz();

    if (drive->window_end_tsc <= drive->window_start_tsc || hz == 0) {
        return 0.0;
    }
    return (double)(drive->window_end_tsc - drive->window_start_tsc) / (double)hz;
}

static int
report(void)
{
    uint64_t hz = spdk_get_ticks_hz();
    uint64_t dst_bytes[3] = { 0, 0, 0 };
    double   dst_window_end[3] = { 0.0, 0.0, 0.0 };
    uint64_t total_bytes = 0;
    uint64_t total_errors = 0;
    uint64_t ep_all_bytes[2] = { 0, 0 };
    double   global_window_start = -1.0;
    double   global_window_end   = 0.0;
    uint32_t i;
    double   total_polls;

    printf("\n=== per-drive results ===\n");
    for (i = 0; i < g_num_drives; i++) {
        struct p2pbw_drive *drive = &g_drives[i];
        double               secs = window_seconds(drive);
        double               iops = secs > 0.0 ? (double)drive->window_completions / secs : 0.0;
        double               gbps = secs > 0.0 ?
                                    (double)drive->window_bytes / secs / P2PBW_GBPS_DIVISOR : 0.0;
        double               mean_us = drive->completions > 0 ?
                                       (double)drive->sum_tsc / (double)drive->completions * 1e6 / (double)hz : 0.0;
        double               min_us = (double)drive->min_tsc * 1e6 / (double)hz;
        double               max_us = (double)drive->max_tsc * 1e6 / (double)hz;

        printf("drive %-14s dst=%-4s completions=%-8" PRIu64 " iops=%-10.0f GBps=%-7.3f "
               "lat_us(mean/min/max)=%.1f/%.1f/%.1f errors=%" PRIu64 "\n",
               drive->traddr, dst_name(drive->dst), drive->window_completions, iops, gbps,
               mean_us, min_us, max_us, drive->errors);
        if (drive->has_error) {
            printf("  first error: sct=0x%02x sc=0x%02x\n", drive->first_err_sct, drive->first_err_sc);
        }

        dst_bytes[drive->dst] += drive->window_bytes;
        if (window_seconds(drive) > 0.0) {
            double end_s = (double)drive->window_end_tsc / (double)hz;
            double start_s = (double)drive->window_start_tsc / (double)hz;

            if (dst_window_end[drive->dst] < end_s) {
                dst_window_end[drive->dst] = end_s;
            }
            if (global_window_start < 0.0 || start_s < global_window_start) {
                global_window_start = start_s;
            }
            if (global_window_end < end_s) {
                global_window_end = end_s;
            }
        }
        total_bytes += drive->window_bytes;
        total_errors += drive->errors;
        if (drive->dst == P2PBW_DST_EP0) {
            ep_all_bytes[0] += drive->bytes;
        } else if (drive->dst == P2PBW_DST_EP1) {
            ep_all_bytes[1] += drive->bytes;
        }
    }

    printf("\n=== per-destination results (windowed) ===\n");
    {
        const char *names[3] = { "dram", "ep0", "ep1" };
        int          d;

        for (d = 0; d < 3; d++) {
            if (dst_bytes[d] == 0 && dst_window_end[d] == 0.0) {
                continue;
            }
            double secs = (global_window_start >= 0.0) ? (dst_window_end[d] - global_window_start) : 0.0;
            double gbps = secs > 0.0 ? (double)dst_bytes[d] / secs / P2PBW_GBPS_DIVISOR : 0.0;

            printf("dst=%-4s bytes=%-14" PRIu64 " GBps=%.3f\n", names[d], dst_bytes[d], gbps);
        }
    }

    {
        double secs = (global_window_start >= 0.0) ? (global_window_end - global_window_start) : 0.0;
        double gbps = secs > 0.0 ? (double)total_bytes / secs / P2PBW_GBPS_DIVISOR : 0.0;

        total_polls = (double)(g_empty_polls + g_nonempty_polls);
        printf("\n=== totals ===\n");
        printf("total window bytes=%" PRIu64 " GBps=%.3f errors=%" PRIu64 "\n",
               total_bytes, gbps, total_errors);
        printf("empty polls=%" PRIu64 "/%.0f (%.2f%%)\n", g_empty_polls, total_polls,
               total_polls > 0 ? 100.0 * (double)g_empty_polls / total_polls : 0.0);

        /* Fixed-format lines for the runner script; EP0/EP1 bytes cover the WHOLE run (fill +
         * window + drain tail) so they can be diffed 1:1 against the card's MI/speed-meter
         * counters, which are snapshotted around the whole process lifetime, not just the window. */
        printf("EP0 bytes %" PRIu64 "\n", ep_all_bytes[0]);
        printf("EP1 bytes %" PRIu64 "\n", ep_all_bytes[1]);
        printf("TOTAL GBps %.3f\n", gbps);
    }

    return total_errors > 0 ? 1 : 0;
}

/* ------------------------------------------------------------------ */
/* main                                                                */
/* ------------------------------------------------------------------ */

int
main(int argc, char **argv)
{
    struct spdk_env_opts     env_opts;
    struct spdk_pci_addr     allowed[P2PBW_MAX_DRIVES + 2];
    size_t                   num_allowed = 0;
    uint32_t                 n_on_ep[2] = { 0, 0 };
    uint32_t                 idx_on_ep[P2PBW_MAX_DRIVES];
    uint32_t                 i;
    int                      rc;
    int                      exit_code = 0;
    uint64_t                 deadline_tsc = 0;
    bool                     time_based;

    g_ep[0].bdf        = g_ep_bdf[0];
    g_ep[0].device_id  = P2PBW_EP0_DEVICE_ID;
    g_ep[1].bdf        = g_ep_bdf[1];
    g_ep[1].device_id  = P2PBW_EP1_DEVICE_ID;

    if (parse_args(argc, argv) != 0) {
        return 1;
    }
    /* --ep0/--ep1 may have been overridden after g_ep[] was pre-filled above */
    g_ep[0].bdf = g_ep_bdf[0];
    g_ep[1].bdf = g_ep_bdf[1];

    if (resolve_dst() != 0) {
        return 1;
    }
    time_based = (g_num_ios_opt == 0);

    if (refuse_if_fzc_running() != 0) {
        return 1;
    }

    for (i = 0; i < g_num_drives; i++) {
        if (g_drives[i].dst == P2PBW_DST_EP0) {
            idx_on_ep[i] = n_on_ep[0]++;
        } else if (g_drives[i].dst == P2PBW_DST_EP1) {
            idx_on_ep[i] = n_on_ep[1]++;
        } else {
            idx_on_ep[i] = 0;
        }
    }

    spdk_env_opts_init(&env_opts);
    snprintf(g_env_name, sizeof(g_env_name), "p2p_bw_%d", (int)getpid());
    env_opts.name       = g_env_name;
    if (getenv("P2P_MEM_MB")) {
        env_opts.mem_size = atoi(getenv("P2P_MEM_MB"));
    }
    env_opts.core_mask  = g_core_mask;

    for (i = 0; i < g_num_drives; i++) {
        rc = spdk_pci_addr_parse(&allowed[num_allowed], g_drives[i].traddr);
        if (rc != 0) {
            fprintf(stderr, "ERROR: cannot parse traddr '%s'\n", g_drives[i].traddr);
            return 1;
        }
        num_allowed++;
    }
    for (i = 0; i < 2; i++) {
        if (g_ep[i].needed) {
            rc = spdk_pci_addr_parse(&allowed[num_allowed], g_ep[i].bdf);
            if (rc != 0) {
                fprintf(stderr, "ERROR: cannot parse endpoint bdf '%s'\n", g_ep[i].bdf);
                return 1;
            }
            num_allowed++;
        }
    }
    env_opts.pci_allowed  = allowed;
    env_opts.num_pci_addr = num_allowed;

    rc = spdk_env_init(&env_opts);
    if (rc != 0) {
        fprintf(stderr, "ERROR: spdk_env_init failed: %d\n", rc);
        return 1;
    }

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    for (i = 0; i < 2; i++) {
        struct spdk_pci_driver *drv;
        struct spdk_pci_addr    addr;

        if (!g_ep[i].needed) {
            continue;
        }
        drv = spdk_pci_get_driver(i == 0 ? "p2pbw_ep0" : "p2pbw_ep1");
        if (drv == NULL) {
            fprintf(stderr, "ERROR: PCI driver for EP%u not registered\n", i);
            exit_code = 1;
            goto cleanup;
        }
        rc = spdk_pci_addr_parse(&addr, g_ep[i].bdf);
        if (rc != 0) {
            fprintf(stderr, "ERROR: cannot parse EP%u bdf '%s'\n", i, g_ep[i].bdf);
            exit_code = 1;
            goto cleanup;
        }
        rc = spdk_pci_device_attach(drv, ep_attach_cb, &g_ep[i], &addr);
        if (rc != 0 || !g_ep[i].attached) {
            fprintf(stderr, "ERROR: attach EP%u (%s) failed: %d\n", i, g_ep[i].bdf, rc);
            exit_code = 1;
            goto cleanup;
        }
        if (setup_ep_window(&g_ep[i], n_on_ep[i]) != 0) {
            exit_code = 1;
            goto cleanup;
        }
    }

    for (i = 0; i < g_num_drives; i++) {
        struct p2pbw_drive           *drive = &g_drives[i];
        struct spdk_nvme_transport_id trid;
        int32_t                       nsid;
        struct spdk_nvme_io_qpair_opts qopts;
        uint64_t                       num_sectors;
        uint32_t                       t;
        uint64_t                       stride;

        memset(&trid, 0, sizeof(trid));
        spdk_nvme_trid_populate_transport(&trid, SPDK_NVME_TRANSPORT_PCIE);
        snprintf(trid.traddr, sizeof(trid.traddr), "%s", drive->traddr);

        drive->ctrlr = spdk_nvme_connect(&trid, NULL, 0);
        if (drive->ctrlr == NULL) {
            fprintf(stderr, "ERROR: spdk_nvme_connect(%s) failed\n", drive->traddr);
            exit_code = 1;
            goto cleanup;
        }

        nsid = spdk_nvme_ctrlr_get_first_active_ns(drive->ctrlr);
        if (nsid <= 0) {
            fprintf(stderr, "ERROR: %s: no active namespace\n", drive->traddr);
            exit_code = 1;
            goto cleanup;
        }
        drive->ns = spdk_nvme_ctrlr_get_ns(drive->ctrlr, (uint32_t)nsid);
        if (drive->ns == NULL) {
            fprintf(stderr, "ERROR: %s: cannot get namespace %d\n", drive->traddr, nsid);
            exit_code = 1;
            goto cleanup;
        }

        drive->sector_size = spdk_nvme_ns_get_sector_size(drive->ns);
        if (drive->sector_size == 0 || (g_io_size % drive->sector_size) != 0) {
            fprintf(stderr, "ERROR: %s: io_size %u not a multiple of sector_size %u\n",
                    drive->traddr, g_io_size, drive->sector_size);
            exit_code = 1;
            goto cleanup;
        }
        drive->lba_count = g_io_size / drive->sector_size;

        if (spdk_nvme_ns_get_max_io_xfer_size(drive->ns) < g_io_size) {
            fprintf(stderr, "ERROR: %s: max_io_xfer_size %u < requested io_size %u\n",
                    drive->traddr, spdk_nvme_ns_get_max_io_xfer_size(drive->ns), g_io_size);
            exit_code = 1;
            goto cleanup;
        }

        num_sectors = spdk_nvme_ns_get_num_sectors(drive->ns);
        drive->num_ios = num_sectors / drive->lba_count;
        if (drive->num_ios == 0) {
            fprintf(stderr, "ERROR: %s: namespace too small for io_size %u\n",
                    drive->traddr, g_io_size);
            exit_code = 1;
            goto cleanup;
        }
        drive->rng_state = (uint64_t)time(NULL) ^ (0x2545F4914F6CDD1DULL * (i + 1)) ^ (uint64_t)i;
        if (drive->rng_state == 0) {
            drive->rng_state = 0x9E3779B97F4A7C15ULL;
        }

        spdk_nvme_ctrlr_get_default_io_qpair_opts(drive->ctrlr, &qopts, sizeof(qopts));
        if (g_qd > 256) {
            qopts.io_queue_size     = g_qd + 1;
            qopts.io_queue_requests = (g_qd + 1) * 2;
        }
        drive->qpair = spdk_nvme_ctrlr_alloc_io_qpair(drive->ctrlr, &qopts, sizeof(qopts));
        if (drive->qpair == NULL) {
            fprintf(stderr, "ERROR: %s: spdk_nvme_ctrlr_alloc_io_qpair failed\n", drive->traddr);
            exit_code = 1;
            goto cleanup;
        }

        drive->qd    = g_qd;
        drive->tasks = calloc(g_qd, sizeof(*drive->tasks));
        if (drive->tasks == NULL) {
            fprintf(stderr, "ERROR: out of memory allocating tasks\n");
            exit_code = 1;
            goto cleanup;
        }

        stride = CEIL_4KB(g_io_size);
        for (t = 0; t < g_qd; t++) {
            struct p2pbw_task *task = &drive->tasks[t];

            task->drive = drive;
            if (drive->dst == P2PBW_DST_DRAM) {
                int numa_id = spdk_nvme_ctrlr_get_numa_id(drive->ctrlr);

                task->buf = spdk_dma_zmalloc_socket(g_io_size, 0x1000, NULL, numa_id);
            } else {
                struct p2pbw_endpoint *ep = &g_ep[drive->dst == P2PBW_DST_EP0 ? 0 : 1];
                uint64_t                slice_off = (uint64_t)idx_on_ep[i] * g_qd * stride +
                                                    (uint64_t)t * stride;

                task->buf = (uint8_t *)ep->reg_va + slice_off;
            }
            if (task->buf == NULL) {
                fprintf(stderr, "ERROR: %s: task %u buffer allocation failed\n", drive->traddr, t);
                exit_code = 1;
                goto cleanup;
            }
            if (spdk_vtophys(task->buf, NULL) == SPDK_VTOPHYS_ERROR) {
                fprintf(stderr, "ERROR: %s: task %u buffer %p does not translate\n",
                        drive->traddr, t, task->buf);
                exit_code = 1;
                goto cleanup;
            }
        }

        printf("%s: sector_size=%u lba_count=%u num_ios=%" PRIu64 " dst=%s qd=%u\n",
               drive->traddr, drive->sector_size, drive->lba_count, drive->num_ios,
               dst_name(drive->dst), drive->qd);
    }

    /* -------------------------------------------------------------- */
    /* Initial fill: submit the full queue depth to every drive       */
    for (i = 0; i < g_num_drives; i++) {
        struct p2pbw_drive *drive = &g_drives[i];
        uint32_t             t;

        for (t = 0; t < drive->qd; t++) {
            rc = submit_task(&drive->tasks[t]);
            if (rc != 0) {
                fprintf(stderr, "ERROR: %s: initial fill submit failed: %d\n", drive->traddr, rc);
                exit_code = 1;
                goto cleanup;
            }
        }
    }

    {
        uint64_t now = spdk_get_ticks();

        for (i = 0; i < g_num_drives; i++) {
            g_drives[i].window_start_tsc = now;
        }
        if (time_based) {
            deadline_tsc = now + (uint64_t)g_duration_s * spdk_get_ticks_hz();
        }
    }

    /* -------------------------------------------------------------- */
    /* Run loop                                                        */
    for (;;) {
        bool all_drained = true;
        uint64_t now = spdk_get_ticks();

        for (i = 0; i < g_num_drives; i++) {
            struct p2pbw_drive *drive = &g_drives[i];

            if (!drive->window_closed) {
                bool close_now = g_stop;

                if (!close_now) {
                    if (time_based) {
                        close_now = (now >= deadline_tsc);
                    } else {
                        close_now = (drive->window_completions >= g_num_ios_opt);
                    }
                }
                if (close_now) {
                    drive->window_closed  = true;
                    drive->window_end_tsc = now;
                    drive->draining       = true;
                }
            }

            rc = spdk_nvme_qpair_process_completions(drive->qpair, 0);
            if (rc == 0) {
                g_empty_polls++;
            } else if (rc > 0) {
                g_nonempty_polls++;
            }

            if (!drive->window_closed || drive->outstanding > 0) {
                all_drained = false;
            }
        }

        if (all_drained) {
            break;
        }
    }

    exit_code = report();

cleanup:
    teardown();
    spdk_env_fini();
    return exit_code;
}
