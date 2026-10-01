/*
 *   iuventus.h: host-side environment for the DMA Iuventus FPGA-to-NVMe peer-to-peer engine
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) Vladislav Válek
 */

/** \file
 * DMA Iuventus environment library.
 *
 * Brings up an FPGA card and the NVMe drives it drives peer-to-peer, then tears the pair down
 * again. One card queue is bound to one drive: the library attaches the drive, creates an NVMe
 * I/O queue pair whose submission and completion rings live in the card's BARs, publishes the
 * PRP lists that let the card name any page of its shared data pool, and enables the engine.
 *
 * The data plane runs entirely on the card. The host configures it, watches it, and stops it;
 * it never sees the payload.
 *
 * Expected order of use:
 *
 * \code
 *   iuventus_opts_init(&opts, sizeof(opts));
 *   iuventus_parse_args(argc, argv, &opts, "si:g", app_parse, app_usage);
 *   spdk_env_init(&env_opts);
 *   iuventus_init(&ctx, &opts);
 *   iuventus_enable(&ctx);
 *      ... run ...
 *   iuventus_disable(&ctx);
 *   iuventus_drain(&ctx, IUVENTUS_DRAIN_TIMEOUT_MS);
 *   iuventus_fini(&ctx);
 * \endcode
 */

#ifndef SPDK_IUVENTUS_H
#define SPDK_IUVENTUS_H

#include "spdk/stdinc.h"

#include "spdk/env.h"
#include "spdk/nvme.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Largest number of drives one card can be bound to. Card queue q drives peer q, so this is
 * also the largest number of card queues the library will configure.
 */
#define IUVENTUS_MAX_PEERS 8
/* Queues the design instantiates (DMA_IUVENTUS NUM_QUEUES). The RTL binds queue q to endpoint
 * q / (IUVENTUS_HW_QUEUES / endpoints) -- queues 0..1 on endpoint 0, 2..3 on endpoint 1 -- and
 * the host must mirror that split exactly, whatever the peer count. */
#define IUVENTUS_HW_QUEUES 4

/**
 * Largest number of PCIe endpoints one card exposes. Single-endpoint firmware attaches only
 * ep[0]; bifurcated firmware attaches ep[0] and ep[1], and splits the card queues evenly across
 * them -- see iuventus_queue_ep().
 */
#define IUVENTUS_MAX_EPS 2

/** Bound recommended for iuventus_drain(), in milliseconds. */
#define IUVENTUS_DRAIN_TIMEOUT_MS 5000

/* Opaque to callers: the library owns the nfb handles and never asks a caller to dereference
 * them, so no nfb header is pulled into this one. */
struct nfb_device;
struct nfb_comp;

struct iuventus_ctx;

/**
 * One NVMe drive bound to one card queue.
 *
 * The first group of fields is the contract iuventus_peer() publishes: which drive this is and
 * which queue pair the card issues its commands on. The rest is library bookkeeping.
 */
struct iuventus_peer {
	/** Controller the drive attached as. */
	struct spdk_nvme_ctrlr		*ctrlr;
	/** First active namespace on that controller. */
	struct spdk_nvme_ns		*ns;
	/** Identifier of that namespace. */
	uint32_t			nsid;
	/** NVMe I/O queue identifier the card submits on, or -1 when none is allocated. */
	int32_t				hw_qid;
	/** Number of sectors in the namespace. */
	uint64_t			lba_space_size;
	/** Largest sector count a single command may carry, as a 0-based mask. */
	uint16_t			lba_num_mask;
	/** Transport address the drive was requested with. */
	struct spdk_nvme_transport_id	trid;
	/** Model name reported by the controller. */
	char				name[64];

	/* Library bookkeeping. */
	uint64_t			doorbell_base;
	uint32_t			doorbell_stride;
	uint16_t			doorbell_mask;
	uint64_t			sqtdbl_paddr;
	uint64_t			cqhdbl_paddr;
	bool				attached;
	bool				qid_alloc;
	bool				cq_created;
	bool				sq_created;
};

/** A mapped card BAR. */
struct iuventus_bar {
	void		*vaddr;
	uint64_t	paddr;
	uint64_t	size;
};

/**
 * A PRP list published to the drives. Entry i addresses page i of the corresponding data BAR,
 * so the card can hand a drive any page of the pool by index alone.
 */
struct iuventus_prp_list {
	void		*vaddr;
	uint64_t	paddr;
	uint32_t	entries;
};

/**
 * One PCIe endpoint of the card: its own pair of NVMe queue BARs, its own pair of data-pool
 * BARs, and its own PRP lists for those pools. A card queue always uses the BARs of the endpoint
 * iuventus_queue_ep() names for it, never another endpoint's.
 */
struct iuventus_ep {
	/** BAR holding the NVMe submission rings, one 4 KiB page per card queue on this endpoint. */
	struct iuventus_bar		sq;
	/** BAR holding the NVMe completion rings, one 4 KiB page per card queue on this endpoint. */
	struct iuventus_bar		cq;
	/** Data pool the card writes to a drive from, on this endpoint. */
	struct iuventus_bar		wrbuff;
	/** Data pool the card reads from a drive into, on this endpoint. */
	struct iuventus_bar		rdbuff;

	/** PRP list published for wrbuff. */
	struct iuventus_prp_list	wrbuff_prp_list;
	/** PRP list published for rdbuff. */
	struct iuventus_prp_list	rdbuff_prp_list;

	/* Library bookkeeping. */
	struct spdk_pci_device		*pci_dev;
	bool				prp_lists_allocated;
};

/**
 * Options for iuventus_init().
 *
 * Always fill this in with iuventus_opts_init() before touching any field: opts_size is what
 * lets a library built against a longer struct stay compatible with a caller built against a
 * shorter one.
 */
struct iuventus_opts {
	/** sizeof(struct iuventus_opts) as the caller knows it. Set by iuventus_opts_init(). */
	size_t		opts_size;
	/** nfb device to open, as accepted by nfb_open(). Defaults to "0". */
	const char	*nfb_dev;
	/** NVMe I/O queue depth in entries; 0 keeps the controller's default. */
	uint16_t	qsize;
	/**
	 * Let the card re-write a doorbell whose value has not changed. Off by default: an
	 * unchanged doorbell write is an NVMe Invalid Doorbell Write Value.
	 */
	bool		rpt_upd_en;
	/** Number of entries of peer_trid[] that are valid. At least one is required. */
	uint32_t	num_peers;
	/** Transport address per peer. Card queue q is bound to peer_trid[q]. */
	struct spdk_nvme_transport_id	peer_trid[IUVENTUS_MAX_PEERS];
	/**
	 * Optional hook, invoked once every peer has attached and before any card state is
	 * touched. This is the only moment at which a controller-level or subsystem-level reset
	 * is safe: the controllers exist, no I/O queue pair has been created on them yet, and the
	 * card holds no configuration that the reset would invalidate.
	 *
	 * Returning non-zero aborts iuventus_init(), which unwinds and returns that value.
	 */
	int		(*on_peers_ready)(struct iuventus_ctx *ctx, void *cb_arg);
	/** Passed to on_peers_ready(). */
	void		*cb_arg;
};

/**
 * Environment handle.
 *
 * Storage belongs to the caller; iuventus_init() fills it in and iuventus_fini() empties it.
 * Read the fields freely -- writing one behind the library's back desynchronises it from the
 * card.
 */
struct iuventus_ctx {
	/** Peers, indexed by card queue. */
	struct iuventus_peer		peers[IUVENTUS_MAX_PEERS];
	/** Number of valid entries in peers[]. */
	uint32_t			num_peers;

	/** PCIe endpoints the card exposes. Card queue q uses ep[iuventus_queue_ep(ctx, q)]. */
	struct iuventus_ep		ep[IUVENTUS_MAX_EPS];
	/** Number of valid entries in ep[]: 1 on single-endpoint firmware, 2 on bifurcated firmware. */
	uint32_t			num_eps;

	/* Library bookkeeping. */
	struct nfb_device		*nfb_dev;
	struct nfb_comp			*nfb_comp;
	const char			*pcie_bdf;
	uint16_t			qsize;
	bool				rpt_upd_en;
	bool				enabled;
};

/** Result of iuventus_parse_args(). */
enum iuventus_parse_args_rvals {
	/** -h was given; usage has been printed and the caller should exit successfully. */
	IUVENTUS_PARSE_ARGS_HELP	= 0,
	/** Arguments accepted. */
	IUVENTUS_PARSE_ARGS_SUCCESS	= 1,
	/** Arguments rejected; usage has been printed. */
	IUVENTUS_PARSE_ARGS_FAIL	= 2,
};

/**
 * Fill opts with its defaults.
 *
 * \param opts Options to initialise.
 * \param opts_size Size of the struct the caller was built against, i.e. sizeof(*opts).
 */
void iuventus_opts_init(struct iuventus_opts *opts, size_t opts_size);

/**
 * Print the usage fragment for the options iuventus_parse_args() owns.
 *
 * Callers that print their own usage should call this from it so both fragments appear together.
 */
void iuventus_usage(void);

/**
 * Parse the command line, taking the options this library owns and handing the rest back.
 *
 * The library owns -h, -d, -t, -q and -R, plus the retired -Q and -N, which are still recognised
 * so an old command line gets an explanation rather than a bare usage dump. Everything in
 * app_getopt_str is passed to app_parse() as it is found; it must not repeat any of those
 * letters.
 *
 * \param argc Count of arguments in argv.
 * \param argv Command line.
 * \param opts Options to fill in; must have been through iuventus_opts_init().
 * \param app_getopt_str getopt(3) string for the caller's own options. May be NULL.
 * \param app_parse Called for each option in app_getopt_str; returns 0 to accept. May be NULL
 *        only if app_getopt_str is NULL.
 * \param app_usage Called before the library's own usage fragment is printed. May be NULL.
 * \return one of enum iuventus_parse_args_rvals.
 */
int iuventus_parse_args(int argc, char **argv, struct iuventus_opts *opts,
			const char *app_getopt_str, int (*app_parse)(int ch, char *arg),
			void (*app_usage)(void));

/**
 * Bring up the card and the drives bound to it.
 *
 * Attaches every peer, runs opts->on_peers_ready(), maps the card BARs, creates one NVMe I/O
 * queue pair per peer with its rings inside those BARs, allocates and publishes the data-pool
 * PRP lists, and programs the per-queue registers. The engine is left disabled; call
 * iuventus_enable() to start it.
 *
 * Must be called after spdk_env_init(). On failure everything already brought up is unwound, so
 * the handle is left as good as untouched.
 *
 * \param ctx Handle to fill in.
 * \param opts Options; copied, so the caller may reuse the struct afterwards.
 * \return 0 on success, negative errno on failure, or whatever on_peers_ready() returned.
 */
int iuventus_init(struct iuventus_ctx *ctx, const struct iuventus_opts *opts);

/**
 * Start the engine.
 *
 * Clears the whole completion-queue BAR first, which is not optional and therefore not exposed
 * separately. A card queue this run does not create still holds the previous run's completions;
 * the card does not clear that queue's doorbell mask on restart, so it stays un-vetoed and is
 * still polled. Its phase resets to 1 and those stale entries carry phase 1, so they are
 * consumed as completions with no command behind them -- each returning an identifier and
 * releasing a data page that was never handed out, until the card stops granting pages at all.
 *
 * The clear writes every byte rather than just the phase tag: a sparse byte write is a
 * read-modify-write, and this BAR does not support reads, so the narrow form never lands.
 *
 * \param ctx Environment handle.
 * \return 0 on success, negative errno on failure.
 */
int iuventus_enable(struct iuventus_ctx *ctx);

/**
 * Stop the engine.
 *
 * Returns as soon as the request is posted; the card quiesces asynchronously. Follow with
 * iuventus_drain() before anything the card is still using is taken away from it.
 *
 * \param ctx Environment handle.
 */
void iuventus_disable(struct iuventus_ctx *ctx);

/**
 * Wait for the card to reach full idle.
 *
 * STATUS.READY stays high while the design quiesces -- it stops accepting new reads, finishes
 * any in-flight write to its end, then blocks -- and drains every outstanding peer-to-peer
 * operation, dropping only at full idle. Wait for that BEFORE the NVMe I/O queues are deleted,
 * else the drives' in-flight fetches are stranded and their controller state is corrupted.
 * Whole-design READY covers every card queue at once, which a per-queue doorbell comparison
 * does not. The wait is bounded so that one stuck drive cannot hang teardown forever.
 *
 * Call after iuventus_disable(); the disable is what starts the quiesce. iuventus_fini() does
 * both itself if the caller has not, so the hazard cannot be walked into by forgetting them.
 *
 * \param ctx Environment handle.
 * \param timeout_ms Bound on the wait, in milliseconds.
 * \return 0 once the design is idle, -ETIMEDOUT if it is still busy when the bound expires.
 */
int iuventus_drain(struct iuventus_ctx *ctx, uint32_t timeout_ms);

/**
 * Tear down everything iuventus_init() brought up, in strict reverse order.
 *
 * Stops and drains the engine first, whether or not the caller already did and whether or not
 * this process is the one that started it. Safe to call on a zeroed handle and safe to call
 * twice.
 *
 * \param ctx Environment handle.
 */
void iuventus_fini(struct iuventus_ctx *ctx);

/**
 * Number of drives bound to the card.
 *
 * \param ctx Environment handle.
 * \return the peer count, which is also the number of card queues in use.
 */
uint32_t iuventus_num_peers(const struct iuventus_ctx *ctx);

/**
 * Look up the drive bound to a card queue.
 *
 * \param ctx Environment handle.
 * \param i Card queue index, below iuventus_num_peers().
 * \return the peer, or NULL if i is out of range.
 */
const struct iuventus_peer *iuventus_peer(const struct iuventus_ctx *ctx, uint32_t i);

/**
 * Endpoint whose BARs a card queue's SQ/CQ and data pool live on.
 *
 * The design splits its IUVENTUS_HW_QUEUES queues into contiguous halves: queue q is on endpoint
 * q / (IUVENTUS_HW_QUEUES / ctx->num_eps), independent of how many peers are attached. On
 * single-endpoint firmware (ctx->num_eps == 1) this is always 0.
 *
 * \param ctx Environment handle.
 * \param q Card queue index, below iuventus_num_peers().
 * \return endpoint index, below ctx->num_eps.
 */
uint32_t iuventus_queue_ep(const struct iuventus_ctx *ctx, uint32_t q);

#ifdef __cplusplus
}
#endif

#endif /* SPDK_IUVENTUS_H */
