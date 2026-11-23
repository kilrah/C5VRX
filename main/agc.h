/* C5VRX-5 by Twotoz and the C5VRX contributors: selectable AGC models.
 *
 * Two alternatives to KEEL-1 (gain.c dg_tick), chosen in the menu GAIN row
 * or console G, for a like-for-like comparison on the board. Both use the
 * same observer windows (dg_observation_t) and the same KEEL-1 home rows
 * (3 dB apart from the table maximum down), so the comparison is about the
 * decision only. Host harness: test/test_agc_compare.c.
 *
 * LEVEL: the input-level AGC. A gain row scales carrier and receiver
 *   noise alike (the noise is set before the baseband gain, FINDINGS
 *   "Direct Gain V7"), so the carrier level referred to the input does not
 *   depend on the row in use. LEVEL keeps a short decaying histogram of it
 *   (~20 ms) and for every home row scores what that row would have given
 *   over the histogram: sync and picture SNR from C/N, quantisation and
 *   the detector's small-signal loss, or a fold of the ultrafine lane. It
 *   moves when another row is clearly better (hysteresis), and at once on
 *   a fold.
 *
 * RSSI: a classic AGC on received power. Every 5 ms the mean window power
 *   (carrier plus noise) is placed at a target radius with a +-3 dB dead
 *   band; a fold drops 6 dB at once. No model of noise or picture.
 *
 * Integer only per window (the C5 has no FPU); the per-row quality table is
 * computed once at reset. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "gain.h"

#ifndef AGC_BIN_DB
#define AGC_BIN_DB 2              /* bins of the input level, dB */
#endif
#define AGC_LMIN_DB (-20)         /* bin 0: carrier power at the top row, dB cells^2 */
/* The top bin must reach a carrier on its best radius at the lowest row
 * (G2 sits ~74 dB under the top row: ~89 dB). Up to 2026-11-13 it ended at
 * +80 (about G9): a stronger carrier, an external LNA close in, read as +79
 * and LEVEL held a row that folded every window (test_agc_strong.c). */
#define AGC_LMAX_DB 100
#define AGC_BINS ((AGC_LMAX_DB - AGC_LMIN_DB) / AGC_BIN_DB)
#define AGC_MAX_HOMES 30u

typedef struct {
    /* Runtime experiments, zero keeps the baseline. Filled by the owner
     * after reset, never by another task while a decision is running. */
    uint32_t fold_memory_us;
    bool fold_radius;
    int8_t target_db;
    uint8_t release;
    uint8_t max_index, homes;
    uint8_t top_index;                /* highest home: the noise-limited top row (agc.c LEVEL_TOP_NOISE_MAX_X16) */
    uint8_t home[AGC_MAX_HOMES];
    int16_t rel_q8[DG_STATES];        /* row gain minus the top row, dB Q8 (model + learned) */
    int16_t learned_q8[DG_STATES];    /* learned correction of the model per row */
    uint16_t noise_x16[DG_STATES];    /* receiver noise power per row, cells^2 x16 */
    uint16_t noise_top_x16;
    int8_t quality[AGC_MAX_HOMES][AGC_BINS];
    uint32_t hist[AGC_BINS];          /* slow level history (LEVEL_DECAY_Q8): the fading */
    uint32_t fast[AGC_BINS];          /* fast level history (LEVEL_FAST_DECAY_Q8): a jump */
    uint64_t decide_us;
    bool fold_pending;
    /* Self-calibration: the input level just before and just after a move
     * must agree (the carrier barely changes in ~2 ms); the difference is
     * the model error of the new row. */
    int32_t level_ema_q8;
    uint8_t level_windows;
    bool move_pending;
    uint8_t move_to, move_windows;
    int32_t move_pre_q8, move_sum_q8;
    uint64_t move_us;
    /* Fold memory: the row that last folded, until when, and the level it
     * folded at (agc.c LEVEL_FOLD_MEMORY_US). */
    uint8_t fold_row;
    uint64_t fold_until_us;
    int32_t fold_level_q8;
    uint32_t folds, decisions, moves, learned, jumps, fold_holds;
} agc_level_t;

typedef struct {
    uint8_t max_index, homes;
    uint8_t home[AGC_MAX_HOMES];
    int16_t rel_q8[DG_STATES];
    uint32_t sum_x16, windows;
    uint64_t decide_us;
    uint32_t folds, moves;
} agc_rssi_t;

/* The noise-limited top row: the highest row whose receiver noise is at most
 * this (cells^2 x16, 3 cells rms). Bare boards keep G83 (4.8-7 cells^2);
 * behind an external LNA higher rows add no C/N, only folds (agc.c,
 * receiver_gain_top). */
#ifndef LEVEL_TOP_NOISE_MAX_X16
#define LEVEL_TOP_NOISE_MAX_X16 (9u * 16u)
#endif
/* The limit in use (EXPERIMENTAL > LEVEL TOP sets it before a re-bind;
 * LEVEL_TOP_NOISE_MAX_X16 by default). */
void agc_level_set_top_noise(uint16_t noise_max_x16);
uint16_t agc_level_top_noise(void);
/* row_db_q8: the gain model per row (direct_gain_t model_db_q8), noise_top_x16:
 * receiver noise power at the table maximum, cells^2 x16 (behind an external
 * LNA it can lie far over the 4-bit window: the homes then end at the highest
 * row whose noise alone does not fold, a->top_index). */
void agc_level_options(agc_level_t *a, bool radius, uint32_t memory_us, int target_db, unsigned release);
void agc_level_reset(agc_level_t *a, const int16_t row_db_q8[DG_STATES], uint8_t max_index, uint16_t noise_top_x16);
uint8_t agc_level_tick(agc_level_t *a, const dg_observation_t *o, uint8_t current, uint64_t now_us);
/* The carrier radius of a folded window at row `current`, cells x16, for
 * dg_observation_t.fold_r_x16; 0 when the window is no clear circle over
 * the 4-bit window or the row's noise is too high to tell (agc.c). raw_x16
 * and fit (optional) receive the best radius before the fit check and the
 * fit (fold_fit units); raw 0 when the row was too noisy to estimate. */
uint16_t agc_level_fold_radius(const agc_level_t *a, const uint8_t *samples, unsigned n, uint8_t current,
                               uint16_t *raw_x16, int8_t *fit);
/* Whether LEVEL acts on the fold radius (LEVEL_FOLD_RADIUS, agc.c) or only logs it. */
bool agc_level_fold_radius_used(void);

void agc_rssi_reset(agc_rssi_t *a, const int16_t row_db_q8[DG_STATES], uint8_t max_index);
uint8_t agc_rssi_tick(agc_rssi_t *a, const dg_observation_t *o, uint8_t current, uint64_t now_us);

/* A window where carrier and noise reached both rails of both axes: folded. */
bool agc_folded(const dg_observation_t *o);
/* 10 log10(x) in dB Q8, x >= 1 (integer). */
int agc_db_q8(uint32_t x);
