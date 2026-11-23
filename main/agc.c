/* C5VRX-5 by Twotoz and the C5VRX contributors: LEVEL and RSSI AGC (agc.h). */
#include "agc.h"
#include "fold_radius_table.h"
#include "level_quality.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* LEVEL parameters (host search, test/test_agc_compare.c). */
#ifndef LEVEL_DECIDE_US
#define LEVEL_DECIDE_US 5000u
#endif
#ifndef LEVEL_DECAY_Q8
#define LEVEL_DECAY_Q8 250u     /* per 5 ms: exp(-5 / 200 ms); 20 ms (199) followed the fading
                                   * itself and wrote 5 a second (test_agc_compare.c, PICTURE_Q) */
#endif
#ifndef LEVEL_FAST_DECAY_Q8
#define LEVEL_FAST_DECAY_Q8 199u   /* per 5 ms: exp(-5 / 20 ms) */
#endif
#ifndef LEVEL_JUMP_Q8
#define LEVEL_JUMP_Q8 (9 * 256)    /* best rows of the two histories this far apart (3 home rows): a jump, not fading; 6 dB wrote 2.6/s, 9 dB 1.7/s at the same score */
#endif
#ifndef LEVEL_WEIGHT
#define LEVEL_WEIGHT 256u     /* one window */
#endif
#ifndef LEVEL_FOLD_WEIGHT
#define LEVEL_FOLD_WEIGHT 256u     /* a folded window counts as one, a few dB over */
#endif
#ifndef LEVEL_HYST
#define LEVEL_HYST 6        /* quality units (of 100) needed to move */
#endif
#ifndef LEVEL_FOLD_OVER_Q8
#define LEVEL_FOLD_OVER_Q8 (3 * 256)   /* a folded window: its measured level this much over */
#endif
#ifndef LEVEL_LEARN_MAX_Q8
#define LEVEL_LEARN_MAX_Q8 (8 * 256)   /* learned model correction per row, at most */
#endif
#ifndef LEVEL_LEARN_WINDOWS
#define LEVEL_LEARN_WINDOWS 8u         /* windows after a move that measure the new row */
#endif
#ifndef LEVEL_LEARN_SETTLE_US
#define LEVEL_LEARN_SETTLE_US 600u     /* the write transient */
#endif
/* Fold radius (agc_level_fold_radius): accepted when the window fits a
 * circle within this much of a true one, per sample (0.2 nat; host model,
 * research/fold_radius.py eval: carriers -0.04 median, noise clouds -0.55),
 * and only beyond the window (a smaller "radius" is noise peaks folding). */
#define FOLD_FIT_MIN    (-(FOLD_SCALE * 16 / 5))
#define FOLD_R_MIN_X16  (8 * 16)
/* Whether LEVEL acts on the fold radius. Off: measured and logged only
 * (attack events). Board walk 290 with it on: the steps down from G77 and
 * below grew from one home row (median 3 rows) to 9, but LEVEL then climbed
 * back (releases 50 -> 90 per walk) and folded again within ~80 ms; folds
 * and video were unchanged and -85..-80 dBm read 46 % against 68-77 %. The
 * radius it took is logged so the next walk can tell an overestimate from a
 * release too early. Host: -DLEVEL_FOLD_RADIUS=1 (test_agc_compare). */
#ifndef LEVEL_FOLD_RADIUS
#define LEVEL_FOLD_RADIUS 0
#endif
/* Fold memory: after a fold LEVEL does not climb back into the row that
 * folded (or a higher one) for this long, unless the carrier has since read
 * LEVEL_FOLD_FAR_Q8 under the fold (gone weaker, not a fade). Blackbox walk
 * 292: nearly every fold was barely over the window (radius 7-10 cells), and
 * after each step down LEVEL climbed back to the row under the edge and folded
 * again 100-600 ms later (G59 -> G56 -> G53 -> G50 in 0.6 s, twice); its
 * 200 ms history forgets the peaks before the next one. KEEL-1 holds a folded
 * row 300 ms (DG7_FOLD_MEMORY_US). 0 switches it off. */
#ifndef LEVEL_FOLD_MEMORY_US
#define LEVEL_FOLD_MEMORY_US 0u
#endif
#define LEVEL_FOLD_FAR_Q8 (6 * 256)

/* RSSI parameters. */
#ifndef RSSI_DECIDE_US
#define RSSI_DECIDE_US 5000u
#endif
#ifndef RSSI_TARGET_X16
#define RSSI_TARGET_X16 (12u * 16u)   /* mean power 12 cells^2: radius ~3.5 cells */
#endif
#ifndef RSSI_DEADBAND_Q8
#define RSSI_DEADBAND_Q8 (3 * 256)
#endif
#ifndef RSSI_FOLD_DROP_Q8
#define RSSI_FOLD_DROP_Q8 (6 * 256)
#endif

int agc_db_q8(uint32_t x)
{
    if (x < 1u) x = 1u;
    /* log2 by the leading bit plus a fraction, then x 10 log10(2). */
    int whole = 31 - __builtin_clz(x);
    uint64_t m = (uint64_t)x << 16 >> whole;   /* [65536, 131072) */
    int frac = 0;
    for (int bit = 7; bit >= 0; --bit) {
        m = (m * m) >> 16;
        if (m >= 131072u) { m >>= 1; frac |= 1 << bit; }
    }
    return ((whole * 256 + frac) * 771 + 128) / 256;   /* 771/256 = 3.0103 */
}

bool agc_folded(const dg_observation_t *o)
{
    return o->clip_pm >= 20u && (o->rails & 3u) == 3u && (o->rails & 12u) == 12u;
}

static unsigned build_homes(uint8_t *home, const int16_t *row_db_q8, uint8_t max_index)
{
    unsigned n = 0;
    int last = 1 << 20;
    for (int k = max_index; k >= 0 && n < AGC_MAX_HOMES; --k)
        if (last - row_db_q8[k] >= 3 * 256 - 1) { home[n++] = (uint8_t)k; last = row_db_q8[k]; }
    return n;
}

/* The home row nearest to `row` (by gain). */
static unsigned nearest_home(const uint8_t *home, unsigned homes, const int16_t *rel_q8, uint8_t row)
{
    unsigned best = 0;
    int best_d = 1 << 20;
    for (unsigned h = 0; h < homes; ++h) {
        const int d = abs(rel_q8[home[h]] - rel_q8[row]);
        if (d < best_d) { best_d = d; best = h; }
    }
    return best;
}

/* ---- LEVEL ---- */

/* Linear interpolation position of x on an ascending axis: index and weight. */
static int axis_at(const int16_t *axis, int n, float x, float *w)
{
    if (x <= axis[0]) { *w = 0.0f; return 0; }
    for (int k = 1; k < n; ++k)
        if (x < axis[k]) { *w = (x - axis[k - 1]) / (float)(axis[k] - axis[k - 1]); return k - 1; }
    *w = 1.0f;
    return n - 2;
}

/* Quality 0..100 of a carrier at input level `level_db` on a row `rel_db`
 * under the top one: the measured U80P64-R1 table (level_quality.h) at the
 * row's C/N and total radius. The gain sets no C/N (the noise enters before
 * it), only the radius: rows differ by the radius window (best ~2.5-6 cells
 * rms; a fold above ~7 costs far more than a small radius below). */
static int quality(float level_db, float rel_db, float noise_top)
{
    const float c = powf(10.0f, (level_db + rel_db) / 10.0f), n = noise_top * powf(10.0f, rel_db / 10.0f);
    const float rms = sqrtf(c + n);
    if (rms * 100.0f >= level_q_rms_x100[LEVEL_Q_RMS - 1]) return 0;
    float wc, wr;
    const int i = axis_at(level_q_cn_x10, LEVEL_Q_CNS, 100.0f * log10f(c / n + 1e-9f), &wc);
    const int j = axis_at(level_q_rms_x100, LEVEL_Q_RMS, rms * 100.0f, &wr);
    const float lo = level_q[i][j] + (level_q[i][j + 1] - level_q[i][j]) * wr;
    const float hi = level_q[i + 1][j] + (level_q[i + 1][j + 1] - level_q[i + 1][j]) * wr;
    float q = lo + (hi - lo) * wc;
    if (rms * 100.0f < level_q_rms_x100[0]) q *= rms * 100.0f / level_q_rms_x100[0];   /* toward no signal */
    return (int)lroundf(q);
}

static void row_noise(agc_level_t *a, unsigned k)
{
    const float n = a->noise_top_x16 * powf(10.0f, a->rel_q8[k] / 2560.0f);
    a->noise_x16[k] = (uint16_t)(n < 1.0f ? 1.0f : n > 65535.0f ? 65535.0f : n);
}

/* Survival row: a carrier at or under LEVEL_EDGE_CN_DB over the receiver noise
 * (or none at all) belongs on the top row. There the measured table is flat
 * within ~1 dB from 1.25 to 7 cells (level_quality_grid.py), so the rows under
 * the top scored as well as it and LEVEL waited one or two home rows down
 * (board 2026-10-29: G79 with no carrier; host edge scenarios G76-G79). The
 * top row cannot fold there (C/N 6 dB at 4.8 cells^2 of noise is 4.9 cells
 * rms) and it is the known high-gain state a returning carrier is met in
 * (AGENTS.md control-work principles), so every lower home scores under it by
 * its distance: no write at the edge unless the level really rose. */
#ifndef LEVEL_EDGE_CN_DB
#define LEVEL_EDGE_CN_DB 6
#endif
static void home_quality(agc_level_t *a, unsigned h)
{
    const float noise_top = a->noise_top_x16 / 16.0f;
    const float edge_db = 10.0f * log10f(noise_top > 0.01f ? noise_top : 0.01f) + LEVEL_EDGE_CN_DB;
    for (int b = 0; b < AGC_BINS; ++b) {
        const float level = AGC_LMIN_DB + (b + 0.5f) * AGC_BIN_DB - a->target_db;
        int q = quality(level, a->rel_q8[a->home[h]] / 256.0f, noise_top);
        if (LEVEL_EDGE_CN_DB > -100 && h && level <= edge_db) {
            const int top = quality(level, a->rel_q8[a->home[0]] / 256.0f, noise_top) - (int)h;
            if (q > top) q = top;
        }
        a->quality[h][b] = (int8_t)(q < -128 ? -128 : q);
    }
}

void agc_level_options(agc_level_t *a, bool radius, uint32_t memory_us, int target_db, unsigned release)
{
    a->fold_radius = radius;
    a->fold_memory_us = memory_us;
    a->target_db = (int8_t)target_db;
    a->release = (uint8_t)release;
    for (unsigned h = 0; h < a->homes; ++h) home_quality(a, h);
}

/* The survival row (home_quality) assumes the receiver noise alone cannot
 * fold the top row: 4.8 cells^2 is 2.2 cells rms on the bare board, 7 cells^2
 * at 5865 MHz (rx_level.h: 1.7 dB over the calibration). Behind an external
 * LNA the noise at G83 is ~17 dB higher (board 2026-11-14, ~21 dB LNA: ~270
 * cells^2, 16 cells rms): with the homes from G83 LEVEL held G72 with the VTX
 * off and folded in 5155 of 5239 decisions, the noise read as a carrier. The
 * homes start at the highest row whose noise stays under 9 cells^2 (3 cells
 * rms, the low end of the table's best radius and 1 dB over the bare A1
 * noise): bare boards keep G83, behind that LNA it is ~G67, where KEEL-1
 * settled on its own (G65-G67). More gain there adds no C/N, only folds. */
static uint16_t s_top_noise_x16 = LEVEL_TOP_NOISE_MAX_X16;
void agc_level_set_top_noise(uint16_t noise_max_x16) { s_top_noise_x16 = noise_max_x16 ? noise_max_x16 : LEVEL_TOP_NOISE_MAX_X16; }
uint16_t agc_level_top_noise(void) { return s_top_noise_x16; }
void agc_level_reset(agc_level_t *a, const int16_t row_db_q8[DG_STATES], uint8_t max_index, uint16_t noise_top_x16)
{
    memset(a, 0, sizeof(*a));
    a->fold_radius = LEVEL_FOLD_RADIUS != 0;
    a->fold_memory_us = LEVEL_FOLD_MEMORY_US;
    a->max_index = max_index;
    a->noise_top_x16 = noise_top_x16;
    for (unsigned k = 0; k <= max_index; ++k) {
        a->rel_q8[k] = (int16_t)(row_db_q8[k] - row_db_q8[max_index]);
        row_noise(a, k);
    }
    int top = max_index;
    while (top > 0 && a->noise_x16[top] > s_top_noise_x16) --top;
    a->top_index = (uint8_t)top;
    a->homes = (uint8_t)build_homes(a->home, row_db_q8, (uint8_t)top);
    for (unsigned h = 0; h < a->homes; ++h) home_quality(a, h);
}

/* The model error of the row a move went to, a quarter at a time, at most
 * LEVEL_LEARN_MAX_Q8 in all; the top row is the anchor (noise measured there). */
static void level_learn(agc_level_t *a, uint8_t row, int err_q8)
{
    if (row == a->max_index) return;
    if (err_q8 > 6 * 256) err_q8 = 6 * 256;
    if (err_q8 < -6 * 256) err_q8 = -6 * 256;
    int l = a->learned_q8[row] + err_q8 / 4;
    l = l > LEVEL_LEARN_MAX_Q8 ? LEVEL_LEARN_MAX_Q8 : l < -LEVEL_LEARN_MAX_Q8 ? -LEVEL_LEARN_MAX_Q8 : l;
    a->rel_q8[row] = (int16_t)(a->rel_q8[row] + l - a->learned_q8[row]);
    a->learned_q8[row] = (int16_t)l;
    row_noise(a, row);
    for (unsigned h = 0; h < a->homes; ++h)
        if (a->home[h] == row) home_quality(a, h);
    ++a->learned;
}

static void level_add(agc_level_t *a, int level_q8, uint32_t weight)
{
    int b = ((level_q8 >> 8) - AGC_LMIN_DB) / AGC_BIN_DB;   /* floor (level >= LMIN) */
    if (level_q8 < AGC_LMIN_DB * 256) b = 0;
    b = b < 0 ? 0 : b >= AGC_BINS ? AGC_BINS - 1 : b;
    a->hist[b] += weight;
    a->fast[b] += weight;
}

static int32_t level_utility(const agc_level_t *a, const uint32_t *hist, unsigned h, uint64_t total)
{
    int64_t u = 0;
    for (int b = 0; b < AGC_BINS; ++b) u += (int64_t)hist[b] * a->quality[h][b];
    return total ? (int32_t)(u * 256 / (int64_t)total) : 0;   /* quality x256 */
}

/* The home with the highest utility over `hist` (ties: listen high). */
static unsigned level_best(const agc_level_t *a, const uint32_t *hist, int32_t *best_u)
{
    uint64_t total = 0;
    for (int b = 0; b < AGC_BINS; ++b) total += hist[b];
    unsigned best = 0;
    *best_u = INT32_MIN;
    for (unsigned h = 0; h < a->homes; ++h) {
        const int32_t u = level_utility(a, hist, h, total) + (h == 0);
        if (u > *best_u) { *best_u = u; best = h; }
    }
    return best;
}

uint8_t agc_level_tick(agc_level_t *a, const dg_observation_t *o, uint8_t current, uint64_t now_us)
{
    if (!a->homes || current > a->max_index) return current;
    const int rel = a->rel_q8[current];
    const uint32_t noise = a->noise_x16[current];
    if (agc_folded(o)) {
        int fold_level;
        if (a->fold_radius && o->fold_r_x16) {
            /* The carrier's own radius, unfolded (agc_level_fold_radius): one
             * decision goes to the row that fits. The window's power reads
             * low, several dB under the carrier, and the old estimate below
             * stepped down one home row per fold (blackbox session 286:
             * G83 -> G79 -> G75 -> G71 -> G68 in 100 ms, a fold each). */
            const uint32_t c = (uint32_t)o->fold_r_x16 * o->fold_r_x16 / 16u;   /* cells^2 x16 */
            fold_level = agc_db_q8(c) - agc_db_q8(16) - rel;
        } else {
            /* No circle (noise peaks folding at the edge, or too much noise
             * to tell): the folded window's own power (the wrapped samples
             * read low), counted LEVEL_FOLD_OVER_Q8 over when the carrier
             * dominates it. Noise peaks folding windows at the edge cost
             * little (level_quality.h). A fixed level at the rails instead
             * read every such window as a strong carrier: with the measured
             * table it held the top row on a strong folding carrier (the
             * rails map to ~6 cells rms, the table's best). */
            const uint32_t c = o->pwr_x16 > noise ? o->pwr_x16 - noise : noise / 50u + 1u;
            const int over = c >= 2u * noise ? LEVEL_FOLD_OVER_Q8 : 0;
            fold_level = agc_db_q8(c) - agc_db_q8(16) - rel + over;
        }
        level_add(a, fold_level, LEVEL_FOLD_WEIGHT);
        a->fold_row = current;
        a->fold_until_us = now_us + a->fold_memory_us;
        a->fold_level_q8 = fold_level;
        a->fold_pending = true;
        a->move_pending = false;   /* a folded level says nothing about the row's gain */
        a->level_windows = 0;
        ++a->folds;
    } else {
        uint32_t c = o->pwr_x16 > noise ? o->pwr_x16 - noise : 0;
        if (c < noise / 50u + 1u) c = noise / 50u + 1u;
        const int level = agc_db_q8(c) - agc_db_q8(16) - rel;
        level_add(a, level, LEVEL_WEIGHT);
        if (a->move_pending && current == a->move_to && now_us - a->move_us >= LEVEL_LEARN_SETTLE_US) {
            a->move_sum_q8 += level;
            if (++a->move_windows >= LEVEL_LEARN_WINDOWS) {
                level_learn(a, current, a->move_sum_q8 / (int)LEVEL_LEARN_WINDOWS - a->move_pre_q8);
                a->move_pending = false;
                a->level_ema_q8 = level;
                a->level_windows = 1;
            }
        } else if (!a->move_pending) {
            /* Only a carrier well above the noise measures a gain. */
            const bool usable = c >= noise;
            if (!usable) a->level_windows = 0;
            else if (!a->level_windows++) a->level_ema_q8 = level;
            else a->level_ema_q8 += (level - a->level_ema_q8) / 4;
        }
    }
    if (!a->fold_pending && now_us - a->decide_us < LEVEL_DECIDE_US) return current;
    a->decide_us = now_us;
    a->fold_pending = false;
    ++a->decisions;
    /* Two histories: the slow one (200 ms) sizes the row for the fading,
     * the fast one (20 ms) sees a jump (a hand over the antenna, an
     * obstacle, coming closer). When their best rows are LEVEL_JUMP_Q8 or
     * more apart the level has moved, not faded: the slow history restarts
     * from the fast one. Board 2026-10-27 (hand on the antenna, -30..-60
     * dBm): the slow history alone held a row that folded 180 windows a
     * second and climbed late when the hand came. */
    int32_t fast_u;
    const unsigned fast_best = level_best(a, a->fast, &fast_u);
    int32_t best_u;
    unsigned best = level_best(a, a->hist, &best_u);
    if (abs(a->rel_q8[a->home[fast_best]] - a->rel_q8[a->home[best]]) >= LEVEL_JUMP_Q8) {
        memcpy(a->hist, a->fast, sizeof(a->hist));
        best = fast_best;
        best_u = fast_u;
        ++a->jumps;
    }
    uint64_t total = 0;
    for (int b = 0; b < AGC_BINS; ++b) total += a->hist[b];
    /* Fold memory: not back into the row that folded (homes run from the top
     * row down), unless the carrier now reads well under the fold. */
    if (a->fold_memory_us && now_us < a->fold_until_us && a->rel_q8[a->home[best]] >= a->rel_q8[a->fold_row] &&
        !(a->level_windows && a->level_ema_q8 < a->fold_level_q8 - LEVEL_FOLD_FAR_Q8)) {
        unsigned below = best;
        while (below + 1u < a->homes && a->rel_q8[a->home[below]] >= a->rel_q8[a->fold_row]) ++below;
        if (a->rel_q8[a->home[below]] < a->rel_q8[a->fold_row]) {
            best = below;
            best_u = level_utility(a, a->hist, best, total);
            ++a->fold_holds;
        }
    }
    const unsigned cur_h = nearest_home(a->home, a->homes, a->rel_q8, current);
    const int32_t cur_u = level_utility(a, a->hist, cur_h, total);
    const unsigned decay = a->release == 1 ? 253u : a->release == 2 ? 244u : LEVEL_DECAY_Q8;
    for (int b = 0; b < AGC_BINS; ++b) {
        a->hist[b] = (uint32_t)(((uint64_t)a->hist[b] * decay) >> 8);
        a->fast[b] = (uint32_t)(((uint64_t)a->fast[b] * LEVEL_FAST_DECAY_Q8) >> 8);
    }
    /* Over the top home (behind an LNA, LEVEL_TOP_NOISE_MAX_X16) no row is
     * home: the noise alone folds there, so go to the best home without the
     * hysteresis (its nearest home would read as "already there"). */
    const bool above_top = current > a->top_index;
    if (!above_top && (best == cur_h || best_u <= cur_u + LEVEL_HYST * 256)) return current;
    ++a->moves;
    a->move_pending = a->level_windows >= 4u;
    if (a->move_pending) {
        a->move_to = a->home[best];
        a->move_pre_q8 = a->level_ema_q8;
        a->move_sum_q8 = 0;
        a->move_windows = 0;
        a->move_us = now_us;
    }
    a->level_windows = 0;
    return a->home[best];
}

/* Magnitude class of a sample, |I| * 8 + |Q| (fold_radius.py mag_class): a
 * token t >= 0 is the cell t + 0.5, a negative one mirrors to -t - 1. */
static inline unsigned fold_class(uint8_t b)
{
    const unsigned i = b >> 4, q = b & 15u;
    return ((i & 8u) ? 15u - i : i) * 8u + ((q & 8u) ? 15u - q : q);
}

/* The ultrafine lane keeps the sign of each component and folds its
 * magnitude modulo 8 cells, so a folded window reads small. An FM carrier
 * is a circle: the radius whose class probabilities (fold_radius_table.h,
 * the exact hardware fold model) explain the window best is the carrier's,
 * folded or not. Host model: exact to the 0.25-cell step for noise up to
 * 0.7 cells per axis, radii 4-24 cells, 64 or 256 samples. Integer only;
 * ~7500 multiply-adds, run on folded windows only. */
uint16_t agc_level_fold_radius(const agc_level_t *a, const uint8_t *samples, unsigned n, uint8_t current,
                               uint16_t *raw_x16, int8_t *fit_out)
{
    if (raw_x16) *raw_x16 = 0;
    if (fit_out) *fit_out = INT8_MIN;
    if (!a->homes || current > a->max_index || !n) return 0;
    /* noise_x16 is cells^2 x16 for both axes; per axis sigma^2 x256 = 8 x it */
    const uint32_t var_x256 = 8u * a->noise_x16[current];
    unsigned s = 0;
    while (s < FOLD_SIGMAS - 1u && var_x256 > (uint32_t)fold_sigma_x16[s] * fold_sigma_x16[s] * 2u) ++s;
    if (var_x256 > (uint32_t)fold_sigma_x16[FOLD_SIGMAS - 1u] * fold_sigma_x16[FOLD_SIGMAS - 1u] * 2u) return 0;
    uint16_t hist[64] = {0};
    for (unsigned k = 0; k < n; ++k) ++hist[fold_class(samples[k])];
    int32_t best = INT32_MIN;
    unsigned best_r = 0;
    for (unsigned r = 0; r < FOLD_R_N; ++r) {
        const int8_t *lp = fold_logp[s][r];
        int32_t score = 0;
        for (unsigned c = 0; c < 64u; ++c) score += (int32_t)hist[c] * lp[c];
        if (score > best) { best = score; best_r = r; }   /* ties: the smaller radius */
    }
    const int32_t fit = best * 16 / (int32_t)n - fold_expect[s][best_r];
    const unsigned r_x16 = FOLD_R0_X16 + best_r * FOLD_R_STEP_X16;
    if (raw_x16) *raw_x16 = (uint16_t)r_x16;
    if (fit_out) *fit_out = (int8_t)(fit < -127 ? -127 : fit > 127 ? 127 : fit);
    return fit >= FOLD_FIT_MIN && r_x16 >= FOLD_R_MIN_X16 ? (uint16_t)r_x16 : 0;
}

bool agc_level_fold_radius_used(void) { return LEVEL_FOLD_RADIUS != 0; }

/* ---- RSSI ---- */

void agc_rssi_reset(agc_rssi_t *a, const int16_t row_db_q8[DG_STATES], uint8_t max_index)
{
    memset(a, 0, sizeof(*a));
    a->max_index = max_index;
    for (unsigned k = 0; k <= max_index; ++k) a->rel_q8[k] = (int16_t)(row_db_q8[k] - row_db_q8[max_index]);
    a->homes = (uint8_t)build_homes(a->home, row_db_q8, max_index);
}

/* The home row whose gain is nearest to `want_q8` (relative to the top). */
static uint8_t rssi_row(const agc_rssi_t *a, int want_q8)
{
    unsigned best = 0;
    int best_d = 1 << 20;
    for (unsigned h = 0; h < a->homes; ++h) {
        const int d = abs(a->rel_q8[a->home[h]] - want_q8);
        if (d < best_d) { best_d = d; best = h; }
    }
    return a->home[best];
}

uint8_t agc_rssi_tick(agc_rssi_t *a, const dg_observation_t *o, uint8_t current, uint64_t now_us)
{
    if (!a->homes || current > a->max_index) return current;
    if (agc_folded(o)) {
        ++a->folds;
        a->sum_x16 = a->windows = 0;
        a->decide_us = now_us;
        const uint8_t to = rssi_row(a, a->rel_q8[current] - RSSI_FOLD_DROP_Q8);
        if (to != current) ++a->moves;
        return to;
    }
    a->sum_x16 += o->pwr_x16;
    ++a->windows;
    if (now_us - a->decide_us < RSSI_DECIDE_US) return current;
    a->decide_us = now_us;
    const uint32_t mean = a->windows ? a->sum_x16 / a->windows : 0;
    a->sum_x16 = a->windows = 0;
    const int err = agc_db_q8(mean ? mean : 1u) - agc_db_q8(RSSI_TARGET_X16);
    if (err > -RSSI_DEADBAND_Q8 && err < RSSI_DEADBAND_Q8) return current;
    const uint8_t to = rssi_row(a, a->rel_q8[current] - err);
    if (to != current) ++a->moves;
    return to;
}
