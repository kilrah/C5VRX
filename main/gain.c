/* C5VRX-5 by Twotoz and the C5VRX contributors: gain core. Decision: KEEL-1 (the V7 peak AGC on 3 dB home rows); measurement and map from Direct Gain V3-V6.
 * Ported from legacy/c5vrx4/main/direct_gain_v3.c (PR #190) with the fixed-lane,
 * staged-recovery, boost-off configuration that ran on the board (V5), plus
 * the V6 carrier memory described in gain.h. */
#include "gain.h"
#include "direct_gain_v3_lut.h"
#include <math.h>
#include <string.h>

/* A Q4 cell represents the half-integer position n+0.5. The lookup is built
 * once; the observer only reads tables and counts a histogram. */
static uint8_t s_power[256];
static uint8_t s_flags[256];
static int8_t s_ci[256], s_cq[256];   /* 2*value+1 per axis */
static bool s_lut_ready;

/* The cell's I and Q value as one-hot bits (I + 8 in bits 0-15, Q + 8 in
 * 16-31): a signed nibble XOR 8 is its value + 8. OR-ing them gives every
 * value seen, so the span needs no per-sample sign extension or min/max. */
static inline uint32_t iq_bits(uint8_t raw)
{
    return 1u << ((raw >> 4) ^ 8u) | 1u << (((raw & 15u) ^ 8u) + 16u);
}

/* Direct mode: act on the first window outside the healthy band and jump
 * straight to the predicted destination. The healthy band stays a strict
 * zero-write zone, so a steady carrier gets no gain writes. */
#define DG_STABLE_WINDOWS 1u
#define DG_HIGH_WINDOWS   1u
#define DG_WEAK_WINDOWS   1u
/* Anti-hunt: two direction reversals within DG_REVERSAL_US arm a damped
 * period in which an out-of-band decision needs DG_DAMPED_WINDOWS windows.
 * Saturation is never damped. */
#define DG_REVERSAL_US     20000u
#define DG_REVERSALS_ARM   2u
#define DG_DAMP_US         200000u
#define DG_DAMPED_WINDOWS  8u
/* A moderate excursion must persist 20 ms (a third of a field) before a
 * write: ordinary FM envelope ripple crossed a band edge 30-100 times a
 * second, and every write is a short PHY transient (board, PR #177). */
#define DG_MODERATE_US     20000u
/* Wi-Fi bursts clip every sample for 0.1-0.5 ms (zerowidth/C5VRX PR #3);
 * saturation counts as overload only after it lasted this long. */
#define DG_BURST_US        1000u
/* Fade hold. A moving drone in an open field passes ground-reflection and
 * multipath nulls of a few milliseconds, after which the carrier is back at
 * its old level. Jumping to the table maximum on the first empty window
 * (+~22 dB from G60) made the returning carrier fold in the ultrafine
 * window: the 1 ms burst gate plus a staged recovery turned every null into
 * ~2 ms of extra loss and 4 writes, and a partial null into ~10 writes
 * (test/test_gain_fade.c, host model). Within DG_FADE_HOLD_US of the last
 * healthy window an empty or weak window does not raise the gain. Motion
 * nulls last ~lambda/2 / speed (2.6 cm: ~3 ms at 10 m/s); a longer hold made
 * a carrier that stays weaker (obstacle) wait: 81 ms instead of 1 ms for a
 * -12 dB step with 100 ms. With 10 ms the cost of a sustained drop is
 * bounded by the hold. A carrier that is really gone still reaches the
 * table maximum; distance fades are slower and tracked as before. */
#define DG_FADE_HOLD_US    10000u
/* V6 carrier memory. After the fade hold a lost carrier climbs from the last
 * healthy gain in steps of DG6_CLIMB_STEP indices (about one RF stage) every
 * DG6_CLIMB_US: a carrier back at its old level, or somewhat weaker, shows up
 * a few dB above it instead of 40 dB above it, and a carrier that is really
 * gone still reaches the table maximum within ~25 ms. Saturation above the
 * remembered gain is that carrier returning: one write back to it, without
 * the 1 ms burst gate and the staged walk (each stage a few ms of folded,
 * clipped picture). A burst there costs one write to a known-good gain. */
#define DG6_CLIMB_STEP     8u
#define DG6_CLIMB_US       3000u

static int clamp_i(int v, int lo, int hi) { return v < lo ? lo : v > hi ? hi : v; }
static int abs_i(int v) { return v < 0 ? -v : v; }

static void prepare_lut(void)
{
    if (s_lut_ready) return;
    for (unsigned raw = 0; raw < 256; ++raw) {
        int q = (int8_t)((raw & 15u) << 4) >> 4;
        int i = (int8_t)(raw & 240u) >> 4;
        int ci = 2 * i + 1, cq = 2 * q + 1;
        int power = (ci * ci + cq * cq + 2) / 4;
        s_power[raw] = (uint8_t)power;
        s_ci[raw] = (int8_t)ci;
        s_cq[raw] = (int8_t)cq;
        s_flags[raw] = (power <= 4 ? 1u : 0u) |
                       ((i == -8 || i == 7 || q == -8 || q == 7) ? 2u : 0u);
    }
    s_lut_ready = true;
}

/* Phase of (x, y) in 256 codes per turn, 0 along +I (c5vrx_phase8_gain_lut
 * on its own grid within one code; test/test_gain_measure.c). */
static uint8_t phase8(int x, int y)
{
    const unsigned ax = (unsigned)abs_i(x), ay = (unsigned)abs_i(y);
    const unsigned lo = ax < ay ? ax : ay, hi = ax < ay ? ay : ax;
    if (!hi) return 0;
    const unsigned t = (lo << 8) / hi;                                  /* tan, Q8 */
    unsigned a = (32u * t + ((11u * t * (256u - t)) >> 8) + 128u) >> 8; /* atan, first octant */
    if (ay > ax) a = 64u - a;
    if (x < 0) a = 128u - a;
    if (y < 0) a = 256u - a;
    return (uint8_t)a;
}

/* A window whose points sit around a DC point far from the origin read as a
 * strong, coherent carrier: power and phase are taken from the origin. On
 * the board (2026-10-19, weak VTX at a distance, after the live switch to
 * MATCHED) 3.5-4 cells of DC with 0.8 cells of carrier and noise around it
 * read P50 ~22 and coherence 96-100, inside the healthy band, so Direct Gain
 * held G71-G77 instead of climbing while the picture was gone. Such a window
 * is judged around its own DC, rounded to whole cells: only when the DC
 * dominates (|mean|^2 >= the spread around it, and >= 1.5 cells) in a probe
 * of >= 256 bytes. A carrier's circle has a small mean against a large
 * spread (research/data/afc_2026-10-18, 840 probes close in: at most 0.74
 * of it; the 2026-10-19 edge snapshots: 1-6 in ~78 % of the probes) and
 * keeps the origin-based result bit for bit; so do the 64-byte
 * sentinel and a fixed IQ point (dg_stuck, measured as before). Clipping,
 * repeats and the span stay raw. */
#define DG_CENTER_MIN_BYTES 256u
static bool dc_dominated(int64_t sx, int64_t sy, uint64_t s2, size_t bytes, int *di, int *dq)
{
    if (bytes < DG_CENTER_MIN_BYTES) return false;
    const int64_t n = (int64_t)bytes;
    const int64_t dc2 = sx * sx + sy * sy;            /* (n * mean)^2, dsp units (2 x cell) */
    const int64_t spread = n * (int64_t)s2 - dc2;     /* n^2 x variance */
    if (dc2 < spread || dc2 < 9 * n * n) return false;
    /* Whole cells: 2n per cell in sums of dsp units; round half away. */
    *di = (int)((sx >= 0 ? sx + n : sx - n) / (2 * n));
    *dq = (int)((sy >= 0 ? sy + n : sy - n) / (2 * n));
    return *di || *dq;
}

static dg_observation_t measure(const uint8_t *samples, size_t bytes,
                                const uint8_t phase8_lut[256], uint64_t observed_us, bool envelope)
{
    dg_observation_t r = {.observed_us = observed_us};
    if (!samples || !bytes || !phase8_lut) return r;
    prepare_lut();
    uint16_t hist[114] = {0};
    unsigned origin = 0, clip = 0, coherent = 0, repeats = 0;
    int32_t cx = s_ci[samples[0]], cy = s_cq[samples[0]];
    uint32_t cp = s_power[samples[0]];
    /* Every per-sample quantity depends only on the byte (tables) or on the
     * byte and its predecessor. The sentinel and the observer run this on
     * ~1.3 MB/s; the former per-sample sign extension, wrap and min/max cost
     * ~125 instructions a byte and with it ~55 % of the CPU, which starved
     * IDLE into the task watchdog (board 2026-10-18, docs/FINDINGS.md).
     * Results are identical (test/test_gain_measure.c). */
    uint32_t iq_seen = iq_bits(samples[0]);
    uint8_t prior_raw = samples[0], prior_phase = phase8_lut[prior_raw];
    ++hist[s_power[prior_raw]];
    unsigned neg_i = prior_raw >> 7, neg_q = (prior_raw >> 3) & 1u;   /* I sign bit 7, Q sign bit 3 */
    origin += s_flags[prior_raw] & 1u;
    clip += s_flags[prior_raw] >> 1;
    for (size_t n = 1; n < bytes; ++n) {
        const uint8_t raw = samples[n], power = s_power[raw], phase = phase8_lut[raw];
        ++hist[power];
        origin += s_flags[raw] & 1u;
        clip += s_flags[raw] >> 1;
        iq_seen |= iq_bits(raw);
        cx += s_ci[raw];
        cy += s_cq[raw];
        neg_i += raw >> 7;
        neg_q += (raw >> 3) & 1u;
        cp += power;
        /* |wrap(phase - prior)| <= 32 codes (45 degrees) */
        coherent += power >= 8u && (uint8_t)(phase - prior_phase + 32u) <= 64u;
        repeats += raw == prior_raw;
        prior_phase = phase;
        prior_raw = raw;
    }
    /* Sum of x^2 + y^2 is 4 * power - 2 per sample (x, y odd). */
    const uint32_t i_seen = iq_seen & 0xffffu, q_seen = iq_seen >> 16;   /* never empty: bytes >= 1 */
    const int i_span = (31 - __builtin_clz(i_seen)) - __builtin_ctz(i_seen);
    const int q_span = (31 - __builtin_clz(q_seen)) - __builtin_ctz(q_seen);
    r.span = (uint8_t)(i_span > q_span ? i_span : q_span);
    r.repeat_pm = bytes > 1u ? (uint16_t)(repeats * 1000u / (bytes - 1u)) : 0;
    r.sum_i = cx;
    r.sum_q = cy;
    r.neg_i = (uint16_t)neg_i;
    r.neg_q = (uint16_t)neg_q;
    r.rails = (uint8_t)((i_seen & 1u) | ((i_seen >> 14) & 2u) | ((q_seen & 1u) << 2) | ((q_seen >> 12) & 8u));
    /* Mean power around the window DC from the sums already taken: in dsp
     * units (2v+1) N * sum(x^2+y^2) - sum(x)^2 - sum(y)^2 over N^2, and a
     * dsp unit is half a cell, so cells^2 x16 is 4x that. */
    {
        const int64_t nb = (int64_t)bytes;
        const int64_t num = (4 * (int64_t)cp - 2 * nb) * nb - (int64_t)cx * cx - (int64_t)cy * cy;
        const int64_t x16 = num > 0 ? 4 * num / (nb * nb) : 0;
        r.pwr_x16 = (uint16_t)(x16 > 65535 ? 65535 : x16);
    }
    int di, dq;
    if (!dg_stuck(&r) && dc_dominated(cx, cy, 4ull * cp - 2ull * bytes, bytes, &di, &dq)) {
        memset(hist, 0, sizeof(hist));
        origin = coherent = 0;
        uint8_t prior = 0;
        for (size_t n = 0; n < bytes; ++n) {
            const int x = s_ci[samples[n]] - 2 * di, y = s_cq[samples[n]] - 2 * dq;
            const int p = (x * x + y * y + 2) / 4;
            const uint8_t power = (uint8_t)(p > 113 ? 113 : p), phase = phase8(x, y);
            ++hist[power];
            origin += power <= 4u;
            if (n) coherent += power >= 8u && (uint8_t)(phase - prior + 32u) <= 64u;
            prior = phase;
        }
        r.centered = 1;
    }
    int64_t sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0, sxp = 0, syp = 0;
    uint64_t spp = 0;
    if (envelope) {
        for (size_t n = 0; n < bytes; ++n) {
            const uint8_t raw = samples[n];
            const int x = 2 * ((int8_t)(raw & 240u) >> 4) + 1;
            const int y = 2 * ((int8_t)((raw & 15u) << 4) >> 4) + 1;
            const int p = x * x + y * y;
            sx += x; sy += y;
            sxx += x * x; syy += y * y; sxy += x * y;
            sxp += x * p; syp += y * p;
            spp += (unsigned)(p * p);
        }
    }
    unsigned cumulative = 0;
    bool p50 = false, p90 = false;
    unsigned r50 = (unsigned)((bytes + 1u) / 2u);
    unsigned r90 = (unsigned)((bytes * 90u + 99u) / 100u);
    unsigned r95 = (unsigned)((bytes * 95u + 99u) / 100u);
    for (unsigned p = 0; p < 114u; ++p) {
        cumulative += hist[p];
        if (!p50 && cumulative >= r50) { r.p50 = (uint8_t)p; p50 = true; }
        if (!p90 && cumulative >= r90) { r.p90 = (uint8_t)p; p90 = true; }
        if (cumulative >= r95) { r.p95 = (uint8_t)p; break; }
    }
    r.origin_pm = (uint16_t)(origin * 1000u / bytes);
    r.clip_pm = (uint16_t)(clip * 1000u / bytes);
    r.coherence = bytes > 1u ? (uint8_t)(coherent * 100u / (bytes - 1u)) : 0;
    /* Envelope C/N: integer raw moments in the existing sample loop, then
     * translate the moments to the window's DC centre once per probe.
     * No per-sample floating point or extra passes in the fast observer. */
    if (envelope) {
        const double inv = 1.0 / (double)bytes;
        const double mx = (double)sx * inv, my = (double)sy * inv;
        const double dc2 = mx * mx + my * my;
        const double mp_raw = (double)(sxx + syy) * inv;
        const double mp = mp_raw - dc2;
        const double mp2 = (double)spp * inv - 4.0 * (mx * (double)sxp + my * (double)syp) * inv
                    + 4.0 * (mx * mx * (double)sxx + 2.0 * mx * my * (double)sxy + my * my * (double)syy) * inv
                    + 2.0 * dc2 * mp_raw - 3.0 * dc2 * dc2;
        const double var = mp2 - mp * mp;
        const double ratio = mp <= 0.0 ? 0.0 : var > 0.0 ? 100.0 * mp * mp / var : 65535.0;
        r.env_ratio_x100 = (uint16_t)(ratio > 65535.0 ? 65535.0 : ratio + .5);
    }
    return r;
}

dg_observation_t dg_measure(const uint8_t *samples, size_t bytes,
                            const uint8_t phase8_lut[256], uint64_t observed_us)
{
    return measure(samples, bytes, phase8_lut, observed_us, true);
}

dg_observation_t dg_measure_gain(const uint8_t *samples, size_t bytes,
                                 const uint8_t phase8_lut[256], uint64_t observed_us)
{
    return measure(samples, bytes, phase8_lut, observed_us, false);
}

static dg_transition_t transition_kind(const direct_gain_t *g, uint8_t from, uint8_t to)
{
    const arc_gain_tuple_t *a = &g->tuple[from], *b = &g->tuple[to];
    if (a->rf_stage != b->rf_stage) return DG_RF;
    if (a->bb_code != b->bb_code) return DG_BB;
    return DG_FINE;
}

static bool carrier(const dg_observation_t *o)
{
    return o->coherence >= 55 && o->p50 >= 5 && o->origin_pm < 650;
}

/* Healthy band P50 13..32 (r ~3.5..5.6 cells), the zero-write zone. */
#define BAND_LO          13
#define BAND_HI          32
#define BAND_P95         65
#define BAND_CLIP_PM     20
#define WEAK_TARGET      17
#define HIGH_TARGET      27
#define HIGH_P90         53
#define HIGH_P95         72
#define SCHMITT_HI       30
#define SCHMITT_P90      47
#define SCHMITT_P95      65
#define SCHMITT_LO       14

static bool valid_learning(const dg_observation_t *o)
{
    return carrier(o) && o->p50 >= 7 && o->p50 <= 45 &&
           o->p95 < 80 && o->clip_pm < 20 && o->origin_pm < 300;
}

static int power_db_q8(unsigned power)
{
    if (power < 1u) power = 1u;
    if (power > 113u) power = 113u;
    return s_dg3_power_db_q8[power];
}

/* Q10 power ratio to Q8 dB with an integer log2 iteration. */
static int ratio_db_q8(int ratio_q10)
{
    if (ratio_q10 < 1) ratio_q10 = 1;
    int64_t x = ((int64_t)ratio_q10 << 16) / 1024;
    int whole = 0;
    while (x < 65536) { x <<= 1; --whole; }
    while (x >= 131072) { x >>= 1; ++whole; }
    int fraction = 0;
    for (int bit = 7; bit >= 0; --bit) {
        x = (x * x) >> 16;
        if (x >= 131072) { x >>= 1; fraction |= 1 << bit; }
    }
    return ((whole * 256 + fraction) * 771 + 128) / 256;
}

#define DG_MAGIC 0x44473521u
static bool same_table(const arc_gain_table_t *a, const arc_gain_table_t *b)
{
    if (!a || !b || a->max_index != b->max_index || a->band5 != b->band5) return false;
    for (unsigned k = 0; k < ARC_RX_STAGE_COUNT; ++k)
        if (a->spans[k] != b->spans[k] || a->start[k] != b->start[k] || a->rf_code[k] != b->rf_code[k]) return false;
    return true;
}

bool dg_map_context_equal(const dg_map_context_t *a, const dg_map_context_t *b)
{
    return a && b && a->freq_mhz == b->freq_mhz && a->lane_mode == b->lane_mode &&
           a->iq_scale == b->iq_scale && a->filter_skirt == b->filter_skirt &&
           a->filter_bw20 == b->filter_bw20 && a->offset_khz == b->offset_khz && a->filter_code == b->filter_code;
}

bool dg_map_matches(const direct_gain_t *g, const arc_gain_table_t *table, const dg_map_context_t *context)
{
    return g && g->magic == DG_MAGIC && g->map_context_valid &&
           dg_map_context_equal(&g->map_context, context) && same_table(&g->table, table);
}
/* ---- KEEL-1: home rows ---------------------------------------------------
 * Every gain row has its own DC, and the live servo learns a row only while
 * the gain stays on it. A gain that visits many rows briefly keeps landing on
 * rows with stale DC (blackbox session 13 at the edge: DC > 1 cell with 15-25 %
 * video, < 0.5 cell with 35-40 %; a steady gain 75 % against 28 % hopping).
 * KEEL-1 uses only home rows KEEL_LATTICE_Q8 apart from the table maximum
 * down: few rows, each visited often enough for the servo, and a write only
 * when the level moved a whole home step. 0 = every row (V7 behaviour).
 *
 * Chosen in test/test_keel_track.c (12 scenarios: close / mid / edge, K 0-10,
 * 5-80 Hz, a -12 dB obstacle; DC per row shrinking with dwell; each write
 * 2 ms of picture): home rows 3 dB apart with a 16 ms peak memory. Score
 * (usable %, edge weighted twice) for a servo time constant of 0.1 / 0.3 /
 * 1 / 3 s: KEEL-1 57.9 / 56.4 / 52.6 / 46.3, V7 (every row, 48 ms) 56.1 /
 * 51.6 / 41.6 / 34.3, V6 52.5 / 49.9 / 44.0 / 37.2. V7 visited so many rows
 * that a slow servo left it worse than V6; home rows 2.5 dB apart scored
 * 2-3 points lower, 3.5 dB about equal. The real servo time per row is not
 * measured: the blackbox (dc per second, gain, writes) will say. */
#ifndef KEEL_LATTICE_Q8
#define KEEL_LATTICE_Q8 768   /* 3 dB */
#endif
static bool magic_ok_early(const direct_gain_t *g) { return g->magic == DG_MAGIC; }
static void keel_build(direct_gain_t *g)
{
    g->keel_rows = 0;
    if (KEEL_LATTICE_Q8 <= 0) return;
    const unsigned top = g->table.max_index;
    uint8_t rows[DG_STATES];
    unsigned n = 0;
    rows[n++] = (uint8_t)top;
    for (int want = g->model_db_q8[top] - KEEL_LATTICE_Q8; want >= g->model_db_q8[2]; want -= KEEL_LATTICE_Q8) {
        unsigned best = 2;
        for (unsigned k = 2; k < rows[n - 1u]; ++k)
            if (abs_i(g->model_db_q8[k] - want) < abs_i(g->model_db_q8[best] - want)) best = k;
        if (best < rows[n - 1u] && g->model_db_q8[rows[n - 1u]] - g->model_db_q8[best] >= KEEL_LATTICE_Q8 / 2)
            rows[n++] = (uint8_t)best;
    }
    for (unsigned k = 0; k < n; ++k) g->keel_row[k] = rows[n - 1u - k];   /* ascending */
    g->keel_rows = (uint8_t)n;
}

/* The highest home row whose model gain is at most that of `from` plus
 * delta (a quarter dB of slack). Up: `from` itself when no higher home row
 * fits. Down: the lowest home row when none fits. */
static uint8_t keel_pick(const direct_gain_t *g, uint8_t from, int delta_q8)
{
    const int want = g->model_db_q8[from] + delta_q8 + 64;
    uint8_t best = 0;
    bool found = false;
    for (unsigned k = 0; k < g->keel_rows; ++k) {
        const uint8_t r = g->keel_row[k];
        const int need = g->model_db_q8[r] + (delta_q8 > 0 && r > from ? (int)g->heat_q8[r] : 0);
        if (need <= want && (delta_q8 > 0 ? r >= from : r < from)) { best = r; found = true; }
    }
    if (found) return best;
    return delta_q8 > 0 ? from : g->keel_row[0];
}


void dg_reset_context(direct_gain_t *g, const arc_gain_table_t *table, uint8_t current_gain,
                      const dg_map_context_t *context)
{
    if (!g) return;
    /* Keep the measured map across tracking resets on an unchanged table:
     * dropping it made V5 re-explore every step (a burst of corrective
     * writes, each a little grain; operator, 2026-10-06). */
    dg_map_blob_t kept;
    dg_map_context_t wanted = {0};
    if (context) wanted = *context; /* context may alias g */
    /* V6 carrier memory survives tracking resets on the same receiver: an
     * AFC step changes only the offset, a PHY lease nothing physical. */
    const bool same_receiver = g->magic == DG_MAGIC && table && same_table(&g->table, table) &&
        g->map_context_valid == (context != NULL) &&
        (!context || (g->map_context.freq_mhz == wanted.freq_mhz && g->map_context.lane_mode == wanted.lane_mode &&
                      g->map_context.iq_scale == wanted.iq_scale && g->map_context.filter_code == wanted.filter_code &&
                      g->map_context.filter_skirt == wanted.filter_skirt && g->map_context.filter_bw20 == wanted.filter_bw20));
    const bool anchor_valid = same_receiver && g->anchor_valid;
    const uint8_t anchor_gain = g->anchor_gain;
    const uint64_t anchor_us = g->anchor_us, carrier_us = g->carrier_us, last_healthy_us = g->last_healthy_us;
    const uint32_t anchor_returns = same_receiver ? g->anchor_returns : 0u, climbs = same_receiver ? g->climbs : 0u;
    /* KEEL-1 state survives the same resets. Board, blackbox session 29:
     * every AFC step (it changes the offset, part of the map context) and
     * every refused write reset the whole core, peak memory and attack
     * memory included, and a core without memory chose again at once: the
     * wrong gains at the edge the model never showed. The counters always
     * survive (a reset zeroed them, and the flight log's per-second
     * differences wrapped to 255). */
    uint8_t keep_peak[sizeof(g->peak_p95)];
    memcpy(keep_peak, g->peak_p95, sizeof(keep_peak));
    const uint8_t keep_peak_i = g->peak_i, keep_folded_row = g->folded_row;
    const uint64_t keep_peak_since = g->peak_since_us, keep_peak_bucket = g->peak_bucket_us, keep_up = g->up_after_us,
                   keep_down = g->down_after_us, keep_folded_until = g->folded_until_us,
                   keep_sat_since = g->saturated_since_us;
    const bool keep_far = g->far_run, keep_sat = g->saturated_run;
    uint16_t keep_heat[DG_STATES];
    memcpy(keep_heat, g->heat_q8, sizeof(keep_heat));
    const uint64_t keep_heat_us = g->heat_us;
    const uint32_t c_heat = magic_ok_early(g) ? g->heat_events : 0u;
    const dg_observation_t keep_last = g->last_tracking;
    const bool magic_ok = g->magic == DG_MAGIC;
    const uint32_t c_writes = magic_ok ? g->writes : 0u, c_mod = magic_ok ? g->writes_moderate : 0u,
                   c_sev = magic_ok ? g->writes_severe : 0u, c_ovl = magic_ok ? g->overloads : 0u,
                   c_holds = magic_ok ? g->holds : 0u, c_bursts = magic_ok ? g->bursts_ignored : 0u,
                   c_one = magic_ok ? g->one_sided_clips : 0u, c_ref = magic_ok ? g->writes_refused : 0u,
                   c_oaw = magic_ok ? g->overloads_after_write : 0u;
    bool keep = g->magic == DG_MAGIC && g->learned && table && same_table(&g->table, table) &&
                g->map_context_valid == (context != NULL) &&
                (!context || dg_map_context_equal(&g->map_context, &wanted)) &&
                dg_export_map(g, &kept) > 0u;
    memset(g, 0, sizeof(*g));
    prepare_lut();
    if (table && table->max_index >= 20u && table->max_index <= ARC_VENDOR_GAIN_MAX) g->table = *table;
    else arc_gain_table_from_bytes(&g->table, NULL, ARC_VENDOR_GAIN_MAX);
    for (unsigned k = 0; k <= g->table.max_index; ++k) {
        (void)arc_gain_tuple_decode(&g->table, (uint8_t)k, &g->tuple[k]);
        const arc_gain_tuple_t *t = &g->tuple[k];
        /* V7 model: 6 dB per RF stage, 6 dB per baseband doubling, 0.83 dB
         * per fine step (fine 0 is the highest). */
        unsigned bb = t->bb_code + 1u, bits_q8 = 0;
        while (bb >= 2u) { bb >>= 1; bits_q8 += 256u; }
        bits_q8 += (unsigned)((((t->bb_code + 1u) << 8) >> (bits_q8 >> 8)) - 256u);   /* linear between powers of two */
        g->model_db_q8[k] = (int16_t)(6 * 256 * (int)t->rf_stage + 6 * (int)bits_q8 + 212 * (5 - (int)t->fine_code));
    }
    keel_build(g);
    /* The receiver floor (apply_target, KEEL-1 home rows) is G2. A floor of
     * G20 here left the core above a hardware gain under G20, and receiver.c
     * rebinds on every window where the two differ: never reached on the bare
     * board (close in G21/G24), every window with an external LNA close in. */
    g->current_gain = (uint8_t)clamp_i(current_gain, 2, g->table.max_index);
    g->target_gain = g->current_gain;
    g->writes = c_writes; g->writes_moderate = c_mod; g->writes_severe = c_sev; g->overloads = c_ovl;
    g->holds = c_holds; g->bursts_ignored = c_bursts; g->one_sided_clips = c_one; g->writes_refused = c_ref;
    g->overloads_after_write = c_oaw;
    if (same_receiver) {
        memcpy(g->peak_p95, keep_peak, sizeof(keep_peak));
        g->peak_i = keep_peak_i; g->peak_since_us = keep_peak_since; g->peak_bucket_us = keep_peak_bucket;
        g->up_after_us = keep_up; g->down_after_us = keep_down; g->far_run = keep_far;
        g->folded_row = keep_folded_row; g->folded_until_us = keep_folded_until;
        g->saturated_run = keep_sat; g->saturated_since_us = keep_sat_since;
        g->last_tracking = keep_last;
        memcpy(g->heat_q8, keep_heat, sizeof(keep_heat));
        g->heat_us = keep_heat_us;
    }
    g->heat_events = c_heat;
    g->relative_power_q10[g->current_gain] = 1024u;
    g->confidence[g->current_gain] = 1u;
    g->uncertainty_pm[g->current_gain] = 50u;
    g->state = DG_ACQUIRE;
    g->magic = DG_MAGIC;
    g->map_context_valid = context != NULL;
    g->map_context = wanted;
    if (keep) (void)dg_import_map(g, &kept);
    if (same_receiver) {
        g->anchor_valid = anchor_valid && anchor_gain >= 2u && anchor_gain <= g->table.max_index;
        g->anchor_gain = anchor_gain;
        g->anchor_us = anchor_us;
        g->carrier_us = carrier_us;
        g->last_healthy_us = last_healthy_us;
        g->anchor_returns = anchor_returns;
        g->climbs = climbs;
    }
}

void dg_reset(direct_gain_t *g, const arc_gain_table_t *table, uint8_t current_gain)
{
    /* Tracking/native handoff on the already-bound context. The observer
     * must use dg_reset_context when the physical transfer context changes. */
    dg_map_context_t context;
    if (g && g->magic == DG_MAGIC && g->map_context_valid) {
        context = g->map_context;
        dg_reset_context(g, table, current_gain, &context);
    } else dg_reset_context(g, table, current_gain, NULL);
    /* A fresh start (no firmware caller; host tests restart their clock):
     * no KEEL-1 memory or counters from before, unlike a re-bind. */
    if (!g) return;
    memset(g->peak_p95, 0, sizeof(g->peak_p95));
    g->peak_i = 0; g->peak_since_us = g->peak_bucket_us = g->up_after_us = g->down_after_us = 0;
    g->far_run = false; g->folded_row = 0; g->folded_until_us = 0;
    g->saturated_run = false; g->saturated_since_us = 0;
    memset(&g->last_tracking, 0, sizeof(g->last_tracking));
    g->writes = g->writes_moderate = g->writes_severe = g->overloads = g->holds = 0;
    g->bursts_ignored = g->one_sided_clips = g->writes_refused = g->overloads_after_write = 0;
    memset(g->heat_q8, 0, sizeof(g->heat_q8)); g->heat_us = 0; g->released_row = 0; g->heat_events = 0;
}

unsigned dg_export_map(const direct_gain_t *g, dg_map_blob_t *blob)
{
    if (!g || !blob) return 0u;
    memset(blob, 0, sizeof(*blob));
    blob->version = DG_MAP_VERSION;
    blob->max_index = g->table.max_index;
    blob->band5 = g->table.band5;
    blob->context_valid = g->map_context_valid;
    blob->context = g->map_context;
    blob->freq_mhz = (uint16_t)g->map_context.freq_mhz;
    blob->lane_mode = (uint8_t)g->map_context.lane_mode;
    /* Canonical fields only: memset above keeps serialization padding zero. */
    memcpy(blob->table.spans, g->table.spans, sizeof(blob->table.spans));
    memcpy(blob->table.start, g->table.start, sizeof(blob->table.start));
    memcpy(blob->table.rf_code, g->table.rf_code, sizeof(blob->table.rf_code));
    blob->table.max_index = g->table.max_index;
    blob->table.band5 = g->table.band5;
    blob->table.runtime_spans_valid = g->table.runtime_spans_valid;
    unsigned confident = 0;
    for (unsigned k = 0; k <= g->table.max_index && k < DG_STATES; ++k) {
        if (!g->confidence[k] || !g->relative_power_q10[k]) continue;
        blob->confidence[k] = g->confidence[k];
        blob->power_q10[k] = g->relative_power_q10[k];
        blob->uncertainty_pm[k] = g->uncertainty_pm[k];
        ++confident;
    }
    return confident > 1u ? confident : 0u;   /* a lone anchor says nothing */
}

bool dg_import_map(direct_gain_t *g, const dg_map_blob_t *blob)
{
    if (!g || !blob || blob->version != DG_MAP_VERSION ||
        blob->max_index != g->table.max_index || blob->band5 != g->table.band5 ||
        !same_table(&blob->table, &g->table) || blob->context_valid != g->map_context_valid ||
        (g->map_context_valid && (!dg_map_context_equal(&blob->context, &g->map_context) ||
         blob->freq_mhz != g->map_context.freq_mhz || blob->lane_mode != g->map_context.lane_mode))) return false;
    const uint8_t anchor = g->current_gain;
    if (!blob->confidence[anchor] || !blob->power_q10[anchor]) return false;
    for (unsigned k = 0; k <= g->table.max_index && k < DG_STATES; ++k) {
        if (!blob->confidence[k] || !blob->power_q10[k]) continue;
        g->relative_power_q10[k] = blob->power_q10[k];
        g->uncertainty_pm[k] = blob->uncertainty_pm[k];
        g->confidence[k] = blob->confidence[k];
    }
    g->learned = 1u;
    return true;
}

bool dg_import_preset(direct_gain_t *g, const uint16_t *power_q10, unsigned count, uint8_t max_index, uint8_t band5)
{
    if (!g || !power_q10 || max_index != g->table.max_index || band5 != g->table.band5) return false;
    const uint8_t anchor = g->current_gain;
    if (anchor >= count || !power_q10[anchor]) return false;
    /* Rescale so the anchor keeps its own value: predict() only uses ratios. */
    const uint32_t base = g->relative_power_q10[anchor] ? g->relative_power_q10[anchor] : 1024u;
    unsigned used = 0;
    for (unsigned k = 0; k <= g->table.max_index && k < count; ++k) {
        if (!power_q10[k] || g->confidence[k] > 1u) continue;   /* own data wins */
        uint32_t v = (uint32_t)power_q10[k] * base / power_q10[anchor];
        g->relative_power_q10[k] = (uint16_t)(v < 1u ? 1u : v > 65535u ? 65535u : v);
        g->uncertainty_pm[k] = 600u;
        g->confidence[k] = 1u;
        ++used;
    }
    return used > 1u;
}

/* Measured responses predict any tuple; without them only a fine step in
 * the same BB bank gets a low-confidence local prior. */
static bool predict(const direct_gain_t *g, uint8_t candidate, int *ratio_q10, int *uncertainty_pm)
{
    uint8_t current = g->current_gain;
    if (g->confidence[current] && g->confidence[candidate]) {
        int base = g->relative_power_q10[current];
        if (!base) return false;
        *ratio_q10 = clamp_i((int)g->relative_power_q10[candidate] * 1024 / base, 1, 8192);
        *uncertainty_pm = clamp_i(g->uncertainty_pm[candidate] + g->uncertainty_pm[current], 80, 900);
        return true;
    }
    if (transition_kind(g, current, candidate) != DG_FINE) return false;
    int steps = (int)g->tuple[current].fine_code - (int)g->tuple[candidate].fine_code;
    if (abs_i(steps) > 2) return false;
    int ratio = 1024;
    while (steps > 0) { ratio = ratio * 121 / 100; --steps; }
    while (steps < 0) { ratio = ratio * 100 / 121; ++steps; }
    *ratio_q10 = ratio;
    *uncertainty_pm = 300;
    return true;
}

static void learn_transition(direct_gain_t *g, const dg_observation_t *after)
{
    if (!g->learn_enabled) { ++g->learn_refused; return; }
    if (!valid_learning(&g->before) || !valid_learning(after) ||
        abs_i((int)g->before.coherence - (int)after->coherence) > 15) return;
    const dg_observation_t *prior = &g->before_previous;
    if (!prior->observed_us || prior->observed_us >= g->before.observed_us ||
        g->before.observed_us - prior->observed_us > 10000u ||
        abs_i((int)prior->p50 - (int)g->before.p50) > clamp_i(g->before.p50 / 4, 2, 10) ||
        abs_i((int)prior->coherence - (int)g->before.coherence) > 12)
        return;
    uint8_t a = g->prior_gain, b = g->current_gain;
    uint32_t base = g->relative_power_q10[a];
    if (!base) return;
    /* A "fine" step the vendor table makes a BB jump is learned anyway, with
     * high uncertainty: rejecting it kept that index mis-predicted. */
    bool mismatch = false;
    if (g->transition == DG_FINE) {
        int steps = abs_i((int)g->tuple[a].fine_code - (int)g->tuple[b].fine_code);
        int high = 1024, low = 1024;
        while (steps-- > 0) { high = high * 18 / 10; low = low * 10 / 18; }
        int observed = (int)after->p50 * 1024 / g->before.p50;
        mismatch = observed < low || observed > high;
    }
    uint32_t estimate = (uint32_t)clamp_i((int)(base * after->p50 / g->before.p50), 1, 65535);
    if (mismatch) ++g->model_mismatches;
    if (!g->confidence[b]) {
        g->relative_power_q10[b] = (uint16_t)estimate;
        g->uncertainty_pm[b] = mismatch ? 600u : 350u;
    } else {
        uint32_t old = g->relative_power_q10[b];
        uint32_t residual_pm = old ? (uint32_t)(abs_i((int)old - (int)estimate) * 1000) / old : 1000u;
        g->uncertainty_pm[b] = (uint16_t)clamp_i((3 * (int)g->uncertainty_pm[b] + (int)residual_pm) / 4, 60, 900);
        if (residual_pm > 450u) {
            if (g->confidence[b]) --g->confidence[b];
            return;
        }
        g->relative_power_q10[b] = (uint16_t)((3u * old + estimate) / 4u);
    }
    if (g->confidence[b] < 15u) ++g->confidence[b];
    ++g->learned;
}

static void note_direction(direct_gain_t *g, int8_t dir, uint64_t now)
{
    if (g->last_write_dir && dir != g->last_write_dir &&
        now >= g->dir_write_us && now - g->dir_write_us < DG_REVERSAL_US) {
        if (++g->reversals >= DG_REVERSALS_ARM) {
            g->damp_until_us = now + DG_DAMP_US;
            g->reversals = 0;
            ++g->damp_events;
        }
    } else if (now < g->dir_write_us || now - g->dir_write_us >= DG_REVERSAL_US) {
        g->reversals = 0;
    }
    g->last_write_dir = dir;
    g->dir_write_us = now;
}

static uint8_t start_write(direct_gain_t *g, const dg_observation_t *o,
                           const dg_observation_t *prior, uint8_t target)
{
    if (target == g->current_gain) return target;
    note_direction(g, target > g->current_gain ? 1 : -1, o->observed_us);
    int ratio_q10 = 0, uncertainty_pm = 0;
    if (predict(g, target, &ratio_q10, &uncertainty_pm)) g->virtual_gain_q8 -= ratio_db_q8(ratio_q10);
    else g->virtual_gain_q8 = 0;
    g->virtual_gain_q8 = clamp_i(g->virtual_gain_q8, -12 * 256, 12 * 256);
    g->before = *o;
    g->before_previous = *prior;
    g->prior_gain = g->current_gain;
    g->target_gain = target;
    g->transition = transition_kind(g, g->current_gain, target);
    g->current_gain = target;
    g->state = DG_SETTLE;
    g->stable_windows = 0;
    g->write_us = o->observed_us;
    ++g->writes;
    return target;
}

static uint64_t settle_guard_us(const direct_gain_t *g)
{
    return g->settle_us[g->transition] > 400u ? (uint64_t)g->settle_us[g->transition] * 3u / 4u : 300u;
}

/* ---- Direct Gain V7: peak AGC ------------------------------------------
 * Board blackbox 2026-10-19 (VTX on, handheld): close in the gain swung over
 * 10-30 steps a second with 255+ writes and overloads, and sync fell to
 * 60-80 %; at the range edge it sat at G66-G81 instead of the maximum. The
 * V6 core caused both: it steered the median level into a band up to P50
 * 32 (radius 5.7 cells, ~2.5 dB below the rails), so every multipath swing
 * clipped, the emergency drop overshot, the next window read weak and the
 * climb (8 steps per 3 ms) folded the carrier again; 2 % of clipping
 * without a rail-side check also dropped the gain for DC. A level loop on a
 * mean (tried first) still followed every fade both ways.
 *
 * V7 is the classic radio AGC on the envelope peaks, fast attack and slow
 * release, which is what the 4-bit window needs: as much gain as the peaks
 * allow.
 * - Attack: clipping on both rails of an axis (DG7_FOLD_PM) in two probes
 *   running, or saturation lasting DG7_SAT_US (Wi-Fi bursts and one-sided
 *   DC clipping never count), steps down to put the peak at DG7_TARGET_P95,
 *   at most DG7_DOWN_MAX steps per DG7_DOWN_GAP_US. P95 alone is no
 *   evidence: at the range edge the noise peaks reach the rails while the
 *   maximum gain is still the best.
 * - Release: only when the highest P95 of the last DG7_HOLD_US stayed below
 *   DG7_RELEASE_P95 does the gain rise, by 70 % of the model difference to
 *   the target, at most DG7_UP_MAX steps per DG7_UP_GAP_US. A multipath
 *   null of a few ms never raises the gain: the peak memory still holds the
 *   level before it.
 * - At the range edge the noise peaks are far below the attack level, so
 *   the gain stays at the table maximum (research/sensitivity_model.py:
 *   2 dB C/N at G83, 3 dB at G72-78, 4 dB at G66).
 * Steps use the model gain per table step: 6 dB per RF stage, 6 dB per
 * baseband doubling, 0.83 dB per fine step (the board's noise fell 4.4-4.9
 * dB per 5-6 steps, 2026-10-19).
 *
 * Parameters from sweeps in test/test_gain_track.c (Rician K 3 and
 * Rayleigh at 30 Hz, Gaussian noise peaks P95 ~ 4.3 x P50 as on the board).
 * Each write costs 2 ms of picture, taken from the blackbox: at the range
 * edge, at equal C/N, a steady gain gave 75 % video and a gain hopping with
 * 255 writes a second gave 28 % (session 13). The cost is the transient plus
 * the DC of briefly visited rows. Chosen: peak memory 48 ms, target P95 30,
 * release below 26, up to 6 steps.
 *
 * Against V6, close in / Rayleigh / mid range at >= 10 dB:
 * 90.5 / 82.9 / 92.0 % vs 83.4 / 77.5 / 84.4 %. At the usable edge (~3 dB
 * C/N), above the threshold: 37.9 vs 35.0 %. Writes: 12-20 instead of
 * 90-125 a second.
 *
 * Rejected:
 * - A 16 ms memory (the first choice, under a 0.4 ms write cost) wrote 29-48
 *   times a second.
 * - A fixed release far under the target (18-22) left the gain a step below
 *   the maximum on receiver noise alone (board, P95 15-19 at G83). */
#ifndef DG7_ATTACK_P95
#define DG7_ATTACK_P95 56u      /* radius ~7.5: the rails */
#endif
#ifndef DG7_FOLD_PM
#define DG7_FOLD_PM 20u      /* two-sided clipping per mille that starts a fold */
#endif
#ifndef DG7_TARGET_P95
#define DG7_TARGET_P95 38u      /* radius ~6.2: 2 dB below the rails, inside the deviation gate (profile_logic.h, P50 <= 40).
                                   * 30 before PR #230: at 540 Hz fading (100 km/h) the 16 ms peak memory always
                                   * holds a fade peak, and the edge sat a home row low (model: 48 -> 51 % usable) */
#endif
#ifndef DG7_RELEASE_P95
#define DG7_RELEASE_P95 30u      /* under the target, above the noise peaks at G83 (P95 15-19); 34 delayed the second climb after a -22 dB step to 31.8 ms */
#endif
#ifndef DG7_HOLD_US
#define DG7_HOLD_US 16000u   /* peak memory (KEEL-1; V7 used 48 ms) */
#endif
#define DG7_BUCKET_US (DG7_HOLD_US / 32u)   /* the memory always spans 32 buckets */
/* Far below the release level (6 dB) for this long, judged on the newest
 * peaks only, a weaker carrier is no multipath null any more: release then
 * instead of after the full memory. A -22 / -12 dB step behind an obstacle
 * recovers in 22 / 14 ms, not 54 / 46 (test_gain_fade); 12 ms released on
 * Rician nulls (close in 89.8 %), 20-32 ms cost the steps 26-38 ms. */
/* Attack memory (KEEL-1): blackbox session 28, walking to the edge
 * (-76..-88 dBm): the gain left a row when a fade peak folded and came back
 * 16 ms later, 255 writes a second between G61 and G79. After an attack the
 * row that folded stays out of reach this long. */
#ifndef DG7_HEAT_STEP_Q8
#define DG7_HEAT_STEP_Q8   384u       /* +1.5 dB per folded release into a row (3 dB parked the edge at G74, PR #230) */
#endif
#define DG7_HEAT_MAX_Q8    3072u      /* at most 12 dB */
#define DG7_HEAT_WINDOW_US 150000u    /* a fold this soon after a release counts */
#define DG7_HEAT_DECAY_US  2000000u   /* -1 dB every 2 s */
#ifndef DG7_FOLD_MEMORY_US
#define DG7_FOLD_MEMORY_US 300000u
#endif
#ifndef DG7_FAR_HOLD_US
#define DG7_FAR_HOLD_US 16000u
#endif
_Static_assert(DG7_HOLD_US / DG7_BUCKET_US <= 32u, "peak memory: 32 buckets (gain.h peak_p95)");
#define DG7_BUCKETS      (DG7_HOLD_US / DG7_BUCKET_US)
#ifndef DG7_STEP_PCT
#define DG7_STEP_PCT 70
#endif
#ifndef DG7_UP_MAX
#define DG7_UP_MAX 6
#endif
#ifndef DG7_UP_GAP_US
#define DG7_UP_GAP_US 8000u
#endif
#ifndef DG7_DOWN_MAX
#define DG7_DOWN_MAX 8
#endif
#ifndef DG7_DOWN_GAP_US
#define DG7_DOWN_GAP_US 1000u
#endif
#ifndef DG7_SAT_US
#define DG7_SAT_US 1000u
#endif
#ifndef DG7_SAT_DROP_DB
#define DG7_SAT_DROP_DB 3   /* one home row; lasting saturation steps again after DG7_DOWN_GAP_US (V6 used 9) */
#endif

/* The table step closest to the model gain of `from` plus delta (dB Q8). */
static uint8_t dg7_index_for(const direct_gain_t *g, uint8_t from, int delta_q8)
{
    if (g->keel_rows) return keel_pick(g, from, delta_q8);
    const int want = g->model_db_q8[from] + delta_q8;
    unsigned best = from;
    int best_err = 1 << 30;
    for (unsigned k = 2; k <= g->table.max_index; ++k) {
        const int err = abs_i(g->model_db_q8[k] - want);
        if (err < best_err || (err == best_err && abs_i((int)k - from) < abs_i((int)best - from))) {
            best = k;
            best_err = err;
        }
    }
    return (uint8_t)best;
}

/* Peak memory: the highest P95 per DG7_BUCKET_US over DG7_HOLD_US,
 * restarted after every write (the gain changed every peak). */
static void dg7_peak_reset(direct_gain_t *g, uint64_t now)
{
    memset(g->peak_p95, 0, sizeof(g->peak_p95));
    g->peak_since_us = now ? now : 1u;
    g->peak_bucket_us = now;
    g->peak_i = 0;
}

static unsigned dg7_peak(direct_gain_t *g, const dg_observation_t *o, unsigned *recent_out)
{
    const uint64_t now = o->observed_us;
    if (now - g->peak_bucket_us > DG7_HOLD_US) {
        memset(g->peak_p95, 0, sizeof(g->peak_p95));
        g->peak_bucket_us = now;
    }
    while (now - g->peak_bucket_us >= DG7_BUCKET_US) {
        g->peak_bucket_us += DG7_BUCKET_US;
        g->peak_i = (uint8_t)((g->peak_i + 1u) % DG7_BUCKETS);
        g->peak_p95[g->peak_i] = 0;
    }
    if (o->p95 > g->peak_p95[g->peak_i]) g->peak_p95[g->peak_i] = o->p95;
    unsigned top = 0, recent = 0;
    for (unsigned k = 0; k < DG7_BUCKETS; ++k) {
        const unsigned v = g->peak_p95[(g->peak_i + DG7_BUCKETS - k) % DG7_BUCKETS];
        if (v > top) top = v;
        if (k < DG7_FAR_HOLD_US / DG7_BUCKET_US && v > recent) recent = v;
    }
    *recent_out = recent;
    return top;
}

uint8_t dg_tick(direct_gain_t *g, const dg_observation_t *o)
{
    if (!g || !o) return 0u;
    dg_observation_t prior = g->last_tracking;
    g->last_tracking = *o;
    const uint64_t now = o->observed_us;
    if (!g->peak_since_us) dg7_peak_reset(g, now);
    if (!g->heat_us || now < g->heat_us) g->heat_us = now;
    else if (now - g->heat_us >= DG7_HEAT_DECAY_US) {
        g->heat_us = now;
        for (unsigned k = 0; k < DG_STATES; ++k) g->heat_q8[k] = g->heat_q8[k] > 256u ? (uint16_t)(g->heat_q8[k] - 256u) : 0u;
    }
    const bool one_sided = o->clip_pm >= 100 && dg_one_sided(o);
    if (one_sided) ++g->one_sided_clips;
    const bool saturated = o->clip_pm >= 100 && !dg_one_sided(o);
    const bool no_carrier = o->p50 <= 4 && o->origin_pm >= 650 && o->coherence < 20;
    if (!no_carrier) g->carrier_us = now;
    /* A write is still settling: its transient is no evidence. */
    if (g->state == DG_SETTLE) {
        if (now <= g->write_us || now - g->write_us < settle_guard_us(g)) {
            /* The write's transient is no saturation evidence either: a run
             * begun on it must not mature across the settle into an overload
             * (blackbox session 32: 3-19 overloads a second at -80..-70 dBm,
             * every overload snapshot clean). */
            g->saturated_run = false;
            return g->current_gain;
        }
        uint64_t elapsed = now - g->write_us;
        if (elapsed < 65535u) {
            uint16_t *settle = &g->settle_us[g->transition];
            *settle = *settle ? (uint16_t)((3u * *settle + elapsed) / 4u) : (uint16_t)elapsed;
        }
        learn_transition(g, o);
        g->state = DG_VERIFY;
        ++g->verified;
        dg7_peak_reset(g, now);
    }
    unsigned recent;
    /* Session 55, G21/G24: an origin-dominated, heavily repeated cloud
     * with sparse outliers has P50=1 but P95=43. It is already classified
     * no_carrier above, yet its P95 kept the release gate closed forever.
     * Use the central population only for release peak memory in this
     * narrow case. Keep the ordinary peak hold, paced gain steps, and raw
     * clipping/attack evidence. In particular this is NOT dg_stuck -> max:
     * off-origin DC and clipped rows must still go through DC correction. */
    dg_observation_t release_o = *o;
    if (no_carrier && o->p50 <= 1u && o->origin_pm >= 850u &&
        o->repeat_pm >= 300u && o->clip_pm == 0u)
        release_o.p95 = o->p50;
    const unsigned peak = dg7_peak(g, &release_o, &recent);
    /* Attack. */
    bool attack = false;
    int delta = 0;
    if (saturated) {
        if (!g->saturated_run) { g->saturated_run = true; g->saturated_since_us = now; }
        if (now - g->saturated_since_us >= DG7_SAT_US) {
            attack = true;
            delta = -DG7_SAT_DROP_DB * 256;
            ++g->overloads;
            if (now - g->write_us < 5000u) ++g->overloads_after_write;
        } else {
            ++g->bursts_ignored;
        }
    } else {
        g->saturated_run = false;
        /* The carrier starts to fold: clipping on both rails of an axis in
         * two probes running. P95 alone is no evidence: at the range edge
         * the noise peaks reach it while the maximum gain is still best. */
        if (o->clip_pm >= DG7_FOLD_PM && prior.clip_pm >= DG7_FOLD_PM && !dg_one_sided(o) && !dg_one_sided(&prior)) {
            attack = true;
            delta = power_db_q8(DG7_TARGET_P95) - power_db_q8(o->p95 > DG7_ATTACK_P95 ? o->p95 : DG7_ATTACK_P95);
        }
    }
    if (attack) {
        if (now < g->down_after_us) return g->current_gain;
        uint8_t target = dg7_index_for(g, g->current_gain, delta);
        if (target >= g->current_gain) target = g->current_gain > 2u ? (uint8_t)(g->current_gain - 1u) : 2u;
        if (!g->keel_rows && target + DG7_DOWN_MAX < g->current_gain) target = (uint8_t)(g->current_gain - DG7_DOWN_MAX);
        g->saturated_run = false;
        g->down_after_us = now + DG7_DOWN_GAP_US;
        g->up_after_us = now + DG7_HOLD_US;
        g->far_run = false;
        /* Attack memory: the row that folded stays out of reach for a while. */
        g->folded_row = g->current_gain;
        g->folded_until_us = now + DG7_FOLD_MEMORY_US;
        /* Learned headroom (KEEL-1): a release that folds within 150 ms was
         * too optimistic for this row, whatever the reason (a table step
         * larger than the model, a fading pattern, an antenna). Blackbox
         * session 34, walking: the attacks were real folds a median 41 ms
         * after a release, P50 ~55 against a target P95 of 30. */
        if (g->released_row == g->current_gain && now - g->released_us < DG7_HEAT_WINDOW_US) {
            const unsigned h = g->heat_q8[g->current_gain] + DG7_HEAT_STEP_Q8;
            g->heat_q8[g->current_gain] = (uint16_t)(h > DG7_HEAT_MAX_Q8 ? DG7_HEAT_MAX_Q8 : h);
            ++g->heat_events;
        }
        g->released_row = 0;
        ++g->writes_severe;
        return start_write(g, o, &prior, target);
    }
    /* Release: only after a full peak memory below the release level. Far
     * below it (6 dB) is no multipath null any more but a carrier gone weaker
     * or gone: DG7_FAR_HOLD_US, the full model step at twice the size, and
     * while it stays far the next step needs only the write to settle. */
    const bool far = recent * 4u < DG7_RELEASE_P95;   /* the last DG7_FAR_HOLD_US only */
    if (!far) g->far_run = false;
    const uint64_t need = far ? (g->far_run ? 0u : DG7_FAR_HOLD_US) : DG7_HOLD_US;
    const unsigned level = far ? recent : peak;
    if (level < DG7_RELEASE_P95 && now - g->peak_since_us >= need &&
        now >= g->up_after_us && g->current_gain < g->table.max_index) {
        delta = (power_db_q8(DG7_TARGET_P95) - power_db_q8(level ? level : 1u)) * (far ? 100 : DG7_STEP_PCT) / 100;
        uint8_t target = dg7_index_for(g, g->current_gain, delta);
        /* KEEL-1 attack memory: never straight back into the row that just
         * folded (unless the carrier is far below: gone weaker). */
        if (!far && now < g->folded_until_us && target >= g->folded_row) {
            uint8_t below = g->current_gain;
            for (unsigned k = 0; k < g->keel_rows; ++k)
                if (g->keel_row[k] < g->folded_row && g->keel_row[k] > below) below = g->keel_row[k];
            if (!g->keel_rows) below = g->folded_row > 0u ? (uint8_t)(g->folded_row - 1u) : 0u;
            target = below;
        }
        if (g->keel_rows) {
            /* KEEL-1: the next home row only when it fits under the target;
             * otherwise stay (no write, no hunting between home rows). */
            if (target > g->current_gain) {
                g->far_run = far;
                g->up_after_us = now + DG7_UP_GAP_US;
                g->released_row = target;
                g->released_us = now;
                ++g->writes_moderate;
                return start_write(g, o, &prior, target);
            }
        } else {
            g->far_run = far;
            if (target <= g->current_gain) target = (uint8_t)(g->current_gain + 1u);
            const unsigned up_max = far ? 2u * DG7_UP_MAX : DG7_UP_MAX;
            if (target > g->current_gain + up_max) target = (uint8_t)(g->current_gain + up_max);
            if (target > g->table.max_index) target = g->table.max_index;
            g->up_after_us = now + DG7_UP_GAP_US;
            ++g->writes_moderate;
            return start_write(g, o, &prior, target);
        }
    }
    g->state = DG_HOLD;
    ++g->holds;
    if (!no_carrier) g->last_healthy_us = now;
    return g->current_gain;
}

/* Board, pair sweep on receiver noise (PR #228): the fine steps of the top
 * rows are 1.0 dB (model 0.83) and the G78/G77 baseband step 0.93 dB (model
 * 1.85); G83 -> G72 is 11.2 dB (model 10.2). Receiver noise is a constant
 * input, so its power per row is the row's real gain. */
#define DG_ROW_STEP_Q8 256   /* 1.0 dB a row (measured, see below) */
unsigned dg_apply_measured_gain(direct_gain_t *g, const float thermal_cells2[DG_STATES])
{
    if (!g || !thermal_cells2) return 0u;
    int top = -1, low = -1;
    for (int k = (int)g->table.max_index; k >= 0; --k)
        if (thermal_cells2[k] > 0.0f) { if (top < 0) top = k; low = k; }
    if (top < 0 || top == low) return 0u;
    int16_t q8[DG_STATES];
    memcpy(q8, g->model_db_q8, sizeof(q8));
    unsigned set = 0;
    for (int k = low; k <= top; ++k) {
        if (thermal_cells2[k] <= 0.0f) continue;
        const float rel = 10.0f * log10f(thermal_cells2[k] / thermal_cells2[top]);
        q8[k] = (int16_t)(g->model_db_q8[top] + (int)lroundf(rel * 256.0f));
        ++set;
    }
    /* Unmeasured rows between measured ones: linear in the row (the board
     * measured ~1 dB a row through the baseband steps the model makes 1.85
     * dB; following the model's shape kept those wrong steps). */
    for (int k = low; k <= top; ++k) {
        if (thermal_cells2[k] > 0.0f) continue;
        int a = k - 1, b = k + 1;
        while (a > low && thermal_cells2[a] <= 0.0f) --a;
        while (b < top && thermal_cells2[b] <= 0.0f) ++b;
        q8[k] = (int16_t)(q8[a] + (q8[b] - q8[a]) * (k - a) / (b - a));
    }
    /* Below the lowest measured row: DG_ROW_STEP_Q8 a row, joined to it. The
     * board's table steps ~1 dB a row wherever it was measured on a carrier
     * or on noise (G19-G31 0.93-1.08, G29-G39, G72-G83 0.94-1.03; PR #231);
     * the RF-stage/BB/fine model has steps of -3.3 and +2.7 dB there and read
     * 4.2 dB short at G24 (the dBm meter 4 dB low on a strong carrier). */
    for (int k = 0; k < low; ++k) q8[k] = (int16_t)(q8[low] - (low - k) * DG_ROW_STEP_Q8);
    memcpy(g->model_db_q8, q8, sizeof(q8));
    keel_build(g);
    return set;
}

void dg_write_refused(direct_gain_t *g, uint8_t actual_gain)
{
    if (!g || g->current_gain == actual_gain) return;
    g->current_gain = actual_gain;
    g->target_gain = actual_gain;
    g->state = DG_HOLD;
    ++g->writes_refused;
}

void dg_sync_applied(direct_gain_t *g, uint8_t gain, uint64_t write_us)
{
    if (!g) return;
    if (gain != g->current_gain) {
        g->current_gain = gain;
        g->target_gain = gain;
        g->stable_windows = 0;
    }
    g->write_us = write_us;
    g->state = DG_SETTLE;
}
