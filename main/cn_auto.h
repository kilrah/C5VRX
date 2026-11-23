/* C5VRX-5 by Twotoz and the C5VRX contributors: C/N-AUTO, a carrier-to-noise
 * estimate that calibrates itself and knows when it cannot be trusted
 * (PR #228, owner review).
 *
 * Why: the envelope estimate (cn_estimate.h) read 3-12 dB low at the range
 * edge (blackbox session 32, against C/N from the IQ snapshots), and a first
 * power fix referred one G83 noise value to every row through the 6 dB gain
 * model, which brings the table's error back in.
 *
 * Design:
 * - Noise is measured per gain row (and per channel and filter, the caller
 *   keys the map). C/N at row k compares the signal power at row k with the
 *   noise measured at row k, so the gain table cancels. A row without its
 *   own measurement borrows the nearest measured row through the model,
 *   with an uncertainty that grows with the model distance.
 * - Noise is learned only with independent evidence of no carrier: a
 *   noise-like envelope over many probes, and a level close to what the row
 *   held before. No video lock alone proves nothing.
 * - Power is taken around a long-term DC of the row, not the window mean:
 *   a short window of a slowly rotating carrier (near 0 Hz) loses its power
 *   to the window mean (research audit, PR #228: up to all of it in one
 *   256-sample window, -0.6 dB on average).
 * - A second estimate from the second and fourth moments (M2M4) checks it.
 *   Disagreement lowers the confidence; it never teaches an offset.
 * - The output carries confidence and source. Unknown is never low.
 * - Only an input-referred power result backed by a noise map may drive
 *   AFC/profile decisions. M2M4 and the envelope model remain useful
 *   diagnostics, but amplitude fading can make either look like thermal
 *   noise without changing the RF C/N (issue #222).
 *
 * Units: moments in cells^2 / cells^4; C/N in 0.1 dB; noise bandwidth in
 * kHz. Pure functions, host test test/test_cn_auto.c. */
#pragma once
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define CNA_ROWS           84u
#define CNA_UNKNOWN_X10    (-990)
#define CNA_MIN_SAMPLES    4u        /* noise windows before a row's value counts as measured */
#define CNA_QUANT_CELLS2   (1.0f / 6.0f)   /* 4-bit quantisation, both axes */

enum { CNA_SRC_NONE = 0, CNA_SRC_POWER = 1, CNA_SRC_M2M4 = 2, CNA_SRC_ENVELOPE = 3 };
enum { CNA_CONF_NONE = 0, CNA_CONF_LOW = 1, CNA_CONF_OK = 2, CNA_CONF_HIGH = 3 };
/* flags */
#define CNA_F_NOISE_BORROWED  0x01u   /* the row's noise came from another row */
#define CNA_F_NOISE_STALE     0x02u
#define CNA_F_DISAGREE        0x04u   /* power and M2M4 differ by more than CNA_AGREE_DB */
#define CNA_F_CLIPPED         0x08u
#define CNA_F_UNSETTLED       0x10u
#define CNA_F_DC_UNKNOWN      0x20u   /* no long-term DC for the row yet */
#define CNA_F_QUANT_LIMITED   0x40u   /* thermal noise below one 4-bit step at this row */
#define CNA_F_NO_CARRIER      0x80u   /* power within 1 dB of the noise: no carrier to rate */
/* cna_reading_t.flags is full: DISTORTED rides in the source (6 bits in the flight log). */
#define CNA_SRC_DISTORTED     0x20u   /* a small carrier on a 4-bit-limited row: read up to +8 dB high */
/* Below this noise-only power (cells^2, thermal sigma ~0.5 cell per axis)
 * a row's noise reading is no noise: with sub-cell noise the samples sit
 * on the cell centres and the reading depends on where the DC falls inside
 * the cell (0 to 0.5 cells^2, test_cn_auto.c). Such rows take the thermal
 * noise of the nearest row above it through the model, plus quantisation. */
#define CNA_QLIM_CELLS2       0.67f
#define CNA_AGREE_DB          4.0f
#define CNA_STALE_US          (3600ll * 1000000ll)   /* a noise value older than an hour is stale */

typedef struct {
    float noise[CNA_ROWS];       /* thermal + quantisation noise power at each row, cells^2 */
    uint16_t n[CNA_ROWS];        /* windows that measured it */
    int64_t us[CNA_ROWS];        /* when it was last measured (0 = loaded/unknown time) */
    float anchor[CNA_ROWS];      /* the clean startup value (two-sided kurtosis gate), 0 = none */
} cna_noise_map_t;

typedef struct {
    int x10;                     /* C/N on the 20 MHz scale of every consumer (cn_estimate.h) */
    int power_x10, m2m4_x10;     /* in the filter's own noise bandwidth; CNA_UNKNOWN_X10 if none */
    uint8_t confidence, source, flags;
    float noise_cells2;          /* the noise used */
} cna_reading_t;

/* The control plane needs input-referred C/N, not merely a plausible number.
 * In particular CNA_SRC_ENVELOPE can read a strong amplitude-varying carrier
 * as a few dB C/N and select a kp level or an AFC gate. Keep non-power estimates in
 * telemetry until a per-context thermal-noise reference exists. */
static inline int cna_control_x10(const cna_reading_t *r)
{
    if (!r || r->x10 == CNA_UNKNOWN_X10 || r->confidence == CNA_CONF_NONE ||
        (r->source & CNA_SRC_DISTORTED) ||
        (r->source & ~CNA_SRC_DISTORTED) != CNA_SRC_POWER ||
        (r->flags & (CNA_F_CLIPPED | CNA_F_UNSETTLED | CNA_F_NO_CARRIER)))
        return CNA_UNKNOWN_X10;
    return r->x10;
}

static inline float cna_db(float r) { return 10.0f * log10f(r > 1e-9f ? r : 1e-9f); }

/* ---- noise map ---- */
/* In service a row may drift only CNA_ANCHOR_DB around its clean startup
 * value (temperature): a carrier near -6 dB C/N passes the absence test
 * (kurtosis ~1.93-1.96) and, learned again and again, crept into the noise
 * (operator, 2026-10-20; B2 row 72 read above row 73). */
#define CNA_ANCHOR_DB 0.5f
static inline void cna_noise_anchor(cna_noise_map_t *m, unsigned row, float power_cells2)
{
    if (row < CNA_ROWS && power_cells2 > 0.0f) m->anchor[row] = power_cells2;
}

static inline void cna_noise_learn(cna_noise_map_t *m, unsigned row, float power_cells2, int64_t now_us)
{
    if (row >= CNA_ROWS || !(power_cells2 > 0.0f)) return;
    if (m->anchor[row] > 0.0f) {
        const float hi = m->anchor[row] * 1.1220185f, lo = m->anchor[row] / 1.1220185f;   /* +-0.5 dB */
        if (power_cells2 > hi) power_cells2 = hi;
        if (power_cells2 < lo) power_cells2 = lo;
    }
    if (!m->n[row]) m->noise[row] = power_cells2;
    else m->noise[row] += (power_cells2 - m->noise[row]) / (m->n[row] < 16u ? (float)(m->n[row] + 1u) : 16.0f);
    if (m->n[row] < 60000u) ++m->n[row];
    m->us[row] = now_us;
}

/* The noise at `row`: its own measurement, or the nearest measured row
 * scaled by the model gain difference (model_q8: dB Q8 per row). The
 * thermal part scales; quantisation does not. *uncert_db grows 0.15 dB per
 * dB of model distance (the model held within ~1 dB per 10 dB over
 * G29-G39, PR #227 pair sweep). */
static inline float cna_noise_at(const cna_noise_map_t *m, unsigned row, const int16_t model_q8[CNA_ROWS],
                                 int64_t now_us, uint8_t *flags, float *uncert_db)
{
    *uncert_db = 99.0f;
    if (row >= CNA_ROWS) return 0.0f;
    if (m->n[row] >= CNA_MIN_SAMPLES && m->noise[row] >= CNA_QLIM_CELLS2) {
        *uncert_db = 0.3f;
        if (m->us[row] && now_us - m->us[row] > CNA_STALE_US) *flags |= CNA_F_NOISE_STALE;
        return m->noise[row];
    }
    /* Borrow the thermal noise of the nearest row that measured real noise. */
    int best = -1, best_d = 1 << 30;
    for (unsigned k = 0; k < CNA_ROWS; ++k) {
        if (m->n[k] < CNA_MIN_SAMPLES || m->noise[k] < CNA_QLIM_CELLS2) continue;
        const int d = model_q8[k] - model_q8[row];
        const int ad = d < 0 ? -d : d;
        if (ad < best_d) { best_d = ad; best = (int)k; }
    }
    if (best < 0) return 0.0f;
    if (m->us[best] && now_us - m->us[best] > CNA_STALE_US) *flags |= CNA_F_NOISE_STALE;
    const float thermal_best = m->noise[best] - CNA_QUANT_CELLS2;
    const float thermal = (thermal_best > 0.0f ? thermal_best : 0.0f) *
                          powf(10.0f, (float)(model_q8[row] - model_q8[best]) / 2560.0f);
    const float total = thermal + CNA_QUANT_CELLS2;
    /* The model's error only matters for the thermal share of the noise. */
    const float share = total > 0.0f ? thermal / total : 0.0f;
    if (m->n[row] >= CNA_MIN_SAMPLES) *flags |= CNA_F_QUANT_LIMITED;
    else *flags |= CNA_F_NOISE_BORROWED;
    *uncert_db = 0.3f + 0.15f * (float)best_d / 256.0f * share;
    return total;
}

/* ---- no-carrier evidence ----
 * Complex Gaussian noise has M4 = 2 M2^2 (|z|^2 exponential); any carrier
 * lowers M4 / M2^2. Averaged over many probes (the caller sums them):
 * kurtosis >= CNA_ABSENT_KURT (a carrier at -6 dB C/N reads ~1.93) and, when
 * the row holds a noise value already, the power within +-2 dB of it. */
#define CNA_ABSENT_KURT 1.95f
static inline bool cna_carrier_absent(double sum_m2, double sum_m4, unsigned probes, float known_noise_cells2)
{
    if (probes < 64u || sum_m2 <= 0.0) return false;
    const double m2 = sum_m2 / probes, m4 = sum_m4 / probes;
    if (m4 / (m2 * m2) < CNA_ABSENT_KURT) return false;
    if (known_noise_cells2 > 0.0f) {
        const float d = cna_db((float)m2 / known_noise_cells2);
        /* Upward only 0.5 dB at a time: a carrier far below the noise that
         * slipped past the kurtosis test can never creep the noise up (and
         * the C/N down); downward up to 2 dB (a cooler chip, a new filter). */
        if (d > 0.5f || d < -2.0f) return false;
    }
    return true;
}

/* ---- a front end that changed (an external LNA fitted or removed) ----
 * The no-carrier test above only takes a row within +0.5/-2 dB of the noise
 * it knows, so a map learned with an LNA never followed its removal (17 dB
 * lower; board 2026-11-14) and a map learned bare would never take the
 * LNA's noise (17 dB higher). A carrier cannot do either: a weak one (the
 * case the 0.5 dB guards) adds under ~1 dB, a strong one fails the kurtosis
 * test, and none lowers the power. So a noise-like window (kurtosis >=
 * CNA_ABSENT_KURT, not clipped: the caller's test) more than
 * CNA_REBASE_UP_DB over or CNA_REBASE_DOWN_DB under the known noise is a
 * new front end: the whole map moves by the same thermal ratio (it scales
 * on every row alike), its anchors go, and the row takes the measurement. */
#define CNA_REBASE_UP_DB   6.0f
#define CNA_REBASE_DOWN_DB 2.0f
static inline bool cna_front_end_changed(double sum_m2, double sum_m4, unsigned probes, float known_noise_cells2)
{
    if (probes < 64u || sum_m2 <= 0.0 || !(known_noise_cells2 > 0.0f)) return false;
    const double m2 = sum_m2 / probes, m4 = sum_m4 / probes;
    if (m4 / (m2 * m2) < CNA_ABSENT_KURT) return false;
    const float d = cna_db((float)m2 / known_noise_cells2);
    return d > CNA_REBASE_UP_DB || d < -CNA_REBASE_DOWN_DB;
}
/* expected_cells2: the row's noise the map expects (cna_noise_at: its own or
 * the nearest real row's through the gain model), at least CNA_QLIM_CELLS2. */
static inline void cna_noise_rebase(cna_noise_map_t *m, unsigned row, float power_cells2, float expected_cells2,
                                    int64_t now_us)
{
    if (row >= CNA_ROWS || !(power_cells2 > CNA_QUANT_CELLS2) || !(expected_cells2 >= CNA_QLIM_CELLS2)) return;
    const float ratio = (power_cells2 - CNA_QUANT_CELLS2) / (expected_cells2 - CNA_QUANT_CELLS2);
    for (unsigned k = 0; k < CNA_ROWS; ++k) {
        m->anchor[k] = 0.0f;
        if (!m->n[k] || k == row) continue;
        const float th = m->noise[k] - CNA_QUANT_CELLS2;
        m->noise[k] = (th > 0.0f ? th * ratio : 0.0f) + CNA_QUANT_CELLS2;
        m->us[k] = now_us;
    }
    m->noise[row] = power_cells2;
    m->n[row] = CNA_MIN_SAMPLES;
    m->us[row] = now_us;
}

/* ---- moments around a given DC ----
 * Integer sums in dsp units (x = 2v + 1 for a cell value v, so a dsp unit is
 * half a cell): cheap on the C5 (no double FPU), exact. p = x^2 + y^2 <= 450,
 * p^2 <= 202500: a probe's sums fit easily in int64. */
typedef struct { int64_t sx, sy, sxx, syy, sxy, sxp, syp, spp; uint32_t n; } cna_sums_t;
static inline void cna_sums_add(cna_sums_t *s, uint8_t raw)
{
    const int32_t x = 2 * ((int8_t)(raw & 240u) >> 4) + 1;          /* I: high nibble */
    const int32_t y = 2 * ((int8_t)((raw & 15u) << 4) >> 4) + 1;    /* Q: low nibble */
    const int32_t p = x * x + y * y;
    s->sx += x; s->sy += y; s->sxx += x * x; s->syy += y * y; s->sxy += x * y;
    s->sxp += x * p; s->syp += y * p; s->spp += (int64_t)p * p; ++s->n;
}
static inline void cna_sums_merge(cna_sums_t *a, const cna_sums_t *b)
{
    a->sx += b->sx; a->sy += b->sy; a->sxx += b->sxx; a->syy += b->syy; a->sxy += b->sxy;
    a->sxp += b->sxp; a->syp += b->syp; a->spp += b->spp; a->n += b->n;
}
/* M2 = E|z - dc|^2, M4 = E|z - dc|^4 in cells (dc in cells). */
static inline void cna_moments(const cna_sums_t *s, float dci, float dcq, double *m2, double *m4)
{
    const double inv = 1.0 / (double)(s->n ? s->n : 1u);
    /* dsp -> cells: x/2, x^2/4, x p / 8, p^2 / 16 */
    const double mx = 0.5 * (double)s->sx * inv, my = 0.5 * (double)s->sy * inv, a = dci, b = dcq;
    const double ex2 = 0.25 * (double)s->sxx * inv, ey2 = 0.25 * (double)s->syy * inv, exy = 0.25 * (double)s->sxy * inv;
    const double ep = ex2 + ey2, exp_ = 0.125 * (double)s->sxp * inv, eyp = 0.125 * (double)s->syp * inv;
    const double ep2 = 0.0625 * (double)s->spp * inv;
    const double d2 = a * a + b * b;
    *m2 = ep - 2.0 * (a * mx + b * my) + d2;
    const double eu = a * mx + b * my;
    const double eu2 = a * a * ex2 + 2.0 * a * b * exy + b * b * ey2;
    const double epu = a * exp_ + b * eyp;
    *m4 = ep2 + 4.0 * eu2 + d2 * d2 - 4.0 * epu + 2.0 * d2 * ep - 4.0 * d2 * eu;
}
static inline void cna_mean(const cna_sums_t *s, float *dci, float *dcq)
{
    const double inv = 1.0 / (double)(s->n ? s->n : 1u);
    *dci = (float)(0.5 * (double)s->sx * inv);
    *dcq = (float)(0.5 * (double)s->sy * inv);
}

/* ---- the estimate ----
 * m2, m4: moments of z - dc over the averaging window (cells). noise: the
 * row's noise (cna_noise_at). nbw_khz: the filter's measured noise
 * bandwidth (0 = unknown: the class + 1.6 MHz). */
static inline cna_reading_t cna_estimate(double m2, double m4, float noise, float noise_uncert_db, uint8_t flags,
                                         unsigned nbw_khz, int class_mhz, int envelope_x10)
{
    cna_reading_t r = {.x10 = CNA_UNKNOWN_X10, .power_x10 = CNA_UNKNOWN_X10, .m2m4_x10 = CNA_UNKNOWN_X10,
                       .flags = flags, .noise_cells2 = noise};
    if (flags & (CNA_F_CLIPPED | CNA_F_UNSETTLED)) return r;
    const float bw_mhz = nbw_khz ? (float)nbw_khz / 1000.0f : (float)class_mhz + 1.6f;
    const float to20 = cna_db(20.7f / bw_mhz);
    bool have_power = false, have_m2m4 = false;
    float cn_power = 0.0f, cn_m2m4 = 0.0f;
    if (noise > 0.0f && noise_uncert_db < 6.0f) {
        /* The carrier over the receiver's thermal noise, the C/N at the pin
         * that the kp curves, the gates and the dBm meter are defined on:
         * on rows near the 4-bit step the quantisation (part of the row's
         * noise) is no noise of the link, and dividing by it read up to ~2 dB
         * low there (walk 2026-10-20, A1). */
        const float c = (float)m2 - noise;
        const float th = noise - CNA_QUANT_CELLS2 > 0.25f * noise ? noise - CNA_QUANT_CELLS2 : 0.25f * noise;
        cn_power = c > 0.02f * th ? cna_db(c / th) : -17.0f;
        have_power = true;
        r.power_x10 = (int)lroundf(10.0f * cn_power);
    }
    {   /* M2M4: S = sqrt(2 M2^2 - M4), N = M2 - S (ideal complex AWGN) */
        const double s2 = 2.0 * m2 * m2 - m4;
        const double s = s2 > 0.0 ? sqrt(s2) : 0.0;
        const double n = m2 - s;
        if (n > 0.0 && m2 > 0.0) {
            cn_m2m4 = s > 0.0 ? cna_db((float)(s / n)) : -17.0f;
            have_m2m4 = true;
            r.m2m4_x10 = (int)lroundf(10.0f * cn_m2m4);
        }
    }
    unsigned conf;
    if (have_power) {
        r.source = CNA_SRC_POWER;
        r.x10 = (int)lroundf(10.0f * (cn_power - to20));
        /* Below the noise (power within 1 dB of it, C/N < -6 dB): no carrier,
         * so no C/N. Consumers held on unknown before; a number here would
         * read as a very weak carrier. */
        if (cn_power < -6.0f) { r.flags |= CNA_F_NO_CARRIER; r.x10 = CNA_UNKNOWN_X10; r.confidence = CNA_CONF_NONE; return r; }
        conf = noise_uncert_db <= 1.0f ? CNA_CONF_HIGH : noise_uncert_db <= 3.0f ? CNA_CONF_OK : CNA_CONF_LOW;
        if (flags & CNA_F_NOISE_STALE) conf = conf > CNA_CONF_LOW ? conf - 1u : conf;
        /* The cross-check only where M2M4 holds: a carrier well inside the
         * window and above the noise, but not so far above that the 4-bit
         * envelope's own variance dominates (2-15 dB). */
        if (have_m2m4 && cn_power >= 2.0f && cn_power <= 15.0f) {
            const float d = fabsf(cn_power - cn_m2m4);
            if (d > CNA_AGREE_DB) { r.flags |= CNA_F_DISAGREE; conf = conf > CNA_CONF_LOW ? conf - 1u : conf; }
        }
    } else if (have_m2m4 && cn_m2m4 >= 0.0f && cn_m2m4 <= 15.0f) {
        r.source = CNA_SRC_M2M4;
        r.x10 = (int)lroundf(10.0f * (cn_m2m4 - to20));
        conf = CNA_CONF_LOW;
    } else if (envelope_x10 != CNA_UNKNOWN_X10) {
        r.source = CNA_SRC_ENVELOPE;
        r.x10 = envelope_x10;
        conf = CNA_CONF_LOW;
    } else {
        conf = CNA_CONF_NONE;
    }
    if (flags & CNA_F_DC_UNKNOWN) conf = conf > CNA_CONF_LOW ? conf - 1u : conf;
    /* A small carrier on a row whose noise is below one step is distorted
     * by the 4-bit steps, not noisy: never more than LOW (the gain core does
     * not hold a weak carrier there; strong ones, radius >= 2 cells, read
     * within ~2 dB). */
    if ((flags & CNA_F_QUANT_LIMITED) && m2 < 4.0) {
        if (conf > CNA_CONF_LOW) conf = CNA_CONF_LOW;
        r.source |= CNA_SRC_DISTORTED;   /* no number for decisions (receiver_obs_cn_x10) */
    }
    r.confidence = (uint8_t)conf;
    if (conf == CNA_CONF_NONE) r.x10 = CNA_UNKNOWN_X10;
    if (r.x10 != CNA_UNKNOWN_X10) { if (r.x10 > 300) r.x10 = 300; if (r.x10 < -30) r.x10 = -30; }
    return r;
}
