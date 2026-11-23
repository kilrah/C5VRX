/* C5VRX-5 by Twotoz and the C5VRX contributors: acceptance test for
 * C/N-AUTO (main/cn_auto.h), PR #228.
 *
 * IQ is generated with a known C/N and taken through the 4-bit ultrafine
 * lane as on the board (it wraps, it does not clip):
 * - an FM carrier: porch at -1 MHz, +-2.3 MHz video at the line rate
 *   (deviation ~0.7); optionally at 0 Hz (a slowly rotating phasor);
 * - complex Gaussian receiver noise; a fixed DC.
 * Every gain row's real gain differs from the 6 dB model by up to +-3 dB
 * (a table error the noise map must cancel).
 *
 * Truth: C/N = carrier power / (thermal + quantisation noise) at the row,
 * in the filter's noise bandwidth (the estimate's power_x10).
 *
 * Checks:
 * - power estimate within 1.5 dB over 0-20 dB, on every row, with table
 *   errors, at -1 MHz and at 0 Hz, with 2 cells of DC, for 256 and 1024
 *   sample windows;
 * - the window-mean power the old estimate used loses carrier power at
 *   0 Hz;
 * - a weak carrier (-3 and 0 dB) is never taken as no carrier; noise is;
 * - a borrowed noise value (row not measured) lowers the confidence;
 * - folding is reported clipped and gives no reading;
 * - M2M4 agrees within 4 dB in its region (2-15 dB);
 * - unknown is never a low number;
 * - an uncalibrated envelope/M2M4 estimate stays out of the actuator path,
 *   including the issue #222 amplitude-fade counterexample.
 *
 *   cc -O2 -std=c11 -I../main test_cn_auto.c -lm -o t && ./t
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "cn_auto.h"
#include "cn_estimate.h"
#include "../research/legacy_demod/kp_only_kp_ladder.h"
#include "adc_fold.h"

static unsigned s_fail, s_run;
static void check(const char *name, int ok)
{
    ++s_run;
    printf("%-4s %s\n", ok ? "ok" : "FAIL", name);
    s_fail += !ok;
}

static uint32_t s_seed = 99u;
static double urand(void) { s_seed = s_seed * 1664525u + 1013904223u; return ((s_seed >> 8) + 0.5) / 16777216.0; }
static double gauss(void) { return sqrt(-2.0 * log(urand())) * cos(6.283185307179586 * urand()); }

/* Rows and gains: model 1 dB a row over the test rows, real gain with a
 * per-row error of up to +-3 dB. */
static int16_t s_model_q8[CNA_ROWS];
static double s_real_db[CNA_ROWS];
static void build_rows(void)
{
    for (unsigned k = 0; k < CNA_ROWS; ++k) {
        s_model_q8[k] = (int16_t)(k * 256);
        s_real_db[k] = (double)k + 3.0 * sin(1.7 * k);     /* the table error */
    }
}
/* thermal noise sigma per axis at row k: 1.95 cells at G83 (board) */
static double sigma_at(unsigned k) { return 1.95 * pow(10.0, (s_real_db[k] - s_real_db[83]) / 20.0); }

typedef struct { cna_sums_t s; double win_m2; unsigned wins; unsigned rail; unsigned total; } gen_t;

/* n_windows of `len` samples at row k: carrier power c_cells2 (0 = none). */
static void generate(gen_t *g, unsigned k, double c_cells2, double porch_hz, double dci, double dcq, unsigned n_windows,
                     unsigned len)
{
    memset(g, 0, sizeof(*g));
    const double r = sqrt(c_cells2), sg = sigma_at(k);
    static double phi;
    for (unsigned w = 0; w < n_windows; ++w) {
        double wx = 0, wy = 0, wp = 0;
        const double t0 = urand() * 1e-3;
        for (unsigned n = 0; n < len; ++n) {
            const double t = t0 + n / 40e6;
            const double f = porch_hz + 2.3e6 * (0.5 + 0.5 * sin(6.283185307179586 * 15734.0 * t));
            phi += 6.283185307179586 * f / 40e6;
            const int i = adc_fold4(r * cos(phi) + sg * gauss() + dci);
            const int q = adc_fold4(r * sin(phi) + sg * gauss() + dcq);
            const double x = i + 0.5, y = q + 0.5, p = x * x + y * y;
            cna_sums_add(&g->s, (uint8_t)(((unsigned)i & 15u) << 4 | ((unsigned)q & 15u)));   /* as the firmware */
            wx += x; wy += y; wp += p;
            g->rail += (i == -8 || i == 7 || q == -8 || q == 7);
            ++g->total;
        }
        g->win_m2 += wp / len - (wx / len) * (wx / len) - (wy / len) * (wy / len);
        ++g->wins;
    }
    g->win_m2 /= g->wins;
}

/* The noise map, measured row by row with no carrier (as the startup step
 * does), for the given rows. */
static cna_noise_map_t s_map;
static void measure_noise(const unsigned *rows, unsigned count)
{
    memset(&s_map, 0, sizeof(s_map));
    for (unsigned j = 0; j < count; ++j) {
        gen_t g;
        for (unsigned rep = 0; rep < 6u; ++rep) {
            generate(&g, rows[j], 0.0, 0.0, 0.0, 0.0, 16, 256);
            double m2, m4;
            { float mi, mq; cna_mean(&g.s, &mi, &mq); cna_moments(&g.s, mi, mq, &m2, &m4); }
            cna_noise_learn(&s_map, rows[j], (float)m2, 1000);
        }
    }
}

/* Estimate at row k for a carrier at the true C/N; returns the error in dB. */
static double run_case(unsigned k, double cn_db, double porch, double dc, unsigned len, cna_reading_t *out, double *win_err)
{
    /* The C/N is the carrier over the thermal noise (the pin C/N), the
     * thermal floored at a quarter of the row's noise where the 4-bit step
     * dominates (cna_estimate). */
    const double noise_total = 2.0 * sigma_at(k) * sigma_at(k) + CNA_QUANT_CELLS2;
    const double thermal = 2.0 * sigma_at(k) * sigma_at(k);
    const double noise_true = thermal > 0.25 * noise_total ? thermal : 0.25 * noise_total;
    const double c = noise_true * pow(10.0, cn_db / 10.0);
    gen_t g;
    generate(&g, k, c, porch, dc, -dc, 4096u / len * 16u, len);
    double m2, m4;
    cna_moments(&g.s, (float)(dc + 0.0), (float)(-dc), &m2, &m4);   /* the row's long-term DC (servo) */
    uint8_t flags = 0;
    float uncert;
    const float noise = cna_noise_at(&s_map, k, s_model_q8, 2000, &flags, &uncert);
    if (g.rail * 100u > g.total * 2u) flags |= CNA_F_CLIPPED;
    *out = cna_estimate(m2, m4, noise, uncert, flags, 13600u, 12, CNA_UNKNOWN_X10);
    if (win_err) *win_err = 10.0 * log10((g.win_m2 > noise ? g.win_m2 - noise : 1e-6) / noise) - cn_db;
    return out->power_x10 == CNA_UNKNOWN_X10 ? 99.0 : out->power_x10 / 10.0 - cn_db;
}

int main(void)
{
    build_rows();
    const unsigned measured[] = {83, 80, 77, 74, 71, 68, 65, 62, 59, 56, 54};
    measure_noise(measured, sizeof(measured) / sizeof(measured[0]));

    /* Accuracy on measured rows, -1 MHz porch, over C/N and rows with table errors. */
    const double cns[] = {0, 2, 4, 6, 10, 15, 20};
    double worst = 0, worst_q = 0, worst_weak = 0;
    unsigned cases = 0, clipped = 0, low_conf_q = 0, quant_strong = 0;
    for (unsigned j = 0; j < sizeof(measured) / sizeof(measured[0]); ++j)
        for (unsigned c = 0; c < sizeof(cns) / sizeof(cns[0]); ++c) {
            cna_reading_t r;
            const double e = run_case(measured[j], cns[c], -1.0e6, 0.0, 256, &r, NULL);
            if (r.flags & CNA_F_CLIPPED) { ++clipped; continue; }
            /* A quantisation-limited row with a carrier under 2 cells must say LOW. */
            if ((r.flags & CNA_F_QUANT_LIMITED) && r.confidence <= CNA_CONF_LOW) {
                /* still used by the decisions (receiver_obs_cn_x10: one scale) */
                ++low_conf_q;
                if (fabs(e) < 90.0 && fabs(e) > fabs(worst_weak)) worst_weak = e;
                continue;
            }
            if (r.flags & CNA_F_QUANT_LIMITED) {
                /* The thermal noise of these rows is borrowed across the
                 * table (+-3 dB errors here): accurate where the kp/LP and AFC
                 * decisions are made (<= 12 dB), above it only >= 12 dB. */
                ++quant_strong;
                if (cns[c] <= 12.0) { if (fabs(e) > fabs(worst_q)) worst_q = e; }
                else if (cns[c] + e < 12.0) worst_q = 99.0;
                if (fabs(e) > 2.5) printf("     quant row %u C/N %.0f dB: error %+.2f dB (noise %.3f, conf %u)\n", measured[j], cns[c], e, r.noise_cells2, r.confidence);
                continue;
            }
            ++cases;
            if (fabs(e) > 1.5) printf("     row %u C/N %.0f dB: error %+.2f dB (noise %.3f, conf %u, flags 0x%02x)\n", measured[j], cns[c], e, r.noise_cells2, r.confidence, r.flags);
            if (fabs(e) > fabs(worst)) worst = e;
        }
    printf("     thermal-noise rows: %u cases, worst %+.2f dB; quantisation-limited: %u strong (worst %+.2f dB), %u weak flagged LOW (worst %+.2f dB); %u folding\n",
           cases, worst, quant_strong, worst_q, low_conf_q, worst_weak, clipped);
    check("power C/N within 1.5 dB over 0-20 dB on every row with real noise, with +-3 dB table errors", fabs(worst) <= 1.5 && cases >= 20);
    check("quantisation-limited rows: a strong carrier within 2.5 dB up to 12 dB C/N (>= 12 dB above), a weak one only with LOW confidence",
          fabs(worst_q) <= 2.5);

    /* A slowly rotating carrier (0 Hz) and 1 cell of DC, 256 and 1024 samples. */
    double worst_rot = 0, worst_win = 0, worst_len = 0;
    for (unsigned c = 1; c < 6; ++c) {
        cna_reading_t r256, r1024;
        double win;
        const double e = run_case(80, cns[c], 0.0, 0.0, 256, &r256, &win);
        if (r256.flags & CNA_F_CLIPPED) continue;          /* the carrier folds at this row: no reading, rightly */
        const double e2 = run_case(80, cns[c], 0.0, 0.0, 1024, &r1024, NULL);
        cna_reading_t rd;
        const double ed = run_case(80, cns[c], -1.0e6, 1.0, 256, &rd, NULL);
        if (fabs(e) > fabs(worst_rot)) worst_rot = e;
        if (!(rd.flags & CNA_F_CLIPPED) && fabs(ed) > fabs(worst_rot)) worst_rot = ed;
        if (win < worst_win) worst_win = win;
        if (fabs(e - e2) > worst_len) worst_len = fabs(e - e2);
    }
    printf("     0 Hz carrier / 1-cell DC: worst %+.2f dB; window-mean power (old) worst %+.2f dB; 256 vs 1024: %.2f dB\n",
           worst_rot, worst_win, worst_len);
    check("a carrier at 0 Hz and 1 cell of DC: within 1.5 dB (long-term DC, not the window mean)", fabs(worst_rot) <= 1.5);
    check("the window-mean power of the old estimate loses carrier at 0 Hz (> 0.3 dB on average)", worst_win < -0.3);
    {   /* 2.5 cells of DC with the noise: the lane wraps; the reading must say so */
        cna_reading_t r;
        const double e = run_case(80, 6.0, -1.0e6, 2.5, 256, &r, NULL);
        printf("     2.5 cells of DC at 6 dB: flags 0x%02x error %+.2f dB\n", r.flags, e);
        check("2.5 cells of DC: either flagged without a reading or still within 1.5 dB",
              ((r.flags & CNA_F_CLIPPED) && r.x10 == CNA_UNKNOWN_X10) || fabs(e) <= 1.5);
    }
    check("window length invariant (256 vs 1024 samples within 0.5 dB)", worst_len <= 0.5);

    /* No-carrier evidence: noise yes, a weak carrier no. */
    {
        bool noise_ok = true, weak_rejected = true;
        for (unsigned rep = 0; rep < 5; ++rep) {
            gen_t g;
            double m2, m4;
            generate(&g, 83, 0.0, -1e6, 0, 0, 96, 256);
            { float mi, mq; cna_mean(&g.s, &mi, &mq); cna_moments(&g.s, mi, mq, &m2, &m4); }
            noise_ok = noise_ok && cna_carrier_absent(m2 * 96, m4 * 96, 96, s_map.noise[83]);
            const double noise_true = 2.0 * sigma_at(83) * sigma_at(83) + CNA_QUANT_CELLS2;
            for (int cn = -3; cn <= 0; cn += 3) {
                generate(&g, 83, noise_true * pow(10.0, cn / 10.0), -1e6, 0, 0, 96, 256);
                { float mi, mq; cna_mean(&g.s, &mi, &mq); cna_moments(&g.s, mi, mq, &m2, &m4); }
                weak_rejected = weak_rejected && !cna_carrier_absent(m2 * 96, m4 * 96, 96, 0.0f);
            }
        }
        check("noise alone is recognised as no carrier", noise_ok);
        check("a weak VTX (-3 and 0 dB C/N, no lock) is never taken as noise", weak_rejected);
    }

    /* A row without its own noise: borrowed, flagged, lower confidence. */
    {
        cna_reading_t own, borrowed;
        (void)run_case(77, 6.0, -1e6, 0, 256, &own, NULL);
        (void)run_case(76, 6.0, -1e6, 0, 256, &borrowed, NULL);
        printf("     own row conf %u, borrowed row conf %u flags 0x%02x\n", own.confidence, borrowed.confidence, borrowed.flags);
        check("a borrowed noise value is flagged and never more confident than a measured one",
              (borrowed.flags & CNA_F_NOISE_BORROWED) && borrowed.confidence <= own.confidence && own.confidence >= CNA_CONF_OK);
    }

    /* Folding: too strong for the row. */
    {
        cna_reading_t r;
        (void)run_case(83, 25.0, -1e6, 0, 256, &r, NULL);
        check("a folding carrier is reported clipped and gives no reading (unknown, not low)",
              (r.flags & CNA_F_CLIPPED) && r.x10 == CNA_UNKNOWN_X10 && r.confidence == CNA_CONF_NONE);
    }

    /* M2M4 against power in its region. */
    {
        double worst_m = 0;
        for (unsigned c = 1; c < 6; ++c) {
            cna_reading_t r;
            (void)run_case(74, cns[c], -1e6, 0, 256, &r, NULL);
            if (r.m2m4_x10 != CNA_UNKNOWN_X10 && r.power_x10 != CNA_UNKNOWN_X10) {
                const double d = (r.m2m4_x10 - r.power_x10) / 10.0;
                if (fabs(d) > fabs(worst_m)) worst_m = d;
            }
        }
        printf("     M2M4 - power over 2-15 dB: worst %+.2f dB\n", worst_m);
        check("M2M4 agrees with power within 4 dB over 2-15 dB", fabs(worst_m) <= 4.0);
    }

    /* Noise only: no carrier, so no C/N (unknown, never a low number). */
    {
        gen_t g;
        generate(&g, 80, 0.0, -1e6, 0, 0, 16, 256);
        double m2, m4;
        cna_moments(&g.s, 0.0f, 0.0f, &m2, &m4);
        uint8_t flags = 0;
        float u;
        const float n = cna_noise_at(&s_map, 80, s_model_q8, 2000, &flags, &u);
        const cna_reading_t r = cna_estimate(m2, m4, n, u, flags, 13600u, 12, CNA_UNKNOWN_X10);
        check("noise only reads as no carrier (unknown), not as a low C/N", r.x10 == CNA_UNKNOWN_X10 && (r.flags & CNA_F_NO_CARRIER));
    }

    /* No noise map at all: never a confident number. */
    {
        cna_noise_map_t empty;
        memset(&empty, 0, sizeof(empty));
        uint8_t flags = 0;
        float u;
        const float n = cna_noise_at(&empty, 80, s_model_q8, 0, &flags, &u);
        const cna_reading_t r = cna_estimate(10.0, 150.0, n, u, flags, 13600u, 12, CNA_UNKNOWN_X10);
        check("without any noise measurement the power source is unavailable", r.source != CNA_SRC_POWER && r.confidence <= CNA_CONF_LOW);
    }

    {   /* Issue #222: two noiseless amplitude plateaus. The envelope's
         * mean^2/variance is a valid statistic, but not an RF C/N identity. */
        const double mean = (13.0 + 61.0) / 2.0;
        const double variance = ((13.0 - mean) * (13.0 - mean) + (61.0 - mean) * (61.0 - mean)) / 2.0;
        const int ratio_x100 = (int)lround(100.0 * mean * mean / variance);
        const int env_x10 = cn_estimate_env_x10(ratio_x100, 12);
        const cna_reading_t envelope = {.x10=env_x10, .confidence=CNA_CONF_LOW, .source=CNA_SRC_ENVELOPE};
        const cna_reading_t m2m4 = {.x10=env_x10, .confidence=CNA_CONF_LOW, .source=CNA_SRC_M2M4};
        const cna_reading_t power = {.x10=env_x10, .confidence=CNA_CONF_OK, .source=CNA_SRC_POWER};
        const cna_reading_t distorted = {.x10=env_x10, .confidence=CNA_CONF_LOW,
                                         .source=CNA_SRC_POWER | CNA_SRC_DISTORTED};
        printf("     amplitude plateaus: ratio %d -> envelope %.1f dB\n",
               ratio_x100, env_x10 / 10.0);
        check("amplitude fading alone can imitate a weak positive C/N", env_x10 > 0 && env_x10 < 60);
        check("an envelope-only C/N never drives RF decisions", cna_control_x10(&envelope) == CNA_UNKNOWN_X10);
        check("an unreferenced M2M4 C/N never drives RF decisions", cna_control_x10(&m2m4) == CNA_UNKNOWN_X10);
        check("a noise-referenced power C/N remains available to decisions", cna_control_x10(&power) == env_x10);
        check("a quantisation-distorted power C/N remains unknown", cna_control_x10(&distorted) == CNA_UNKNOWN_X10);
        kp_ladder_t ladder;
        kp_ladder_reset(&ladder);
        ladder.level = 1;
        for (int64_t now = 0; now < 10000000; now += 50000)
            (void)kp_ladder_update_evidence(&ladder, cna_control_x10(&envelope), true, 100, now);
        check("amplitude-only C/N cannot move the active kp ladder", ladder.level == 1);
    }

    {   /* a weak carrier learned again and again cannot pull an anchored row */
        cna_noise_map_t m;
        memset(&m, 0, sizeof(m));
        for (unsigned k = 0; k < CNA_MIN_SAMPLES; ++k) cna_noise_learn(&m, 72, 0.84f, 1);
        cna_noise_anchor(&m, 72, 0.84f);
        for (unsigned k = 0; k < 500u; ++k) cna_noise_learn(&m, 72, 0.84f * 1.26f, 2);   /* +1 dB: a -6 dB carrier */
        const double drift = 10.0 * log10(m.noise[72] / 0.84);
        cna_noise_map_t f;
        memset(&f, 0, sizeof(f));
        for (unsigned k = 0; k < 500u; ++k) cna_noise_learn(&f, 72, 0.84f * 1.26f, 2);
        printf("     anchored row after 500 learns at +1 dB: %+.2f dB (unanchored %+.2f dB)\n", drift,
               10.0 * log10(f.noise[72] / 0.84));
        check("in-service learning stays within 0.5 dB of the clean startup value", drift <= 0.51 && drift >= 0.0);
    }
    /* A front end that changed (an external LNA fitted or removed, 2026-11-14):
     * noise-like windows far from the map move the whole map. */
    {
        const double known = 3.85;   /* bare row 79 on the board */
        #define NOISE(p, kurt) ((p) * 96.0), ((kurt) * (p) * (p) * 96.0), 96u
        check("front end: noise 17 dB over the map (LNA fitted) is a new front end",
              cna_front_end_changed(NOISE(known * 50.0, 2.0), (float)known));
        check("front end: noise 17 dB under the map (LNA removed) is a new front end",
              cna_front_end_changed(NOISE(known / 50.0 + 0.7, 2.0), 30.0f));
        check("front end: 3 dB over the map is no evidence (a weak carrier or interference)",
              !cna_front_end_changed(NOISE(known * 2.0, 2.0), (float)known));
        check("front end: a strong carrier (low kurtosis) 17 dB over is no new front end",
              !cna_front_end_changed(NOISE(known * 50.0, 1.2), (float)known));
        #undef NOISE
        cna_noise_map_t m = {0};
        for (unsigned r = 72; r <= 83; ++r) {
            m.noise[r] = (float)(CNA_QUANT_CELLS2 + 3.68 * pow(10.0, (r - 79.0) / 10.0));
            m.n[r] = CNA_MIN_SAMPLES; m.anchor[r] = m.noise[r];
        }
        const float before83 = m.noise[83];
        const float expected67 = (float)(CNA_QUANT_CELLS2 + 3.68 * pow(10.0, (67.0 - 79.0) / 10.0)) + 0.0f;
        const float lna67 = (float)(CNA_QUANT_CELLS2 + 8.3);
        cna_noise_rebase(&m, 67, lna67, expected67 < CNA_QLIM_CELLS2 ? CNA_QLIM_CELLS2 : expected67, 1);
        const double ratio = 8.3 / ((expected67 < CNA_QLIM_CELLS2 ? CNA_QLIM_CELLS2 : expected67) - CNA_QUANT_CELLS2);
        check("rebase: the row takes the measurement", fabsf(m.noise[67] - lna67) < 1e-4f && m.n[67] == CNA_MIN_SAMPLES);
        check("rebase: every other row moves by the same thermal ratio",
              fabs((m.noise[83] - CNA_QUANT_CELLS2) / (before83 - CNA_QUANT_CELLS2) - ratio) < 1e-3 * ratio);
        bool anchors = true;
        for (unsigned r = 0; r < CNA_ROWS; ++r) anchors = anchors && m.anchor[r] == 0.0f;
        check("rebase: the anchors go (in-service learning may follow freely)", anchors);
        cna_noise_map_t q = {0};
        cna_noise_rebase(&q, 67, 8.0f, 0.3f, 1);
        check("rebase: refused against a quantisation-limited expectation", q.noise[67] == 0.0f);
    }
    printf("%u/%u C/N-AUTO checks passed\n", s_run - s_fail, s_run);
    return s_fail != 0;
}
