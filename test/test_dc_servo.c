/* C5VRX-5 by Twotoz and the C5VRX contributors: host test of the live DC
 * servo (main/dc_servo.h) against a simulated receiver: a DAC with an unknown
 * slope and polarity per axis, a boot offset of up to 4 cells, a 4-bit
 * quantiser that clips, receiver noise, and a wandering DC.
 *
 *   cc -O2 -std=c11 -I../main test_dc_servo.c -lm -o t && ./t
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "dc_servo.h"

static unsigned s_fail, s_run;
static void check(const char *name, int ok)
{
    ++s_run;
    printf("%-4s %s\n", ok ? "ok" : "FAIL", name);
    s_fail += !ok;
}

static uint32_t s_seed = 2026;
static double gauss(void)
{
    double u = 0;
    for (int k = 0; k < 12; ++k) { s_seed = s_seed * 1664525u + 1013904223u; u += (s_seed >> 8) / 16777216.0; }
    return u - 6.0;
}

/* Mean of n quantised samples (dsp units summed like gain.c) of noise sigma
 * around a true DC (cells), clipped to the 4-bit range. */
static uint32_t s_neg[2];   /* sign-bit counts of the last measure() */
static void measure(double dc_i, double dc_q, double sigma, uint32_t n, int64_t *si, int64_t *sq)
{
    *si = *sq = 0;
    s_neg[0] = s_neg[1] = 0;
    for (uint32_t k = 0; k < n; ++k) {
        int i = (int)floor(dc_i + sigma * gauss()), q = (int)floor(dc_q + sigma * gauss());
        /* ultrafine {9,6,5,4}: the sign bit stays, the magnitude bits fold
         * (legacy/c5vrx3/main/rf.c: +288 reads as +32); -9 & 7 = 7 -> -1 */
        i = i >= 0 ? (i & 7) : (i & 7) - 8;
        q = q >= 0 ? (q & 7) : (q & 7) - 8;
        s_neg[0] += i < 0;
        s_neg[1] += q < 0;
        *si += 2 * i + 1;
        *sq += 2 * q + 1;
    }
}

/* Run the loop: returns the |DC| (mcells) after `periods`, worst axis. */
static int run(double off_i, double off_q, double slope_i, double slope_q, double sigma, double wander,
               unsigned periods, dc_servo_t *s, int *code_out)
{
    int code[2] = {0, 0}, dc[2] = {0, 0};
    for (unsigned p = 0; p < periods; ++p) {
        const double w = wander * sin(p * 0.21);
        const double ti = off_i + w + code[0] * slope_i / 1000., tq = off_q - w + code[1] * slope_q / 1000.;
        int64_t si, sq;
        measure(ti, tq, sigma, DC_SERVO_MIN_SAMPLES, &si, &sq);
        if (!dc_servo_mean(si, sq, DC_SERVO_MIN_SAMPLES, dc)) return 99999;
        int step[2];
        const int pn[2] = {dc_servo_pneg(s_neg[0], DC_SERVO_MIN_SAMPLES), dc_servo_pneg(s_neg[1], DC_SERVO_MIN_SAMPLES)};
        dc_servo_decide_sign(s, 83u, dc, pn, step);
        if (step[0] || step[1]) {
            code[0] += step[0];
            code[1] += step[1];
            dc_servo_applied(s, 83u, dc, step);
        }
    }
    if (code_out) { code_out[0] = code[0]; code_out[1] = code[1]; }
    const double ti = off_i + code[0] * slope_i / 1000., tq = off_q + code[1] * slope_q / 1000.;
    const double e = fabs(ti) > fabs(tq) ? fabs(ti) : fabs(tq);
    return (int)(e * 1000.);
}

int main(void)
{
    dc_servo_t s;
    int dc[2];
    check("too few samples: no estimate", !dc_servo_mean(0, 0, DC_SERVO_MIN_SAMPLES - 1u, dc));
    check("mean in mcells (dsp units are twice a cell)",
          dc_servo_mean(3 * (int64_t)DC_SERVO_MIN_SAMPLES, -(int64_t)DC_SERVO_MIN_SAMPLES, DC_SERVO_MIN_SAMPLES, dc) &&
          dc[0] == 1500 && dc[1] == -500);

    /* The baseline boots: 1-4 cells off, noise 1.9 cells, slopes 50-250
     * mcells per code of either polarity. */
    static const double offs[][2] = {{-1.37, .21}, {1.18, 1.72}, {2.22, 1.97}, {1.54, -3.30}, {2.28, -1.73},
                                     {.49, -3.36}, {-3.9, -3.5}, {4.0, 4.0}};
    static const double slopes[][2] = {{150, 150}, {246, 127}, {-130, 220}, {60, -200}, {-250, -50}};
    /* Slopes in the board's range (170-550 measured) within 20 periods (5 s);
     * the weak ones of the set (|slope| <= 60, three times below anything
     * measured) within 40: a quarter of the step per code, with the
     * magnitude bits folding at the start (4 cells, sigma 1.9). */
    int worst = 0, worst_weak = 0;
    unsigned cases = 0;
    for (unsigned o = 0; o < sizeof(offs) / sizeof(offs[0]); ++o)
        for (unsigned k = 0; k < sizeof(slopes) / sizeof(slopes[0]); ++k) {
            const bool weak = fabs(slopes[k][0]) <= 60. || fabs(slopes[k][1]) <= 60.;
            dc_servo_init(&s);
            const int e = run(offs[o][0], offs[o][1], slopes[k][0], slopes[k][1], 1.9, 0., weak ? 40 : 20, &s, NULL);
            if (weak) worst_weak = e > worst_weak ? e : worst_weak;
            else worst = e > worst ? e : worst;
            ++cases;
        }
    printf("     boot offsets x slopes: %u cases, worst |DC| %d mcells (20 periods), weak slopes %d (40 periods)\n",
           cases, worst, worst_weak);
    check("every boot offset, either polarity, board slopes: below 0.35 cells within 20 periods (5 s)", worst <= 350);
    check("weak slopes (<= 60 mcells/code): below 0.35 cells within 40 periods (10 s)", worst_weak <= 350);

    /* Weak noise (sigma 0.8) with a 3.5-cell DC: the cloud sits on a rail. */
    dc_servo_init(&s);
    const int clipped = run(3.5, -3.6, 180, 140, .8, 0., 20, &s, NULL);
    check("clipped cloud (3.5 cells, sigma 0.8): converges anyway", clipped <= 350);

    /* Wandering DC (+-1 cell over ~30 periods): tracked. */
    dc_servo_init(&s);
    const int tracked = run(2.0, -1.0, 150, 150, 1.9, 1.0, 60, &s, NULL);
    check("wander of +-1 cell: tracked within 0.5 cells", tracked <= 500);

    /* Already centred: no steps (no hunting on noise). */
    dc_servo_init(&s);
    int code[2];
    (void)run(.05, -.08, 150, 150, 1.9, 0., 40, &s, code);
    check("centred: no hunting in 40 periods", s.steps == 0 && code[0] == 0 && code[1] == 0);

    /* A wrong default polarity is relearned. */
    dc_servo_init(&s);
    const int flipped = run(2.5, 2.5, -150, -150, 1.9, 0., 25, &s, NULL);
    /* The sign-bit bracket finds the polarity by its fold walk (a turn) and
     * often reaches zero before a linear-zone step could measure a slope. */
    check("inverted DAC polarity: found, converges", flipped <= 350 && s.slope[0] < 0 && s.slope[1] < 0 &&
          (s.learned > 0 || s.turns > 0));

    /* Steps never exceed the DAC trial limit. */
    dc_servo_init(&s);
    int big[2] = {9000, -9000}, step[2];
    dc_servo_decide(&s, 83u, big, step);
    check("step clamped to DC_SERVO_STEP_MAX", abs(step[0]) == DC_SERVO_STEP_MAX && abs(step[1]) == DC_SERVO_STEP_MAX);
    int small[2] = {200, -240};
    dc_servo_decide(&s, 83u, small, step);
    check("inside the deadband: no step", step[0] == 0 && step[1] == 0);

    /* A DC jump that is not the step's effect (a channel round): forgotten
     * by the caller, or, if not, a measured slope does not flip sign. */
    dc_servo_init(&s);
    (void)run(2.0, 2.0, 400, 400, 1.9, 0., 12, &s, NULL);   /* slopes measured, ~+400 */
    const int learned_i = s.slope[0], learned_q = s.slope[1];
    int d0[2] = {1500, 1500}, st0[2], st1[2];
    static const int lin[2] = {500, 500};               /* sign share: no folding */
    dc_servo_decide_sign(&s, 83u, d0, lin, st0);
    dc_servo_applied(&s, 83u, d0, st0);
    int jump[2] = {1500 + 600, 1500 + 700};            /* new channel: unrelated jump against the step */
    dc_servo_decide_sign(&s, 83u, jump, lin, st1);
    check("unrelated DC jump: a measured slope keeps its sign", s.known[0] && s.known[1] && s.slope[0] > 0 &&
          s.slope[1] > 0 && s.sign_rejects >= 1u && learned_i > 0 && learned_q > 0);
    dc_servo_init(&s);
    dc_servo_decide_sign(&s, 83u, d0, lin, st0);
    dc_servo_applied(&s, 83u, d0, st0);
    dc_servo_forget_step(&s, 83u);
    dc_servo_decide_sign(&s, 83u, jump, lin, st1);
    check("forgotten step: no slope evidence taken", s.slope[0] == DC_SERVO_SLOPE_DEFAULT &&
          s.slope[1] == DC_SERVO_SLOPE_DEFAULT && s.forgotten == 1u);
    /* Two rows with different offsets (board: G83 0.3 cells, G82 3.9 after
     * a channel round), Direct Gain alternating faster than one mean: each
     * row's sum only grows during its own visits, and both converge. */
    {
        dc_servo_t t;
        dc_servo_init(&t);
        const double off[2][2] = {{.3, -.2}, {1.1, -3.9}};
        int code[2][2] = {{0, 0}, {0, 0}};
        int64_t acc[2][2] = {{0, 0}, {0, 0}};
        uint32_t n[2] = {0, 0};
        for (unsigned visit = 0; visit < 400; ++visit) {
            const unsigned r = visit % 3u ? 0u : 1u;          /* 2:1 between G83 and G82 */
            int64_t si, sq;
            measure(off[r][0] + code[r][0] * .4, off[r][1] + code[r][1] * .4, 1.9, 8000u, &si, &sq);
            acc[r][0] += si; acc[r][1] += sq; n[r] += 8000u;   /* a short visit: 8000 samples */
            int dc[2], step[2];
            if (!dc_servo_mean(acc[r][0], acc[r][1], n[r], dc)) continue;
            acc[r][0] = acc[r][1] = 0; n[r] = 0;
            dc_servo_decide(&t, r ? 82u : 83u, dc, step);
            if (step[0] || step[1]) {
                code[r][0] += step[0]; code[r][1] += step[1];
                dc_servo_applied(&t, r ? 82u : 83u, dc, step);
            }
        }
        double worst = 0;
        for (unsigned r = 0; r < 2; ++r)
            for (unsigned a = 0; a < 2; ++a) {
                const double e = fabs(off[r][a] + code[r][a] * .4);
                worst = e > worst ? e : worst;
            }
        printf("     rows G83/G82 alternating: worst |DC| %.2f cells\n", worst);
        check("alternating rows with different offsets: both below 0.35 cells", worst <= .35);
    }
    /* Flicker: the DC itself wanders on the decision scale (board
     * 2026-10-19: ~0.3 cells between 250 ms means, 1/f below ~100 kHz). The
     * servo must not read a step drowned in it as a wrong slope: 300 steps,
     * 32 relearns and 50 sign rejects in 100 s on the board. */
    {
        dc_servo_t f;
        dc_servo_init(&f);
        int code[2] = {0, 0};
        double wander[2] = {0, 0};
        unsigned steps = 0;
        double sq = 0;
        for (unsigned p = 0; p < 400; ++p) {
            for (int a = 0; a < 2; ++a) wander[a] = 0.9 * wander[a] + 0.3 * gauss();   /* AR(1), ~0.7 cells rms */
            const double ti = 1.2 + wander[0] + code[0] * 0.4, tq = -0.8 + wander[1] + code[1] * 0.4;
            int64_t si, sqq;
            measure(ti, tq, 1.9, DC_SERVO_MIN_SAMPLES, &si, &sqq);
            int dc[2], step[2];
            if (!dc_servo_mean(si, sqq, DC_SERVO_MIN_SAMPLES, dc)) continue;
            dc_servo_decide(&f, 83u, dc, step);
            if (step[0] || step[1]) { code[0] += step[0]; code[1] += step[1]; dc_servo_applied(&f, 83u, dc, step); ++steps; }
            if (p >= 100) sq += ti * ti + tq * tq;
        }
        const double rms = sqrt(sq / 300.);
        printf("     flicker: %u steps in 400 periods, relearned %lu, sign rejects %lu, |DC| rms %.2f cells\n", steps,
               (unsigned long)f.relearned, (unsigned long)f.sign_rejects, rms);
        check("flicker: no slope relearning from noise", f.relearned <= 1u && f.sign_rejects <= 10u);
        check("flicker: DC stays near the wander floor (<= 1.0 cell rms)", rms <= 1.0);
    }
    /* Board, PR #228: a template table 4.5 cells off on Q, the slope lost
     * (relearn), the cloud partly on a rail: the +3 probe made it worse,
     * its effect stayed under the learning threshold, and the probe repeated
     * up to the trim clamp (+188 codes, 3.5 cells left). */
    {
        unsigned bad_cases = 0, cases_run = 0;
        int worst_wrong = 0, worst_dc = 0;
        static const double sl[] = {150, -150, 300, -300, 60};
        for (unsigned k = 0; k < sizeof(sl) / sizeof(sl[0]); ++k)
            for (int sign = -1; sign <= 1; sign += 2) {
                dc_servo_init(&s);
                s.slope[1] = 0;                         /* lost, as after a relearn */
                int code_end[2];
                const int e = run(0.0, sign * 9.0, 150, sl[k], 1.5, 0., 120, &s, code_end);
                /* codes moved the wrong way: the correct code is -off/slope */
                const double right = -sign * 9.0 * 1000. / sl[k];
                const int wrong = (int)fabs(code_end[1] - right) - (int)fabs(right);
                worst_wrong = wrong > worst_wrong ? wrong : worst_wrong;
                worst_dc = e > worst_dc ? e : worst_dc;
                bad_cases += e > 500;
                ++cases_run;
            }
        printf("     lost slope, 9 cells (pinned), sigma 1.5: %u cases, worst |DC| after 120 periods %d mcells\n", cases_run, worst_dc);
        check("lost slope on a clipped cloud: converges (<= 0.5 cells), never runs to the clamp", bad_cases == 0u);
    }
    {   /* Board 2026-10-20 (new antenna): a proven positive polarity, then a
         * strong carrier folds the IQ for seconds; the sign share says
         * nothing about the DC then. The polarity must hold. */
        dc_servo_t p;
        dc_servo_init(&p);
        (void)run(1.5, 1.5, 200, 200, 1.9, 0., 20, &p, NULL);   /* linear zone: polarity learned */
        const int votes_ok = p.pol_votes[0] >= 1 && p.pol_votes[1] >= 1;
        static const int folded_dc[2] = {3800, -4100}, folded_share[2] = {30, 970};
        for (unsigned k = 0; k < 200u; ++k) {
            int st[2];
            dc_servo_decide_sign(&p, 83u, folded_dc, folded_share, st);
            dc_servo_applied(&p, 83u, folded_dc, st);
        }
        printf("     polarity votes %d/%d, slope after folding %d/%d, turns %lu\n", p.pol_votes[0], p.pol_votes[1], p.slope[0], p.slope[1], (unsigned long)p.turns);
        check("a proven polarity is not flipped by seconds of folded IQ", votes_ok && p.slope[0] > 0 && p.slope[1] > 0);
    }
    {   /* A row whose noise is under a cell (sigma 0.3): every sample in one
         * cell, the sign share jumps 0 <-> 1000. The bracket must settle on
         * the one-code crossing instead of hunting +-8 codes. */
        dc_servo_t b;
        dc_servo_init(&b);
        int code_end[2];
        unsigned steps_before;
        const int e = run(1.1, -0.9, 250, 250, 0.3, 0., 30, &b, code_end);
        steps_before = b.steps;
        (void)run(1.1 + code_end[0] * .25, -0.9 + code_end[1] * .25, 250, 250, 0.3, 0., 30, &b, NULL);
        printf("     sigma 0.3: |DC| %d mcells, brackets %lu, steps %u then %lu more in 30 periods\n", e,
               (unsigned long)b.brackets, steps_before, (unsigned long)(b.steps - steps_before));
        check("sub-cell noise: settles within one code (<= 0.25 cells), then holds", e <= 250 &&
              b.steps - steps_before <= 2u);
    }
    {   /* Blackbox 2026-10-27: edge rows 13-48 codes off their table codes. A
         * 48-code error (12 cells at 250 mcells/code, folded) with sigma 0.8
         * reaches 0.35 cells in few decisions; 8-code steps needed >= 6 plus
         * the fold walk. */
        unsigned worst_periods = 0;
        static const double err[] = {-12.0, -6.0, 6.0, 12.0};
        for (unsigned k = 0; k < sizeof(err) / sizeof(err[0]); ++k) {
            unsigned p = 1;
            for (; p <= 40u; ++p) {
                dc_servo_t t;
                dc_servo_init(&t);
                t.pol_votes[0] = t.pol_votes[1] = 1;        /* a learned polarity (stored per board) */
                if (run(err[k], -err[k] / 2, 250, 250, 0.8, 0., p, &t, NULL) <= 350) break;
            }
            worst_periods = p > worst_periods ? p : worst_periods;
        }
        printf("     template rows 6-12 cells off, sigma 0.8: within 0.35 cells after <= %u decisions\n", worst_periods);
        check("template rows 6-12 cells off: corrected within 8 decisions", worst_periods <= 8u);
    }
    /* ---- shared DC model (dc_model_t) ----
     * Rows 50..83; the true error of row g in codes is drift + own(g). An
     * ideal servo visit measures dc = 200 mcells/code * (true - offset) and
     * steps by the rounded remainder (at most 16 codes, the bracket limit). */
    {
        enum { LO = 50, HI = 83 };
        static dc_model_t m;
        static bool templ[DC_MODEL_ROWS];
        static int own_err[DC_MODEL_ROWS];
        int drift = 0;
        const int slope = 200;
        #define ERR(g, a) (drift + own_err[g] + (int)(a) * 3)
        #define VISIT(g) do { \
            int dc_[2], st_[2]; \
            for (unsigned a_ = 0; a_ < 2; ++a_) dc_[a_] = slope * (ERR(g, a_) - dc_model_offset(&m, (g), a_, templ[g])); \
            dc_model_note(&m, (g), dc_, false); \
            for (unsigned a_ = 0; a_ < 2; ++a_) { \
                st_[a_] = abs(dc_[a_]) <= DC_SERVO_DEADBAND ? 0 : dc_[a_] / slope; \
                if (st_[a_] > 16) st_[a_] = 16; \
                if (st_[a_] < -16) st_[a_] = -16; } \
            dc_model_apply(&m, (g), st_, templ[g]); } while (0)
        #define WORST(out) do { out = 0; for (unsigned g_ = LO; g_ <= HI; ++g_) for (unsigned a_ = 0; a_ < 2; ++a_) { \
            const int e_ = abs(ERR(g_, a_) - dc_model_offset(&m, g_, a_, templ[g_])); if (e_ > out) out = e_; } } while (0)
        int worst;
        srand(14);

        /* 1. Calibrated rows (bare board), a common shift of 20 codes after a
         * channel setup: the first visit of any row fixes every row. */
        dc_model_reset(&m);
        for (unsigned g = 0; g < DC_MODEL_ROWS; ++g) { templ[g] = false; own_err[g] = 0; }
        drift = 20;
        VISIT(83); VISIT(83);
        WORST(worst);
        check("model: a common shift on calibrated rows: two visits of one row fix all rows", worst <= 1);
        const int shared_after = m.shared[0];

        /* 2. Template rows 50..67 (the LNA edge), each off by a smooth amount
         * of its own (30..47 codes) plus the drift: hopping visits. */
        for (unsigned g = LO; g <= 67; ++g) { templ[g] = true; own_err[g] = 30 + (int)(g - LO); }
        unsigned visits = 0;
        for (; visits < 400; ++visits) {
            { const unsigned row_ = (unsigned)(LO + rand() % 18); VISIT(row_); }
            WORST(worst);
            if (worst <= 1) break;
        }
        printf("     template rows 50-67 (30-47 codes off): every row within a code after %u hopping visits\n", visits + 1);
        check("model: template rows converge by inheritance, under 6 visits per row", worst <= 1 && visits + 1 <= 6u * 18u);
        unsigned settle = 0;
        for (unsigned k = 0; k < 200; ++k) { const unsigned row_ = (unsigned)(LO + rand() % 34); VISIT(row_); }
        /* A later drift of 10 codes (temperature, another antenna): rows
         * now settled move together. */
        drift += 10;
        for (; settle < 60; ++settle) {
            { const unsigned row_ = (unsigned)(LO + rand() % 34); VISIT(row_); }
            WORST(worst);
            if (worst <= 1) break;
        }
        printf("     drift of 10 codes after settling: every row within a code after %u visits\n", settle + 1);
        check("model: a drift after settling reaches every row within 6 visits", worst <= 1 && settle + 1 <= 6u);
        check("model: the first shift went to the shared part", shared_after == 20);

        /* 3. One calibrated row off by 15 of its own: its steps turn local,
         * the others end where they belong. */
        dc_model_reset(&m);
        for (unsigned g = 0; g < DC_MODEL_ROWS; ++g) { templ[g] = false; own_err[g] = 0; }
        drift = 0;
        own_err[70] = 15;
        for (unsigned k = 0; k < 300; ++k) { const unsigned row_ = (unsigned)(LO + rand() % 34); VISIT(row_); }
        WORST(worst);
        printf("     one calibrated row 15 codes off: worst row %d codes, row 70 own=%u local=%d\n", worst, m.own[70], m.local[70][0]);
        check("model: a lasting per-row error turns local, no row left off", worst <= 1 && m.own[70]);

        /* 4. Adoption keeps the offset and what the neighbours inherit. */
        dc_model_reset(&m);
        for (unsigned g = 0; g < DC_MODEL_ROWS; ++g) { templ[g] = true; own_err[g] = 0; }
        m.shared[0] = 7; m.local[60][0] = 12; m.own[60] = true;
        const int before60 = dc_model_offset(&m, 60, 0, true), before61 = dc_model_offset(&m, 61, 0, true);
        dc_model_adopt(&m, 60, true);
        /* the caller moved 19 codes into row 60's table codes */
        check("model: adoption moves the offset into the codes (row offset 0)", dc_model_offset(&m, 60, 0, true) == 0 && before60 == 19);
        check("model: a template neighbour inherits the same after adoption", dc_model_offset(&m, 61, 0, true) == before61);
        m.shared[0] += 5;
        check("model: an adopted row still follows a later shared drift", dc_model_offset(&m, 60, 0, true) == 5);
        #undef ERR
        #undef VISIT
        #undef WORST
    }
    printf("%u/%u dc servo checks passed\n", s_run - s_fail, s_run);
    return s_fail != 0;
}
