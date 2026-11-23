/* C5VRX-5 by Twotoz and the C5VRX contributors: the received-level meter
 * (main/rx_level.h) and the probe power it reads (gain.c pwr_x16).
 *
 *   cc -O2 -std=c11 -I../main test_rx_level.c ../main/gain.c ../main/arc_phy.c -lm -o t && ./t
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gain.h"
#include "phase8_gain_lut.h"
#include "rx_level.h"

unsigned char phy_param[2048];

static unsigned s_fail, s_run;
static void check(const char *name, int ok)
{
    ++s_run;
    printf("%-4s %s\n", ok ? "ok" : "FAIL", name);
    s_fail += !ok;
}

static direct_gain_t g;
static uint32_t sum[RX_LEVEL_ROWS], n[RX_LEVEL_ROWS];

/* probes at one row with a mean power in cells^2 */
static void put(unsigned row, double cells2, unsigned probes)
{
    sum[row] += (uint32_t)lround(cells2 * 16.0) * probes;
    n[row] += probes;
}
static void clear(void) { for (unsigned k = 0; k < RX_LEVEL_ROWS; ++k) sum[k] = n[k] = 0; }

int main(void)
{
    arc_gain_table_t table;
    arc_gain_table_from_bytes(&table, NULL, ARC_5G_GAIN_MAX);
    const dg_map_context_t c = {.freq_mhz = 5865, .lane_mode = 2, .iq_scale = 1, .filter_bw20 = 1, .filter_code = 60};
    dg_reset_context(&g, &table, 83, &c);
    const unsigned top = table.max_index;

    /* Probe power: DC-removed mean of |z|^2 at the half-cell centres. */
    bool power_ok = true;
    srand(5);
    for (int t = 0; t < 200 && power_ok; ++t) {
        uint8_t s[256];
        double si = 0, sq = 0, spp = 0;
        const int dci = rand() % 7 - 3, dcq = rand() % 7 - 3, spread = 1 + rand() % 5;
        for (int k = 0; k < 256; ++k) {
            int i = dci + rand() % (2 * spread + 1) - spread, q = dcq + rand() % (2 * spread + 1) - spread;
            if (i < -8) i = -8;
            if (i > 7) i = 7;
            if (q < -8) q = -8;
            if (q > 7) q = 7;
            s[k] = (uint8_t)(((unsigned)i & 15u) << 4 | ((unsigned)q & 15u));
            const double x = i + .5, y = q + .5;
            si += x; sq += y; spp += x * x + y * y;
        }
        const double want = spp / 256 - (si / 256) * (si / 256) - (sq / 256) * (sq / 256);
        const dg_observation_t o = dg_measure_gain(s, sizeof(s), c5vrx_phase8_gain_lut, 1000);
        power_ok = fabs(o.pwr_x16 / 16.0 - want) <= 1.0 / 16.0 + 1e-9;
        if (!power_ok) printf("     power %.3f vs %.3f\n", o.pwr_x16 / 16.0, want);
    }
    check("probe power is the DC-removed mean |z|^2 (200 random windows)", power_ok);

    clear();
    put(top, 4.93, 1000);
    rx_level_t r = rx_level_compute(sum, n, 0, g.model_db_q8, top, 0.0f);
    printf("     noise only at G83: input %.1f dBm, above_noise %u\n", r.input_dbm_x10 / 10.0, r.above_noise);
    check("receiver noise at G83 reads -96.7 dBm and no carrier", r.valid && !r.above_noise && abs(r.input_dbm_x10 + 967) <= 2);

    clear();
    put(top, 4.93 + 47.6, 1000);   /* a carrier 10 dB over the noise */
    r = rx_level_compute(sum, n, 0, g.model_db_q8, top, 0.0f);
    printf("     carrier 10 dB over the noise at G83: %.1f dBm, C/N %.1f dB\n", r.carrier_dbm_x10 / 10.0, r.cn_x10 / 10.0);
    check("a carrier 10 dB over the noise reads -86.7 dBm, C/N 10 dB",
          r.above_noise && abs(r.carrier_dbm_x10 + 867) <= 2 && abs(r.cn_x10 - 100) <= 2 && r.exact);

    /* The same pin level at G60: the model refers it back. */
    const double at60 = 47.6 * pow(10.0, (g.model_db_q8[60] - g.model_db_q8[top]) / 2560.0);
    clear();
    put(60, at60 + 4.76 * pow(10.0, (g.model_db_q8[60] - g.model_db_q8[top]) / 2560.0) + 1.0 / 6.0, 1000);
    r = rx_level_compute(sum, n, 0, g.model_db_q8, top, 0.0f);
    printf("     same pin level at G60: %.1f dBm (model %.1f dB under G83)\n", r.carrier_dbm_x10 / 10.0,
           (g.model_db_q8[top] - g.model_db_q8[60]) / 256.0);
    check("the same pin level reads the same at G60", abs(r.carrier_dbm_x10 + 867) <= 5 && r.exact);

    clear();
    put(30, 20.0, 1000);
    r = rx_level_compute(sum, n, 0, g.model_db_q8, top, 0.0f);
    printf("     strong carrier at G30: %.1f dBm\n", r.carrier_dbm_x10 / 10.0);
    check("below G54 the reading is flagged as not exact", r.valid && !r.exact && r.carrier_dbm_x10 > -700);

    clear();
    put(top, 30.0, 40);
    r = rx_level_compute(sum, n, 960, g.model_db_q8, top, 0.0f);
    check("too few unclipped probes: no reading, clipping counted", !r.valid && r.clipped_pm == 960);

    /* A learned noise 1.7 dB higher (5865 MHz on the board) is subtracted. */
    const float nrel = (float)pow(10.0, 0.17);
    clear();
    put(top, 4.76 * nrel + 1.0 / 6.0, 1000);
    r = rx_level_compute(sum, n, 0, g.model_db_q8, top, 0.0f);
    check("unlearned: noise 1.7 dB high reads as a false carrier", r.above_noise);
    r = rx_level_compute(sum, n, 0, g.model_db_q8, top, nrel);
    check("learned noise: no carrier, noise reads -95.0 dBm", !r.above_noise && abs(r.noise_dbm_x10 + 950) <= 2);
    clear();
    put(top, 4.76 * nrel * 11.0 + 1.0 / 6.0, 1000);
    r = rx_level_compute(sum, n, 0, g.model_db_q8, top, nrel);
    check("learned noise: C/N 10 dB over it", abs(r.cn_x10 - 100) <= 2);

    /* The noise from C/N-AUTO's map: with and without an external LNA. */
    {
        cna_noise_map_t m = {0};
        uint8_t row = 0;
        check("map: empty map gives no noise (fallback)", rx_level_noise_from_map(&m, g.model_db_q8, top, &row) == 0.0f &&
              row == 255u);

        /* Bare board: the top row measured the calibration noise, 1.7 dB up at 5865 MHz. */
        m.noise[top] = (float)(4.76 * nrel + 1.0 / 6.0); m.n[top] = CNA_MIN_SAMPLES;
        const float bare = rx_level_noise_from_map(&m, g.model_db_q8, top, &row);
        clear();
        put(top, 4.76 * nrel + 1.0 / 6.0, 1000);
        r = rx_level_compute(sum, n, 0, g.model_db_q8, top, bare);
        printf("     bare: noise %.1f dBm from G%u\n", r.noise_dbm_x10 / 10.0, row);
        check("map, bare: the top row gives the learned -95.0 dBm, no carrier",
              row == top && !r.above_noise && abs(r.noise_dbm_x10 + 950) <= 2);

        /* Behind a ~21 dB LNA: the noise at the pin is 17.5 dB over the
         * calibration (board 2026-11-14), rows 70..top clip and stay empty,
         * G67 measured it. A quantisation-limited low row is no noise. */
        memset(&m, 0, sizeof(m));
        const double lna_rel = pow(10.0, 1.75);
        const double down67 = pow(10.0, (g.model_db_q8[67] - g.model_db_q8[top]) / 2560.0);
        m.noise[67] = (float)(4.76 * lna_rel * down67 + 1.0 / 6.0); m.n[67] = CNA_MIN_SAMPLES;
        m.noise[64] = (float)(4.76 * lna_rel * pow(10.0, (g.model_db_q8[64] - g.model_db_q8[top]) / 2560.0) + 1.0 / 6.0);
        m.n[64] = CNA_MIN_SAMPLES;
        m.noise[10] = 0.3f; m.n[10] = CNA_MIN_SAMPLES;
        m.noise[75] = 30.0f; m.n[75] = 1u;   /* one window: not measured yet */
        const float lna = rx_level_noise_from_map(&m, g.model_db_q8, top, &row);
        clear();
        put(67, 4.76 * lna_rel * down67 + 1.0 / 6.0, 1000);   /* VTX off, AGC at G67 */
        r = rx_level_compute(sum, n, 0, g.model_db_q8, top, lna);
        printf("     LNA: noise %.1f dBm from G%u, input %.1f dBm, above_noise %u\n", r.noise_dbm_x10 / 10.0, row,
               r.input_dbm_x10 / 10.0, r.above_noise);
        check("map, LNA: the highest measured row (G67), noise -79.2 dBm, VTX off reads no carrier",
              row == 67u && !r.above_noise && abs(r.noise_dbm_x10 + 792) <= 3);
        /* A row learned while the noise clipped (G72 19.8 cells^2 on the board)
         * reads low; the clean G67 below it wins, and alone it is still used. */
        cna_noise_map_t mc = m;
        mc.noise[72] = 19.8f; mc.n[72] = CNA_MIN_SAMPLES;
        uint8_t row_c = 0;
        const float with_clip = rx_level_noise_from_map(&mc, g.model_db_q8, top, &row_c);
        check("map: a clipped higher row (G72, 19.8 cells2) is skipped for the clean G67",
              row_c == 67u && fabsf(with_clip - lna) < 1e-3f * lna);
        memset(&mc, 0, sizeof(mc));
        mc.noise[72] = 19.8f; mc.n[72] = CNA_MIN_SAMPLES; mc.noise[78] = 40.0f; mc.n[78] = CNA_MIN_SAMPLES;
        (void)rx_level_noise_from_map(&mc, g.model_db_q8, top, &row_c);
        check("map: only clipped rows: the lowest (least clipped) is used", row_c == 72u);
        /* An empty map with a clipped no-carrier window at G72 (LEVEL parked
         * there, every window clipped, nothing learned): the lower bound is the
         * first reference; a clean map row below wins over it. */
        {
            cna_noise_map_t empty = {0};
            float floor_cells2[RX_LEVEL_ROWS] = {0};
            floor_cells2[72] = 19.8f;
            const float fl = rx_level_noise_from_map_floor(&empty, floor_cells2, g.model_db_q8, top, &row_c);
            const float want = (float)((19.8 - 1.0 / 6.0) * pow(10.0, (g.model_db_q8[top] - g.model_db_q8[72]) / 2560.0) / 4.76);
            check("floor: an empty map takes the clipped G72 window as a lower bound", row_c == 72u &&
                  fabsf(fl - want) < 1e-3f * want);
            check("floor: without the floor the empty map gives nothing",
                  rx_level_noise_from_map(&empty, g.model_db_q8, top, &row_c) == 0.0f && row_c == 255u);
            empty.noise[67] = m.noise[67]; empty.n[67] = CNA_MIN_SAMPLES;
            (void)rx_level_noise_from_map_floor(&empty, floor_cells2, g.model_db_q8, top, &row_c);
            check("floor: a clean map row (G67) wins over the floor", row_c == 67u);
            /* A bound under 12 cells^2 (a few clipped peaks at the useful top,
             * board: G67 7-15 % of the probes) also counts with an empty map. */
            cna_noise_map_t none = {0};
            float low_floor[RX_LEVEL_ROWS] = {0};
            low_floor[67] = 8.5f; low_floor[72] = 19.8f;
            (void)rx_level_noise_from_map_floor(&none, low_floor, g.model_db_q8, top, &row_c);
            check("floor: a bound under 12 cells2 counts, the lowest row wins", row_c == 67u);
        }
        rx_level_t old = rx_level_compute(sum, n, 0, g.model_db_q8, top, 0.0f);
        check("LNA without the map: the calibration reads the noise as a carrier (the bug)", old.above_noise &&
              old.cn_x10 > 150);
        clear();
        put(67, 4.76 * lna_rel * 11.0 * down67 + 1.0 / 6.0, 1000);   /* a carrier 10 dB over the LNA's noise */
        r = rx_level_compute(sum, n, 0, g.model_db_q8, top, lna);
        check("map, LNA: a carrier 10 dB over the noise reads C/N 10 dB", r.above_noise && abs(r.cn_x10 - 100) <= 3);
    }

    {   /* the display gate: noise 1.3 dB over its reference is no carrier without video */
        rx_level_t r = {.valid = true, .above_noise = true, .input_dbm_x10 = -952, .carrier_dbm_x10 = -1000,
                        .cn_x10 = -55};
        const rx_level_t a = rx_level_gate(r, false), b = rx_level_gate(r, true);
        check("gate: 1.3 dB over the noise without video reads no carrier", !a.above_noise && a.cn_x10 == -990 &&
              a.carrier_dbm_x10 == -952);
        check("gate: the same with video keeps the carrier (range edge)", b.above_noise && b.carrier_dbm_x10 == -1000);
        r.cn_x10 = 20; r.carrier_dbm_x10 = -925;
        check("gate: C/N >= 0 dB shows without video", rx_level_gate(r, false).above_noise);
    }
    printf("%u/%u rx level checks passed\n", s_run - s_fail, s_run);
    return s_fail != 0;
}
