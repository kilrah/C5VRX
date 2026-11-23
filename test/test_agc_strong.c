/* C5VRX-5 by Twotoz and the C5VRX contributors: LEVEL and the core near the
 * bottom of the gain table (an external LNA close in, 2026-11-13).
 *
 * A Discord report (an RX5818-style LNA in front of the XIAO ESP32-C5): far
 * more range, a worse picture close in. Without an LNA the board already
 * parked at G21/G24 close in (FINDINGS, session 55); an LNA of 15-20 dB
 * moves that into G2-G9, where two limits of the firmware sat:
 *  - LEVEL's input-level histogram ended at +80 dB cells^2 at the top row,
 *    about G9 for a carrier on its best radius; a stronger carrier was read
 *    as +79 and held a row that folded (this test, LEVEL part);
 *  - dg_reset clamped the core to G20 or higher, so with the hardware under
 *    G20 the receiver saw a core/hardware mismatch and rebound the whole
 *    gain core on every window (receiver.c, core part).
 * Host model only: an ideal carrier, the gain model of the vendor table; no
 * LNA, compression or range claim. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "agc.h"
#include "gain.h"
#include "strong_lane.h"

unsigned char phy_param[1024];
static agc_level_t lv;
static direct_gain_t dg;

#define CHECK(c, ...) do { if (!(c)) { printf("FAIL " __VA_ARGS__); printf("\n"); exit(1); } } while (0)

int main(void)
{
    arc_gain_table_t t;
    arc_gain_table_from_bytes(&t, NULL, ARC_5G_GAIN_MAX);
    /* Core part: a reset keeps a low hardware gain. */
    for (unsigned g = 2; g <= 20; ++g) {
        dg_reset(&dg, &t, (uint8_t)g);
        CHECK(dg.current_gain == g, "core reset at G%u gave G%u", g, dg.current_gain);
    }
    /* LEVEL part: the carrier power referred to the top row (dB cells^2),
     * from 10 dB (the range edge) through ~71 dB (the bare board close in,
     * ~5 cells at G21) to 92 dB (an LNA of ~20 dB close in). */
    const float noise_top = 4.8f;   /* cells^2, the AGC default order */
    agc_level_reset(&lv, dg.model_db_q8, t.max_index, (uint16_t)(noise_top * 16.0f));
    unsigned worst_folds = 0;
    for (int level = 10; level <= 92; level += 6) {
        agc_level_reset(&lv, dg.model_db_q8, t.max_index, (uint16_t)(noise_top * 16.0f));
        uint8_t row = (uint8_t)t.max_index;
        unsigned folds_late = 0;
        for (unsigned w = 0; w < 4000; ++w) {
            const float rel = lv.rel_q8[row] / 256.0f;
            const float c = powf(10.0f, (level + rel) / 10.0f), n = noise_top * powf(10.0f, rel / 10.0f);
            dg_observation_t o = {.observed_us = 1000u + w * 250u};
            if (sqrtf(c + n) > 7.0f) {   /* past the 4-bit window: both rails, a folded (low) power */
                o.rails = 15; o.clip_pm = 300; o.pwr_x16 = (uint16_t)(42.0f * 16.0f);   /* ~uniform over the 16 codes per axis */
                if (w >= 2000) ++folds_late;
            } else {
                const float p = (c + n) * 16.0f;
                o.pwr_x16 = (uint16_t)(p > 65535.0f ? 65535.0f : p);
            }
            row = agc_level_tick(&lv, &o, row, o.observed_us);
            if (row < 2u) row = 2u;   /* receiver.c apply_target */
        }
        const float rel = lv.rel_q8[row] / 256.0f, rms = sqrtf(powf(10.0f, (level + rel) / 10.0f));
        printf("level %2d dB: G%02u carrier %.1f cells rms, folded windows in the last half %u\n", level, row, rms, folds_late);
        if (folds_late > worst_folds) worst_folds = folds_late;
        CHECK(folds_late == 0 && rms <= 7.0f, "level %d dB held G%u folding", level, row);
    }
    /* Over the window on the floor row: LEVEL asks for its lowest home (G0,
     * under the receiver floor G2). That is no room left, and STRONG LANE
     * steps (board log 2026-11-13: it did not, the raw target read as room). */
    {
        agc_level_reset(&lv, dg.model_db_q8, t.max_index, (uint16_t)(noise_top * 16.0f));
        strong_lane_t sl = {0};
        uint8_t row = 2;
        bool below_floor = false;
        for (unsigned w = 0; w < 400 && !sl.steps; ++w) {
            dg_observation_t o = {.observed_us = 1000u + w * 250u, .rails = 15, .clip_pm = 400, .pwr_x16 = (uint16_t)(42.0f * 16.0f)};
            const uint8_t target = agc_level_tick(&lv, &o, row, o.observed_us);
            below_floor |= target < STRONG_LANE_GAIN_FLOOR;
            if (strong_lane_tick(&sl, strong_lane_no_room(row, target), true, true, o.pwr_x16, o.observed_us) > 0)
                strong_lane_applied(&sl, 1, o.observed_us);
            row = target < 2u ? 2u : target;
        }
        CHECK(row == 2 && sl.steps == 1, "folding on G2: row G%u, lane steps %u", row, sl.steps);
        printf("folding on the floor row: LEVEL target under G2 %s, lane step taken\n", below_floor ? "seen" : "not seen");
    }
    /* LNA at the range edge (board 2026-11-14, ~21 dB LNA): the receiver noise
     * at G83 is ~17.5 dB over the bare board, ~270 cells^2. Bound with the
     * bare default (4.8) LEVEL takes that noise for a carrier and holds G72
     * with no VTX, the row the board held (noise ~27 cells^2, 5.2 cells rms:
     * the board folded there in 5155 of 5239 decisions; this model counts a
     * fold only over 7 cells rms). Bound with the real noise its homes start
     * where the noise is <= 9 cells^2 and no level folds, with or without a
     * carrier. */
    {
        const float lna_noise = 4.8f * powf(10.0f, 1.75f);
        unsigned old_folds = 0, new_worst = 0, old_edge_row = 0;
        for (int pass = 0; pass < 2; ++pass) {
            for (int level = -20; level <= 88; level += 4) {   /* -20: no carrier; 24: C/N 0 dB; 88: the table floor */
                agc_level_reset(&lv, dg.model_db_q8, t.max_index, (uint16_t)((pass ? lna_noise : noise_top) * 16.0f));
                uint8_t row = (uint8_t)t.max_index;
                unsigned folds_late = 0;
                for (unsigned w = 0; w < 4000; ++w) {
                    const float rel = lv.rel_q8[row] / 256.0f;
                    const float c = level > -20 ? powf(10.0f, (level + rel) / 10.0f) : 0.0f;
                    const float n = lna_noise * powf(10.0f, rel / 10.0f);
                    dg_observation_t o = {.observed_us = 1000u + w * 250u};
                    if (sqrtf(c + n) > 7.0f) {
                        o.rails = 15; o.clip_pm = 300; o.pwr_x16 = (uint16_t)(42.0f * 16.0f);
                        if (w >= 2000) ++folds_late;
                    } else {
                        const float p = (c + n) * 16.0f;
                        o.pwr_x16 = (uint16_t)(p > 65535.0f ? 65535.0f : p);
                    }
                    row = agc_level_tick(&lv, &o, row, o.observed_us);
                    if (row < 2u) row = 2u;
                }
                if (!pass) { old_folds += folds_late; if (level == -20) old_edge_row = row; }
                else {
                    const float rel = lv.rel_q8[row] / 256.0f;
                    const float n = lna_noise * powf(10.0f, rel / 10.0f);
                    if (folds_late > new_worst) new_worst = folds_late;
                    CHECK(folds_late == 0, "LNA level %d dB held G%u folding (%u)", level, row, folds_late);
                    if (level <= 24) CHECK(row == lv.top_index && n <= 9.0f,
                                           "LNA edge level %d dB: G%u (top G%u), noise %.1f cells2", level, row, lv.top_index, n);
                }
            }
            if (pass) printf("LNA noise %.0f cells2 at G83: homes from G%u, no late folds at any level; "
                             "bare-default bound held G%u with no carrier (%u late folds in this model)\n",
                             lna_noise, lv.top_index, old_edge_row, old_folds);
        }
        {   /* the old bound's no-carrier row: noise over the 9 cells^2 (the board's G72) */
            const float n_old = lna_noise * powf(10.0f, lv.rel_q8[old_edge_row] / 2560.0f);
            CHECK(old_edge_row > lv.top_index && n_old > 9.0f, "bare-default bound with no carrier held G%u (noise %.1f)",
                  old_edge_row, n_old);
        }
        CHECK(lv.top_index < t.max_index && lv.top_index >= 60, "LNA top row G%u", lv.top_index);
        /* Bare board: unchanged top (the default and the 1.7 dB higher A1 noise). */
        agc_level_reset(&lv, dg.model_db_q8, t.max_index, (uint16_t)(4.8f * 16.0f));
        CHECK(lv.top_index == t.max_index, "bare default top G%u", lv.top_index);
        agc_level_reset(&lv, dg.model_db_q8, t.max_index, (uint16_t)(4.76f * 1.48f * 16.0f));
        CHECK(lv.top_index == t.max_index, "bare A1 top G%u", lv.top_index);
    }
    printf("PASS test_agc_strong (worst late folds %u)\n", worst_folds);
    return 0;
}
