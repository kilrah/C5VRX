/* C5VRX-5 by Twotoz and the C5VRX contributors: STRONG LANE, a coarser IQ
 * lane for a carrier that still folds on the lowest gain rows.
 *
 * Extends the lane work of C5VRX-3/4 and docs/FINDINGS.md "Edge window"
 * (2026-11-10): a lane change is a GPIO-matrix route, with no analog step,
 * no other DC row and no decoder restart (the decoder's token plane is the
 * angle of the signed nibble pair, so the same table decodes every lane).
 * The fine lane holds 6 dB and the coarse lane 12 dB more than the ultrafine
 * one at the same analog gain. The general live lane change stays retired
 * (2026-10-04): this one acts only where the analog gain has no room left,
 * a state the bare board does not reach (close in G21/G24) and an external
 * LNA does (Discord report 2026-11-13: after the gain floor fixes the
 * picture still degraded at 20 cm).
 *
 * Coarser: the gain sits on the lowest rows, its controller has no lower row
 * to go to, and the windows keep folding. Finer: the carrier radius on the
 * coarser lane stayed small enough for the finer one (twice the radius) for
 * a while. No carrier reads as small, so a lost carrier returns to the
 * ultrafine lane and the normal gain control (the known high-gain state).
 * Pure logic; host test test/test_strong_lane.c. No board result yet. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define STRONG_LANE_MAX 2u            /* 0 ultrafine, 1 fine, 2 coarse */
#define STRONG_LANE_FLOOR_ROW 9u      /* RF stage 0 of the 5 GHz table: rows 0..9 */
#define STRONG_LANE_UP_SCORE 16u      /* folded windows over clean ones (200 us each) */
#define STRONG_LANE_UP_DWELL_US 20000u
/* 2.5 cells rms on this lane is 5 on the finer one; a fold needs ~8, so a
 * step up lands at 4 or more: 4 dB between the two decisions. */
#define STRONG_LANE_DOWN_PWR_X16 100u
#define STRONG_LANE_DOWN_US 200000u

#define STRONG_LANE_GAIN_FLOOR 2u      /* the lowest row the receiver writes (receiver.c apply_target) */

/* The gain has no room left: it is on the floor rows and its controller asks
 * for no lower row than the receiver can write. LEVEL's lowest home row is
 * G0, under that floor: on G2 it kept asking for G0, which read as room left,
 * and the lane never stepped on G2 (board log 2026-11-13, PR #304 build:
 * runs of clipped windows on G2, every step taken on G4). */
static inline bool strong_lane_no_room(uint8_t current, uint8_t target)
{
    if (target < STRONG_LANE_GAIN_FLOOR) target = STRONG_LANE_GAIN_FLOOR;
    return current <= STRONG_LANE_FLOOR_ROW && target >= current;
}

typedef struct {
    uint8_t steps;
    uint8_t score;
    uint64_t change_us, small_since_us;
    bool small;
    uint32_t ups, downs;
} strong_lane_t;

/* One observer window. no_room: the gain is on the floor rows and its
 * controller asked for no lower row (ignored on a coarser lane, where the
 * gain is held). Returns +1 (one lane coarser), -1 (one finer) or 0; the
 * caller routes the lane and reports the step with strong_lane_applied. */
static inline int strong_lane_tick(strong_lane_t *s, bool no_room, bool folded, bool clipped, uint16_t pwr_x16, uint64_t now_us)
{
    if (folded) {
        s->small = false;
        if (s->steps == 0u && !no_room) { s->score = 0; return 0; }
        if (s->score < 255u) ++s->score;
        if (s->steps < STRONG_LANE_MAX && s->score >= STRONG_LANE_UP_SCORE && now_us - s->change_us >= STRONG_LANE_UP_DWELL_US)
            return 1;
        return 0;
    }
    if (s->score) --s->score;
    if (s->steps == 0u) return 0;
    if (clipped || pwr_x16 > STRONG_LANE_DOWN_PWR_X16) { s->small = false; return 0; }
    if (!s->small) { s->small = true; s->small_since_us = now_us; }
    return now_us - s->small_since_us >= STRONG_LANE_DOWN_US && now_us - s->change_us >= STRONG_LANE_DOWN_US ? -1 : 0;
}

static inline void strong_lane_applied(strong_lane_t *s, int step, uint64_t now_us)
{
    if (step > 0 && s->steps < STRONG_LANE_MAX) { ++s->steps; ++s->ups; }
    else if (step < 0 && s->steps) { --s->steps; ++s->downs; }
    s->score = 0;
    s->small = false;
    s->change_us = now_us;
}
