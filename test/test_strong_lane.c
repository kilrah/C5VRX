/* C5VRX-5 by Twotoz and the C5VRX contributors: STRONG LANE policy
 * (main/strong_lane.h). An ideal constant-envelope carrier of a given radius
 * (ultrafine cells) on the floor row; the lane halves the radius per step and
 * a window folds above 8 cells. Policy only: no RF, route or picture claim. */
#include <stdio.h>
#include <stdlib.h>
#include "strong_lane.h"

#define CHECK(c, ...) do { if (!(c)) { printf("FAIL " __VA_ARGS__); printf("\n"); exit(1); } } while (0)

static strong_lane_t s;
static uint64_t now;
static unsigned folded_windows;

/* Run `ms` of 200 us windows of a carrier with radius r (ultrafine cells). */
static void run(float r, unsigned ms, bool no_room)
{
    for (unsigned w = 0; w < ms * 5u; ++w, now += 200u) {
        const float rl = r / (float)(1u << s.steps);
        const bool folded = rl > 8.0f;
        folded_windows += folded;
        const float p = rl * rl * 16.0f;
        const int step = strong_lane_tick(&s, no_room, folded, false, (uint16_t)(folded ? 42 * 16 : p > 65535.0f ? 65535.0f : p), now);
        if (step) strong_lane_applied(&s, step, now);
    }
}

int main(void)
{
    now = 1000000u;
    /* A folding carrier while the gain still has room: the gain's job. */
    run(20.0f, 500, false);
    CHECK(s.steps == 0 && !s.ups, "moved with gain room left");
    /* In range on the floor row: nothing. */
    run(6.0f, 500, true);
    CHECK(s.steps == 0 && !s.ups, "moved without a fold");
    /* 6 dB over the window on the floor row: the fine lane, within 25 ms. */
    folded_windows = 0;
    run(12.0f, 500, true);
    CHECK(s.steps == 1 && s.ups == 1 && folded_windows <= 125u, "12 cells: steps %u folded %u", s.steps, folded_windows);
    /* 12 dB over: the coarse lane, and it stays (no hunting). */
    folded_windows = 0;
    run(28.0f, 1000, true);
    CHECK(s.steps == 2 && s.ups == 2 && s.downs == 0 && folded_windows <= 125u, "28 cells: steps %u folded %u", s.steps, folded_windows);
    /* Beyond the coarse lane there is no further step. */
    run(80.0f, 300, true);
    CHECK(s.steps == 2 && s.ups == 2, "stepped past coarse");
    /* The radius between the two thresholds on either lane: no step (4 dB apart). */
    run(28.0f, 300, true);
    const uint32_t ups = s.ups, downs = s.downs;
    run(14.0f, 2000, true);   /* 3.5 cells on coarse, 7 on fine */
    CHECK(s.ups == ups && s.downs == downs, "hunted at 14 cells: ups %u downs %u", (unsigned)s.ups, (unsigned)s.downs);
    /* Moving away: back one lane at a time, never into a fold. */
    folded_windows = 0;
    run(9.0f, 1000, true);    /* 2.25 on coarse -> fine 4.5 */
    CHECK(s.steps == 1 && folded_windows == 0, "9 cells: steps %u", s.steps);
    run(4.0f, 1000, true);    /* 2 on fine -> ultrafine 4 */
    CHECK(s.steps == 0 && folded_windows == 0, "4 cells: steps %u", s.steps);
    /* The carrier goes away on the coarse lane: ultrafine within a second. */
    run(40.0f, 300, true);
    CHECK(s.steps == 2, "not coarse again");
    run(0.0f, 900, true);
    CHECK(s.steps == 0, "no carrier kept lane step %u", s.steps);
    /* A clipped (not folded) window is not a small carrier. */
    run(40.0f, 300, true);
    for (unsigned w = 0; w < 5000u; ++w, now += 200u) {
        const int step = strong_lane_tick(&s, true, false, true, 10, now);
        CHECK(step == 0, "clipped window released the lane");
    }
    printf("PASS test_strong_lane (ups %u downs %u)\n", (unsigned)s.ups, (unsigned)s.downs);
    return 0;
}
