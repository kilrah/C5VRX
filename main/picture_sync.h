/* C5VRX by Twotoz and the C5VRX contributors: a goggle-like picture measure
 * for the flight log, the share of expected sync pulses present in the
 * demodulated video of one IQ window.
 *
 * FIELD_DIAG video_pct counts a control window only when the AFC can measure
 * it: a stationary envelope and timing-confirmed sync and porch pairs within
 * ~3 lines. Fading, a gain write or one noisy sync fails the whole window,
 * and the edge read 20-40 % where the operator saw a clean picture (walk 296,
 * -85..-90 dBm; FINDINGS 2026-10-27: windows called lost held clean syncs
 * through the decoder). A goggle's line PLL rides through missing syncs, so
 * this counts them one by one, with none of the AFC's gates.
 *
 * The window is decoded in software with the live decoder's LUT16 (the
 * pair/state rule of the TX BitScrambler, research/sim.py _pair_codes): one
 * DAC code per byte pair at 20 MS/s, then a 4-pair sum (~5 MHz, the goggle
 * filter). Sync tip and blanking differ with the deviation (P64 and U85Q:
 * tip ~12-16, blanking ~25 codes at deviation 0.66), so the slicer sits
 * halfway between the 3rd percentile and the median. A pulse is >= 2 us low
 * (horizontal 4.7 us, VBI equalising and broad pulses too); pulses within
 * 0.7 line of each other count once.
 *
 * One decoding pass: the slicer comes from the previous window's histogram
 * (the levels do not change in a control tick), this window's histogram sets
 * the next one; a window without a slicer is not counted (found and expected
 * 0). No buffer, so no heap. The control task shares priority 3 with the gain
 * observer: PICTURE_YIELD() runs every PICTURE_YIELD_PAIRS so the observer
 * waits at most ~0.25 ms (board: ~3.8 ms wall per window with two passes and
 * no yield). Host test: test/test_picture_sync.py (against a NumPy reference
 * on simulated windows, both decoders). */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PICTURE_LINE_PAIRS_X10 12711u   /* NTSC line, 63.556 us at 20 MS/s, x10 */
#define PICTURE_MIN_LOW_PAIRS 40u       /* 2 us */
#define PICTURE_SETTLE_PAIRS 32u        /* decoder state settles from 0 */
#define PICTURE_MIN_SPAN 20u            /* median - tip in 4-pair sums (5 codes): else no sync */
#define PICTURE_YIELD_PAIRS 512u
#ifndef PICTURE_YIELD
#define PICTURE_YIELD() ((void)0)
#endif

typedef struct { uint16_t found, expected; } picture_sync_t;
/* The slicer carried from one window to the next (zero: none yet). */
typedef struct {
    const uint16_t *lut; uint16_t threshold_x2; bool valid;
    /* The last window's levels in 4-pair sums (DAC code x4) of the unscaled
     * decoder, for AUTO OUTPUT (output_auto.h): the 3rd percentile (sync
     * tip), the median and the 99.5th percentile (peak white, chroma
     * included). levels: a sync level stood apart from the bulk. */
    uint16_t tip_x4, mid_x4, top_x4; bool levels;
} picture_state_t;

/* One DAC code per byte pair, as the TX BitScrambler: lut16 with `bits`-bit
 * tokens (P64 4, U85Q 5). */
static inline uint8_t picture_decode(const uint16_t *lut, unsigned bits, unsigned *state, uint8_t a, uint8_t b)
{
    const unsigned address = (a >> 4) | ((a & 15u) << 4) | (((b >> 7) & 1u) << 8) | (((b >> 3) & 1u) << 9);
    const unsigned token = lut[address] >> (16u - bits);
    const unsigned value = lut[(*state << bits) + token];
    *state = (value >> 6) & ((1u << (10u - bits)) - 1u);
    return (uint8_t)(value & 63u);
}

static inline picture_sync_t picture_sync_window(picture_state_t *ps, const uint8_t *raw, size_t bytes,
                                                 const uint16_t *lut, unsigned bits)
{
    picture_sync_t r = {0, 0};
    const size_t pairs = bytes / 2u;
    if (!lut || pairs < PICTURE_SETTLE_PAIRS + 2u * PICTURE_LINE_PAIRS_X10 / 10u) return r;
    if (ps->lut != lut) { ps->lut = lut; ps->valid = false; }   /* another decoder: other levels */
    const bool count = ps->valid;
    const unsigned threshold_x2 = ps->threshold_x2;              /* sum < threshold_x2 / 2 */
    uint16_t hist[253] = {0};
    uint8_t last[4] = {0};
    unsigned state = 0, sum = 0, n = 0, run = 0, found = 0;
    unsigned since = PICTURE_LINE_PAIRS_X10;   /* 10 lines: the first pulse counts; x100 below stays in range */
    for (size_t k = 0; k < pairs; ++k) {
        if (k && !(k % PICTURE_YIELD_PAIRS)) PICTURE_YIELD();
        const uint8_t c = picture_decode(lut, bits, &state, raw[2 * k], raw[2 * k + 1]);
        if (k < PICTURE_SETTLE_PAIRS) continue;
        sum += c; sum -= last[n & 3u]; last[n & 3u] = c; ++n;
        if (n < 4u) continue;
        ++hist[sum];
        ++since;
        if (2u * sum < threshold_x2) {
            if (++run == PICTURE_MIN_LOW_PAIRS && (since - run) * 100u > 7u * PICTURE_LINE_PAIRS_X10) {   /* > 0.7 line */
                ++found;
                since = run;   /* measured from the pulse's start */
            }
        } else {
            run = 0;
        }
    }
    const unsigned total = n >= 4u ? n - 3u : 0u;
    /* This window's levels set the next window's slicer. */
    unsigned tip = 0, mid = 0, top = 0, acc = 0;
    bool have_mid = false;
    for (unsigned v = 0; v < 253u && total; ++v) {
        const unsigned before = acc;
        acc += hist[v];
        if (before * 100u < 3u * total && acc * 100u >= 3u * total) tip = v;
        if (!have_mid && before * 2u < total && acc * 2u >= total) { mid = v; have_mid = true; }
        if (before * 200u < 199u * total && acc * 200u >= 199u * total) { top = v; break; }
    }
    /* No sync level apart from the bulk (noise): a slicer under everything. */
    ps->threshold_x2 = (uint16_t)(mid >= tip + PICTURE_MIN_SPAN ? tip + mid : 0u);
    ps->valid = total != 0;
    ps->tip_x4 = (uint16_t)tip; ps->mid_x4 = (uint16_t)mid; ps->top_x4 = (uint16_t)top;
    ps->levels = total != 0 && mid >= tip + PICTURE_MIN_SPAN && top >= mid;
    if (!count) return r;
    r.expected = (uint16_t)(total * 10u / PICTURE_LINE_PAIRS_X10);
    if (!r.expected) r.expected = 1;
    r.found = (uint16_t)(found > r.expected ? r.expected : found);
    return r;
}
