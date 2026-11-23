/* C5VRX-5 by Twotoz and the C5VRX contributors: AUTO OUTPUT, the video level
 * a normal analog receiver delivers, set from the received signal.
 *
 * Extends issue #200 (manual OUTPUT) and its study research/output_level.py:
 * at OUTPUT 1.0x the DAC maps -6..+9 MHz onto its 64 codes, so a VTX of
 * deviation 0.69 uses ~19 of them and its sync is ~5.5 codes deep (model:
 * 0.060 V into 75 ohm where CVBS has 0.286 V; deviation 1.0: 0.089 V). A
 * goggle's sync slicer then works on a third of the level it was made for.
 * The study's ideal sync/white-referenced gain doubled that depth without
 * clipping, and listed what a live version needs: sync, porch and white
 * headroom measured, context resets, a failed-write rollback, at least 2 s of
 * stable evidence, the shared 5 s application gap, and a hold on weak or
 * invalid observations. This is that policy; transport.c applies the word by
 * the existing verified restart and rollback.
 *
 * Evidence per control window (control.c), all of the unscaled live decoder:
 *   tip_x4, top_x4  sync tip (3rd percentile) and peak white (99.5th) of the
 *                   decoded video, DAC code x4 (picture_sync.h);
 *   depth_x4        sync tip to back porch from the AFC's burst/timing-
 *                   confirmed levels (afc2_measure), DAC code x4.
 * A window counts only with confirmed syncs, at least 80 % of its sync
 * pulses present and levels apart from the bulk: weak, fading or missing
 * video adds nothing, so the setting is held, never walked by a fade.
 *
 * OA_BATCH windows give medians; a batch whose depth or tip scatter is rejected.
 * The composite span is the larger of the observed tip-to-white and the
 * standard's 3.5 x sync depth (a dark scene must not raise the gain), and
 * its largest value is kept for the whole context: a scene that turns dark
 * again never raises the gain, so picture content cannot make the level
 * hunt (each change is a restart). The gain puts that span on OA_SPAN_CODES
 * with the tip
 * at OUTPUT_AFFINE_TIP, at most OA_SYNC_CODES of sync and OA_GAIN_MAX_Q3.
 * The tip keeps OUTPUT_AFFINE_TIP codes of room under it (output_level.h):
 * noise and FM clicks under the tip must not be cut on one side at the edge.
 * Excursions past the DAC saturate, as in a video amplifier.
 *
 * Changes cost one ~5 ms verified restart, so they are rare by rule:
 *   first setting   two agreeing batches, gain >= 1.25x;
 *   down / tip      two agreeing batches when the gain must fall by >= 0.25x
 *                   or the tip moved >= 3 codes (clipping is at stake);
 *   up              four agreeing batches, >= 0.75x more and 30 s since the
 *                   last change (only a deeper sync bound can ask for it:
 *                   the span itself never shrinks within a context).
 * A context change (channel, decoder, standard, manual OUTPUT) resets all of
 * it and the caller returns to the manual level. Host test:
 * test/test_output_auto.c. No voltage is claimed: the DAC's loaded span is
 * not calibrated (research/output_level.py), only its use of the range. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "output_level.h"

#define OA_BATCH          16u
#define OA_SPAN_CODES     (60 - OUTPUT_AFFINE_TIP)   /* peak white at code 60 */
#define OA_SYNC_CODES     18      /* 40/140 of the DAC range */
#define OA_GAIN_MIN_Q3    8       /* 1.0x */
#define OA_GAIN_MAX_Q3    32      /* 4.0x */
#define OA_FIRST_MIN_Q3   10      /* below 1.25x a restart buys nothing */
#define OA_DOWN_Q3        2
#define OA_UP_Q3          6
#define OA_TIP_CODES      3
#define OA_UP_AFTER_US    30000000LL
#define OA_MIN_DEPTH_X4   12      /* 3 codes: under it no sync to reference */

typedef struct {
    uint16_t tip[OA_BATCH], top[OA_BATCH], depth[OA_BATCH];
    unsigned n;
    unsigned span_hold_x4;        /* largest tip-to-white span of this context; 0 none yet */
    unsigned word;                /* applied affine word, 0 = the manual level */
    unsigned want;                /* candidate word, 0 none */
    unsigned agree;               /* consecutive batches agreeing with want */
    int64_t applied_us;
    uint32_t batches, rejected, applied;
    uint16_t last_tip_x4, last_depth_x4, last_span_x4;   /* telemetry: the last accepted batch */
} output_auto_t;

static inline void output_auto_reset(output_auto_t *a) { memset(a, 0, sizeof(*a)); }

static inline unsigned oa_median(uint16_t *v, unsigned n)
{
    for (unsigned i = 1; i < n; ++i)
        for (unsigned j = i; j > 0 && v[j - 1] > v[j]; --j) { const uint16_t t = v[j]; v[j] = v[j - 1]; v[j - 1] = t; }
    return n & 1u ? v[n / 2] : (unsigned)(v[n / 2 - 1] + v[n / 2]) / 2u;
}
static inline unsigned oa_mad(const uint16_t *v, unsigned n, unsigned med)
{
    uint16_t d[OA_BATCH];
    for (unsigned i = 0; i < n; ++i) d[i] = (uint16_t)(v[i] > med ? v[i] - med : med - v[i]);
    return oa_median(d, n);
}

/* The gain (Q3) and source tip for measured levels. */
static inline unsigned output_auto_word(unsigned tip_x4, unsigned depth_x4, unsigned span_x4)
{
    unsigned g = 8u * 4u * OA_SPAN_CODES / (span_x4 ? span_x4 : 1u);
    const unsigned g_sync = 8u * 4u * OA_SYNC_CODES / (depth_x4 ? depth_x4 : 1u);
    if (g > g_sync) g = g_sync;
    if (g > OA_GAIN_MAX_Q3) g = OA_GAIN_MAX_Q3;
    if (g < OA_GAIN_MIN_Q3) g = OA_GAIN_MIN_Q3;
    unsigned tip = (tip_x4 + 2u) / 4u;
    if (tip > 63u) tip = 63u;
    return output_affine(g, tip);
}

static inline bool oa_near(unsigned a, unsigned b)
{
    const int dg = (int)output_affine_gain_q3(a) - (int)output_affine_gain_q3(b);
    const int dt = (int)output_affine_tip(a) - (int)output_affine_tip(b);
    return dg >= -1 && dg <= 1 && dt >= -1 && dt <= 1;
}

/* One qualified window. Returns true when `*word` should be applied now; the
 * caller reports the outcome with output_auto_applied() on success and simply
 * observes on (the candidate stays) when the transport refuses. */
static inline bool output_auto_observe(output_auto_t *a, unsigned tip_x4, unsigned top_x4, unsigned depth_x4,
                                       int64_t now_us, unsigned *word)
{
    if (depth_x4 < OA_MIN_DEPTH_X4 || top_x4 <= tip_x4) return false;
    a->tip[a->n] = (uint16_t)tip_x4; a->top[a->n] = (uint16_t)top_x4; a->depth[a->n] = (uint16_t)depth_x4;
    if (++a->n < OA_BATCH) return false;
    a->n = 0;
    uint16_t v[OA_BATCH];
    memcpy(v, a->depth, sizeof(v));
    const unsigned depth = oa_median(v, OA_BATCH);
    memcpy(v, a->tip, sizeof(v));
    const unsigned tip = oa_median(v, OA_BATCH);
    memcpy(v, a->top, sizeof(v));
    const unsigned top = oa_median(v, OA_BATCH);
    /* Scatter: 15 % of the depth, 1.5 codes of tip. */
    if (oa_mad(a->depth, OA_BATCH, depth) * 100u > depth * 15u || oa_mad(a->tip, OA_BATCH, tip) > 6u ||
        depth < OA_MIN_DEPTH_X4 || top <= tip) {
        ++a->rejected;
        a->agree = 0; a->want = 0;
        return false;
    }
    ++a->batches;
    unsigned span = top - tip;
    const unsigned standard = depth * 7u / 2u;      /* sync 40, white 100 of 140 */
    if (span < standard) span = standard;
    /* Peak hold for the context: a bright scene returns. */
    if (span > a->span_hold_x4) a->span_hold_x4 = span;
    a->last_tip_x4 = (uint16_t)tip; a->last_depth_x4 = (uint16_t)depth; a->last_span_x4 = (uint16_t)a->span_hold_x4;
    const unsigned cand = output_auto_word(tip, depth, a->span_hold_x4);
    const int g = (int)output_affine_gain_q3(cand), t = (int)output_affine_tip(cand);
    bool wanted;
    unsigned need = 2u;
    if (!a->word) {
        wanted = g >= OA_FIRST_MIN_Q3;
    } else {
        const int cg = (int)output_affine_gain_q3(a->word), ct = (int)output_affine_tip(a->word);
        const bool down = g <= cg - OA_DOWN_Q3, moved = t - ct >= OA_TIP_CODES || ct - t >= OA_TIP_CODES;
        const bool up = g >= cg + OA_UP_Q3 && now_us - a->applied_us >= OA_UP_AFTER_US;
        wanted = down || moved || up;
        if (up && !down && !moved) need = 4u;
    }
    if (!wanted) { a->agree = 0; a->want = 0; return false; }
    if (a->want && oa_near(a->want, cand)) ++a->agree;
    else { a->want = cand; a->agree = 1; }
    if (a->agree < need) return false;
    *word = a->want;
    return true;
}

static inline void output_auto_applied(output_auto_t *a, unsigned word, int64_t now_us)
{
    a->word = word;
    a->applied_us = now_us;
    a->want = 0; a->agree = 0;
    ++a->applied;
}
