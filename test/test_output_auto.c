/* C5VRX by Twotoz and the C5VRX contributors: AUTO OUTPUT policy
 * (output_auto.h) and the affine output word (output_level.h). Host logic
 * only: no voltage, goggle or board claim. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "output_auto.h"

static unsigned rnd(unsigned *s) { *s = *s * 1103515245u + 12345u; return *s >> 16; }

/* Levels of a VTX in DAC code x4: porch at code 21 (-1 MHz), sync depth
 * deviation x 1914 kHz, white 2.5 x depth above the porch (white_scale: a
 * VTX with more picture swing, issue #200). */
typedef struct { unsigned tip, top, depth; } levels_t;
static levels_t vtx(double deviation, double white_scale, double porch_code)
{
    const double depth = deviation * 1914.0 * 63.0 / 15000.0;
    levels_t l = {(unsigned)(4.0 * (porch_code - depth) + 0.5), (unsigned)(4.0 * (porch_code + 2.5 * depth * white_scale) + 0.5),
                  (unsigned)(4.0 * depth + 0.5)};
    return l;
}

/* Feed `batches` batches with a little measurement noise; returns the number
 * of applications and leaves the applied word in the state. */
static unsigned feed(output_auto_t *a, levels_t l, unsigned batches, int64_t *now, unsigned *seed, bool apply_ok)
{
    unsigned applied = 0;
    for (unsigned k = 0; k < batches * OA_BATCH; ++k) {
        *now += 55000;   /* ~18 windows a second */
        unsigned word = 0;
        const unsigned j = rnd(seed) % 3u;
        if (output_auto_observe(a, l.tip + j - 1u, l.top + j - 1u, l.depth + (rnd(seed) & 1u), *now, &word)) {
            assert(output_is_affine(word));
            if (apply_ok) { output_auto_applied(a, word, *now); ++applied; }
        }
    }
    return applied;
}

static void mapped(unsigned word, levels_t l, int *tip, int *porch, int *white)
{
    *tip = output_entry((uint16_t)((l.tip + 2u) / 4u), word, NULL) & 63;
    *porch = output_entry((uint16_t)((l.tip + l.depth + 2u) / 4u), word, NULL) & 63;
    *white = output_entry((uint16_t)((l.top + 2u) / 4u), word, NULL) & 63;
}

int main(void)
{
    unsigned seed = 7, checks = 0;
    int64_t now = 1000000;
    /* 1. The affine word: identity gain keeps every code relative to the tip,
     * metadata bits are untouched, saturation stays inside the DAC. */
    for (unsigned g = OA_GAIN_MIN_Q3; g <= OA_GAIN_MAX_Q3; ++g)
        for (unsigned tip = 0; tip < 40u; ++tip)
            for (unsigned c = 0; c < 64u; ++c) {
                bool clipped;
                const uint16_t out = output_entry((uint16_t)(0xabc0u | c), output_affine(g, tip), &clipped);
                assert((out & 0xffc0u) == 0xabc0u);
                const double want = OUTPUT_AFFINE_TIP + g / 8.0 * ((double)c - (double)tip);
                const double sat = want < 0 ? 0 : want > 63 ? 63 : want;
                assert(abs((int)(out & 63u) - (int)(sat + 0.5)) <= 1);
                assert(clipped == (want < -0.5 || want >= 63.5) || abs((int)(want * 8) % 8) == 4);
                ++checks;
            }
    assert((output_entry(17, output_affine(8, 17), NULL) & 63u) == OUTPUT_AFFINE_TIP);
    /* 2. A low-deviation VTX (0.69, the board's): settles once, after two
     * batches, with the composite inside the DAC, the sync >= 2x deeper and
     * the tip well clear of code 0 (research/output_tip_headroom.py). */
    {
        output_auto_t a; output_auto_reset(&a);
        const levels_t l = vtx(0.69, 1.0, 21.0);
        assert(feed(&a, l, 1, &now, &seed, true) == 0);          /* one batch decides nothing */
        assert(feed(&a, l, 1, &now, &seed, true) == 1);
        const unsigned g = output_affine_gain_q3(a.word);
        int tip, porch, white;
        mapped(a.word, l, &tip, &porch, &white);
        printf("deviation 0.69: gain %.2fx tip %d porch %d white %d (1x: sync %.1f codes)\n", g / 8.0, tip, porch, white, l.depth / 4.0);
        assert(g >= 16 && g <= OA_GAIN_MAX_Q3 && tip >= OUTPUT_AFFINE_TIP - 1 && tip <= OUTPUT_AFFINE_TIP + 1 &&
               white <= 62 && white >= 50);
        assert(porch - tip >= 11 && porch - tip <= OA_SYNC_CODES + 1);
        /* Room under the tip: the unscaled codes down to the decoder's floor
         * that still come out above code 0 (at least 1 MHz under the tip). */
        assert((output_entry((uint16_t)((l.tip + 2u) / 4u - 5u), a.word, NULL) & 63u) > 0u);
        /* Steady signal: no further restart, however long. */
        assert(feed(&a, l, 400, &now, &seed, true) == 0);
        ++checks;
    }
    /* 3. Nominal and wide VTXs: the gain shrinks with the deviation and a
     * wide one (1.7) is left alone (no restart for less than 1.25x). */
    {
        output_auto_t a; output_auto_reset(&a);
        const levels_t n = vtx(1.0, 1.0, 21.0);
        assert(feed(&a, n, 2, &now, &seed, true) == 1);
        int tip, porch, white;
        mapped(a.word, n, &tip, &porch, &white);
        printf("deviation 1.00: gain %.2fx tip %d porch %d white %d\n", output_affine_gain_q3(a.word) / 8.0, tip, porch, white);
        assert(tip >= OUTPUT_AFFINE_TIP - 1 && white <= 62 && porch - tip >= 11);
        output_auto_reset(&a);
        assert(feed(&a, vtx(1.7, 1.0, 21.0), 20, &now, &seed, true) == 0 && !a.word);
        ++checks;
    }
    /* 4. More picture swing than the standard (issue #200 levels): the
     * observed white bounds the gain, white stays on the DAC. */
    {
        output_auto_t a; output_auto_reset(&a);
        const levels_t l = vtx(0.69, 1.58, 18.9);
        assert(feed(&a, l, 2, &now, &seed, true) == 1);
        int tip, porch, white;
        mapped(a.word, l, &tip, &porch, &white);
        printf("issue #200 levels: gain %.2fx tip %d porch %d white %d\n", output_affine_gain_q3(a.word) / 8.0, tip, porch, white);
        assert(white <= 62 && tip >= OUTPUT_AFFINE_TIP - 1);
        ++checks;
    }
    /* 5. A dark scene must not raise the gain (the standard span bounds it),
     * and a bright scene after a dark one lowers it only when clipping is at
     * stake, within two batches. */
    {
        output_auto_t a; output_auto_reset(&a);
        levels_t dark = vtx(0.69, 1.0, 21.0);
        dark.top = dark.tip + dark.depth + 8u;                   /* nearly black picture */
        assert(feed(&a, dark, 2, &now, &seed, true) == 1);
        const unsigned g_dark = output_affine_gain_q3(a.word);
        output_auto_t b; output_auto_reset(&b);
        (void)feed(&b, vtx(0.69, 1.0, 21.0), 2, &now, &seed, true);
        assert(g_dark == output_affine_gain_q3(b.word));
        const levels_t bright = vtx(0.69, 1.58, 21.0);
        const unsigned n = feed(&a, bright, 3, &now, &seed, true);
        assert(n == 1 && output_affine_gain_q3(a.word) < g_dark);
        /* Back to a normal scene: the span is kept, the level never hunts
         * with the picture content (~6 minutes of it here). */
        const unsigned g_bright = output_affine_gain_q3(a.word);
        assert(feed(&a, vtx(0.69, 1.0, 21.0), 400, &now, &seed, true) == 0 && output_affine_gain_q3(a.word) == g_bright);
        assert(feed(&a, dark, 400, &now, &seed, true) == 0);
        ++checks;
    }
    /* 6. Scatter (fading, a wrong sync) and missing sync add nothing; a
     * refused application keeps asking without inventing a new state. */
    {
        output_auto_t a; output_auto_reset(&a);
        const levels_t l = vtx(0.69, 1.0, 21.0);
        for (unsigned k = 0; k < 40u * OA_BATCH; ++k) {
            unsigned word;
            now += 55000;
            const unsigned wild = rnd(&seed) % 160u;
            assert(!output_auto_observe(&a, l.tip + wild, l.top, l.depth + wild, now, &word));
        }
        assert(a.rejected && !a.word && !a.batches);
        unsigned word;
        assert(!output_auto_observe(&a, l.tip, l.top, 4u, now, &word));          /* no sync depth */
        assert(!output_auto_observe(&a, l.top, l.tip, l.depth, now, &word));     /* no span */
        output_auto_reset(&a);
        assert(feed(&a, l, 6, &now, &seed, false) == 0 && !a.word && a.want);    /* transport refused */
        assert(feed(&a, l, 1, &now, &seed, true) == 1);
        ++checks;
    }
    /* 7. A carrier that moved (AFC step, VTX drift): the tip is followed
     * from 3 codes on, and small shifts cost no restart. */
    {
        output_auto_t a; output_auto_reset(&a);
        (void)feed(&a, vtx(0.69, 1.0, 21.0), 2, &now, &seed, true);
        assert(feed(&a, vtx(0.69, 1.0, 22.5), 30, &now, &seed, true) == 0);
        assert(feed(&a, vtx(0.69, 1.0, 25.0), 3, &now, &seed, true) == 1);
        ++checks;
    }
    printf("PASS: AUTO OUTPUT affine word and policy (%u checks)\n", checks);
    return 0;
}
