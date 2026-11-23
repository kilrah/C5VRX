#include "menu_raster.h"
#include <string.h>

/* ITU-R BT.470: PAL 625/50, NTSC 525/59.94. Round cumulative NTSC
 * half-line time (11440/9 samples) to a DMA word, never each line independently.
 * Within a cycle the longest edge error is 44.45 ns. */
uint32_t menu_half_sample(video_standard_t standard, unsigned half)
{
    return standard == VIDEO_STD_PAL ? half * 1280u :
           ((half * 2860u + 4u) / 9u) * 4u;
}

/* Monochrome raster (2026-11-14): the menu has no colour, so no burst.
 * Every line then starts with the same prefix, and a line's blank tail and
 * the next line's prefix are one DMA node (tail_prefix): a UI line costs two
 * nodes (UI row, tail + next prefix), a blank line one. With eight burst
 * phases each line needed its own prefix node: NTSC interlaced took 1499
 * nodes (~18 KiB of descriptors), which never fitted at boot (also not on
 * 5.0.593), so the menu ran in the progressive 240p fallback, which the
 * operator's goggles showed inverting and desyncing. */
void menu_raster_init(menu_raster_t *r, video_standard_t standard, bool interlaced)
{
    (void)interlaced;
    memset(r, 20, sizeof(*r));
    memset(r->tail_prefix + MENU_TAIL_BYTES, 0, 188); /* 4.7 us H sync at the line start */
    memset(r->equalizing, 0, standard == VIDEO_STD_PAL ? 94 : 92);
    /* Broad pulse ends 4.7 us before the following half-line edge.
     * NTSC short/long half-lines select a shifted start below. */
    memset(r->broad, 0, 1280 - 188);
}

/* Whether half-line h of the loop starts a normal (sync + picture) line. */
static bool line_start(video_standard_t standard, bool interlaced, unsigned h)
{
    const bool pal = standard == VIDEO_STD_PAL;
    const unsigned eq = pal ? 5 : 6, field_halves = menu_field_halves(standard, interlaced);
    h %= menu_loop_halves(standard, interlaced);
    return !(h & 1u) && (h + (pal ? eq : 0)) % field_halves >= 3 * eq;
}

bool menu_raster_emit(const menu_raster_t *r, video_standard_t standard, bool interlaced,
                      menu_segment_fn emit, void *ctx)
{
    const bool pal = standard == VIDEO_STD_PAL;
    const unsigned field_halves = menu_field_halves(standard, interlaced);
    const unsigned eq = pal ? 5 : 6;
    /* Interlaced: a complete two-field frame. Progressive: one field; NTSC's
     * 18 vertical half-lines leave 253 full lines, PAL's 15 leave one
     * half-line before the pre-equalizing pulses, as in its first field. */
    const unsigned total_halves = menu_loop_halves(standard, interlaced);
    /* Centre the UI in the active picture, clear of vertical blanking. */
    const unsigned text_start = menu_ui_first_line(standard);
    const unsigned y_repeat = menu_ui_y_repeat(standard);
    const unsigned prefix_bytes = menu_prefix_bytes(standard);
    const uint8_t *prefix = r->tail_prefix + MENU_TAIL_BYTES;
    bool prefix_sent = false;   /* the previous node ended with this line's prefix */
    for (unsigned h = 0; h < total_halves;) {
        /* PAL line 1 starts with broad sync; its five pre-equalizing
         * half-lines belong to the end of the preceding field. NTSC line 1
         * starts with pre-equalization (BT.470 figures 2-1 and 2-2). */
        unsigned pos = (h + (pal ? eq : 0)) % field_halves;
        unsigned start = menu_half_sample(standard, h);
        unsigned length = menu_half_sample(standard, h + 1) - start;
        if (pos < 3 * eq) {
            const uint8_t *data = (pos >= eq && pos < 2 * eq) ?
                r->broad + 1280 - length : r->equalizing;
            if (!emit(ctx, data, length)) return false;
            ++h;
            prefix_sent = false;
            continue;
        }
        /* Horizontal sync stays on the global full-line grid. The second
         * field's vertical train starts on the intervening half-line. */
        if (h & 1u) {
            if (!emit(ctx, r->tail_prefix, length)) return false;
            ++h;
            prefix_sent = false;
            continue;
        }
        unsigned halves = pos + 1 < field_halves ? 2 : 1;
        length = menu_half_sample(standard, h + halves) - start;
        if (!prefix_sent && !emit(ctx, prefix, prefix_bytes)) return false;
        unsigned line = pos / 2;
        unsigned remaining = length - prefix_bytes;
        if (line >= text_start && line < text_start + y_repeat * MENU_UI_LINES) {
            if (!emit(ctx, r->ui[(line - text_start) / y_repeat], MENU_UI_BYTES)) return false;
            remaining -= MENU_UI_BYTES;
        }
        h += halves;
        /* The blank tail, with the next line's prefix when one follows. */
        prefix_sent = h < total_halves && line_start(standard, interlaced, h);
        if (prefix_sent) {
            if (!emit(ctx, prefix - remaining, remaining + prefix_bytes)) return false;
        } else if (!emit(ctx, r->tail_prefix, remaining)) {
            return false;
        }
    }
    return true;
}
