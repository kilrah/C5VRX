#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Standalone CVBS menu raster at 40 MHz.
 *
 * The modern UI is 384 x 56 logical pixels. Horizontal coordinates use a
 * 208/50 sample scale (40.0 us). Each logical Y row is repeated on four NTSC
 * or five PAL video lines (224 / 280 lines per field, ~93% / ~98% of the
 * active picture height); repetition reuses the same SRAM row, so the taller
 * menu costs no UI memory.
 *
 * The UI is centred in the active picture on both axes. Horizontal centring
 * lengthens the shared sync prefix to the UI start. Monochrome, no burst
 * (2026-11-14): every line has the same prefix, so a line's blank tail and
 * the next line's prefix are one DMA node: a UI line is UI row + (tail +
 * next prefix), a blank line one node.
 * NTSC active video is 9.4..62.06 us after 0H, PAL 10.35..62.35 us.
 */
#define MENU_FONT_HEIGHT 8u
#define MENU_UI_WIDTH 384u
#define MENU_UI_LINES 56u
#define MENU_UI_X_SCALE_NUM 208u
#define MENU_UI_X_SCALE_DEN 50u
#define MENU_UI_BYTES 1600u
#define MENU_NTSC_UI_Y_REPEAT 4u
#define MENU_PAL_UI_Y_REPEAT 5u
/* First UI line as a field-line index of menu_raster_emit (NTSC index k is
 * field line k+1, PAL index k is field line k-1). Active picture: NTSC
 * k=21..261, PAL k=25..311 in both fields. */
#define MENU_NTSC_UI_FIRST_LINE 29u
#define MENU_PAL_UI_FIRST_LINE 28u

/* Line start (sync leading edge) to UI start; DMA word aligned. */
#define MENU_NTSC_PREFIX_BYTES 628u
#define MENU_PAL_PREFIX_BYTES 652u
#define MENU_PREFIX_BYTES MENU_PAL_PREFIX_BYTES
/* Longest line remainder after the shortest prefix (NTSC lines are <= 2544). */
#define MENU_TAIL_BYTES (2560u - MENU_NTSC_PREFIX_BYTES)
/* Two loop forms:
 * - interlaced (default): one complete BT.470 two-field frame; with the
 *   shared tail+prefix nodes NTSC needs ~1000 descriptors (~12 KiB; with
 *   per-line burst prefixes it was 1499 / 18 KiB and never fitted at boot);
 * - progressive fallback: one field repeated (NTSC 262 / PAL 312 lines, the
 *   "240p/288p" form), 748 / 905 descriptors, used only when the interlaced
 *   chain cannot be allocated. The operator's display showed the progressive
 *   menu briefly, then inverted and went black (board 2026-10-18), so it is
 *   never the first choice. Earlier: eight fields ~76 KiB. */
#define MENU_NTSC_FIELD_HALVES 525u
#define MENU_PAL_FIELD_HALVES 625u
#define MENU_MAX_NODES 1840u

typedef enum { VIDEO_STD_NTSC, VIDEO_STD_PAL } video_standard_t;

typedef struct {
    /* Blank level (a line tail, a blank half-line), then the line prefix
     * (H sync at its start) at offset MENU_TAIL_BYTES: tail + next prefix
     * is one contiguous node. */
    uint8_t tail_prefix[MENU_TAIL_BYTES + MENU_PREFIX_BYTES];
    uint8_t equalizing[1280];
    uint8_t broad[1280];
    uint8_t ui[MENU_UI_LINES][MENU_UI_BYTES];
} menu_raster_t;

typedef bool (*menu_segment_fn)(void *ctx, const uint8_t *data, unsigned length);
void menu_raster_init(menu_raster_t *raster, video_standard_t standard, bool interlaced);
bool menu_raster_emit(const menu_raster_t *raster, video_standard_t standard, bool interlaced,
                      menu_segment_fn emit, void *ctx);
uint32_t menu_half_sample(video_standard_t standard, unsigned half);

/* Half-lines per field: the interlaced field has the odd BT.470 count, the
 * progressive field drops one half-line so every loop starts on the H grid. */
static inline unsigned menu_field_halves(video_standard_t standard, bool interlaced)
{
    return (standard == VIDEO_STD_PAL ? MENU_PAL_FIELD_HALVES : MENU_NTSC_FIELD_HALVES) - (interlaced ? 0u : 1u);
}

/* Half-lines of one cyclic loop. */
static inline unsigned menu_loop_halves(video_standard_t standard, bool interlaced)
{
    return menu_field_halves(standard, interlaced) * (interlaced ? 2u : 1u);
}

static inline unsigned menu_prefix_bytes(video_standard_t standard)
{
    return standard == VIDEO_STD_PAL ? MENU_PAL_PREFIX_BYTES : MENU_NTSC_PREFIX_BYTES;
}

static inline unsigned menu_ui_y_repeat(video_standard_t standard)
{
    return standard == VIDEO_STD_PAL ? MENU_PAL_UI_Y_REPEAT : MENU_NTSC_UI_Y_REPEAT;
}

static inline unsigned menu_ui_first_line(video_standard_t standard)
{
    return standard == VIDEO_STD_PAL ? MENU_PAL_UI_FIRST_LINE : MENU_NTSC_UI_FIRST_LINE;
}
