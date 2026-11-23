/* C5VRX by Twotoz/contributors: actual DMA raster timing at both standards. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "menu_raster.h"
static menu_raster_t raster;
static uint8_t *s_wave;
static unsigned s_wave_at;
static bool capture(void *ctx, const uint8_t *data, unsigned length)
{
    (void)ctx; memcpy(s_wave + s_wave_at, data, length); s_wave_at += length; return true;
}
/* Waveform check of one cyclic loop: every sync leading edge sits on the
 * half-line grid, consecutive edges are a half or a full line apart
 * (including across the loop wrap), and the loop holds one field's broad
 * pulses (one field progressive, two fields interlaced). */
static void check_wave(video_standard_t v, bool interlaced, unsigned halves)
{
    const unsigned total = menu_half_sample(v, halves);
    s_wave = malloc(total); assert(s_wave); s_wave_at = 0;
    assert(menu_raster_emit(&raster, v, interlaced, capture, NULL) && s_wave_at == total);
    unsigned edges = 0, broad = 0, first = 0, last = 0;
    for (unsigned n = 0; n < total; ++n) {
        if (s_wave[n] || !s_wave[(n + total - 1u) % total]) continue;
        unsigned run = 0;
        while (run < total && !s_wave[(n + run) % total]) ++run;
        broad += run > 800u;   /* > 20 us */
        unsigned half = 0;
        while (half <= halves && menu_half_sample(v, half) < n) ++half;
        assert(menu_half_sample(v, half) == n); /* on the half-line grid */
        if (edges) {
            unsigned d = n - last;
            assert(d == menu_half_sample(v, half) - menu_half_sample(v, half - 1u) ||
                   (half >= 2u && d == menu_half_sample(v, half) - menu_half_sample(v, half - 2u)));
        } else first = n;
        last = n; ++edges;
    }
    unsigned wrap = total - last + first, line = v == VIDEO_STD_PAL ? 2560u : 2540u;
    assert(wrap <= line + 4u && wrap + 4u >= line / 2u);
    assert(broad == (v == VIDEO_STD_PAL ? 5u : 6u) * (interlaced ? 2u : 1u));
    free(s_wave);
}
typedef struct { uint64_t samples; unsigned nodes, sync, burst; } stats_t;
static bool segment(void *ctx, const uint8_t *data, unsigned length)
{
    stats_t *s = ctx;
    assert(length && length <= 4092 && !(length & 3) && !((uintptr_t)data & 3));
    s->samples += length; ++s->nodes;
    for (unsigned k = 0; k < length; ++k) {
        assert(data[k] <= 63);
        s->sync += data[k] == 0;
        s->burst += data[k] >= 25 && data[k] <= 28;
    }
    return true;
}
int main(void)
{
    for (unsigned mode = 0; mode < 2; ++mode) {
        const bool interlaced = mode == 0;
        for (unsigned standard = 0; standard < 2; ++standard) {
            video_standard_t v = (video_standard_t)standard;
            menu_raster_init(&raster, v, interlaced);
            memset(raster.ui, 20, sizeof(raster.ui));
            stats_t s = {0};
            assert(menu_raster_emit(&raster, v, interlaced, segment, &s));
            unsigned halves = menu_loop_halves(v, interlaced);
            assert(s.samples == menu_half_sample(v, halves));
            assert(s.nodes && s.nodes <= MENU_MAX_NODES && s.sync > 1000 && s.burst == 0);   /* monochrome */
            if (interlaced) assert(s.nodes <= (v == VIDEO_STD_PAL ? 1250u : 1060u));   /* fits at boot (was 1811 / 1499) */
            assert(halves == (v == VIDEO_STD_PAL ? (interlaced ? 1250u : 624u) : (interlaced ? 1050u : 524u)));
            if (!interlaced) assert(s.nodes <= (v == VIDEO_STD_PAL ? 940u : 780u));
            check_wave(v, interlaced, halves);
            printf("%s %s nodes=%u descriptor_bytes=%u\n", v == VIDEO_STD_PAL ? "PAL" : "NTSC",
                   interlaced ? "interlaced" : "progressive", s.nodes, s.nodes * 12u);
            for (unsigned h = 1; h <= halves; ++h) {
                unsigned length = menu_half_sample(v, h) - menu_half_sample(v, h - 1);
                assert(v == VIDEO_STD_PAL ? length == 1280 : length == 1268 || length == 1272);
            }
        }
    }
    puts("PASS: PAL/NTSC interlaced and progressive menu rasters, half-line sync grid across the wrap, field broad pulses, sync, no burst, node budget, DAC bounds and DMA alignment");
    return 0;
}
