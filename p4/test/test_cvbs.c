/* C5VRX P4 display by Twotoz and the C5VRX contributors: host check of
 * p4/main/cvbs.c on a synthetic composite signal (PAL and NTSC with broad
 * and equalising pulses, half-line fields, a row gradient and a 5 us white
 * marker; clock error and noise). Checks lock, standard, field parity and
 * row order, levels and the horizontal position. Run from p4/test:
 *   cc -O2 -std=c11 -Istub -I../main test_cvbs.c -lm -o test_cvbs && ./test_cvbs
 * The stub/ headers stand in for ESP-IDF; nothing of the capture runs. */
#include <stdio.h>
#include <math.h>
#include "stub/common.h"
int64_t g_fake_time;
axi_dma_dev_t AXI_DMA;
#include "cvbs.c"

enum { HN, HEQ, HBR, HBL, HVID };   /* half-line kinds: none(blank), eq, broad, blank no pulse, video */
static bool g_pal;
static bool g_colour;        /* colour bars instead of the grey gradient */
/* Colour bars: four 13 us segments: grey, +V (red), +U (blue), -U-V (green). */
static const double g_bar_u[4] = {0, 0, 0.2, -0.15}, g_bar_v[4] = {0, 0.2, 0, -0.15};
static double g_H;           /* line, samples */
static int g_lines;
/* kinds of the two halves of line ln (1-based) */
static void kinds(int ln, int *a, int *b)
{
    *a = HN; *b = HN;
    if (g_pal) {
        if (ln <= 2) { *a = HBR; *b = HBR; }
        else if (ln == 3) { *a = HBR; *b = HEQ; }
        else if (ln <= 5) { *a = HEQ; *b = HEQ; }
        else if (ln <= 22) { *a = HN; *b = HN; }
        else if (ln <= 310) { *a = HVID; *b = HVID; }
        else if (ln <= 312) { *a = HEQ; *b = HEQ; }
        else if (ln == 313) { *a = HEQ; *b = HBR; }
        else if (ln <= 315) { *a = HBR; *b = HBR; }
        else if (ln <= 317) { *a = HEQ; *b = HEQ; }
        else if (ln == 318) { *a = HEQ; *b = HBL; }
        else if (ln <= 335) { *a = HN; *b = HN; }
        else if (ln <= 622) { *a = HVID; *b = HVID; }
        else if (ln == 623) { *a = HVID; *b = HEQ; }
        else { *a = HEQ; *b = HEQ; }
    } else {
        if (ln <= 3) { *a = HEQ; *b = HEQ; }
        else if (ln <= 6) { *a = HBR; *b = HBR; }
        else if (ln <= 9) { *a = HEQ; *b = HEQ; }
        else if (ln <= 20) { *a = HN; *b = HN; }
        else if (ln <= 262) { *a = HVID; *b = HVID; }
        else if (ln == 263) { *a = HVID; *b = HEQ; }
        else if (ln <= 265) { *a = HEQ; *b = HEQ; }
        else if (ln == 266) { *a = HEQ; *b = HBR; }
        else if (ln <= 268) { *a = HBR; *b = HBR; }
        else if (ln == 269) { *a = HBR; *b = HEQ; }
        else if (ln <= 271) { *a = HEQ; *b = HEQ; }
        else if (ln == 272) { *a = HEQ; *b = HBL; }
        else if (ln <= 282) { *a = HN; *b = HN; }
        else { *a = HVID; *b = HVID; }
    }
}
/* expected frame row of an active line, -1 if none */
static int frame_row(int ln)
{
    if (g_pal) {
        if (ln >= 23 && ln <= 310) return 2 * (ln - 23);
        if (ln >= 336 && ln <= 622) return 2 * (ln - 336) + 1;
    } else {
        if (ln >= 283 && ln <= 524) return 2 * (ln - 283);
        if (ln >= 21 && ln <= 262) return 2 * (ln - 21) + 1;
    }
    return -1;
}
static int rows_total(void) { return g_pal ? 576 : 480; }

static uint8_t level(int64_t n, double noise_amp, unsigned *rng)
{
    const double us = n / 20.0;
    const double Hus = g_H / 20.0;
    const double tf = fmod(us, Hus * g_lines);
    const int li = (int)(tf / Hus);
    const double pos = tf - li * Hus;
    int a, b;
    kinds(li + 1, &a, &b);
    const int half = pos >= Hus / 2;
    const int k = half ? b : a;
    const double hp = half ? pos - Hus / 2 : pos;
    double v = 20, chroma_u = 0, chroma_v = 0;
    switch (k) {
    case HEQ: v = hp < 2.35 ? 0 : 20; break;
    case HBR: v = hp < Hus / 2 - 4.7 ? 0 : 20; break;
    case HBL: v = 20; break;
    case HN: v = (!half && hp < 4.7) ? 0 : 20; break;
    case HVID: {
        if (!half && hp < 4.7) { v = 0; break; }
        const double act0 = g_pal ? 10.5 : 9.4, act1 = act0 + (g_pal ? 52.0 : 52.6);
        if (pos < act0 || pos >= act1) { v = 20; break; }
        const int r = frame_row(li + 1);
        if (g_colour) {
            const int seg = (int)((pos - act0) / 13.0);
            const double white = 20.0 * (g_pal ? 2.333 : 2.5);
            v = 20 + 0.4 * white;
            chroma_u = g_bar_u[seg > 3 ? 3 : seg] * white;
            chroma_v = g_bar_v[seg > 3 ? 3 : seg] * white;
        } else if (pos < act0 + 5.0) v = 63;                           /* white marker, 5 us */
        else v = 20 + 43.0 * (r < 0 ? 0 : r) / (rows_total() - 1);    /* row gradient */
        break;
    }
    }
    if (g_colour && !half && (k == HVID || k == HN) && hp > (g_pal ? 5.6 : 5.3) && hp < (g_pal ? 7.85 : 7.8)) {
        /* burst: half the sync depth; PAL (-U +- V)/sqrt2, NTSC -U */
        if (g_pal) { chroma_u = -10.0 / sqrt(2.0); chroma_v = 10.0 / sqrt(2.0); }
        else { chroma_u = -10.0; chroma_v = 0; }
    }
    if (chroma_u != 0 || chroma_v != 0) {
        const double w = 2 * M_PI * (g_pal ? 4.43361875 : 3.57954545) * us;   /* continuous carrier */
        const int vsw = g_pal && (li & 1) ? -1 : 1;                           /* PAL V switch */
        v += chroma_u * sin(w) + vsw * chroma_v * cos(w);
    }
    if (noise_amp > 0) { *rng = *rng * 1103515245u + 12345u; v += noise_amp * (((*rng >> 16) & 0x7fff) / 16384.0 - 1.0); }
    int c = (int)lrint(v);
    return (uint8_t)(c < 0 ? 0 : c > 63 ? 63 : c);
}

static int run(bool pal, double noise, double clock_ppm, bool colour)
{
    g_colour = colour;
    memset(&s_cl, 0, sizeof(s_cl));
    memset(s_hist, 0, sizeof(s_hist));
    s_hw = 0; s_search = 0; s_locked = false; s_line_in_field = 0; s_parity = 0; s_vs_missed = VS_REACQUIRE; s_seq = 0; s_frame_fields = 0;
    s_period = pal ? PERIOD_PAL : PERIOD_NTSC; s_frame_pal = true; s_ready_idx = -1; s_held_idx = -1; s_w_idx = 0;
    memset(&s_cnt, 0, sizeof(s_cnt));
    for (int k = 0; k < 3; ++k) memset(s_frames[k], 0, CVBS_ROW_BYTES * CVBS_H_MAX);
    g_pal = pal; g_lines = pal ? 625 : 525;
    g_H = (pal ? 1280.0 : 1271.1111) * (1.0 + clock_ppm * 1e-6);
    unsigned rng = 1;
    const int64_t total = (int64_t)(g_H * g_lines * 6);   /* six frames */
    int64_t n = 12345;   /* start mid-frame; 20 MS/s codes, summed in pairs as fill_history() */
    while (n < total) {
        const int64_t need = (s_locked ? (s_next >> 16) : s_search) - 32;
        int64_t room = HIST - (s_hw - need) - 32;
        if (room > 4096) room = 4096;
        for (int64_t i = 0; i < room; ++i, n += 2) {
            const int a = level(n, noise, &rng), b = level(n + 1, noise, &rng);
            s_raw[(2 * s_hw) & RM] = (uint8_t)a;
            s_raw[(2 * s_hw + 1) & RM] = (uint8_t)b;
            s_hist[(s_hw++) & HM] = (uint8_t)(a + b);
        }
        decode_lines();
    }
    cvbs_frame_t f;
    if (!cvbs_acquire(&f)) { printf("%s: no frame (fields=%u lines=%u hsync=%u locked=%d)\n", pal ? "PAL" : "NTSC", s_cnt.fields, s_cnt.lines, s_cnt.hsync, s_locked); return 1; }
    int bad = 0, worst = 0;
    const double gain = 255.0 / (s_blank + (s_blank - s_sync) * (pal ? 2.333 : 2.5) - s_blank);
#define YPX(r, x) (f.yuv[(size_t)(r) * CVBS_ROW_BYTES + 3 * ((x) / 2) + ((x) & 1 ? YUV_Y1 : YUV_Y0)])
#define CPX(r, x) (f.yuv[(size_t)(r) * CVBS_ROW_BYTES + 3 * ((x) / 2) + YUV_C])
    if (colour) {
        /* Bars: Cb (even rows) / Cr (odd rows) at each segment's centre,
         * averaged over 40 rows in the middle of the picture. */
        const double ku = 291.9, kv = 207.4;
        static const int px_pal[4] = {61, 191, 321, 451}, px_ntsc[4] = {58, 188, 318, 448};
        for (int seg = 0; seg < 4; ++seg) {
            const int x = pal ? px_pal[seg] : px_ntsc[seg];
            double cb = 0, cr = 0;
            for (int r = 200; r < 240; r += 2) { cb += CPX(r, x); cr += CPX(r + 1, x); }
            cb /= 20; cr /= 20;
            const double wcb = 128 + ku * g_bar_u[seg], wcr = 128 + kv * g_bar_v[seg];
            const int ok = fabs(cb - wcb) < 18 && fabs(cr - wcr) < 18;
            printf("  %s bar %d: Cb %.0f (want %.0f) Cr %.0f (want %.0f) %s\n", pal ? "PAL" : "NTSC", seg, cb, wcb, cr,
                   wcr, ok ? "ok" : "BAD");
            bad += !ok;
        }
        printf("%s colour noise=%.1f: colour_lines=%u killed=%u vote=%d %s\n", pal ? "PAL " : "NTSC", noise,
               s_cl.colour_lines, s_cl.killed_lines, s_cl.vote, bad ? "FAIL" : "ok");
        return bad != 0;
    }
    /* PAL's last row is half-line 623, which this generator leaves blank. */
    for (int r = 0; r < f.height - (pal ? 1 : 0); ++r) {
        if (20 + 43.0 * r / (f.height - 1) + noise > 63) break;   /* clipped by the noise: mean is low */
        const int want = (int)lrint(43.0 * r / (f.height - 1) * gain);
        int sum = 0;
        for (int x = CVBS_W / 4; x < CVBS_W * 7 / 8; ++x) sum += YPX(r, x);
        const int cnt = CVBS_W * 7 / 8 - CVBS_W / 4, got = (sum + cnt / 2) / cnt;
        const int d = abs(got - (want > 255 ? 255 : want));
        if (d > worst) worst = d;
        if (d > 5 + (int)noise) {   /* fixed-point levels (luma.S): ~1 % gain, truncating shift */ if (bad < 5) printf("  row %d: got %d want %d\n", r, got, want); ++bad; }
    }
    /* marker: 5 us white = 50 samples at 10 MS/s, less the 0.4/0.7 us trim */
    int mark = 0;
    for (int x = 0; x < 120; ++x) mark += YPX(300, x) > 200;
    printf("%s noise=%.1f ppm=%.0f: pal=%d height=%d frames=%u fields=%u hsync/lines=%u/%u sync=%.2f blank=%.2f "
           "line_us=%.4f bad_rows=%d worst=%d marker_px=%d\n",
           pal ? "PAL " : "NTSC", noise, clock_ppm, f.pal, f.height, s_cnt.frames, s_cnt.fields, s_cnt.hsync, s_cnt.lines,
           s_sync, s_blank, (double)s_period / Q16 / 10.0, bad, worst, mark);
    return bad > 0 || f.pal != pal || mark < (pal ? 42 : 39) || mark > (pal ? 50 : 47) + (int)noise / 2;
}

int main(void)
{
    for (int k = 0; k < 3; ++k) s_frames[k] = calloc(1, CVBS_ROW_BYTES * CVBS_H_MAX);
    set_slice();
    build_lut();
    chroma_init();
    s_saturation = 100;   /* the bars' expected values are at nominal saturation */
    int fail = 0;
    fail |= run(true, 0, 0, false);
    fail |= run(false, 0, 0, false);
    fail |= run(true, 0, 300, false);
    fail |= run(false, 0, -300, false);
    fail |= run(true, 6, 0, false);
    fail |= run(false, 6, 0, false);
    fail |= run(true, 12, 0, false);
    fail |= run(false, 12, 100, false);
    fail |= run(true, 0, 0, true);
    fail |= run(false, 0, 0, true);
    fail |= run(true, 4, 50, true);
    fail |= run(false, 4, -50, true);
    /* Levels learnt on a bad signal, far above this one's: it must still lock. */
    s_sync = 25.0f; s_blank = 31.0f; set_slice(); build_lut();
    fail |= run(true, 0, 0, false);
    s_sync = 25.0f; s_blank = 31.0f; set_slice(); build_lut();
    fail |= run(false, 6, 0, false);
    printf(fail ? "FAIL\n" : "PASS\n");
    return fail;
}
