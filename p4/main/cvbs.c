/* C5VRX P4 display by Twotoz and the C5VRX contributors: capture and
 * composite decode of the C5's DAC codes (docs/P4LINK.md).
 *
 * Bus: the C5's PARLIO TX sends its U80P64-R1 (or U85Q) output, six-bit
 * codes at 40 MHz with every code twice ([D,D], unique CVBS at 20 MS/s), and
 * its clock. PARLIO RX here samples it on that clock into a ring in internal
 * RAM.
 *
 * The continuous receive follows zerowidth's p4usb decoder (fork
 * zerowidth/C5VRX, branch p4usb, commit 3dfac08, p4usb/main/decode.c): whole
 * 4032-byte DMA nodes, partial receive with a soft-delimiter EOF every eight
 * nodes, and the DMA's write position read from the AXI-GDMA descriptor
 * register one node behind it, since the register names the next node as
 * soon as the current one starts. Rewritten here for an 8-bit code bus.
 *
 * Decode at 10 MS/s: each history sample is the sum of two successive codes
 * (0..126; bytes 0 and 2 of every 32-bit ring word), one store per word and
 * a free low-pass. On the board the decoder at 20 MS/s took all of a core
 * for half the samples (2026-10-31): a P4 load costs ~3-5 cycles (p4usb
 * found the same). Code 0 = sync tip, about 20 = blanking: sync slicing on
 * 4-sample sums at the midpoint of the measured sync and blanking levels, a
 * line clock that follows each valid hsync and coasts through the vertical
 * interval and missing pulses, broad pulses for the field start, their
 * position in the line for the field parity. Each active line goes into the
 * frame at source resolution, shifted to its sub-sample sync phase, through
 * the level table as RGB565: 512 samples, the 52 us (PAL) / 52.6 us (NTSC)
 * active line less 0.4 / 0.7 us each side, so the PPA scales it to 800
 * columns by exactly 25/16. No colour. */
#include "cvbs.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "board.h"
#include "driver/gpio.h"
#include "driver/parlio_rx.h"
#include "esp_cache.h"
#include "esp_private/esp_cache_private.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/dma_types.h"
#include "soc/axi_dma_struct.h"
#include "soc/gdma_channel.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "cvbs";

/* ---- capture ---- */
#define NODE_BYTES      4032
#define RING_NODES      24   /* 32 no longer fitted one internal DMA block next to the colour history */
#define RING_BYTES      (RING_NODES * NODE_BYTES)        /* 2.4 ms at 40 MB/s */
#define EOF_BYTES       (8 * NODE_BYTES)
#define RING_US         ((int64_t)RING_BYTES * 1000000 / LINK_CLK_HZ)

static parlio_rx_unit_handle_t s_rx;
static parlio_rx_delimiter_handle_t s_delim;
static uint8_t *s_ring;
static int s_dma_ch = -1;
static uint32_t s_node_desc[RING_NODES];   /* descriptor address of each node, by ring offset */
static uint32_t s_rd;                      /* ring byte offset read up to (a multiple of 4) */
static int64_t s_rd_us;
static size_t s_cache_align = 64;
static TaskHandle_t s_task;        /* decode task */
static volatile bool s_edge_neg, s_restart;

/* ---- decode, 10 MS/s ---- */
#define FS_HZ           10000000
#define HIST            32768                   /* samples, 3.3 ms */
#define HM              (HIST - 1)
#define Q16             65536
#define WIN             15                      /* hsync search, +-1.5 us */
#define PERIOD_PAL      (640 * Q16)
#define PERIOD_NTSC     ((int64_t)(635.556 * Q16))
#define PERIOD_MIN      (625 * Q16)
#define PERIOD_MAX      (655 * Q16)
#define MISS_UNLOCK     40
#define LINE_SAMPLES    660                     /* a line and the search window */

static uint8_t s_hist[HIST] __attribute__((aligned(16)));
/* The same samples' two codes at 20 MS/s (raw index 2n and 2n+1 belong to
 * history sample n), for the colour subcarrier: at 10 MS/s the demodulated
 * chroma's 2 fsc image would alias down to about 1 MHz. */
#define RAW             (2 * HIST)
#define RM              (RAW - 1)
static uint8_t s_raw[RAW] __attribute__((aligned(16)));
/* The history is single-producer (fill task, core 0) / single-consumer
 * (decode task, core 1): the write count and the oldest sample the decoder
 * still needs cross cores under s_hw_lock (64-bit values on a 32-bit CPU). */
static int64_t s_hw;                /* samples written to s_hist (fill task) */
static int64_t s_need;              /* oldest sample the decoder still reads */
static volatile bool s_resync;      /* fill -> decode: data was lost, search again */
static portMUX_TYPE s_hw_lock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_fill_task;
static int64_t s_next;              /* expected next hsync, Q16 */
static int64_t s_period = PERIOD_PAL;
static int64_t s_search;            /* unlocked: next sample to search from */
static uint32_t s_search_fail;      /* unlocked: failed searches (seed_levels) */
static bool s_locked;
static int s_miss;
static int s_line_in_field = 0, s_parity;
static int s_same_parity;
static uint32_t s_coasts, s_late_vs;           /* TEMP: fields coasted without a vsync */
#define VS_WINDOW       6
#define VS_REACQUIRE    3
static int s_vs_missed;              /* fields in a row without a vertical sync in the window */
static bool s_w_prog, s_frame_prog;
static uint32_t s_flog[32], s_flog_n, s_rows_field, s_pub_flag;   /* TEMP field log */
static uint32_t s_black_field, s_bflog[32], s_hsync_field;   /* the frame being written / the newest is progressive */
           /* fields in a row with the same parity: progressive */
static float s_sync = 0.0f, s_blank = 20.0f;   /* code units (a sample is two codes) */
static int s_thr, s_thr4;           /* slice level in sample units, and x4 for 4-sums */
static uint8_t s_lut[128];          /* sample (two codes) -> Y */
/* The same mapping for cvbs_luma16 (luma.S): offset, w0, w1, w2, 0x80s;
 * [1 2 1] on colour lines (kf), none (ku). */
static int8_t s_kf[5][16] __attribute__((aligned(16))), s_ku[5][16] __attribute__((aligned(16)));
static int s_kshift, s_kshift_u;

#ifdef __riscv
void cvbs_luma16(const int8_t *x, uint8_t *y, int n, const int8_t *k, int shift);
#else
static void cvbs_luma16(const int8_t *x, uint8_t *y, int n, const int8_t *k, int shift)
{
    for (int i = 0; i < 16 * n; ++i) {
        const int8_t o = k[0];
        const int a = (int8_t)(x[i - 1] - o), b = (int8_t)(x[i] - o), c = (int8_t)(x[i + 1] - o);
        int v = (k[16] * a + k[32] * b + k[48] * c) >> shift;
        v = v < -128 ? -128 : v > 127 ? 127 : v;
        y[i] = (uint8_t)(v ^ 0x80);
    }
}
#endif
static int s_contrast = 100, s_brightness = 0;
static bool s_swap;
static volatile bool s_lut_dirty = true;

/* ---- frames: the decoder writes one, one is the newest, the display holds one ---- */
static uint8_t *s_frames[3];
static int s_w_idx, s_ready_idx = -1, s_held_idx = -1;
static uint32_t s_seq;
static bool s_frame_pal = true;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static struct { uint32_t lines, hsync, fields, frames; } s_cnt, s_rate;
static uint32_t s_overruns, s_restarts;
static int64_t s_rate_us, s_last_data_us, s_busy_us, s_busy_rate_us;

/* History reads on 32-bit positions: the ring index only needs the low bits. */
static inline int XU(uint32_t n) { return s_hist[n & HM]; }
static inline int S4U(uint32_t n) { return XU(n) + XU(n + 1) + XU(n + 2) + XU(n + 3); }

static IRAM_ATTR int64_t hw_get(void)
{
    portENTER_CRITICAL(&s_hw_lock);
    const int64_t v = s_hw;
    portEXIT_CRITICAL(&s_hw_lock);
    return v;
}

/* n history samples from position from into a linear buffer. */
static IRAM_ATTR void hist_copy(uint8_t *dst, int64_t from, int n)
{
    const uint32_t a = (uint32_t)from & HM;
    const int first = (int)(HIST - a) < n ? (int)(HIST - a) : n;
    memcpy(dst, s_hist + a, (size_t)first);
    if (n > first) memcpy(dst + first, s_hist, (size_t)(n - first));
}

/* ------------------------------------------------------------------ */
/* capture                                                             */
/* ------------------------------------------------------------------ */

/* Only a wake-up; the decoder reads the DMA position from the hardware. */
static bool IRAM_ATTR on_partial(parlio_rx_unit_handle_t rx, const parlio_rx_event_data_t *e, void *arg)
{
    BaseType_t woken = pdFALSE;
    if (s_fill_task) vTaskNotifyGiveFromISR(s_fill_task, &woken);
    return woken == pdTRUE;
}

static int find_dma_channel(void)
{
    for (int ch = 0; ch < 3; ++ch)
        if (AXI_DMA.in[ch].conf.in_peri_sel.peri_in_sel_chn == SOC_GDMA_TRIG_PERIPH_PARLIO0) return ch;
    return -1;
}

/* The driver's descriptor of each ring node, by walking its circular list. */
static void map_descriptors(void)
{
    memset(s_node_desc, 0, sizeof(s_node_desc));
    uint32_t d = AXI_DMA.in[s_dma_ch].conf.in_dscr_bf0.val;
    for (int k = 0; k < 2 * RING_NODES && d; ++k) {
        const dma_descriptor_align8_t *u = (const dma_descriptor_align8_t *)d;
        const uint32_t off = (uint32_t)u->buffer - (uint32_t)s_ring;
        if (off < RING_BYTES && off % NODE_BYTES == 0) s_node_desc[off / NODE_BYTES] = d;
        d = (uint32_t)u->next;
    }
    unsigned found = 0;
    for (int k = 0; k < RING_NODES; ++k) found += s_node_desc[k] != 0;
    if (found != RING_NODES) ESP_LOGW(TAG, "descriptor walk found %u of %u nodes", found, RING_NODES);
}

/* Ring offset up to which the DMA's data is in memory: the start of the node
 * before the one the register names. */
static uint32_t dma_done(void)
{
    const uint32_t d = AXI_DMA.in[s_dma_ch].conf.in_dscr_bf0.val;
    for (int k = 0; k < RING_NODES; ++k)
        if (s_node_desc[k] == d) return (uint32_t)((k + RING_NODES - 1) % RING_NODES) * NODE_BYTES;
    return s_rd;
}

static void no_putc(char c) { (void)c; }

/* The driver's node interrupt reports the first node of a receive with an
 * early log straight to the UART, holding interrupts off for milliseconds;
 * the ROM printer is muted around a start (as in p4usb). */
static void mute_rom_log(bool mute)
{
    if (mute) esp_rom_install_channel_putc(1, no_putc);
    else esp_rom_install_uart_printf();
}

static esp_err_t rx_begin(void)
{
    const parlio_rx_soft_delimiter_config_t dcfg = {
        .sample_edge = s_edge_neg ? PARLIO_SAMPLE_EDGE_NEG : PARLIO_SAMPLE_EDGE_POS,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
        .eof_data_len = EOF_BYTES,
        .timeout_ticks = 0,
    };
    esp_err_t err = parlio_new_rx_soft_delimiter(&dcfg, &s_delim);
    if (err != ESP_OK) return err;
    mute_rom_log(true);
    err = parlio_rx_unit_enable(s_rx, true);
    const parlio_receive_config_t rcfg = {.delimiter = s_delim, .flags.partial_rx_en = 1};
    if (err == ESP_OK) err = parlio_rx_unit_receive(s_rx, s_ring, RING_BYTES, &rcfg);
    if (err == ESP_OK && s_dma_ch < 0 && (s_dma_ch = find_dma_channel()) < 0) err = ESP_ERR_NOT_FOUND;
    const uint32_t stale = err == ESP_OK ? AXI_DMA.in[s_dma_ch].conf.in_dscr_bf0.val : 0;
    if (err == ESP_OK) err = parlio_rx_soft_delimiter_start_stop(s_rx, s_delim, true);
    if (err == ESP_OK) {
        /* Until a node of the new receive is done the register still names
         * one of the last receive (with no bus clock it stays there). */
        const int64_t t0 = esp_timer_get_time();
        while (AXI_DMA.in[s_dma_ch].conf.in_dscr_bf0.val == stale && esp_timer_get_time() - t0 < 2000) {}
        map_descriptors();
        s_rd = dma_done();
        s_rd_us = esp_timer_get_time();
    }
    vTaskDelay(1);
    mute_rom_log(false);
    return err;
}

static void rx_end(void)
{
    (void)parlio_rx_soft_delimiter_start_stop(s_rx, s_delim, false);
    (void)parlio_rx_unit_disable(s_rx);
    (void)parlio_del_rx_delimiter(s_delim);
    s_delim = NULL;
}

static esp_err_t rx_create(void)
{
    parlio_rx_unit_config_t cfg = {
        .trans_queue_depth = 1,
        .max_recv_size = RING_BYTES,
        .data_width = 8,
        .clk_src = PARLIO_CLK_SRC_EXTERNAL,
        .ext_clk_freq_hz = LINK_CLK_HZ,
        .exp_clk_freq_hz = LINK_CLK_HZ,
        .clk_in_gpio_num = LINK_CLK_GPIO,
        .clk_out_gpio_num = -1,
        .valid_gpio_num = -1,
        .flags.free_clk = 1,
    };
    const int pins[LINK_DATA_BITS] = LINK_DATA_GPIOS;
    for (int k = 0; k < 8; ++k) cfg.data_gpio_nums[k] = k < LINK_DATA_BITS ? pins[k] : -1;
    esp_err_t err = parlio_new_rx_unit(&cfg, &s_rx);
    if (err != ESP_OK) return err;
    const parlio_rx_event_callbacks_t cbs = {.on_partial_receive = on_partial};
    return parlio_rx_unit_register_event_callbacks(s_rx, &cbs, NULL);
}

/* ------------------------------------------------------------------ */
/* decode                                                              */
/* ------------------------------------------------------------------ */

static IRAM_ATTR void build_lut(void)
{
    /* Black at blanking; white from the sync depth by the standard's ratio
     * (PAL 700/300 mV, NTSC 100/40 IRE). Index: a sample, two codes. */
    const float depth = s_blank - s_sync > 4.0f ? s_blank - s_sync : 4.0f;
    const float white = s_blank + depth * (s_frame_pal ? 2.333f : 2.5f);
    const float gain = 255.0f / (white - s_blank) * s_contrast / 100.0f;
    for (int v = 0; v < 128; ++v) {
        const float g = (v / 2.0f - s_blank) * gain + s_brightness;
        s_lut[v] = g < 0.0f ? 0 : g > 255.0f ? 255 : (uint8_t)(g + 0.5f);
    }
    /* y - 128 = 4 w (x - o) >> shift with x in samples (two codes):
     * 4 w / 2^shift = G = gain / 2, o = 2 blank - (brightness - 128) / G. */
    const float G = gain * 0.5f;
    int sh = 0;
    while (sh < 12 && G * (float)(1 << (sh + 1)) / 4.0f <= 63.0f) ++sh;
    int w = (int)(G * (float)(1 << sh) / 4.0f + 0.5f);
    if (w < 1) w = 1;
    if (w > 63) w = 63;
    float of = 2.0f * s_blank - (s_brightness - 128.0f) / G;
    const int o = of < 0.0f ? 0 : of > 127.0f ? 127 : (int)(of + 0.5f);
    for (int i = 0; i < 16; ++i) {
        s_kf[0][i] = s_ku[0][i] = (int8_t)o;
        s_kf[1][i] = (int8_t)w; s_kf[2][i] = (int8_t)(2 * w); s_kf[3][i] = (int8_t)w;
        s_ku[1][i] = 0; s_ku[2][i] = (int8_t)(4 * w > 127 ? 127 : 4 * w); s_ku[3][i] = 0;
        s_kf[4][i] = s_ku[4][i] = (int8_t)0x80;
    }
    /* unfiltered: 4w may not fit s8; then (2w)(x) twice as (w0 = 0, w1 = 2w, w2 = 0) with shift - 1 */
    if (4 * w > 127) for (int i = 0; i < 16; ++i) s_ku[2][i] = (int8_t)(2 * w);
    s_kshift = sh;
    s_kshift_u = 4 * w > 127 ? sh - 1 : sh;
    s_lut_dirty = false;
}

/* A falling sync edge in [from, to] (samples) with a sync body behind it and
 * blanking after it: not an equalising pulse (2.35 us) or a broad pulse. */
static IRAM_ATTR bool find_hsync(int64_t from, int64_t to, int64_t *edge_q16)
{
    const int thr = s_thr4;
    const uint32_t base = (uint32_t)from, count = (uint32_t)(to - from) + 1u;
    int prev = S4U(base - 1u);
    for (uint32_t i = 0; i < count; ++i) {
        const uint32_t n = base + i;
        const int cur = S4U(n);
        if (prev >= thr && cur < thr) {
            int low = 0, high = 0;
            for (uint32_t k = 4; k < 40; k += 2) low += S4U(n + k) < thr;    /* 18 checks, 0.4-4 us */
            for (uint32_t k = 78; k < 91; k += 2) high += S4U(n + k) >= thr;  /* 7 checks, 7.8-9.3 us */
            if (low >= 15 && high >= 5) {
                const int frac = (int)(((int64_t)(prev - thr) << 16) / (prev - cur));
                *edge_q16 = ((from + (int64_t)i - 1) << 16) + frac;
                return true;
            }
        }
        prev = cur;
    }
    return false;
}

static IRAM_ATTR void set_slice(void)
{
    s_thr = (int)(s_sync + s_blank + 0.5f);   /* the midpoint, in samples of two codes */
    s_thr4 = 4 * s_thr;
}

static IRAM_ATTR void measure_levels(uint32_t l)
{
    int sync = 0, blank = 0;
    for (uint32_t k = 15; k < 35; ++k) sync += XU(l + k);     /* 1.5-3.5 us */
    for (uint32_t k = 80; k < 92; ++k) blank += XU(l + k);    /* 8-9.2 us, after the burst */
    s_sync += (sync / 40.0f - s_sync) * (1.0f / 32.0f);
    s_blank += (blank / 24.0f - s_blank) * (1.0f / 32.0f);
    if (s_blank - s_sync < 4.0f) s_sync = s_blank - 4.0f;
    set_slice();
}

/* Unlocked: the levels from the signal itself. measure_levels learns only on
 * accepted hsyncs, so after levels learnt on a bad signal (sync 25, blank 31)
 * a clean one (the C5 menu, sync 0, blank 20) lay wholly below the slice, no
 * hsync passed and the decoder stayed unlocked for good (board 2026-10-31).
 * Over four lines the sync tip is ~7 % of the samples and the porches another
 * ~11 % at blanking: the 3rd percentile is the sync tip, the 12th blanking (or
 * black near it). A vertical interval (mostly sync) gives no usable spread
 * and is skipped. */
#define SEED_SPAN 2560
static IRAM_ATTR void seed_levels(int64_t to)
{
    uint16_t h[128] = {0};
    for (int64_t n = to - SEED_SPAN; n < to; ++n) ++h[XU((uint32_t)n) & 127];
    int acc = 0, v = 0, sync = -1, blank = -1;
    for (; v < 128; ++v) {
        acc += h[v];
        if (sync < 0 && acc * 100 >= SEED_SPAN * 3) sync = v;
        if (acc * 100 >= SEED_SPAN * 12) { blank = v; break; }
    }
    if (sync < 0 || blank < 0 || blank - sync < 8) return;   /* samples: 4 code units */
    s_sync = sync / 2.0f;
    s_blank = blank / 2.0f;
    set_slice();
    s_lut_dirty = true;
}

static volatile uint32_t s_row_head, s_row_tail;  /* the row queue (below) */
static uint32_t s_pub_head[3];       /* row queue position a frame's last row reaches */
static IRAM_ATTR void publish_frame(void)
{
    portENTER_CRITICAL(&s_lock);
    s_ready_idx = s_w_idx;
    s_frame_prog = s_w_prog;
    s_pub_head[s_w_idx] = s_row_head;
    for (int k = 0; k < 3; ++k)
        if (k != s_ready_idx && k != s_held_idx) { s_w_idx = k; break; }
    ++s_seq;
    portEXIT_CRITICAL(&s_lock);
    ++s_cnt.frames;
}

/* ---- colour ----
 * Burst-referenced quadrature demodulation on the 20 MS/s codes, per line:
 * the burst and 64 windows across the active line (one per 8 pixels), each
 * correlated with zero-mean sin/cos tables over two subcarrier cycles, so
 * luma and the 2 fsc product cancel. A line's carrier phase comes from its
 * own burst; PAL's V switch from the burst's +-45 degree swing between
 * successive lines (a vote keeps the alternation), then the PAL delay line
 * (the mean of two lines' U and V) cancels phase errors. The burst amplitude
 * sets the colour gain; without a burst the line stays grey. With
 * C = sum x sin(wt) + j sum x cos(wt) and x = U sin(wt+p) + V cos(wt+p),
 * C = N/2 (U + jV) e^(jp); the PAL burst is (-U +- V)/sqrt2, NTSC -U. */
#define CW_N            64                      /* chroma windows a line */
#define CW_STEP         16                      /* raw samples between windows */
#define CW_MAX          11
#define CB_MAX          40
#define CB_BLOCKS       3                       /* the burst in 16-sample blocks */
#define TAB_AMP         63.0f                   /* int8 table amplitude */
typedef struct {
    /* Zero-mean sin, cos per 16-sample block, zero past the window: the
     * layout cvbs_corr16 reads (corr.S). */
    int8_t btab[CB_BLOCKS][2][16] __attribute__((aligned(16)));
    int8_t wtab[CW_N][2][16] __attribute__((aligned(16)));
    int burst_off, burst_len, win_len, act_off; /* raw samples from the hsync edge */
    float omega;                                /* subcarrier, radians per raw sample */
    int rot_base;                               /* e^(-j w d) for d = rot_base .. +63 */
    float rot_c[64], rot_s[64];
    float burst_ratio;                          /* burst / white amplitude */
} chroma_std_t;
static chroma_std_t s_cs[2] __attribute__((aligned(16)));   /* [0] NTSC, [1] PAL */
static uint8_t s_bbuf[16 * CB_BLOCKS] __attribute__((aligned(16)));
static uint8_t s_cbuf[16 * CW_N] __attribute__((aligned(16)));

/* The PIE routine on the P4 (corr.S), the same sums in C elsewhere (host test). */
#ifdef __riscv
void cvbs_corr16(const int8_t *x, const int8_t *tab, int32_t *out, int n);
#else
static void cvbs_corr16(const int8_t *x, const int8_t *tab, int32_t *out, int n)
{
    for (int k = 0; k < n; ++k, x += 16, tab += 32, out += 2) {
        int32_t a = 0, b = 0;
        for (int i = 0; i < 16; ++i) { a += x[i] * tab[i]; b += x[i] * tab[16 + i]; }
        out[0] = a;
        out[1] = b;
    }
}
#endif
static uint16_t s_pair_idx[CVBS_W / 2];         /* chroma pair -> window, and weight (Q8) */
static uint8_t s_pair_w[CVBS_W / 2];
static int s_saturation = 30;      /* ui.c sets the saved value */
static struct {
    float br, bi;                               /* previous line's burst */
    uint32_t r0, line;                          /* its raw edge, line number */
    float u[CW_N], v[CW_N];                     /* previous drawn line, for the PAL delay line */
    uint32_t uv_line;
    int vsw, vote;                              /* PAL V switch of line `line`, and its vote */
    uint32_t colour_lines, killed_lines;
} s_cl;
static uint32_t s_line_no;


static void tables_for(chroma_std_t *c, bool pal)
{
    c->omega = 2.0f * (float)M_PI * (pal ? 4433618.75f : 3579545.45f) / 20.0e6f;
    c->burst_off = pal ? 116 : 110;
    c->burst_len = pal ? 36 : 34;                 /* 8.0 / 6.1 cycles */
    c->win_len = pal ? 9 : 11;                    /* 2.0 / 1.97 cycles */
    c->act_off = 2 * (pal ? 109 : 101);
    c->burst_ratio = pal ? 0.2143f : 0.2f;
    c->rot_base = (pal ? 1280 : 1271) - 32;     /* raw samples between line starts */
    for (int d = 0; d < 64; ++d) {
        c->rot_c[d] = cosf(c->omega * (c->rot_base + d));
        c->rot_s[d] = sinf(c->omega * (c->rot_base + d));
    }
    float s0[CB_MAX], c0[CB_MAX], ms = 0, mc = 0;
    memset(c->btab, 0, sizeof(c->btab));
    memset(c->wtab, 0, sizeof(c->wtab));
    for (int i = 0; i < c->burst_len; ++i) {
        s0[i] = sinf(c->omega * (c->burst_off + i)); c0[i] = cosf(c->omega * (c->burst_off + i));
        ms += s0[i]; mc += c0[i];
    }
    for (int i = 0; i < c->burst_len; ++i) {
        c->btab[i / 16][0][i % 16] = (int8_t)lrintf((s0[i] - ms / c->burst_len) * TAB_AMP);
        c->btab[i / 16][1][i % 16] = (int8_t)lrintf((c0[i] - mc / c->burst_len) * TAB_AMP);
    }
    for (int k = 0; k < CW_N; ++k) {
        const int o = c->act_off + CW_STEP * k + CW_STEP / 2 - c->win_len / 2;
        ms = mc = 0;
        for (int i = 0; i < c->win_len; ++i) {
            s0[i] = sinf(c->omega * (o + i)); c0[i] = cosf(c->omega * (o + i));
            ms += s0[i]; mc += c0[i];
        }
        for (int i = 0; i < c->win_len; ++i) {
            c->wtab[k][0][i] = (int8_t)lrintf((s0[i] - ms / c->win_len) * TAB_AMP);
            c->wtab[k][1][i] = (int8_t)lrintf((c0[i] - mc / c->win_len) * TAB_AMP);
        }
    }
}

static void chroma_init(void)
{
    tables_for(&s_cs[0], false);
    tables_for(&s_cs[1], true);
    /* Pair p (pixels 2p, 2p+1, centre 2p + 0.5) between the window centres
     * 8k + 3.5 and 8k + 11.5. */
    for (int p = 0; p < CVBS_W / 2; ++p) {
        float pos = (2.0f * p + 0.5f - 3.5f) / 8.0f;
        if (pos < 0) pos = 0;
        if (pos > CW_N - 1.001f) pos = CW_N - 1.001f;
        s_pair_idx[p] = (uint16_t)pos;
        s_pair_w[p] = (uint8_t)((pos - (int)pos) * 256.0f);
    }
}

#include "esp_cpu.h"

/* Frame rows reach PSRAM through a copier task on core 0: the decoder fills
 * a ring of row buffers in internal RAM and moves on. A row's copy into
 * PSRAM cost the decoder's core 3.3-5k cycles in stalls on the busy bus;
 * esp_async_memcpy cost 39k cycles a call (board 2026-10-31). The copier
 * also writes the frame back in 32-row blocks as fields finish them: the
 * whole 590 KB frame in one burst, next to the PPA's traffic, had starved
 * the panel's scanout (white flashes). */
#define ROW_BUFS        48                      /* > 2 ms of rows: the copier wakes every 1 ms */
static uint8_t s_rowbuf[ROW_BUFS][CVBS_ROW_BYTES] __attribute__((aligned(16)));
static struct { uint8_t *dst, *flush; } s_rowq[ROW_BUFS];
static volatile uint32_t s_row_head, s_row_tail;  /* decoder fills head, copier drains tail */
static uint32_t s_row_waits, s_rows_copied;

static uint8_t *row_buffer(void)
{
    while (s_row_head - s_row_tail >= ROW_BUFS) { ++s_row_waits; vTaskDelay(1); }
    return s_rowbuf[s_row_head % ROW_BUFS];
}

/* Queue the row just built for dst; flush: a 32-row block to write back
 * once this row is in it, or NULL. */
static void row_send(uint8_t *dst, uint8_t *flush)
{
#ifndef __riscv
    memcpy(dst, s_rowbuf[s_row_head % ROW_BUFS], CVBS_ROW_BYTES);   /* host test: no copier task */
    (void)flush;
    return;
#endif
    s_rowq[s_row_head % ROW_BUFS].dst = dst;
    s_rowq[s_row_head % ROW_BUFS].flush = flush;
    __atomic_store_n(&s_row_head, s_row_head + 1u, __ATOMIC_RELEASE);
}

static void copier_task(void *arg)
{
    (void)arg;
    for (;;) {
        /* At most 24 rows a pass, then the lower tasks get the core. */
        for (int n = 0; n < 24 && s_row_tail != __atomic_load_n(&s_row_head, __ATOMIC_ACQUIRE); ++n) {
            const uint32_t k = s_row_tail % ROW_BUFS;
            memcpy(s_rowq[k].dst, s_rowbuf[k], CVBS_ROW_BYTES);
            if (s_rowq[k].flush)
                (void)esp_cache_msync(s_rowq[k].flush, 32 * CVBS_ROW_BYTES, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
            __atomic_store_n(&s_row_tail, s_row_tail + 1u, __ATOMIC_RELEASE);
            ++s_rows_copied;
        }
        vTaskDelay(1);
    }
}

/* n raw codes from position from into a linear buffer. */
static IRAM_ATTR void raw_copy(uint8_t *dst, uint32_t from, int n)
{
    const uint32_t a = from & RM;
    const int first = (int)(RAW - a) < n ? (int)(RAW - a) : n;
    memcpy(dst, s_raw + a, (size_t)first);
    if (n > first) memcpy(dst + first, s_raw, (size_t)(n - first));
}

/* The chroma (Cb when cr is false, else Cr) of each pixel pair of the line
 * from hsync l; false (grey) when the line has no usable burst or colour is
 * off. Both components are demodulated (the PAL delay line needs them); only
 * the row's one is interpolated. */
/* Window values (x16) -> one byte per pixel pair. */
static IRAM_ATTR void chroma_pairs(const float *comp, float k, uint8_t *out)
{
    int32_t cw[CW_N + 1];
    for (int i = 0; i < CW_N; ++i) cw[i] = (int32_t)(comp[i] * k * 16.0f) + 128 * 16;
    cw[CW_N] = cw[CW_N - 1];
    /* Window i's centre is between pairs 4i+1 and 4i+2: linear steps of a
     * quarter window. */
    uint8_t *o = out;
    for (int i = 0; i < CW_N; ++i) {
        const int32_t step = cw[i + 1] - cw[i];
        int32_t acc = cw[i] * 4 - step * 3 / 2;
        for (int j = 0; j < 4; ++j, acc += step) {
            const int32_t val = acc >> 6;
            *o++ = val < 0 ? 0 : val > 255 ? 255 : (uint8_t)val;
        }
    }
}

/* other: also the other component (a progressive field's twin row), or NULL. */
static IRAM_ATTR bool chroma_line(int64_t l, bool want_cr, uint8_t *out, uint8_t *other)
{
    const chroma_std_t *c = &s_cs[s_frame_pal];
    const uint32_t r0 = 2u * (uint32_t)(l >> 16);
    const uint32_t line = s_line_no;
    int32_t bsum[2 * CB_BLOCKS];
    raw_copy(s_bbuf, r0 + (uint32_t)c->burst_off, sizeof(s_bbuf));
    cvbs_corr16((const int8_t *)s_bbuf, &c->btab[0][0][0], bsum, CB_BLOCKS);
    const int32_t bre = bsum[0] + bsum[2] + bsum[4], bim = bsum[1] + bsum[3] + bsum[5];
    const float br = (float)bre, bi = (float)bim, bmag = sqrtf(br * br + bi * bi);
    /* Nominal burst: half the sync depth (codes), times N/2, times the table. */
    const float bnom = 0.5f * (s_blank - s_sync) * c->burst_len * 0.5f * TAB_AMP;
    const bool prev_ok = s_cl.line + 1u == line && (s_cl.br != 0.0f || s_cl.bi != 0.0f);
    const float pr = s_cl.br, pi = s_cl.bi;
    const uint32_t prev_r0 = s_cl.r0;
    s_cl.br = br; s_cl.bi = bi; s_cl.r0 = r0; s_cl.line = line;
    if (s_saturation <= 0 || bmag < 0.25f * bnom) { ++s_cl.killed_lines; s_cl.uv_line = 0; return false; }

    int vsw = 1;
    float qr = br / bmag, qi = -bi / bmag;        /* e^(-j angle(B)) */
    if (s_frame_pal) {
        /* The V switch alternates; the measured swing votes for or against
         * the running sequence: P = B conj(Bprev) e^(-j w dr), Im < 0 for +V. */
        int predicted = s_cl.vsw ? -s_cl.vsw : 1;
        if (prev_ok) {
            const int32_t dr = (int32_t)(r0 - prev_r0) - c->rot_base;
            const float rr = br * pr + bi * pi, ri = bi * pr - br * pi;     /* B conj(Bprev) */
            const float cd = dr >= 0 && dr < 64 ? c->rot_c[dr] : cosf(c->omega * (float)(dr + c->rot_base));
            const float sd = dr >= 0 && dr < 64 ? c->rot_s[dr] : sinf(c->omega * (float)(dr + c->rot_base));
            const float im = ri * cd - rr * sd;
            const int measured = im < 0 ? 1 : -1;
            if (measured == predicted) { if (s_cl.vote < 8) ++s_cl.vote; }
            else if (--s_cl.vote < -3) { predicted = measured; s_cl.vote = 0; }
        }
        vsw = s_cl.vsw = predicted;
        /* e^(-jp) = conj(Bhat) e^(+j135 vsw) */
        const float k = 0.70710678f, sr = -k, si = vsw > 0 ? k : -k;
        const float nr = qr * sr - qi * si, ni = qr * si + qi * sr;
        qr = nr; qi = ni;
    } else {
        qr = -qr; qi = -qi;                       /* e^(-jp) = -conj(Bhat) */
    }
    /* U, V in white units: (C/Nc) / (|B|/Nb) * burst_ratio; Cb = 291.9 U,
     * Cr = 207.4 V (BT.601 full range), times the saturation. */
    const float g = c->burst_ratio * (float)c->burst_len / (float)c->win_len / bmag * s_saturation / 100.0f;
    float u[CW_N], v[CW_N];
    static int32_t wsum[2 * CW_N];
    raw_copy(s_cbuf, r0 + (uint32_t)(c->act_off + CW_STEP / 2 - c->win_len / 2), sizeof(s_cbuf));
    cvbs_corr16((const int8_t *)s_cbuf, &c->wtab[0][0][0], wsum, CW_N);
    for (int k = 0; k < CW_N; ++k) {
        const int32_t cre = wsum[2 * k], cim = wsum[2 * k + 1];
        const float wr = cre * qr - cim * qi, wi = cre * qi + cim * qr;
        u[k] = wr * g;
        v[k] = (float)vsw * wi * g;
    }
    if (s_frame_pal && s_cl.uv_line + 1u == line) {
        for (int k = 0; k < CW_N; ++k) {
            const float uu = u[k], vv = v[k];
            u[k] = 0.5f * (uu + s_cl.u[k]); v[k] = 0.5f * (vv + s_cl.v[k]);
            s_cl.u[k] = uu; s_cl.v[k] = vv;
        }
    } else {
        memcpy(s_cl.u, u, sizeof(u));
        memcpy(s_cl.v, v, sizeof(v));
    }
    s_cl.uv_line = line;
    /* Cb = 291.9 U, Cr = 207.4 V (BT.601 full range). */
    chroma_pairs(want_cr ? v : u, want_cr ? 207.4f : 291.9f, out);
    if (other) chroma_pairs(want_cr ? u : v, want_cr ? 291.9f : 207.4f, other);
    ++s_cl.colour_lines;
    return true;
}

/* One frame row: the active line's Y (PAL from 10.5 us, NTSC from 9.4 us
 * after the hsync's leading edge, less the trim) at source resolution, from
 * the nearest sample (+-25 ns), through the levels, with [1 2 1] (-30 dB at
 * 4.43 MHz, -15 dB at 3.58) on colour lines against the subcarrier's dots:
 * 16 pixels a step in PIE (luma.S). Packed with the row's chroma c (C Y Y
 * per pixel pair, four pairs as three words) into a row buffer for the
 * copier. Scalar, the Y loop cost ~23 cycles a pixel (board 2026-10-31). */
static IRAM_ATTR void pack_send(const uint8_t *yb, const uint8_t *c, uint8_t *dst, uint8_t *flush)
{
    uint32_t *o = (uint32_t *)row_buffer();
    const uint32_t *y32 = (const uint32_t *)yb, *c32 = (const uint32_t *)c;
    for (int q = 0; q < CVBS_W / 8; ++q, o += 3) {
        const uint32_t w0 = y32[2 * q], w1 = y32[2 * q + 1], cw = c32[q];
        o[0] = (cw & 0xffu) | (w0 & 0xffffu) << 8 | (cw & 0xff00u) << 16;
        o[1] = (w0 >> 16) | (cw & 0xff0000u) | (w1 & 0xffu) << 24;
        o[2] = ((w1 >> 8) & 0xffu) | (cw >> 24) << 8 | (w1 >> 16) << 16;
    }
    row_send(dst, flush);
}

/* dst2/c2: the twin row of a progressive field (same Y), or NULL. */
static IRAM_ATTR void row_out(int64_t l, bool filter, const uint8_t *c, uint8_t *dst, uint8_t *flush,
                              const uint8_t *c2, uint8_t *dst2, uint8_t *flush2)
{
    const int64_t start = l + (int64_t)(s_frame_pal ? 109 : 101) * Q16 + Q16 / 2;
    static uint8_t a[16 + CVBS_W + 32] __attribute__((aligned(16)));
    static uint8_t yb[CVBS_W] __attribute__((aligned(16)));
    hist_copy(a, (start >> 16) - 16, 16 + CVBS_W + 16);   /* a[16 + i] = sample i */
    cvbs_luma16((const int8_t *)a + 16, yb, CVBS_W / 16, filter ? &s_kf[0][0] : &s_ku[0][0],
                filter ? s_kshift : s_kshift_u);
    {   /* TEMP: near-black rows as drawn */
        uint32_t sum = 0;
        for (int i = 0; i < CVBS_W; i += 4) sum += yb[i];
        if (sum < 128u * 3u) ++s_black_field;
    }
    pack_send(yb, c, dst, flush);
    if (dst2) pack_send(yb, c2, dst2, flush2);
}

/* One line from its hsync l (Q16): vertical sync, field bookkeeping, levels
 * and the picture row. */
static IRAM_ATTR void process_line(int64_t l, bool hsync_valid)
{
    const uint32_t lu = (uint32_t)(l >> 16);
    const int period = (int)(s_period >> 16);
    ++s_cnt.lines;
    ++s_line_no;
    if (hsync_valid) { ++s_cnt.hsync; ++s_hsync_field; measure_levels(lu); }

    /* Broad pulses: most of the line below the slice level, in a run longer
     * than 8 us. A run from mid-line starts the field that begins half a
     * line late. Steps of 0.4 us, each the mean of 8 samples (0.8 us); a
     * single high step inside a pulse is noise, not its end. Single samples
     * failed on a VTX with a shallow sync (6 codes deep, board 2026-10-31):
     * noise broke the run from mid-line, the odd field read as even, and
     * its rows went into the wrong frame lines (the top of the picture
     * flickered, 29-35 frames/s published). */
    /* Cheap test first: 8 points across the line; the whole walk only where
     * most of them are low (the vertical interval). */
    const int thr8 = 2 * s_thr4;
#define LOW8(n) (S4U((n) - 4u) + S4U(n) < thr8)
    /* Cheap test first: 7 points (8-sample means) at 1/8..7/8 of the line;
     * a broad pulse from mid-line covers 3-4. With single samples at
     * 1/16..15/16 it covered exactly the 3 needed (15/16 is the serration),
     * noise took one and the odd field was missed (board 2026-10-31). */
    int probe = 0;
    for (int j = 1; j < 8; ++j) probe += LOW8(lu + (uint32_t)(j * period / 8));
    int low = 0, run = 0, first = -1;
    if (probe >= 3)
    for (int k = 0; k < period; k += 4) {
        const uint32_t n = lu + (uint32_t)k + 4u;
        if (LOW8(n) || (LOW8(n - 4u) && LOW8(n + 4u))) {
            ++low;
            if (++run == 20 && first < 0) first = k - 19 * 4;
        } else {
            run = 0;
        }
    }
    ++s_line_in_field;
    /* A vertical sync counts only near where the line count expects one
     * (PAL 312/313, NTSC 262/263 lines, +-VS_WINDOW), unless VS_REACQUIRE
     * fields in a row went without one there. A dark stretch or a dropout
     * read as broad pulses mid-picture had restarted the field early, and
     * the next field's lines overwrote the top of the frame: the top third
     * blinked (board 2026-10-31). As zerowidth's p4usb decoder does. */
    const int field_lines = s_frame_pal ? 312 : 262;
    const bool vs_window = s_line_in_field >= field_lines - VS_WINDOW && s_line_in_field <= field_lines + 1 + VS_WINDOW;
    if (low * 4 > period * 35 / 100 && s_line_in_field > 100 && (vs_window || s_vs_missed >= VS_REACQUIRE)) {
        s_vs_missed = 0;
        int parity = first > period * 3 / 10 && first < period * 7 / 10;
        /* Seen a line late: the previous line high in its first half and low
         * in its second is the odd field's mid-line start (a broad line before
         * an even field's is low in both halves). That field began there. */
        int late = 0;
        if (!parity) {
            const uint32_t pl = lu - (uint32_t)period;
            int a = 0, b = 0, na = 0, nb = 0;
            for (int k = period / 10; k < period * 9 / 20; k += 8, ++na) a += LOW8(pl + (uint32_t)k);
            for (int k = period * 11 / 20; k < period * 9 / 10; k += 8, ++nb) b += LOW8(pl + (uint32_t)k);
            if (a * 10 < na * 3 && b * 10 > nb * 7) { parity = 1; late = 1; ++s_late_vs; }
        }
        /* A frame ends with its second field; without alternating parity
         * (progressive, or weak broad pulses) every field ends one. */
        s_pub_flag = s_parity == 1 || parity == s_parity;
        s_flog[s_flog_n++ & 31] = (uint32_t)parity << 31 | s_pub_flag << 30 | (uint32_t)(first & 0x7ff) << 19 |
                                  (s_rows_field & 0x3ff) << 9 | (uint32_t)(s_line_in_field & 0x1ff);
        s_rows_field = 0;
        s_bflog[(s_flog_n - 1) & 31] = s_hsync_field;
        s_black_field = 0;
        s_hsync_field = 0;
        if (s_parity == 1 || parity == s_parity) publish_frame();
        /* Progressive with hysteresis: one field read with the other parity
         * (the C5 menu raster, about one in 25) must not switch modes, which
         * wrote that field interlaced and split the picture (board
         * 2026-10-31); interlaced video, alternating, still leaves at once. */
        s_same_parity = parity == s_parity ? (s_same_parity < 16 ? s_same_parity + 1 : 16)
                                           : (s_same_parity > 8 ? s_same_parity - 8 : 0);
        s_parity = parity;
        s_line_in_field = late;
        ++s_cnt.fields;
        if (s_lut_dirty || (s_cnt.fields & 7u) == 0) build_lut();
        return;
    }
    if (s_line_in_field > field_lines + 1 + VS_WINDOW) {
        /* No vertical sync where expected: coast a field at the same phase. */
        ++s_vs_missed;
        ++s_coasts;
        if (s_parity == 1 || s_same_parity >= 2) publish_frame();
        if (s_same_parity < 2) s_parity ^= 1;
        s_line_in_field -= field_lines;
        if (s_line_in_field > field_lines) s_line_in_field = 0;
    }
    /* Standard from the line period; frame geometry follows at a frame start. */
    const bool pal = s_period > (int64_t)(637.75 * Q16);
    if (pal != s_frame_pal && s_line_in_field == 1) { s_frame_pal = pal; s_lut_dirty = true; }
    const int height = s_frame_pal ? 576 : 480;
    /* A progressive field (the C5's menu and boot screen in the P4-link
     * build, some cameras) goes into the frame as its own rows; the PPA
     * scales it twice as much (display.c). Doubling its lines here doubled
     * the rows to copy, and the decoder fell behind (board 2026-10-31). */
    const bool prog = s_same_parity >= 2;
    s_w_prog = prog;
    const int first_active = s_frame_pal ? (s_parity && !prog ? 23 : 22) : 17;
    const int k = s_line_in_field - first_active;
    if (!s_locked || k < 0 || k >= height / 2) return;
    /* PAL: the field starting on a whole line is the top one; NTSC: the
     * half-line field. */
    const int top_parity = s_frame_pal ? 0 : 1;
    const int row = prog ? k : 2 * k + ((s_parity != top_parity) ^ s_swap);
    /* YUV420: Cb on even rows, Cr on odd ones. A progressive field (the
     * C5's menu and boot screen in the P4-link build, some cameras) fills
     * both rows of its line. */
    static uint8_t c[CVBS_W / 2] __attribute__((aligned(4)));
    const bool colour = chroma_line(l, row & 1, c, NULL);
    if (!colour) memset(c, 128, sizeof(c));
    /* After a field's 16th line in a block, its 32-row block goes back. */
    uint8_t *const base = s_frames[s_w_idx];
    uint8_t *flush = k % 16 == 15 ? base + (size_t)(prog ? k - 15 : 2 * (k - 15)) * CVBS_ROW_BYTES : NULL;
    row_out(l, colour, c, base + (size_t)row * CVBS_ROW_BYTES, flush, NULL, NULL, NULL);
    ++s_rows_field;
}

/* Decode every whole line the history holds. */
static IRAM_ATTR void decode_lines(void)
{
    const int64_t hw = hw_get();
    for (;;) {
        if (!s_locked) {
            /* Search a line and a half for the first valid hsync. */
            const int64_t to = s_search + 960;
            if (hw < to + 200) break;
            int64_t edge;
            if (find_hsync(s_search, to, &edge)) {
                s_locked = true;
                s_miss = 0;
                s_next = edge;
            } else {
                s_search = to;
                if ((++s_search_fail & 15u) == 0) seed_levels(to);   /* every ~24 lines */
                continue;
            }
        }
        const int64_t exp = s_next >> 16;
        if (hw < exp + (s_period >> 16) + WIN + 200) break;
        int64_t edge, l = s_next;
        const bool ok = find_hsync(exp - WIN, exp + WIN, &edge);
        if (ok) {
            const int64_t e = edge - s_next;
            l = s_next + e / 2;
            s_period += e / 32;
            if (s_period < PERIOD_MIN) s_period = PERIOD_MIN;
            if (s_period > PERIOD_MAX) s_period = PERIOD_MAX;
            s_miss = 0;
        } else if (++s_miss > MISS_UNLOCK) {
            s_locked = false;
            s_search = exp;
            continue;
        }
        process_line(l, ok);
        s_next = l + s_period;
    }
    const int64_t need = (s_locked ? (s_next >> 16) : s_search) - 32;
    portENTER_CRITICAL(&s_hw_lock);
    s_need = need;
    portEXIT_CRITICAL(&s_hw_lock);
}

static const uint8_t s_mask3f = 0x3F;
#ifdef __riscv
void cvbs_fill64(const uint8_t *ring, uint8_t *raw, uint8_t *hist, int n, const uint8_t *mask);
#endif
static void fill64_c(const uint8_t *ring, uint8_t *raw, uint8_t *hist, uint32_t n)
{
    for (uint32_t g = 0; g < n; ++g, ring += 64, raw += 32, hist += 16)
        for (int i = 0; i < 16; ++i) {
            raw[2 * i] = ring[4 * i] & 0x3F;
            raw[2 * i + 1] = ring[4 * i + 2] & 0x3F;
            hist[i] = (uint8_t)(raw[2 * i] + raw[2 * i + 1]);
        }
}
static bool s_fill_pie;   /* the PIE routine passed its self-test */
static inline void fill64(const uint8_t *ring, uint8_t *raw, uint8_t *hist, uint32_t n)
{
#ifdef __riscv
    if (s_fill_pie) { cvbs_fill64(ring, raw, hist, (int)n, &s_mask3f); return; }
#endif
    fill64_c(ring, raw, hist, n);
}

/* The PIE routine against the C one on a test pattern (lane order). */
static bool fill_selftest(void)
{
#ifdef __riscv
    static uint8_t in[128] __attribute__((aligned(16)));
    static uint8_t r1[64] __attribute__((aligned(16))), h1[32] __attribute__((aligned(16)));
    static uint8_t r2[64] __attribute__((aligned(16))), h2[32] __attribute__((aligned(16)));
    for (int i = 0; i < 128; ++i) in[i] = (uint8_t)(i * 37 + 11);
    fill64_c(in, r1, h1, 2);
    cvbs_fill64(in, r2, h2, 2, &s_mask3f);
    return !memcmp(r1, r2, sizeof(r1)) && !memcmp(h1, h2, sizeof(h1));
#else
    return false;
#endif
}

/* New ring data into the history: per 32-bit word (codes D,D,E,E or D,E,E,F)
 * bytes 0 and 2, summed, as far as the decoder has room. True when more is
 * waiting for room. */
static IRAM_ATTR bool fill_history(void)
{
    const int64_t now = esp_timer_get_time();
    const uint32_t done = dma_done();
    uint32_t avail = (done + RING_BYTES - s_rd) % RING_BYTES;
    if (avail) s_last_data_us = now;
    if ((now - s_rd_us > RING_US * 3 / 4 && avail) || avail > RING_BYTES - 2 * NODE_BYTES) {
        /* Away for most of a ring: the DMA may have lapped us. */
        ++s_overruns;
        s_rd = done;
        s_rd_us = now;
        s_resync = true;
        return false;
    }
    s_rd_us = now;
    portENTER_CRITICAL(&s_hw_lock);
    const int64_t need = s_need;
    portEXIT_CRITICAL(&s_hw_lock);
    const int64_t room = HIST - (s_hw - need) - 32;
    if (room <= 0) return false;
    const bool more = (int64_t)avail / 4 > room;
    if (more) avail = (uint32_t)room * 4u & ~63u;   /* whole 64-byte groups */
    while (avail) {
        uint32_t n = RING_BYTES - s_rd;
        if (n > avail) n = avail;
        /* Only the DMA writes the ring: invalidating whole lines around the
         * new bytes loses nothing. */
        const uint32_t al = (uint32_t)s_cache_align, lo = s_rd / al * al, hi = (s_rd + n + al - 1u) / al * al;
        (void)esp_cache_msync(s_ring + lo, (hi < RING_BYTES ? hi : RING_BYTES) - lo, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
        /* 64 ring bytes -> 32 codes (raw) and 16 sums (history) a step, in
         * PIE (fill.S). n and s_rd are multiples of 64 (4032-byte nodes are
         * 63 groups), so s_hw stays a multiple of 16, both histories' writes
         * stay 16-byte aligned and no group straddles a wrap; split at it. */
        uint32_t done_b = 0;
        while (done_b < n) {
            const uint32_t hw = (uint32_t)s_hw + done_b / 4u;
            uint32_t groups = (n - done_b) / 64u;
            const uint32_t to_wrap = (HIST - (hw & HM)) / 16u;
            if (groups > to_wrap) groups = to_wrap;
            fill64(s_ring + s_rd + done_b, s_raw + ((2u * hw) & RM), s_hist + (hw & HM), groups);
            done_b += groups * 64u;
        }
        portENTER_CRITICAL(&s_hw_lock);
        s_hw += n / 4u;
        portEXIT_CRITICAL(&s_hw_lock);
        s_rd = (s_rd + n) % RING_BYTES;
        avail -= n;
    }
    return more;
}

/* Core 0: ring -> history. Woken by the receive's node interrupts. */
static void fill_task(void *arg)
{
    (void)arg;
    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));
        if (s_restart) {
            s_restart = false;
            rx_end();
            esp_err_t err = rx_begin();
            ++s_restarts;
            s_resync = true;
            if (err != ESP_OK) ESP_LOGE(TAG, "capture restart: %s", esp_err_to_name(err));
        }
        const int64_t before = s_hw;
        while (fill_history()) {
            if (s_task) xTaskNotifyGive(s_task);
            vTaskDelay(1);   /* the decoder makes room */
        }
        if (s_hw != before && s_task) xTaskNotifyGive(s_task);
    }
}

/* Core 1: history -> lines -> frames. */
static void decode_task(void *arg)
{
    (void)arg;
    s_rate_us = esp_timer_get_time();
    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2));
        if (s_resync) {
            s_resync = false;
            s_locked = false;
            s_search = hw_get();
        }
        const int64_t t0 = esp_timer_get_time();
        decode_lines();
        const int64_t now = esp_timer_get_time();
        s_busy_us += now - t0;
        if (now - s_rate_us >= 1000000) {
            s_rate = s_cnt;
            memset(&s_cnt, 0, sizeof(s_cnt));
            s_busy_rate_us = s_busy_us;
            s_busy_us = 0;
            s_rate_us = now;
        }
    }
}

esp_err_t cvbs_start(void)
{
    (void)esp_cache_get_alignment(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA, &s_cache_align);
    if (s_cache_align < 4) s_cache_align = 4;
    s_ring = heap_caps_aligned_calloc(128, 1, RING_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    for (int k = 0; k < 3; ++k) s_frames[k] = heap_caps_aligned_calloc(128, 1, CVBS_ROW_BYTES * CVBS_H_MAX, MALLOC_CAP_SPIRAM);
    if (!s_ring || !s_frames[0] || !s_frames[1] || !s_frames[2]) return ESP_ERR_NO_MEM;
    set_slice();
    build_lut();
    chroma_init();
    s_fill_pie = fill_selftest();
    esp_err_t err = rx_create();
    if (err == ESP_OK) err = rx_begin();
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "capture on %u MHz bus clock, DMA channel %d", (unsigned)(LINK_CLK_HZ / 1000000), s_dma_ch);
    if (xTaskCreatePinnedToCore(decode_task, "cvbs", 6144, NULL, configMAX_PRIORITIES - 2, &s_task, 1) != pdPASS)
        return ESP_ERR_NO_MEM;
    if (xTaskCreatePinnedToCore(copier_task, "cvbs_rows", 3072, NULL, configMAX_PRIORITIES - 3, NULL, 0) != pdPASS)
        return ESP_ERR_NO_MEM;
    return xTaskCreatePinnedToCore(fill_task, "cvbs_fill", 4096, NULL, configMAX_PRIORITIES - 2, &s_fill_task, 0) == pdPASS
               ? ESP_OK : ESP_ERR_NO_MEM;
}

bool cvbs_acquire(cvbs_frame_t *f)
{
    bool got = false;
    portENTER_CRITICAL(&s_lock);
    /* Not before the copier has put the frame's last rows into PSRAM (and
     * written them back): published 3 lines after its last row, a frame
     * taken at once showed rows from an older one (board 2026-10-31). */
    if (s_ready_idx >= 0 && (int32_t)(__atomic_load_n(&s_row_tail, __ATOMIC_ACQUIRE) - s_pub_head[s_ready_idx]) >= 0) {
        s_held_idx = s_ready_idx;
        s_ready_idx = -1;
        f->yuv = s_frames[s_held_idx];
        f->pal = s_frame_pal;
        f->height = (s_frame_pal ? 576 : 480) / (s_frame_prog ? 2 : 1);
        f->seq = s_seq;
        got = true;
    }
    portEXIT_CRITICAL(&s_lock);
    return got;
}

void cvbs_release(void)
{
    portENTER_CRITICAL(&s_lock);
    s_held_idx = -1;
    portEXIT_CRITICAL(&s_lock);
}

void cvbs_get_stats(cvbs_stats_t *s)
{
    s->locked = s_locked;
    s->pal = s_frame_pal;
    s->line_us = (float)s_period / Q16 / (FS_HZ / 1e6f);
    s->lines_ps = s_rate.lines;
    s->hsync_ps = s_rate.hsync;
    s->fields_ps = s_rate.fields;
    s->frames_ps = s_rate.frames;
    s->overruns = s_overruns;
    s->restarts = s_restarts;
    s->sync_level = s_sync;
    s->blank_level = s_blank;
    s->clock = esp_timer_get_time() - s_last_data_us < 100000;
}

void cvbs_set_picture(int contrast_pct, int brightness)
{
    s_contrast = contrast_pct;
    s_brightness = brightness;
    s_lut_dirty = true;
}

void cvbs_set_field_swap(bool swap) { s_swap = swap; }
void cvbs_set_saturation(int pct) { s_saturation = pct; }

void cvbs_set_sample_edge_neg(bool neg)
{
    if (neg == s_edge_neg) return;
    s_edge_neg = neg;
    if (s_task) s_restart = true;
}

/* Bench line (main.c, every 2 s): decoder load, bus clock and DMA progress,
 * pin activity, the newest samples' histogram (16 bins of 8, two codes per
 * sample) and a few samples, and the decoder state. */
void cvbs_debug_print(void)
{
    static uint32_t last_desc;
    static int64_t last_hw;
    const uint32_t desc = s_dma_ch >= 0 ? AXI_DMA.in[s_dma_ch].conf.in_dscr_bf0.val : 0;
    const int64_t hw = hw_get();
    unsigned hist[16] = {0};
    for (uint32_t k = 0; k < 4096; ++k) ++hist[XU((uint32_t)hw - 4096u + k) >> 3];
    char h[96];
    int n = 0;
    for (int k = 0; k < 16; ++k) n += snprintf(h + n, sizeof(h) - (size_t)n, "%s%u", k ? "," : "", hist[k] * 100u / 4096u);
    static const int pins[LINK_DATA_BITS + 1] = {LINK_CLK_GPIO, 2, 3, 4, 32, 33, 47};
    char act[96];
    n = 0;
    for (int p = 0; p < LINK_DATA_BITS + 1; ++p) {
        int changes = 0, prev = gpio_get_level(pins[p]);
        for (int k = 0; k < 500; ++k) {
            const int v = gpio_get_level(pins[p]);
            changes += v != prev;
            prev = v;
        }
        n += snprintf(act + n, sizeof(act) - (size_t)n, "%sg%d:%d", p ? "," : "", pins[p], changes);
    }
    printf("FILL pie=%d\n", s_fill_pie);
    printf("FLOG lines/rows/hsyncs:");
    for (int i = 0; i < 32; ++i) {
        const uint32_t v = s_flog[(s_flog_n + i) & 31];
        printf(" %lu/%lu/%lu%c%c%lu", (unsigned long)(v & 0x1ff), (unsigned long)((v >> 9) & 0x3ff),
               (unsigned long)s_bflog[(s_flog_n + i) & 31], v >> 31 ? 'o' : 'e', (v >> 30) & 1 ? 'P' : '-',
               (unsigned long)((v >> 19) & 0x7ff));
    }
    printf(" same=%d coast=%lu late=%lu\n", s_same_parity, (unsigned long)s_coasts, (unsigned long)s_late_vs);
    printf("ROWS queued=%lu copied=%lu waits=%lu\n", (unsigned long)s_row_head, (unsigned long)s_rows_copied,
           (unsigned long)s_row_waits);
    printf("CVBS colour=%lu/%lu vsw_vote=%d cpu=%lld%% moved=%u samples_ps=%lld locked=%d pal=%d line_us=%.3f lines=%lu hsync=%lu fields=%lu "
           "frames=%lu sync=%.1f blank=%.1f thr=%d overruns=%lu restarts=%lu edge=%s pins=[%s] hist%%=[%s]\n",
           (unsigned long)s_cl.colour_lines, (unsigned long)s_cl.killed_lines, s_cl.vote,
           (long long)(s_busy_rate_us / 10000), desc != last_desc, (long long)((hw - last_hw) / 2), s_locked,
           s_frame_pal, (double)((float)s_period / Q16 / 10.0f), (unsigned long)s_rate.lines,
           (unsigned long)s_rate.hsync, (unsigned long)s_rate.fields, (unsigned long)s_rate.frames, (double)s_sync,
           (double)s_blank, s_thr, (unsigned long)s_overruns, (unsigned long)s_restarts, s_edge_neg ? "neg" : "pos",
           act, h);
    last_desc = desc;
    last_hw = hw;
}
