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

static const char *TAG = "cvbs";

/* ---- capture ---- */
#define NODE_BYTES      4032
#define RING_NODES      32
#define RING_BYTES      (RING_NODES * NODE_BYTES)        /* 3.2 ms at 40 MB/s */
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

static uint8_t s_hist[HIST] __attribute__((aligned(4)));
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
static bool s_locked;
static int s_miss;
static int s_line_in_field = 1000, s_parity;
static int s_same_parity;           /* fields in a row with the same parity: progressive */
static float s_sync = 0.0f, s_blank = 20.0f;   /* code units (a sample is two codes) */
static int s_thr, s_thr4;           /* slice level in sample units, and x4 for 4-sums */
static uint16_t s_lut565[128];      /* sample (two codes) -> RGB565 grey */
static int s_contrast = 100, s_brightness = 0;
static bool s_swap;
static volatile bool s_lut_dirty = true;

/* ---- frames: the decoder writes one, one is the newest, the display holds one ---- */
static uint16_t *s_frames[3];
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

static int64_t hw_get(void)
{
    portENTER_CRITICAL(&s_hw_lock);
    const int64_t v = s_hw;
    portEXIT_CRITICAL(&s_hw_lock);
    return v;
}

/* n history samples from position from into a linear buffer. */
static void hist_copy(uint8_t *dst, int64_t from, int n)
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

static void build_lut(void)
{
    /* Black at blanking; white from the sync depth by the standard's ratio
     * (PAL 700/300 mV, NTSC 100/40 IRE). Index: a sample, two codes. */
    const float depth = s_blank - s_sync > 4.0f ? s_blank - s_sync : 4.0f;
    const float white = s_blank + depth * (s_frame_pal ? 2.333f : 2.5f);
    const float gain = 255.0f / (white - s_blank) * s_contrast / 100.0f;
    for (int v = 0; v < 128; ++v) {
        const float gf = (v / 2.0f - s_blank) * gain + s_brightness;
        const unsigned g = gf < 0.0f ? 0u : gf > 255.0f ? 255u : (unsigned)(gf + 0.5f);
        s_lut565[v] = (uint16_t)(((g >> 3) << 11) | ((g >> 2) << 5) | (g >> 3));
    }
    s_lut_dirty = false;
}

/* A falling sync edge in [from, to] (samples) with a sync body behind it and
 * blanking after it: not an equalising pulse (2.35 us) or a broad pulse. */
static bool find_hsync(int64_t from, int64_t to, int64_t *edge_q16)
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

static void set_slice(void)
{
    s_thr = (int)(s_sync + s_blank + 0.5f);   /* the midpoint, in samples of two codes */
    s_thr4 = 4 * s_thr;
}

static void measure_levels(uint32_t l)
{
    int sync = 0, blank = 0;
    for (uint32_t k = 15; k < 35; ++k) sync += XU(l + k);     /* 1.5-3.5 us */
    for (uint32_t k = 80; k < 92; ++k) blank += XU(l + k);    /* 8-9.2 us, after the burst */
    s_sync += (sync / 40.0f - s_sync) * (1.0f / 32.0f);
    s_blank += (blank / 24.0f - s_blank) * (1.0f / 32.0f);
    if (s_blank - s_sync < 4.0f) s_sync = s_blank - 4.0f;
    set_slice();
}

static void publish_frame(void)
{
    portENTER_CRITICAL(&s_lock);
    s_ready_idx = s_w_idx;
    for (int k = 0; k < 3; ++k)
        if (k != s_ready_idx && k != s_held_idx) { s_w_idx = k; break; }
    ++s_seq;
    portEXIT_CRITICAL(&s_lock);
    ++s_cnt.frames;
}

/* The active line (PAL from 10.5 us, NTSC from 9.4 us after the hsync's
 * leading edge, less the trim) at source resolution, moved by the edge's
 * sub-sample phase, through the level table. */
static void draw_line(int64_t l, uint16_t *dst)
{
    const int64_t start = l + (int64_t)(s_frame_pal ? 109 : 101) * Q16;
    static uint8_t a[CVBS_W + 4];
    hist_copy(a, start >> 16, CVBS_W + 1);
    const int f = (int)((start >> 8) & 255), g = 256 - f;
    for (int i = 0; i < CVBS_W; ++i) dst[i] = s_lut565[((a[i] * g + a[i + 1] * f) >> 8) & 127];
}

/* One line from its hsync l (Q16): vertical sync, field bookkeeping, levels
 * and the picture row. */
static void process_line(int64_t l, bool hsync_valid)
{
    const uint32_t lu = (uint32_t)(l >> 16);
    const int period = (int)(s_period >> 16);
    ++s_cnt.lines;
    if (hsync_valid) { ++s_cnt.hsync; measure_levels(lu); }

    /* Broad pulses: most of the line below the slice level, in a run longer
     * than 8 us. A run from mid-line starts the field that begins half a
     * line late. Steps of 0.4 us; a single high sample inside a pulse is
     * noise, not its end. */
    const int thr = s_thr;
    /* Cheap test first: 8 points across the line; the whole walk only where
     * most of them are low (the vertical interval). */
    int probe = 0;
    for (int k = period / 16; k < period; k += period / 8) probe += XU(lu + (uint32_t)k) < thr;
    int low = 0, run = 0, first = -1;
    if (probe >= 3)
    for (int k = 0; k < period; k += 4) {
        const uint32_t n = lu + (uint32_t)k;
        if (XU(n) < thr || (XU(n - 4u) < thr && XU(n + 4u) < thr)) {
            ++low;
            if (++run == 20 && first < 0) first = k - 19 * 4;
        } else {
            run = 0;
        }
    }
    ++s_line_in_field;
    if (low * 4 > period * 35 / 100 && s_line_in_field > 100) {
        const int parity = first > period * 3 / 10 && first < period * 7 / 10;
        /* A frame ends with its second field; without alternating parity
         * (progressive, or weak broad pulses) every field ends one. */
        if (s_parity == 1 || parity == s_parity) publish_frame();
        s_same_parity = parity == s_parity ? s_same_parity + 1 : 0;
        s_parity = parity;
        s_line_in_field = 0;
        ++s_cnt.fields;
        if (s_lut_dirty || (s_cnt.fields & 7u) == 0) build_lut();
        return;
    }
    if (s_line_in_field > 400) {   /* no vertical sync: free-run the fields */
        if (s_parity == 1) publish_frame();
        s_parity ^= 1;
        s_line_in_field = 0;
    }
    /* Standard from the line period; frame geometry follows at a frame start. */
    const bool pal = s_period > (int64_t)(637.75 * Q16);
    if (pal != s_frame_pal && s_line_in_field == 1) { s_frame_pal = pal; s_lut_dirty = true; }
    const int height = s_frame_pal ? 576 : 480;
    const int first_active = s_frame_pal ? (s_parity ? 23 : 22) : 17;
    const int k = s_line_in_field - first_active;
    if (!s_locked || k < 0 || k >= height / 2) return;
    /* PAL: the field starting on a whole line is the top one; NTSC: the
     * half-line field. */
    const int top_parity = s_frame_pal ? 0 : 1;
    const int row = 2 * k + ((s_parity != top_parity) ^ s_swap);
    uint16_t *dst = s_frames[s_w_idx] + (size_t)row * CVBS_W;
    draw_line(l, dst);
    /* A progressive field (the C5's menu and boot screen in the P4-link
     * build, some cameras) fills both rows of its line. */
    if (s_same_parity >= 2) {
        uint16_t *other = s_frames[s_w_idx] + (size_t)(row ^ 1) * CVBS_W;
        memcpy(other, dst, CVBS_W * 2);
    }
    /* The PPA reads the frame from memory: rows go back from the cache in
     * blocks of 32 (32 KB) as a field finishes them. One write-back per row
     * cost the decoder a third of its core; the whole frame in one 590 KB
     * burst, next to the PPA's traffic, starved the panel's scanout: frames
     * of 31.7 ms instead of 16.4 seen as white flashes (board 2026-10-31,
     * 6 a minute; 0 in 90 s without that burst). */
    if (k % 16 == 15) {
        uint16_t *block = s_frames[s_w_idx] + (size_t)(2 * (k - 15)) * CVBS_W;
        (void)esp_cache_msync(block, 32 * CVBS_W * 2, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    }
}

/* Decode every whole line the history holds. */
static void decode_lines(void)
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

/* New ring data into the history: per 32-bit word (codes D,D,E,E or D,E,E,F)
 * bytes 0 and 2, summed, as far as the decoder has room. True when more is
 * waiting for room. */
static bool fill_history(void)
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
    if (more) avail = (uint32_t)room * 4u & ~15u;   /* whole groups of four words */
    while (avail) {
        uint32_t n = RING_BYTES - s_rd;
        if (n > avail) n = avail;
        /* Only the DMA writes the ring: invalidating whole lines around the
         * new bytes loses nothing. */
        const uint32_t al = (uint32_t)s_cache_align, lo = s_rd / al * al, hi = (s_rd + n + al - 1u) / al * al;
        (void)esp_cache_msync(s_ring + lo, (hi < RING_BYTES ? hi : RING_BYTES) - lo, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
        /* Four words -> four samples -> one 32-bit store. n and s_rd are
         * multiples of 16 (4032-byte nodes), so s_hw stays a multiple of 4
         * and no group straddles the history's wrap. */
        const uint32_t *src = (const uint32_t *)(s_ring + s_rd);
        uint32_t hw = (uint32_t)s_hw;
        for (uint32_t i = 0; i < n / 4u; i += 4u, hw += 4u) {
            const uint32_t a = src[i], b = src[i + 1], c = src[i + 2], d = src[i + 3];
            const uint32_t sa = (a & 0x3Fu) + ((a >> 16) & 0x3Fu), sb = (b & 0x3Fu) + ((b >> 16) & 0x3Fu);
            const uint32_t sc = (c & 0x3Fu) + ((c >> 16) & 0x3Fu), sd = (d & 0x3Fu) + ((d >> 16) & 0x3Fu);
            *(uint32_t *)(s_hist + (hw & HM)) = sa | sb << 8 | sc << 16 | sd << 24;
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
    for (int k = 0; k < 3; ++k) s_frames[k] = heap_caps_aligned_calloc(128, 1, CVBS_W * 2 * CVBS_H_MAX, MALLOC_CAP_SPIRAM);
    if (!s_ring || !s_frames[0] || !s_frames[1] || !s_frames[2]) return ESP_ERR_NO_MEM;
    set_slice();
    build_lut();
    esp_err_t err = rx_create();
    if (err == ESP_OK) err = rx_begin();
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "capture on %u MHz bus clock, DMA channel %d", (unsigned)(LINK_CLK_HZ / 1000000), s_dma_ch);
    if (xTaskCreatePinnedToCore(decode_task, "cvbs", 6144, NULL, configMAX_PRIORITIES - 2, &s_task, 1) != pdPASS)
        return ESP_ERR_NO_MEM;
    return xTaskCreatePinnedToCore(fill_task, "cvbs_fill", 4096, NULL, configMAX_PRIORITIES - 2, &s_fill_task, 0) == pdPASS
               ? ESP_OK : ESP_ERR_NO_MEM;
}

bool cvbs_acquire(cvbs_frame_t *f)
{
    bool got = false;
    portENTER_CRITICAL(&s_lock);
    if (s_ready_idx >= 0) {
        s_held_idx = s_ready_idx;
        s_ready_idx = -1;
        f->rgb = s_frames[s_held_idx];
        f->pal = s_frame_pal;
        f->height = s_frame_pal ? 576 : 480;
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
    printf("CVBS cpu=%lld%% moved=%u samples_ps=%lld locked=%d pal=%d line_us=%.3f lines=%lu hsync=%lu fields=%lu "
           "frames=%lu sync=%.1f blank=%.1f thr=%d overruns=%lu restarts=%lu edge=%s pins=[%s] hist%%=[%s]\n",
           (long long)(s_busy_rate_us / 10000), desc != last_desc, (long long)((hw - last_hw) / 2), s_locked,
           s_frame_pal, (double)((float)s_period / Q16 / 10.0f), (unsigned long)s_rate.lines,
           (unsigned long)s_rate.hsync, (unsigned long)s_rate.fields, (unsigned long)s_rate.frames, (double)s_sync,
           (double)s_blank, s_thr, (unsigned long)s_overruns, (unsigned long)s_restarts, s_edge_neg ? "neg" : "pos",
           act, h);
    last_desc = desc;
    last_hw = hw;
}
