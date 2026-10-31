/* C5VRX-5 by Twotoz and the C5VRX contributors: realtime transport.
 * MODEM_DIAG -> PARLIO RX 40 MS/s -> 32 KiB cyclic ring -> TX BitScrambler
 * (U80P64-R1) -> PARLIO TX 40 MHz -> 6-bit resistor DAC. Both GDMA rings run
 * with interrupts off and cleared suc_eof (espressif/esp-idf#19091), so no
 * DMA boundary is a stream boundary; the CPU never paces samples. */
#include "receiver.h"
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include "driver/bitscrambler.h"
#include "driver/parlio_rx.h"
#include "driver/parlio_tx.h"
#include "esp_attr.h"
#include "esp_cache.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/misc.h"
#include "hal/bitscrambler_ll.h"
#include "transport_lut.h"
#include "transport_guard.h"
#include "transport_recovery.h"
#include "output_level.h"
#include "hal/parlio_ll.h"
#include "p64_tables.h"
#include "u85q_table.h"
#include "snapshot.h"
#include "control_window.h"
#include "phase8_gain_lut.h"
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "esp_private/esp_clk.h"
#include "soc/soc_caps.h"
#include "phy.h"
#include "soc/ahb_dma_struct.h"
#include "soc/bitscrambler_struct.h"
#include "soc/parl_io_struct.h"
#include "soc/pcr_struct.h"
#ifdef C5VRX_LCD
#include "driver/gpio.h"
#include "esp_private/gpio.h"
#include "hal/parlio_periph.h"
#include "hal/usb_serial_jtag_ll.h"
#include "soc/gpio_sig_map.h"
#include "soc/io_mux_reg.h"
#endif

/* Two fixed decoders, each a program with its own LUT16 and no runtime policy
 * (no kp ladder, no LP): U80P64-R1 (default) and U85Q (operator decision
 * 2026-10-28, selectable for a like-for-like board comparison; host model:
 * U85Q holds sync ~0.5-1 dB longer at deviation 0.7, P64 longer at 1.0). The
 * selection is applied by the same verified restart as OUTPUT and profiles;
 * a rollback returns to the last verified decoder with its own LUT. */
BITSCRAMBLER_PROGRAM(s_p64, "p64");
BITSCRAMBLER_PROGRAM(s_u85q, "u85q");
static bool s_started;
static uint8_t s_demod = TRANSPORT_DEMOD_P64, s_verified_demod = TRANSPORT_DEMOD_P64;
static const void *demod_program(uint8_t d) { return d == TRANSPORT_DEMOD_U85Q ? s_u85q : s_p64; }
static const uint16_t *demod_table(uint8_t d) { return d == TRANSPORT_DEMOD_U85Q ? u85q_table : p64_table; }
static const void *tracker_program(void) { return demod_program(s_demod); }

/* 6-bit resistor DAC, LSB first (XIAO D4..D9; 8.2k/3.9k/2k/1k/470R/240R
 * plus 200R). Do not change the tested pin order. D10/GPIO10 and D3/GPIO7
 * are reserved for future seventh/eighth branches; keep them disconnected
 * here until a complete wider-DAC implementation is hardware-validated. */
#ifdef C5VRX_LCD
/* Waveshare ESP32-C5-LCD-1.47: the same 6-bit codes, on header pins beside
 * the LCD (GPIO23/24 are its CS/DC). Bits 1..5 always on GPIO9/15/3/12/11
 * (header 5/4/16/8/9; 3.9k/2k/1k/470R/240R); the 8.2k LSB on GPIO13 (USB
 * D-, header 6) only in 6-bit mode, which turns USB off (transport_set_dac_bits).
 * Not GPIO27/2: boot straps (GPIO27 has the board's 10k pull-up). */
static const int s_dac_gpio[8] = {-1, 9, 15, 3, 12, 11, -1, -1};
#define DAC_LSB_USB_GPIO 13
static uint8_t s_dac_bits = 5u;
#else
static const int s_dac_gpio[8] = {23, 24, 11, 12, 8, 9, -1, -1};
#endif

DMA_ATTR __attribute__((aligned(64))) uint8_t s_raw_ring[RAW_RING_BYTES];
static parlio_rx_unit_handle_t s_rx;
static parlio_rx_delimiter_handle_t s_rx_delimiter;
static parlio_tx_unit_handle_t s_tx;
static bitscrambler_handle_t s_bs;
static bool s_tx_enabled;
static bool s_rx_enabled, s_bs_enabled;
static bool s_transport_fault;
static unsigned s_output, s_verified_output, s_output_clipped;
static bool s_have_verified;
static uint32_t s_restart_errors, s_rollbacks, s_rollback_fail;
#define LIVE_TX_NODES ((RAW_RING_BYTES + 4091u) / 4092u)
DMA_ATTR __attribute__((aligned(64))) static dma_descriptor_t s_live_tx_nodes[LIVE_TX_NODES];
int s_rx_dma_ch = -1, s_tx_dma_ch = -1;
ring_node_t s_rx_nodes[MAX_RING_DESCRIPTORS];
int s_rx_node_count;
static uint32_t s_tx_empty, s_rx_overflow, s_dma_faults, s_bs_overload;
static struct {
    uint32_t calls, not_ready, source_invalid, pointer_invalid, lapped, timed_out;
    uint32_t max_us, max_advance;
} s_control_copy;

void sync_dma_c2m(const void *addr, size_t size)
{
    if (!addr || !size) return;
    uint32_t start = (uint32_t)addr & ~63u, end = ((uint32_t)addr + size + 63u) & ~63u;
    (void)esp_cache_msync((void *)start, end - start,
                          ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
}

void sync_dma_m2c(const void *addr, size_t size)
{
    if (!addr || !size) return;
    uint32_t start = (uint32_t)addr & ~63u, end = ((uint32_t)addr + size + 63u) & ~63u;
    (void)esp_cache_msync((void *)start, end - start,
                          ESP_CACHE_MSYNC_FLAG_DIR_M2C | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
}

static esp_err_t enable_tx(void)
{
    esp_err_t err = parlio_tx_unit_enable(s_tx);
    if (err == ESP_OK) s_tx_enabled = true;
    return err;
}

static esp_err_t disable_tx(void)
{
    esp_err_t err = parlio_tx_unit_disable(s_tx);
    if (err == ESP_OK) s_tx_enabled = false;
    return err;
}

static esp_err_t prepare_rx(void)
{
    const parlio_rx_unit_config_t cfg = {
        .trans_queue_depth = 1u,
        .max_recv_size = sizeof(s_raw_ring),
        .dma_burst_size = 32u,
        .data_width = 8u,
        .clk_src = PARLIO_CLK_SRC_DEFAULT,
        .exp_clk_freq_hz = IQ_RATE_HZ,
        .clk_in_gpio_num = -1,
        .clk_out_gpio_num = -1,
        .valid_gpio_num = -1,
        /* Share MODEM_DIAG routing's GPIO27 I-LSB / GPIO2 Q-sign pin order. */
        .data_gpio_nums = RF_IQ_GPIO_NUMS,
        .flags = {.free_clk = true, .clk_gate_en = false, .allow_pd = false},
    };
    esp_err_t err = parlio_new_rx_unit(&cfg, &s_rx);
    if (err != ESP_OK) return err;
    const parlio_rx_soft_delimiter_config_t delim = {
        .sample_edge = PARLIO_SAMPLE_EDGE_POS,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
        .eof_data_len = sizeof(s_raw_ring),
        .timeout_ticks = 0u,
    };
    err = parlio_new_rx_soft_delimiter(&delim, &s_rx_delimiter);
    if (err != ESP_OK) return err;
    err = parlio_rx_unit_enable(s_rx, false);
    if (err == ESP_OK) s_rx_enabled = true;
    return err;
}

static esp_err_t prepare_tx(void)
{
    parlio_tx_unit_config_t cfg = {
        .clk_src = PARLIO_CLK_SRC_DEFAULT,
        .clk_in_gpio_num = -1,
        .output_clk_freq_hz = 40000000u,
        .data_width = 8u,
        .clk_out_gpio_num = -1,
        .valid_gpio_num = -1,
        .trans_queue_depth = 1u,
        .max_transfer_size = sizeof(s_raw_ring),
        .dma_burst_size = 32u,
        .shift_edge = PARLIO_SHIFT_EDGE_NEG,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
    };
    for (unsigned k = 0; k < 8u; ++k) cfg.data_gpio_nums[k] = s_dac_gpio[k];
    esp_err_t err = parlio_new_tx_unit(&cfg, &s_tx);
    if (err != ESP_OK) return err;
    const bitscrambler_config_t bs = {.dir = BITSCRAMBLER_DIR_TX, .attach_to = SOC_BITSCRAMBLER_ATTACH_PARL_IO};
    err = bitscrambler_new(&bs, &s_bs);
    if (err != ESP_OK) return err;
    return enable_tx();
}

#ifdef C5VRX_LCD
/* DAC bit 0 onto GPIO13 (USB D-), or back to USB. The TX unit, program and
 * LUT stay as they are: only the pad changes owner, so no restart. The
 * console never blocks on output (main.c).
 * Switching the pads off alone left the host attached (board 2026-10-31: it
 * saw no disconnect, so 6 -> 5 bit never enumerated again). So the PHY's
 * pull-up is overridden off first and both lines sit low (SE0) until the
 * host has seen the detach, and D+ (GPIO14) stays pulled low while the DAC
 * owns D-. Back to USB: SE0 again, then the pull-up returns: a fresh attach,
 * the host resets the bus and enumerates. The D+ pull-up is the PHY's
 * together with GPIO14's IO_MUX pull-up (on at reset; gpio_ll.h), which the
 * GPIO calls here clear: both pads' IO_MUX words are saved and written back
 * (board 2026-10-31: without that the host saw the detach but no re-attach). */
#define USB_DETACH_MS 100
static uint32_t s_usb_iomux[2];
static void route_dac_lsb(bool dac)
{
    const gpio_num_t pin = DAC_LSB_USB_GPIO, dp = DAC_LSB_USB_GPIO + 1;
    const usb_serial_jtag_pull_override_vals_t detach = {.dp_pu = false, .dm_pu = false, .dp_pd = true, .dm_pd = true};
    if (dac) {
        s_usb_iomux[0] = REG_READ(IO_MUX_GPIO13_REG);
        s_usb_iomux[1] = REG_READ(IO_MUX_GPIO14_REG);
        usb_serial_jtag_ll_phy_enable_pull_override(&detach);
        vTaskDelay(pdMS_TO_TICKS(USB_DETACH_MS));
        usb_serial_jtag_ll_phy_enable_pad(false);
        gpio_func_sel(dp, PIN_FUNC_GPIO);
        gpio_set_direction(dp, GPIO_MODE_INPUT);
        gpio_set_pull_mode(dp, GPIO_PULLDOWN_ONLY);
        gpio_func_sel(pin, PIN_FUNC_GPIO);
        gpio_set_pull_mode(pin, GPIO_FLOATING);
        esp_rom_gpio_connect_out_signal(pin, soc_parlio_signals[0].tx_units[0].data_sigs[0], false, false);
    } else {
        esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX, false, false);
        gpio_set_direction(pin, GPIO_MODE_DISABLE);
        gpio_set_direction(dp, GPIO_MODE_DISABLE);
        REG_WRITE(IO_MUX_GPIO13_REG, s_usb_iomux[0]);   /* pulls and function as USB left them */
        REG_WRITE(IO_MUX_GPIO14_REG, s_usb_iomux[1]);
        usb_serial_jtag_ll_phy_enable_pull_override(&detach);
        usb_serial_jtag_ll_phy_enable_pad(true);
        vTaskDelay(pdMS_TO_TICKS(USB_DETACH_MS));
        usb_serial_jtag_ll_phy_disable_pull_override();
    }
}

uint8_t transport_dac_bits(void) { return s_dac_bits; }

/* 5 (USB on) or 6 (USB off, LSB on GPIO13); before the first start only
 * recorded, transport_start routes it. */
bool transport_set_dac_bits(uint8_t bits)
{
    if (bits != 5u && bits != 6u) return false;
    if (bits == s_dac_bits) return true;
    if (s_tx) route_dac_lsb(bits == 6u);
    s_dac_bits = bits;
    return true;
}
#endif

static esp_err_t start_rx(void)
{
    esp_err_t err = parlio_rx_soft_delimiter_start_stop(s_rx, s_rx_delimiter, true);
    if (err != ESP_OK) return err;
    const parlio_receive_config_t cfg = {
        .delimiter = s_rx_delimiter,
        .flags = {.partial_rx_en = true, .indirect_mount = false},
    };
    return parlio_rx_unit_receive(s_rx, s_raw_ring, sizeof(s_raw_ring), &cfg);
}

/* Use an owned fixed raw-ring TX chain, just as the menu uses an owned
 * raster chain. Re-submitting IDF loop transactions after repeated P hung
 * in its unbounded FIFO-ready spin. This chain never remounts driver loop
 * buffers, and the readiness wait yields with a bounded deadline. */
static esp_err_t start_tx(void)
{
    size_t offset = 0;
    for (unsigned k = 0; k < LIVE_TX_NODES; ++k) {
        dma_descriptor_t *d = &s_live_tx_nodes[k];
        size_t length = sizeof(s_raw_ring) - offset;
        if (length > 4092u) length = 4092u;
        memset(d, 0, sizeof(*d));
        d->dw0.size = length;
        d->dw0.length = length;
        d->dw0.owner = 1;
        d->buffer = s_raw_ring + offset;
        d->next = &s_live_tx_nodes[(k + 1u) % LIVE_TX_NODES];
        offset += length;
    }
    sync_dma_c2m(s_live_tx_nodes, sizeof(s_live_tx_nodes));
    parlio_ll_tx_enable_clock(&PARL_IO, false);
    parlio_ll_tx_reset_clock(&PARL_IO);
    parlio_ll_tx_reset_fifo(&PARL_IO);
    parlio_ll_tx_set_idle_data_value(&PARL_IO, DAC_IDLE_CODE);
    parlio_ll_tx_set_eof_condition(&PARL_IO, PARLIO_LL_TX_EOF_COND_DATA_LEN);
    parlio_ll_tx_set_trans_bit_len(&PARL_IO, 1);
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    AHB_DMA.out_link_addr[s_tx_dma_ch].val = (uint32_t)&s_live_tx_nodes[0];
    AHB_DMA.channel[s_tx_dma_ch].out.out_link.outlink_start_chn = 1;
    int64_t deadline = esp_timer_get_time() + 20000;
    while (!parlio_ll_tx_is_ready(&PARL_IO)) {
        if (esp_timer_get_time() >= deadline) return ESP_ERR_TIMEOUT;
        vTaskDelay(1);   /* taskYIELD alone would still starve lower-priority IDLE */
    }
    parlio_ll_tx_start(&PARL_IO, true);
    parlio_ll_tx_enable_clock(&PARL_IO, true);
    return ESP_OK;
}

/* The CPU LUT port follows lut_mode (ESP32-C5 TRM 44.3.1). On this
 * board, writes/readback under RUN or PAUSE fail; HALT with the normal
 * load/reset/start sequence succeeds. Do not claim a seamless live update. */
static uint16_t s_lut_live[1024];   /* what the LUT RAM holds */
static bool s_lut_known;            /* shadow valid: the verified decoder's LUT16 */
/* IDF V1 image: 3 header words, 8 instructions of 9 words, LUT16.
 * Reuse the target LUT storage; only the 300-byte prefix is additional.
 * Prepare before halting so the driver writes the target exactly once. */
typedef struct {
    uint32_t prefix[3u + 8u * 9u];
    uint16_t lut[1024];
} tracker_image_t;
static tracker_image_t s_program_work;
#define s_lut_work s_program_work.lut
_Static_assert(offsetof(tracker_image_t, lut) == 300u, "BitScrambler V1 LUT offset");
static uint32_t s_lut_writes, s_lut_verify_fail;
static bool resume_live(const uint16_t *restore_lut);
static transport_guard_t s_switch_guard;

static void select_lut_entry(void *ctx, unsigned entry)
{
    (void)ctx;
    BITSCRAMBLER.lut_cfg[BITSCRAMBLER_DIR_TX].cfg0.lut_idx = entry;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
}

static uint32_t read_lut_entry(void *ctx)
{
    (void)ctx;
    return BITSCRAMBLER.lut_cfg[BITSCRAMBLER_DIR_TX].cfg1.lut;
}

/* Apply one complete target using the normal pipeline restart. Copy the
 * shadow only after the halted RAM verifies; a failed load resumes baseline
 * video instead of repeatedly retrying writes under the running core. */
static int64_t stage_now(void);
static void stage_note(int stage, int64_t us);
static void stage_hooks(void) { tr_hook_now = stage_now; tr_hook_note = stage_note; }

static uint32_t s_prog_verify_fail;

static bool program_words(const uint8_t *bin, const uint8_t **inst)
{
    /* bitscrambler_program_hdr_t (esp_driver_bitscrambler): version, hw_rev,
     * hdr_len (words), inst_ct, ... */
    if (bin[0] != 1u || bin[2] != 3u || bin[3] != 8u || bin[4] != 0u || bin[5] != 2u || bin[6] != 1u)
        return false;
    *inst = bin + (size_t)bin[2] * 4u;
    return true;
}

/* The analog profile has already changed its filter; restart the same
 * decoder image through the shared guard and verified rollback coordinator. */
bool transport_restart_profile(void)
{
    if (s_transport_fault || !s_lut_known || !s_started || s_menu_active ||
        !transport_guard_ready(&s_switch_guard, esp_timer_get_time())) return false;
    stage_hooks();
    const bool ok = resume_live(demod_table(s_demod));
    transport_guard_mark(&s_switch_guard, esp_timer_get_time());
    return ok;
}

uint32_t transport_switch_wait_ms(void)
{
    int64_t now = esp_timer_get_time();
    return transport_guard_ready(&s_switch_guard, now) ? 0 :
           (uint32_t)((TRANSPORT_SWITCH_GAP_US - (now - s_switch_guard.last_switch_us) + 999) / 1000);
}

const char *transport_demod_name(uint8_t d) { return d == TRANSPORT_DEMOD_U85Q ? U85Q_MODEL_ID : P64_MODEL_ID; }
const char *transport_program_name(void) { return transport_demod_name(s_demod); }
uint8_t transport_demod(void) { return s_demod; }
/* The live, verified decoder's LUT16 (without the OUTPUT scaling) and its
 * token width, for the picture measure (picture_sync.h); NULL while unknown. */
const uint16_t *transport_decoder_lut(unsigned *token_bits)
{
    if (!s_lut_known || s_transport_fault || !s_started) return NULL;
    *token_bits = s_verified_demod == TRANSPORT_DEMOD_U85Q ? 5u : 4u;
    return demod_table(s_verified_demod);
}

/* Select the decoder: at once while the menu owns TX or before the first
 * start, else a verified restart behind the shared guard. A failed restart
 * rolls back to the last verified decoder (restart_stage). */
bool transport_set_demod(uint8_t d)
{
    if (s_transport_fault || d >= TRANSPORT_DEMODS) return false;
    if (d == s_demod) return true;
    if (s_menu_active || !s_started) { s_demod = d; return true; }
    if (!transport_guard_ready(&s_switch_guard, esp_timer_get_time())) return false;
    s_demod = d;
    const bool ok = resume_live(demod_table(d));
    transport_guard_mark(&s_switch_guard, esp_timer_get_time());
    return ok;
}
bool transport_live(void) { return s_started; }
bool transport_faulted(void) { return s_transport_fault; }
uint32_t transport_restart_errors(void) { return s_restart_errors; }
unsigned transport_output(void) { return s_output; }
const char *transport_output_name(void)
{
    static const char *const names[OUTPUT_LEVELS] = {
        "1.0x", "1.5x", "2.0x", "2.5x", "3.0x", "3.5x", "4.0x", "4.5x", "5.0x", "0.5x"
    };
    return names[s_output];
}
bool transport_set_output(unsigned level)
{
    if (s_transport_fault || level >= OUTPUT_LEVELS) return false;
    if (level == s_output) return true;
    if (s_menu_active || !s_started) { s_output = level; return true; }
    if (!transport_guard_ready(&s_switch_guard, esp_timer_get_time())) return false;
    s_output = level;
    bool ok = resume_live(s_lut_known ? demod_table(s_demod) : NULL);
    transport_guard_mark(&s_switch_guard, esp_timer_get_time());
    return ok;
}

/* Selected configuration stays separate from the last fully verified live
 * configuration. Publication waits for both DMA starts, including readback
 * of the complete program and LUT on boot/menu/rollback as well as OUTPUT/format changes. */
typedef struct {
    const uint8_t *program;
    unsigned output, clipped;
} restart_t;

static bool stop_part(void *ctx, tr_part_t part)
{
    (void)ctx;
    esp_err_t err = part == TR_TX ? disable_tx() : part == TR_BS ? bitscrambler_disable(s_bs) :
                    parlio_rx_unit_disable(s_rx);
    if (err != ESP_OK) printf("TRANSPORT teardown part=%d error=%s\n", part, esp_err_to_name(err));
    return err == ESP_OK;
}
/* Where a restart's picture gap goes (T TRANSPORT_STAGES_US, range audit
 * PR #230): the last restart's time per stage. */
static uint32_t s_stage_us[TR_STAGES], s_quiesce_us;
static int64_t stage_now(void) { return esp_timer_get_time(); }
static void stage_note(int stage, int64_t us)
{
    if (stage < 0) s_quiesce_us = (uint32_t)us;
    else if (stage < TR_STAGES) s_stage_us[stage] = (uint32_t)us;
}
static bool quiesce(void *ctx)
{
    bool ok = tr_stop_parts(ctx, stop_part, &s_tx_enabled, &s_bs_enabled, &s_rx_enabled, true);
    s_started = false;
    s_rx_node_count = 0; /* observers cannot consume stopped/stale DMA */
    return ok;
}

static esp_err_t start_rings_tail(void);
static bool restart_stage(void *ctx, tr_stage_t stage, bool rollback)
{
    restart_t *r = ctx;
    if (stage == TR_BS_ENABLE && rollback) {
        s_output = s_verified_output;
        r->output = s_verified_output;
        s_demod = s_verified_demod;   /* its program goes with the verified LUT below */
        r->program = tracker_program();
        const uint8_t *inst;
        if (!program_words(r->program, &inst)) return false;
        memcpy(s_program_work.prefix, r->program, sizeof(s_program_work.prefix));
        memcpy(s_lut_work, s_lut_live, sizeof(s_lut_work));
    }
    esp_err_t err = ESP_OK;
    switch (stage) {
    case TR_BS_ENABLE:
        err = bitscrambler_enable(s_bs);
        if (err == ESP_OK) s_bs_enabled = true;
        break;
    case TR_PROGRAM_LOAD:
        err = bitscrambler_load_program(s_bs, &s_program_work);
        if (err == ESP_OK) s_lut_writes += TRANSPORT_LUT_ENTRIES;
        break;
    case TR_INSTRUCTION_VERIFY: {
        const uint8_t *inst;
        if (!program_words(r->program, &inst)) { ++s_prog_verify_fail; return false; }
        bool verified = true;
        for (unsigned i = 0; i < 8u; ++i)
            for (unsigned w = 0; w < BITSCRAMBLER_LL_INST_LEN_WORDS; ++w) {
                uint32_t expected;
                memcpy(&expected, inst + (i * BITSCRAMBLER_LL_INST_LEN_WORDS + w) * 4u, 4);
                /* Instructions have 257 bits: only bit 0 of word 8 exists. */
                uint32_t mask = w == 8u ? 1u : UINT32_MAX;
                uint32_t actual = bitscrambler_ll_instmem_read(&BITSCRAMBLER, BITSCRAMBLER_DIR_TX, i, w);
                if ((actual & mask) != (expected & mask)) { ++s_prog_verify_fail; verified = false; }
            }
        if (!verified) {
            printf("TRANSPORT restart instruction_verify failed rollback=%u\n", rollback);
            return false;
        }
        break;
    }
    case TR_LUT_VERIFY:
        if (!transport_lut_verify(s_lut_work, NULL, select_lut_entry, read_lut_entry, &s_lut_verify_fail)) {
            printf("TRANSPORT restart lut_verify failed rollback=%u\n", rollback);
            return false;
        }
        break;
    case TR_BS_RESET: err = bitscrambler_reset(s_bs); break;
    case TR_BS_START: err = bitscrambler_start(s_bs); break;
    case TR_RX_ENABLE:
        err = parlio_rx_unit_enable(s_rx, false);
        if (err == ESP_OK) s_rx_enabled = true;
        break;
    case TR_TX_ENABLE:
        err = enable_tx();
        break;
    case TR_RX_START: err = start_rx(); break;
    case TR_TX_START: err = start_rings_tail(); break;
    default: return false;
    }
    if (err != ESP_OK)
        printf("TRANSPORT restart stage=%d rollback=%u error=%s\n", stage, rollback, esp_err_to_name(err));
    return err == ESP_OK;
}


static int find_dma(bool rx)
{
    for (int k = 0; k < 3; ++k) {
        if (rx && AHB_DMA.channel[k].in.in_peri_sel.peri_in_sel_chn == 9) return k;
        if (!rx && AHB_DMA.channel[k].out.out_peri_sel.peri_out_sel_chn == 9) return k;
    }
    return -1;
}

/* Clear suc_eof on every ring descriptor: the ring becomes endless. */
static int clear_eof(int ch, bool rx)
{
    if (ch < 0 || ch >= 3) return 0;
    uint32_t first = rx ? AHB_DMA.channel[ch].in.in_dscr_bf0.val : AHB_DMA.channel[ch].out.out_dscr_bf0.val;
    if (first < 0x40800000u || first >= 0x40860000u) return 0;
    dma_descriptor_t *d = (dma_descriptor_t *)(uintptr_t)first;
    int count = 0;
    while (d && count < MAX_RING_DESCRIPTORS) {
        d->dw0.suc_eof = 0;
        sync_dma_c2m(d, sizeof(*d));
        if (rx) {
            s_rx_nodes[count].dscr = d;
            s_rx_nodes[count].buffer = (uint8_t *)d->buffer;
            s_rx_nodes[count].length = d->dw0.size ? d->dw0.size : CONTROL_BYTES;
        }
        d = d->next;
        ++count;
        if ((uintptr_t)d == first) break;
    }
    if (rx) s_rx_node_count = count;
    __asm__ __volatile__("fence rw, rw" ::: "memory");
    return count;
}

static void quiet_interrupts(void)
{
    for (int k = 0; k < 3; ++k) { AHB_DMA.in_intr[k].ena.val = 0; AHB_DMA.out_intr[k].ena.val = 0; }
    PARL_IO.int_ena.val = 0;
    /* External-enable EOF selection: soft-delimiter RX never stalls on EOF. */
    PARL_IO.rx_genrl_cfg.rx_eof_gen_sel = 1;
}

/* RX first, TX three quarters of a ring later (nominal614us). Compared with
 * half a ring this buys8192 bytes for whole-descriptor/write-history guards,
 * for205us more video latency and no extra memory. */
static esp_err_t start_rings_tail(void)
{
    quiet_interrupts();
    /* Make RX endless BEFORE any wait for TX. start_tx may wait/yield for
     * FIFO readiness; deferring this until afterward could let RX reach EOF. */
    s_rx_dma_ch = find_dma(true);
    if (s_rx_dma_ch < 0) return ESP_ERR_NOT_FOUND;
    const int rx_channel = s_rx_dma_ch;
    if (clear_eof(s_rx_dma_ch, true) < 3) return ESP_ERR_INVALID_STATE;
    esp_rom_delay_us((uint64_t)LIVE_PIPELINE_LAG_BYTES * 1000000ULL / IQ_RATE_HZ);
    s_tx_dma_ch = find_dma(false);
    if (s_tx_dma_ch < 0) return ESP_ERR_NOT_FOUND;
    const int tx_channel = s_tx_dma_ch;
    esp_err_t err = start_tx();
    if (err != ESP_OK) return err;
    int rx_after = find_dma(true), tx_after = find_dma(false);
    if (rx_after < 0 || tx_after < 0) return ESP_ERR_NOT_FOUND;
    if (rx_after != rx_channel || tx_after != tx_channel) return ESP_ERR_INVALID_STATE;
    quiet_interrupts();
    if (clear_eof(s_tx_dma_ch, false) < 1)
        return ESP_ERR_INVALID_STATE;
    PARL_IO.int_clr.val = UINT32_MAX;
    if (s_rx_dma_ch >= 0) AHB_DMA.in_intr[s_rx_dma_ch].clr.val = UINT32_MAX;
    if (s_tx_dma_ch >= 0) AHB_DMA.out_intr[s_tx_dma_ch].clr.val = UINT32_MAX;
    BITSCRAMBLER.state[BITSCRAMBLER_DIR_TX].val = 1u << 31;
    return ESP_OK;
}

esp_err_t transport_start(void)
{
    memset(s_raw_ring, 0, sizeof(s_raw_ring));
    sync_dma_c2m(s_raw_ring, sizeof(s_raw_ring));
    esp_err_t err = prepare_rx();
    if (err == ESP_OK) err = prepare_tx();
    if (err != ESP_OK) {
        (void)quiesce(NULL);
        s_transport_fault = true;
        return err;
    }
#ifdef C5VRX_LCD
    if (s_dac_bits == 6u) route_dac_lsb(true);
#endif
    return resume_live(NULL) ? ESP_OK : ESP_FAIL;
}

/* Live -> menu: the raster takes TX, but RX keeps collecting IQ for filter,
 * DC and profile calibration. Full recovery quiesce stops RX as well. */
void transport_stop_tx(void)
{
    bool ok = tr_stop_parts(NULL, stop_part, &s_tx_enabled, &s_bs_enabled, &s_rx_enabled, false);
    s_started = false;
    if (!ok) {
        s_transport_fault = true;
        printf("TRANSPORT ERROR: teardown failed; live recovery latched OFF, USB remains available\n");
    }
}

/* Start a standalone descriptor chain (menu raster) on the TX GDMA. */
void transport_start_chain(dma_descriptor_t *first)
{
    if (s_transport_fault || enable_tx() != ESP_OK) return;
    s_tx_dma_ch = find_dma(false);
    if (s_tx_dma_ch < 0) { (void)disable_tx(); s_transport_fault = true; return; }
    AHB_DMA.out_intr[s_tx_dma_ch].ena.val = 0;
    AHB_DMA.out_intr[s_tx_dma_ch].clr.val = UINT32_MAX;
    PARL_IO.int_ena.val = 0;
    parlio_ll_tx_enable_clock(&PARL_IO, false);
    parlio_ll_tx_reset_clock(&PARL_IO);
    parlio_ll_tx_reset_fifo(&PARL_IO);
    parlio_ll_tx_set_idle_data_value(&PARL_IO, DAC_IDLE_CODE);
    parlio_ll_tx_set_eof_condition(&PARL_IO, PARLIO_LL_TX_EOF_COND_DATA_LEN);
    parlio_ll_tx_set_trans_bit_len(&PARL_IO, 1);
    __asm__ __volatile__("fence rw, rw" ::: "memory");
    AHB_DMA.out_link_addr[s_tx_dma_ch].val = (uint32_t)first;
    AHB_DMA.channel[s_tx_dma_ch].out.out_link.outlink_start_chn = 1;
    /* Wait up to 50 ms, yielding; PARLIO outputs the idle code meanwhile. */
    int64_t deadline = esp_timer_get_time() + 50000;
    while (!parlio_ll_tx_is_ready(&PARL_IO) && esp_timer_get_time() < deadline) vTaskDelay(1);
    if (!parlio_ll_tx_is_ready(&PARL_IO)) {
        (void)disable_tx(); s_transport_fault = true;
        printf("TRANSPORT ERROR: menu TX timeout, USB remains available\n");
        return;
    }
    parlio_ll_tx_start(&PARL_IO, true);
    parlio_ll_tx_enable_clock(&PARL_IO, true);
}

void transport_stop_chain(void)
{
    if (s_tx_enabled && disable_tx() != ESP_OK) s_transport_fault = true;
}

/* Menu -> live: the exact boot topology again. */
static bool resume_live(const uint16_t *restore_lut)
{
    if (s_transport_fault) return false;
    stage_hooks();
    restart_t r = {.program = tracker_program(), .output = s_output};
    const uint8_t *program_inst;
    if (!program_words(r.program, &program_inst)) return false;
    memcpy(s_program_work.prefix, r.program, sizeof(s_program_work.prefix));
    if (!restore_lut) restore_lut = demod_table(s_demod);
    if (restore_lut == s_lut_live) {
        memcpy(s_lut_work, s_lut_live, sizeof(s_lut_work));
        r.clipped = s_output_clipped;
    } else {
        const uint8_t *inst;
        if (!program_words(r.program, &inst) || r.program[6] != 1u) return false;
        for (unsigned i = 0; i < 1024u; ++i) {
            bool clipped;
            s_lut_work[i] = output_entry(restore_lut[i], s_output, &clipped);
            r.clipped += clipped;
        }
    }
    tr_result_t result = tr_restart(&r, quiesce, restart_stage, s_have_verified);
    if (result != TR_APPLIED) ++s_restart_errors;
    if (result == TR_LATCHED) {
        ++s_rollback_fail;
        s_transport_fault = true;
        s_lut_known = false;
        printf("TRANSPORT ERROR: verified recovery failed; live OFF latched, USB/reflash available\n");
        return false;
    }
    s_started = true;
    if (result == TR_ROLLED_BACK) {
        ++s_rollbacks;
        printf("TRANSPORT rollback=verified program=%s output=%s\n",
               transport_program_name(), transport_output_name());
        return false;
    }
    memcpy(s_lut_live, s_lut_work, sizeof(s_lut_live));
    const bool output_changed = s_have_verified && s_verified_output != r.output;
    s_verified_output = r.output;
    s_verified_demod = s_demod;
    s_output_clipped = r.clipped;
    s_have_verified = true;
    s_lut_known = true;
    /* A failed restart or
     * rollback returns above and cannot replace the remembered output. */
    if (output_changed) settings_save_output(r.output);
    return true;
}

void transport_resume_live(void)
{
    if (!resume_live(NULL)) printf("TRANSPORT menu return failed; inspect T recovery status\n");
    transport_guard_mark(&s_switch_guard, esp_timer_get_time());
}

void transport_poll_faults(void)
{
    if (PARL_IO.int_raw.tx_fifo_rempty_int_raw) { ++s_tx_empty; PARL_IO.int_clr.tx_fifo_rempty_int_clr = 1; }
    if (PARL_IO.int_raw.rx_fifo_wovf_int_raw) { ++s_rx_overflow; PARL_IO.int_clr.rx_fifo_wovf_int_clr = 1; }
    if (s_rx_dma_ch >= 0) {
        uint32_t raw = AHB_DMA.in_intr[s_rx_dma_ch].raw.val & 0xfcu;
        if (raw) { ++s_dma_faults; AHB_DMA.in_intr[s_rx_dma_ch].clr.val = raw; }
    }
    if (s_tx_dma_ch >= 0) {
        uint32_t raw = AHB_DMA.out_intr[s_tx_dma_ch].raw.val & 0x7cu;
        if (raw) { ++s_dma_faults; AHB_DMA.out_intr[s_tx_dma_ch].clr.val = raw; }
    }
    if (BITSCRAMBLER.state[BITSCRAMBLER_DIR_TX].eof_overload) {
        ++s_bs_overload;
        BITSCRAMBLER.state[BITSCRAMBLER_DIR_TX].val = 1u << 31;
    }
}

uint32_t transport_fault_count(void)
{
    return s_tx_empty + s_rx_overflow + s_dma_faults + s_bs_overload;
}

void transport_flog_identity(uint8_t *demod, uint8_t *output_level)
{
    const bool verified = s_started && s_tx_enabled && s_bs_enabled && s_have_verified && s_lut_known &&
                          !s_transport_fault;
    if (demod) *demod = !verified ? FLOG_DEMOD_UNKNOWN :
                        s_verified_demod == TRANSPORT_DEMOD_U85Q ? FLOG_DEMOD_U85Q_1A09F5C092E8 : FLOG_DEMOD_P64_R1;
    if (output_level) *output_level = verified ? (uint8_t)s_verified_output : 0xffu;
}

/* Flight-log V1 output fields keep their wire slots. Without an output
 * observer only the verified identity is known; lock percentages read
 * 0xff (no fresh samples) and the counters stay zero. */
void transport_flog_output(transport_flog_output_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    transport_flog_identity(&out->demod, &out->output_level);
    out->flags = out->demod ? FLOG_OUT_DEMOD_VERIFIED : 0u;
    out->h_lock_pct = out->v_lock_pct = 0xffu;
}

void transport_print(void)
{
    const char *verified_demod = s_have_verified && s_lut_known && !s_transport_fault ?
        transport_demod_name(s_verified_demod) : "unknown";
    printf("DEMOD selected=%s verified=%s p64_model=%s p64_table_sha256=%s u85q_table_sha256=%s (console m switches)\n",
           transport_program_name(), verified_demod, P64_MODEL_ID, P64_TABLE_SHA256, U85Q_TABLE_SHA256);
    printf("TRANSPORT_STAGES_US quiesce=%lu bs_enable=%lu program_load=%lu instruction_verify=%lu lut_verify=%lu "
           "bs_reset=%lu bs_start=%lu rx_enable=%lu tx_enable=%lu rx_start=%lu tx_start=%lu (last restart)\n",
           (unsigned long)s_quiesce_us, (unsigned long)s_stage_us[TR_BS_ENABLE], (unsigned long)s_stage_us[TR_PROGRAM_LOAD],
           (unsigned long)s_stage_us[TR_INSTRUCTION_VERIFY], (unsigned long)s_stage_us[TR_LUT_VERIFY],
           (unsigned long)s_stage_us[TR_BS_RESET], (unsigned long)s_stage_us[TR_BS_START],
           (unsigned long)s_stage_us[TR_RX_ENABLE], (unsigned long)s_stage_us[TR_TX_ENABLE],
           (unsigned long)s_stage_us[TR_RX_START], (unsigned long)s_stage_us[TR_TX_START]);
    printf("VIDEO_OUTPUT source=%s standard=%s\n", s_menu_active ? "MENU" : s_started ? "LIVE" : "OFF",
           s_video_std == VIDEO_STD_PAL ? "PAL" : "NTSC");
    printf("OUTPUT level=%s boot_restore=saved default=1.0x clipped_lut_entries=%u recovery_errors=%lu rollbacks=%lu "
           "rollback_fail=%lu fault_latched=%u live=%u\n", transport_output_name(), s_output_clipped,
           (unsigned long)s_restart_errors, (unsigned long)s_rollbacks, (unsigned long)s_rollback_fail,
           s_transport_fault, s_started);
    printf("TRANSPORT rx_descriptors=%d tx_empty=%lu rx_overflow=%lu dma_faults=%lu bs_overload=%lu "
           "lut_writes=%lu lut_verify_fail=%lu program=%s switch_wait_ms=%lu prog_verify_fail=%lu\n",
           s_rx_node_count, (unsigned long)s_tx_empty, (unsigned long)s_rx_overflow,
           (unsigned long)s_dma_faults, (unsigned long)s_bs_overload,
           (unsigned long)s_lut_writes, (unsigned long)s_lut_verify_fail,
           transport_program_name(),
           (unsigned long)transport_switch_wait_ms(), (unsigned long)s_prog_verify_fail);
    printf("CONTROL_COPY calls=%lu not_ready=%lu source_invalid=%lu pointer_invalid=%lu "
           "lapped=%lu timed_out=%lu max_us=%lu max_advance=%lu\n",
           (unsigned long)s_control_copy.calls, (unsigned long)s_control_copy.not_ready,
           (unsigned long)s_control_copy.source_invalid, (unsigned long)s_control_copy.pointer_invalid,
           (unsigned long)s_control_copy.lapped, (unsigned long)s_control_copy.timed_out,
           (unsigned long)s_control_copy.max_us, (unsigned long)s_control_copy.max_advance);
}

int ring_node_index(uint32_t addr)
{
    for (int k = 0; k < s_rx_node_count; ++k)
        if ((uintptr_t)s_rx_nodes[k].dscr == addr) return k;
    return -1;
}

static uint32_t rx_active_addr(void) { return AHB_DMA.channel[s_rx_dma_ch].in.in_dscr_bf0.val; }

/* Software readers of the whole ring (the LCD frame grabber, grab.c): the
 * ring offset of the descriptor RX is writing now; every byte before it
 * (cyclically) is complete. False while RX is stopped or restarting.
 * in_dscr_bf0 runs one descriptor ahead of the data: it names node N+1 from
 * the moment the GDMA starts writing node N (marker probe, Waveshare C5 LCD
 * board 2026-10-30: the register switched within 3 us of node N's first
 * byte landing, every time). Rows read up to that register showed the
 * previous lap's samples, black sync streaks on the LCD. */
bool transport_rx_write_offset(uint32_t *offset)
{
    if (s_rx_dma_ch < 0 || s_rx_node_count < 3) return false;
    int k = ring_node_index(rx_active_addr());
    if (k < 0) return false;
    k = (k + s_rx_node_count - 1) % s_rx_node_count;
    if (!s_rx_nodes[k].buffer) return false;
    *offset = (uint32_t)(s_rx_nodes[k].buffer - s_raw_ring);
    return true;
}

/* How far the GDMA may be ahead of that offset. */
uint32_t transport_rx_max_descriptor(void)
{
    uint32_t m = 0;
    for (int k = 0; k < s_rx_node_count; ++k)
        if (s_rx_nodes[k].length > m) m = s_rx_nodes[k].length;
    return m ? m : CONTROL_BYTES;
}

static bool completed_plan(int active, size_t need, control_copy_plan_t *out)
{
    if (active < 0 || s_rx_node_count > MAX_RING_DESCRIPTORS) return false;
    size_t lengths[MAX_RING_DESCRIPTORS];
    for (int k = 0; k < s_rx_node_count; ++k) lengths[k] = s_rx_nodes[k].length;
    return control_copy_plan(lengths, (unsigned)s_rx_node_count, (unsigned)active, need, out);
}


/* The newest completed descriptor, copied and proven not lapped. */
bool copy_control_window(uint8_t *dst, size_t bytes)
{
    ++s_control_copy.calls;
    if (s_rx_dma_ch < 0 || s_rx_node_count < 3) { ++s_control_copy.not_ready; return false; }
    /* The control task (priority 3) was preempted mid-copy by the gain
     * observer (same priority, up to ~0.5 ms per window) and the sentinel
     * (4) often enough that 30-44 % of control windows failed the overrun
     * check on the board (T LADDER copy_fail, 2026-10-13). The copy itself
     * takes tens of microseconds: hold the scheduler for it. Interrupts
     * (GDMA, timers) still run. */
    vTaskSuspendAll();
    int64_t start = esp_timer_get_time();
    int active = ring_node_index(rx_active_addr());
    if (active < 0) {
        ++s_control_copy.pointer_invalid;
        (void)xTaskResumeAll();
        return false;
    }
    control_copy_plan_t plan;
    if (!completed_plan(active, bytes, &plan)) {
        ++s_control_copy.source_invalid;
        (void)xTaskResumeAll();
        return false;
    }
    uint8_t *src = s_rx_nodes[plan.index].buffer;
    if (!src || src < s_raw_ring ||
        src + bytes > s_raw_ring + sizeof(s_raw_ring)) {
        ++s_control_copy.source_invalid;
        (void)xTaskResumeAll();
        return false;
    }
    sync_dma_m2c(src, bytes);
    memcpy(dst, src, bytes);
    int after = ring_node_index(rx_active_addr());
    const int64_t took = esp_timer_get_time() - start;
    (void)xTaskResumeAll();
    if ((uint64_t)took > s_control_copy.max_us) s_control_copy.max_us = (uint32_t)took;
    int advance = after < 0 ? s_rx_node_count : (after - active + s_rx_node_count) % s_rx_node_count;
    if (after < 0) { ++s_control_copy.pointer_invalid; return false; }
    if ((uint32_t)advance > s_control_copy.max_advance) s_control_copy.max_advance = (uint32_t)advance;
    if ((unsigned)advance >= plan.distance) { ++s_control_copy.lapped; return false; }
    if (!control_copy_current(&plan, (unsigned)s_rx_node_count, (unsigned)active, (unsigned)after,
                              (uint64_t)took, IQ_RATE_HZ)) { ++s_control_copy.timed_out; return false; }
    return true;
}

/* Four separated 64-byte regions of the newest completed descriptor. */
bool copy_probe_window(uint8_t dst[PROBE_BYTES], int *node)
{
    static const size_t offset[PROBE_REGIONS] = {512u, 1536u, 2560u, 4028u};
    if (s_rx_dma_ch < 0 || s_rx_node_count < 2) return false;
    int64_t start = esp_timer_get_time();
    int active = ring_node_index(rx_active_addr());
    if (active < 0) return false;
    control_copy_plan_t plan;
    if (!completed_plan(active, CONTROL_BYTES, &plan)) return false;
    uint8_t *src = s_rx_nodes[plan.index].buffer;
    if (!src || src < s_raw_ring ||
        src + CONTROL_BYTES > s_raw_ring + sizeof(s_raw_ring)) return false;
    for (unsigned r = 0; r < PROBE_REGIONS; ++r) {
        sync_dma_m2c(src + offset[r], PROBE_REGION_BYTES);
        memcpy(dst + r * PROBE_REGION_BYTES, src + offset[r], PROBE_REGION_BYTES);
    }
    const uint64_t took = (uint64_t)(esp_timer_get_time() - start);
    const int after = ring_node_index(rx_active_addr());
    if (took > 300 || after < 0 || !control_copy_current(&plan, (unsigned)s_rx_node_count,
        (unsigned)active, (unsigned)after, took, IQ_RATE_HZ)) return false;
    if (node) *node = (int)plan.index;
    return true;
}

bool copy_sentinel_window(uint8_t dst[64])
{
    if (s_rx_dma_ch < 0 || s_rx_node_count < 3) return false;
    const int64_t start = esp_timer_get_time();
    const int active = ring_node_index(rx_active_addr());
    control_copy_plan_t plan;
    if (!completed_plan(active, 1984u + 64u, &plan)) return false;
    uint8_t *src = s_rx_nodes[plan.index].buffer;
    if (!src || src < s_raw_ring || src + 2048u > s_raw_ring + sizeof(s_raw_ring)) return false;
    sync_dma_m2c(src + 1984u, 64u);
    memcpy(dst, src + 1984u, 64u);
    const int after = ring_node_index(rx_active_addr());
    return after >= 0 && control_copy_current(&plan, (unsigned)s_rx_node_count,
        (unsigned)active, (unsigned)after, (uint64_t)(esp_timer_get_time() - start), IQ_RATE_HZ);
}

/* 8190 contiguous bytes (204.75 us) ending where the active descriptor
 * began, valid only if RX could not have reached them during the copy. */
bool copy_snapshot(uint8_t *dst)
{
    if (s_rx_dma_ch < 0 || s_rx_node_count < 4) return false;
    int64_t start = esp_timer_get_time();
    int active = ring_node_index(rx_active_addr());
    if (active < 0) return false;
    uintptr_t base = (uintptr_t)s_raw_ring, first = (uintptr_t)s_rx_nodes[0].buffer;
    if (first < base || first - base >= sizeof(s_raw_ring)) return false;
    size_t total = 0;
    for (int k = 0; k < s_rx_node_count; ++k) {
        uintptr_t a = (uintptr_t)s_rx_nodes[k].buffer;
        if (a < base || !c5v4_snapshot_segment(sizeof(s_raw_ring), first - base, total, a - base,
                                               s_rx_nodes[k].length)) return false;
        total += s_rx_nodes[k].length;
    }
    if (total != sizeof(s_raw_ring)) return false;
    size_t end = (size_t)(s_rx_nodes[active].buffer - s_raw_ring);
    c5v4_snapshot_plan_t plan;
    if (!c5v4_snapshot_plan(total, end, s_rx_nodes[active].length, C5V4_LEVEL_SAMPLE_BYTES, &plan)) return false;
    sync_dma_m2c(s_raw_ring + plan.offset, plan.first);
    memcpy(dst, s_raw_ring + plan.offset, plan.first);
    size_t rest = C5V4_LEVEL_SAMPLE_BYTES - plan.first;
    if (rest) { sync_dma_m2c(s_raw_ring, rest); memcpy(dst + plan.first, s_raw_ring, rest); }
    uint64_t elapsed = (uint64_t)(esp_timer_get_time() - start);
    int later = ring_node_index(rx_active_addr());
    if (later < 0) return false;
    size_t after = (size_t)(s_rx_nodes[later].buffer - s_raw_ring);
    return c5v4_snapshot_current(total, C5V4_LEVEL_SAMPLE_BYTES, (after + total - end) % total,
                                 s_rx_nodes[later].length, elapsed, plan.safe_bytes, IQ_RATE_HZ);
}

/* Endpoint bytes (odd ring byte of each pair) of the newest `n` pairs. */
bool copy_recent_endpoints(uint8_t *dst, size_t n)
{
    if (s_rx_dma_ch < 0 || s_rx_node_count < 2 || 2u * n > sizeof(s_raw_ring) / 2u) return false;
    int idx = ring_node_index(rx_active_addr());
    if (idx < 0) return false;
    uint8_t *buf = s_rx_nodes[idx].buffer;
    if (buf < s_raw_ring || buf >= s_raw_ring + sizeof(s_raw_ring)) return false;
    size_t end = (size_t)(buf - s_raw_ring) & ~(size_t)1u;
    size_t pos = (end + sizeof(s_raw_ring) - 2u * n) % sizeof(s_raw_ring);
    sync_dma_m2c(s_raw_ring, sizeof(s_raw_ring));
    for (size_t k = 0; k < n; ++k) {
        dst[k] = s_raw_ring[pos + 1u];
        pos = (pos + 2u) % sizeof(s_raw_ring);
    }
    return true;
}

/* Sampling phase: holding the RX divider one count higher for ~1 us moves
 * the RX edge against the MODEM_DIAG update by a few 4.17 ns ticks
 * (zerowidth PR #3 does this on its sample clock). */
void rx_clock_slip(uint32_t us)
{
    phy_observation_changed();
    static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&mux);
    uint32_t div = PCR.parl_clk_rx_conf.parl_clk_rx_div_num;
    HAL_FORCE_MODIFY_U32_REG_FIELD(PCR.parl_clk_rx_conf, parl_clk_rx_div_num, div + 1u);
    esp_rom_delay_us(us);
    HAL_FORCE_MODIFY_U32_REG_FIELD(PCR.parl_clk_rx_conf, parl_clk_rx_div_num, div);
    portEXIT_CRITICAL(&mux);
}

metrics_t measure_window(const uint8_t *s, size_t n)
{
    metrics_t m = {0};
    uint16_t hist[129] = {0};
    int clip = 0, origin = 0, coherent = 0, pi = 0, pq = 0;
    for (size_t k = 0; k < n; ++k) {
        int q = (int8_t)((s[k] & 0x0fu) << 4) >> 4, i = (int8_t)(s[k] & 0xf0u) >> 4;
        if (i == -8 || i == 7 || q == -8 || q == 7) ++clip;
        int p = i * i + q * q;
        if (p <= 4) ++origin;
        if (k) {
            int dot = i * pi + q * pq, cross = q * pi - i * pq;
            if (p >= 8 && dot > 0 && (cross < 0 ? -cross : cross) <= dot) ++coherent;
        }
        ++hist[p > 128 ? 128 : p];
        pi = i; pq = q;
    }
    unsigned cumulative = 0;
    for (unsigned p = 0; p <= 128u; ++p) {
        cumulative += hist[p];
        if (cumulative >= (n + 1u) / 2u) { m.p_median = (int)p; break; }
    }
    m.q_phase = n > 1u ? coherent * 100 / (int)(n - 1u) : 0;
    m.clip_pm = n ? clip * 1000 / (int)n : 0;
    m.origin_pm = n ? origin * 1000 / (int)n : 1000;
    return m;
}
