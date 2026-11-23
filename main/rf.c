/* C5VRX-5 by Twotoz and the C5VRX contributors: receive-only RF front end.
 * From legacy/c5vrx4/main/rf.c (C5VRX-2 wifi5.c lineage), native-AGC, lab and
 * adaptive-lane paths removed. */
#include "rf.h"
#include "phy.h"
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "experimental.h"
#include "soc/gpio_sig_map.h"
#ifdef HWX_MENU_ON_IQ
#include "hal/parlio_periph.h"
#endif
#include "soc/lp_aon_reg.h"

#define REG32(a)         (*(volatile uint32_t *)(uintptr_t)(a))
#define MAC_TXQ0_CONF    0x600a4d6cu
#define MAC_TXQ_STRIDE   0x10u
#define MAC_TXQ_ENABLE   0x80000000u
#define MAC_TXQ_COUNT    5u
/* Continuous modem front end: keeps the ADC streaming into MODEM_DIAG
 * without 802.11 packets (C5VRX-2 autonomous dump-first writer). */
#define DUMP_CTRL       0x600a9004u
#define DUMP_PTR_MODE   0x600a9008u
#define DUMP_FORMAT     0x600a9018u
#define FE_PATH         0x600a20b4u
#define FE_ENABLE       0x600a0800u
#define SOURCE_CTRL     0x600a08ccu
#define SOURCE_MUX      0x600a70b8u
#define MODEM_CLOCK     0x600a9c04u
#define CTRL_ENABLE     0x80000000u
#define CTRL_DUMP_FIRST 0x00020000u
#define TX_START_SELECT 0x00060000u
#define SELECTOR_MASK   0x01fe0000u
#define HP_SRAM_USAGE   0x60095004u

static const char *TAG = "c5vrx5_rf";

/* PARLIO RX data pins, in data-bit order: Q on bits 0..3, I on 4..7. */
static const gpio_num_t s_iq_pins[8] = RF_IQ_GPIO_NUMS;
/* Ultrafine lane: sign bit 9 plus ADC bits 6..4, i.e. 16-code steps in a
 * +-128 window (+12 dB over the coarse {9,8,7,6} slice, an exact
 * power-of-two scale inside the window). Q on DIAG[4,5,6,9], I on
 * DIAG[14,15,16,19] (bit-exact against the RF dump, PR #122). Fixed for
 * the session: Direct Gain V5 brings a folding carrier back into the
 * window with analog gain (operator default since 2026-10-08). */
static const uint8_t s_lane_diag[RF_LANES][8] = {
    {6u, 7u, 8u, 9u, 16u, 17u, 18u, 19u},   /* coarse    {9,8,7,6} */
    {5u, 6u, 7u, 9u, 15u, 16u, 17u, 19u},   /* fine      {9,7,6,5} */
    {4u, 5u, 6u, 9u, 14u, 15u, 16u, 19u},   /* ultrafine {9,6,5,4} */
};
static const char *const s_lane_names[RF_LANES] = {"coarse", "fine", "ultrafine"};
static unsigned s_lane = RF_LANE_ULTRAFINE;

extern int lmac_stop_hw_txq(void);
extern void phy_wifi_fbw_sel(uint32_t val);
extern void phy_force_rx_gain(bool enable, uint8_t gain_idx);
extern void phy_disable_agc(void);
extern void phy_rfagc_disable(void);
extern void phy_set_freq(uint16_t freq_mhz, int offset);
extern void phy_chip_set_chan_offset(int offset_khz);

static arc_gain_table_t s_table;
static uint32_t s_table_generation;
static uint8_t s_gain = 52u;
static void (*volatile s_post_gain_hook)(uint8_t gain);
static rf_filter_t s_filter = {PHY_FILTER_CALIBRATED, 0, false};

static esp_err_t lock_rx_only(void)
{
    (void)lmac_stop_hw_txq();
    for (unsigned q = 0; q < MAC_TXQ_COUNT; ++q)
        REG32(MAC_TXQ0_CONF - q * MAC_TXQ_STRIDE) &= ~MAC_TXQ_ENABLE;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    for (unsigned q = 0; q < MAC_TXQ_COUNT; ++q)
        if (REG32(MAC_TXQ0_CONF - q * MAC_TXQ_STRIDE) & MAC_TXQ_ENABLE) return ESP_ERR_INVALID_STATE;
    return ESP_OK;
}

static esp_err_t route_modem_iq(unsigned lane)
{
    uint64_t mask = 0;
    for (unsigned k = 0; k < 8; ++k) mask |= 1ULL << s_iq_pins[k];
    const gpio_config_t cfg = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) return err;
    for (unsigned k = 0; k < 8; ++k)
        esp_rom_gpio_connect_out_signal(s_iq_pins[k], MODEM_DIAG0_IDX + s_lane_diag[lane][k], false, false);
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    s_lane = lane;
    return ESP_OK;
}

esp_err_t rf_set_lane(unsigned lane)
{
    if (lane >= RF_LANES) return ESP_ERR_INVALID_ARG;
    phy_begin();
    esp_err_t result=route_modem_iq(lane);
    phy_end();
    return result;
}

/* Live route change (STRONG LANE, receiver.c): the eight GPIO-matrix output
 * selects only, on pins route_modem_iq configured at boot. No PHY register;
 * the caller invalidates the observations made on the old lane. */
void rf_reroute_lane(unsigned lane)
{
    if (lane >= RF_LANES) return;
    for (unsigned k = 0; k < 8; ++k)
        esp_rom_gpio_connect_out_signal(s_iq_pins[k], MODEM_DIAG0_IDX + s_lane_diag[lane][k], false, false);
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    s_lane = lane;
}

unsigned rf_lane(void) { return s_lane; }

#ifdef HWX_MENU_ON_IQ
/* GPIO matrix output selects only; the pads stay configured by
 * route_modem_iq. PARLIO TX data bit k -> IQ pad k. */
void rf_hwx_pads_tx(bool tx)
{
    for (unsigned k = 0; k < 8; ++k)
        esp_rom_gpio_connect_out_signal(s_iq_pins[k], tx ? soc_parlio_signals[0].tx_units[0].data_sigs[k]
                                                         : MODEM_DIAG0_IDX + s_lane_diag[s_lane][k], false, false);
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
}
#endif
const char *rf_lane_name(unsigned lane) { return lane < RF_LANES ? s_lane_names[lane] : "?"; }

static void enable_continuous_modem(void)
{
    REG32(HP_SRAM_USAGE) = (REG32(HP_SRAM_USAGE) & 0xfffef0ffu) | 0x00010000u;
    REG32(SOURCE_CTRL) &= 0xff87ffffu;
    REG32(SOURCE_MUX) = (REG32(SOURCE_MUX) & 0xfffffff8u) | 1u;
    REG32(MODEM_CLOCK) = UINT32_MAX;
    REG32(FE_ENABLE) |= 4u;
    REG32(FE_PATH) &= ~1u;
    uint32_t v = REG32(DUMP_FORMAT);
    v = (v & 0xff03ffffu) | 0x006c0000u;
    REG32(DUMP_FORMAT) = v;
    v = (REG32(DUMP_FORMAT) & 0xfffc0fffu) | 0x0001a000u;
    REG32(DUMP_FORMAT) = v;
    v = (REG32(DUMP_FORMAT) & 0xfffff03fu) | 0x00000640u;
    REG32(DUMP_FORMAT) = v;
    v = (REG32(DUMP_FORMAT) & 0xffffffc0u) | 0x18u;
    REG32(DUMP_FORMAT) = v | 0x01000000u;
    /* Pre-trigger circular mode on TX_START: the MAC queues are disabled,
     * so it never fires and the writer streams continuously. */
    REG32(DUMP_PTR_MODE) = (REG32(DUMP_PTR_MODE) & ~SELECTOR_MASK) | TX_START_SELECT;
    uint32_t ctrl = REG32(DUMP_CTRL);
    ctrl &= ~(CTRL_ENABLE | 0x00080000u | 0x00040000u);
    ctrl |= CTRL_DUMP_FIRST;
    ctrl = (ctrl & ~0x0001ffffu) | 16384u;
    REG32(DUMP_CTRL) = ctrl;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    REG32(DUMP_CTRL) = ctrl | CTRL_ENABLE;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
}

static void apply_filter(void)
{
    if (!phy_filter_set_code(s_filter.code) && s_filter.code != PHY_FILTER_CALIBRATED)
        ESP_EARLY_LOGW(TAG, "filter code %d not applied", s_filter.code);
    /* Written even at 0: leaving a skirt must undo it. */
    if (!phy_filter_set_skirt(s_filter.skirt) && s_filter.skirt)
        ESP_EARLY_LOGW(TAG, "filter skirt %u not applied", s_filter.skirt);
}

/* Every tuning/filter transaction restores the same receive state: vendor
 * AGC off, forced gain, 11p baseline, analog profile, digital filter, DC hold. */
static void restore_lock(void)
{
    experimental_phy_prepare_restore();
    (void)phy_dco_release();          /* the restore must replay vendor rows */
    enable_continuous_modem();
    ESP_ERROR_CHECK(lock_rx_only());
    phy_disable_agc();
    phy_rfagc_disable();
    phy_wifi_fbw_sel(s_filter.bw20 ? 0u : 1u);
    phy_force_rx_gain(true, s_gain);
    const esp_err_t p11 = phy_11p_apply_default();
    if (p11 == ESP_ERR_NOT_SUPPORTED)
        ESP_EARLY_LOGW(TAG, "11p baseline not applied (unsupported PHY)");
    else
        ESP_ERROR_CHECK(p11);        /* never resume after failed entry/rollback */
    apply_filter();
    if (!phy_dfilt_apply_default())
        ESP_EARLY_LOGW(TAG, "digital RX filter mode not applied (unsupported PHY or readback mismatch)");
    (void)experimental_phy_restore();
    if (s_post_gain_hook) s_post_gain_hook(s_gain);
}

/* The reconstructed table against the PHY's AGC memory (T GAIN_TABLE_HW).
 * The window is mapped as vendor apb_get_mem() does it: MEM_CONF and the
 * SYSCON test config cleared, every modem clock forced on, the AGC memory
 * selected, then all restored; 64 words per critical section. */
#define LPCON_MEM_CONF    0x600af02cu
#define LPCON_APB_MEM_SEL 0x600af038u
#define APB_SEL_AGC       4u
#define SYSCON_TEST_CONF  0x600a9c00u
#define SYSCON_CLK_CONF1  0x600a9c14u
#define APB_MEM_WINDOW    0x600ae000u
#define AGC_CHECK_WORDS   ((ARC_5G_GAIN_MAX + 1u) * ARC_AGC_ENTRY_WORDS)
static unsigned s_hw_checked, s_hw_match;
static int s_hw_first_bad = -1;
static portMUX_TYPE s_agc_window_lock = portMUX_INITIALIZER_UNLOCKED;

/* Words start..start+n-1 of the window into words[0..n-1]. */
static void read_agc_range(uint32_t *words, unsigned start, unsigned n)
{
    for (unsigned first = start; first < start + n; first += 64u) {
        const unsigned count = start + n - first < 64u ? start + n - first : 64u;
        taskENTER_CRITICAL(&s_agc_window_lock);
        const uint32_t mem_conf = REG32(LPCON_MEM_CONF), test_conf = REG32(SYSCON_TEST_CONF);
        const uint32_t clk_conf1 = REG32(SYSCON_CLK_CONF1), sel = REG32(LPCON_APB_MEM_SEL);
        REG32(LPCON_MEM_CONF) = 0;
        REG32(SYSCON_TEST_CONF) = 0;
        REG32(SYSCON_CLK_CONF1) = UINT32_MAX;
        REG32(LPCON_APB_MEM_SEL) = APB_SEL_AGC;
        __asm__ __volatile__("fence iorw, iorw" ::: "memory");
        for (unsigned k = 0; k < count; ++k) words[first - start + k] = REG32(APB_MEM_WINDOW + 4u * (first + k));
        REG32(LPCON_APB_MEM_SEL) = sel;
        REG32(LPCON_MEM_CONF) = mem_conf;
        REG32(SYSCON_TEST_CONF) = test_conf;
        REG32(SYSCON_CLK_CONF1) = clk_conf1;
        __asm__ __volatile__("fence iorw, iorw" ::: "memory");
        taskEXIT_CRITICAL(&s_agc_window_lock);
    }
}

static void read_agc_memory(uint32_t *words, unsigned n) { read_agc_range(words, 0, n); }

static void check_table_against_hw(void)
{
    static uint32_t words[AGC_CHECK_WORDS];
    if (!s_table.band5) { s_hw_checked = 0; return; }   /* the AGC layout is only confirmed for 5 GHz */
    read_agc_memory(words, AGC_CHECK_WORDS);
    s_hw_checked = s_table.max_index + 1u;
    s_hw_match = arc_gain_table_check_agc(&s_table, words, AGC_CHECK_WORDS, &s_hw_first_bad);
    if (s_hw_match != s_hw_checked)
        printf("GAIN_TABLE_HW MISMATCH rows=%u match=%u first_bad=G%d (the reconstructed gain table differs from "
               "the PHY's AGC memory: check the esp-phy-lib version)\n", s_hw_checked, s_hw_match, s_hw_first_bad);
}

void rf_gain_table_hw(unsigned *checked, unsigned *match, int *first_bad)
{
    *checked = s_hw_checked;
    *match = s_hw_match;
    *first_bad = s_hw_first_bad;
}

/* FINDINGS 2026-10-27: for one whole session bank 1 of every top-stage row
 * read 245/224, against 279/227 on every boot, while G48/G42 were normal:
 * one source entry changed. The table is written by the boot RX DC
 * calibration only and reloaded into the AGC memory by every channel setup
 * (Ghidra, pinned libphy), so a change shows up in one of these layers.
 * RAM: a hash of the table and one AGC row at a time (the standard image
 * has about 2 KB of boot heap to spare). */
static uint32_t s_vbank1_hash, s_bank1_sig, s_bank1_events;

void rf_bank1_check(bool print)
{
    /* capture_table() already owns the recursive PHY lock; console polls
     * must share it so a retune cannot split this snapshot or its state. */
    phy_read_begin();
    unsigned grp;
    const int16_t *tab = phy_vendor_bank1_table(&grp);
    if (!tab || !s_table.band5) {
        phy_read_end();
        if (print) printf("BANK1_CHECK supported=0\n");
        return;
    }
    uint32_t hash = 2166136261u;
    for (unsigned k = 0; k < PHY_VBANK1_GROUPS * PHY_VBANK1_STAGES * 2u; ++k)
        hash = (hash ^ (uint16_t)tab[k]) * 16777619u;
    if (!s_vbank1_hash) s_vbank1_hash = hash;
    const unsigned tab_changed = hash != s_vbank1_hash;
    unsigned mem_bad = 0;
    int mem_first = -1, got[2] = {-1, -1}, want[2] = {-1, -1}, top[2] = {-1, -1}, b2[2];
    for (unsigned g = 0; g <= s_table.max_index; ++g) {
        arc_gain_tuple_t t;
        if (!arc_gain_tuple_decode(&s_table, (uint8_t)g, &t)) continue;
        uint32_t words[ARC_AGC_ENTRY_WORDS];
        read_agc_range(words, g * ARC_AGC_ENTRY_WORDS, ARC_AGC_ENTRY_WORDS);
        int b1[2];
        arc_gain_entry_dc(words, b1, b2);
        const int16_t *w = &tab[(grp * PHY_VBANK1_STAGES + t.rf_stage) * 2u];
        if (g == s_table.max_index) { top[0] = b1[0]; top[1] = b1[1]; }
        if ((b1[0] != w[0] || b1[1] != w[1]) && !mem_bad++) {
            mem_first = (int)g;
            got[0] = b1[0]; got[1] = b1[1]; want[0] = w[0]; want[1] = w[1];
        }
    }
    int live1[2], live2[2];
    const bool live = phy_dco_banks(live1, live2);
    /* Anomalies only: a retune into another group legitimately moves top_row. */
    const uint32_t sig = 1u + tab_changed * 65536u + mem_bad;
    const bool changed = s_bank1_sig && sig != s_bank1_sig;
    s_bank1_events += changed;
    s_bank1_sig = sig;
    /* Release ownership before USB formatting/output. Keep only the current
     * group's nine pairs on the stack, never another full table in RAM. */
    int16_t table_now[PHY_VBANK1_STAGES * 2u];
    if (tab_changed && (print || changed))
        memcpy(table_now, &tab[grp * PHY_VBANK1_STAGES * 2u], sizeof(table_now));
    const unsigned rows = s_table.max_index + 1u, held = phy_dco_held();
    const uint32_t events = s_bank1_events;
    phy_read_end();
    if (!print && !changed) return;
    printf("BANK1_CHECK%s group=%u table_changed=%u", changed ? " CHANGED" : "", grp, tab_changed);
    if (tab_changed) {   /* the boot values are in the boot log's first BANK1_CHECK */
        printf(" table_now=");
        for (unsigned s = 0; s < PHY_VBANK1_STAGES; ++s)
            printf("%s%d/%d", s ? "," : "", table_now[s * 2u], table_now[s * 2u + 1u]);
    }
    printf(" mem_rows=%u mem_bad=%u", rows, mem_bad);
    if (mem_first >= 0) printf(" first_bad=G%d mem=%d/%d want=%d/%d", mem_first, got[0], got[1], want[0], want[1]);
    printf(" top_row=%d/%d live=%d/%d held=%u events=%lu\n", top[0], top[1], live ? live1[0] : -1, live ? live1[1] : -1,
           held, (unsigned long)events);
}

static void capture_table(void)
{
    arc_phy_capture_gain_table(&s_table);
    check_table_against_hw();
    rf_bank1_check(false);
    ++s_table_generation;
}

esp_err_t rf_start(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        err = nvs_flash_erase();
        if (err == ESP_OK) err = nvs_flash_init();
    }
    if (err != ESP_OK) return err;
    /* No esp_netif_init: no network interface exists, and its TCP/IP task
     * (3 KiB stack plus queues) took heap the menu needs. */
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    /* sta_disconnected_pm must be off: Wi-Fi power management would gate
     * the RF/PHY clocks that keep MODEM_DIAG streaming. */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.sta_disconnected_pm = false;
    if ((err = esp_wifi_init(&cfg)) != ESP_OK) return err;
    if ((err = esp_wifi_set_storage(WIFI_STORAGE_RAM)) != ESP_OK) return err;
    if ((err = esp_wifi_set_mode(WIFI_MODE_STA)) != ESP_OK) return err;
    if ((err = esp_wifi_start()) != ESP_OK) return err;
    if ((err = esp_wifi_set_band_mode(WIFI_BAND_MODE_5G_ONLY)) != ESP_OK) return err;
    if ((err = esp_wifi_set_ps(WIFI_PS_NONE)) != ESP_OK) return err;
    wifi_protocols_t protocols = {
        .ghz_2g = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX,
        .ghz_5g = WIFI_PROTOCOL_11A | WIFI_PROTOCOL_11N,
    };
    if ((err = esp_wifi_set_protocols(WIFI_IF_STA, &protocols)) != ESP_OK) return err;
    wifi_bandwidths_t bandwidths = {.ghz_2g = WIFI_BW20, .ghz_5g = WIFI_BW40};
    if ((err = esp_wifi_set_bandwidths(WIFI_IF_STA, &bandwidths)) != ESP_OK) return err;
    if ((err = esp_wifi_set_channel(173, WIFI_SECOND_CHAN_NONE)) != ESP_OK) return err;
    /* Promiscuous RX keeps the receive path alive; a zero filter keeps the
     * LMAC from buffering packets or raising interrupts. */
    if ((err = esp_wifi_set_promiscuous(true)) != ESP_OK) return err;
    wifi_promiscuous_filter_t filter = {.filter_mask = 0};
    (void)esp_wifi_set_promiscuous_filter(&filter);
    if ((err = lock_rx_only()) != ESP_OK) return err;
    experimental_load();
    if ((err = route_modem_iq(experimental_boot_lane())) != ESP_OK) return err;
    phy_begin();
    /* The filter capacitors still hold the per-chip calibration from PHY
     * init; capture them before anything else writes the filter. */
    (void)phy_filter_capture_base();
    restore_lock();
    capture_table();
    phy_end();
    return ESP_OK;
}

void rf_reboot_to_download(void)
{
    /* esptool's USB reset does not reset the modem domain; the forced clocks
     * and the running dump writer would keep the ROM loader from answering. */
    printf("C5VRX5 download_boot=requested\n");
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(50));
    REG32(DUMP_CTRL) &= ~CTRL_ENABLE;
    REG32(HP_SRAM_USAGE) &= ~0x00010f00u;
    (void)esp_wifi_stop();
    REG_SET_FIELD(LP_AON_SYS_CFG_REG, LP_AON_FORCE_DOWNLOAD_BOOT, 1u);
    esp_restart();
}

bool rf_set_gain(uint8_t gain, uint32_t expected_generation)
{
    if (!phy_try_actuator(expected_generation)) return false;
    /* Re-writing the gain in force while the DC pair is held changes
     * nothing but would leave debug mode for ~2 us: skip it (V5 used to
     * re-write the same gain 50-100 times a second; seen as rolling). */
    if (gain == s_gain && phy_dco_held()) { phy_end_actuator(); return true; }
    phy_observation_changed();
    experimental_phy_prepare_restore();
    (void)phy_dco_release();          /* a held hold keeps PBUS in debug mode */
    phy_force_rx_gain(true, gain);
    s_gain = gain;
    (void)experimental_phy_restore();
    if (s_post_gain_hook) s_post_gain_hook(gain);
    phy_end_actuator();
    return true;
}

void rf_force_gain(uint8_t gain)
{
    phy_begin();
    experimental_phy_prepare_restore();
    (void)phy_dco_release();
    phy_force_rx_gain(true, gain);
    s_gain = gain;
    (void)experimental_phy_restore();
    if (s_post_gain_hook) s_post_gain_hook(gain);
    phy_end();
}

void rf_set_post_gain_hook(void (*hook)(uint8_t gain)) { s_post_gain_hook = hook; }
const arc_gain_table_t *rf_gain_table(void) { return &s_table; }
uint32_t rf_table_generation(void) { return s_table_generation; }

/* The analog filter branch follows the PHY bandwidth mode at the last
 * channel setup, not the later phy_wifi_fbw_sel(): set up in BW20 the RX0
 * capacitor code sets the width and the second-stage skirt has no effect,
 * set up in BW40 it is the other way round (board 2026-10-19, same boot:
 * code 63 / skirt 0 11.9 vs 19.7 MHz noise bandwidth, code 16 / skirt 32
 * 21.1 vs 13.0 MHz; ESPARGOS esp-sdr: "individual digital register writes do
 * not update the PHY's PBUS analog-control tables"). channel_setup() selects
 * the mode first; a filter that changes the mode repeats the setup. */
static esp_err_t channel_setup(uint16_t mhz);
static bool s_setup_valid, s_setup_bw20;
static uint16_t s_freq;   /* tentative: defined with the channel table below */
static int s_offset_khz;

bool rf_set_filter(rf_filter_t filter)
{
    phy_begin();
    s_filter = filter;
    if (!s_setup_valid || s_setup_bw20 != filter.bw20) {
        const int offset = s_offset_khz;
        if (channel_setup(s_freq) == ESP_OK) {
            if (offset) phy_chip_set_chan_offset(offset);
            capture_table();   /* a setup may regenerate the vendor gain table */
        }
    }
    restore_lock();
    bool ok = rf_filter_applied();
    phy_end();
    return ok;
}

rf_filter_t rf_filter(void) { return s_filter; }

/* Requested versus read back (phy.c verifies every register write and
 * falls back to the 11p baseline bytes). Without the pinned PHY nothing can
 * be written, so only the chip's own calibration counts as applied. */
bool rf_filter_applied(void)
{
    const int digital = phy_dfilt_hw_mode();
    const int p11 = phy_11p_hw_enabled();
    return phy_filter_code() == s_filter.code && phy_filter_skirt() == (int)s_filter.skirt &&
           (digital < 0 || digital == phy_dfilt_mode()) && p11 != 0;
}

/* 6 bands x 8 channels. */
static const fpv_channel_t s_channels[FPV_BANDS][8] = {
    [FPV_BAND_R] = {{"R1", 5658}, {"R2", 5695}, {"R3", 5732}, {"R4", 5769},
                    {"R5", 5806}, {"R6", 5843}, {"R7", 5880}, {"R8", 5917}},
    [FPV_BAND_A] = {{"A1", 5865}, {"A2", 5845}, {"A3", 5825}, {"A4", 5805},
                    {"A5", 5785}, {"A6", 5765}, {"A7", 5745}, {"A8", 5725}},
    [FPV_BAND_B] = {{"B1", 5733}, {"B2", 5752}, {"B3", 5771}, {"B4", 5790},
                    {"B5", 5809}, {"B6", 5828}, {"B7", 5847}, {"B8", 5866}},
    [FPV_BAND_E] = {{"E1", 5705}, {"E2", 5685}, {"E3", 5665}, {"E4", 5645},
                    {"E5", 5885}, {"E6", 5905}, {"E7", 5925}, {"E8", 5945}},
    [FPV_BAND_F] = {{"F1", 5740}, {"F2", 5760}, {"F3", 5780}, {"F4", 5800},
                    {"F5", 5820}, {"F6", 5840}, {"F7", 5860}, {"F8", 5880}},
    [FPV_BAND_L] = {{"L1", 5362}, {"L2", 5399}, {"L3", 5436}, {"L4", 5473},
                    {"L5", 5510}, {"L6", 5547}, {"L7", 5584}, {"L8", 5621}},
};
static const char *const s_band_names[FPV_BANDS] = {
    "RACEBAND R", "BOSCAM A", "BOSCAM B", "BOSCAM E", "FATSHARK F", "LOWBAND L",
};
static size_t s_index = FPV_BAND_A * 8u;   /* A1, 5865 MHz */
static uint16_t s_freq = 5865u;
static int s_offset_khz;

const fpv_channel_t *rf_channel(void) { return &s_channels[s_index / 8u][s_index % 8u]; }
uint16_t rf_channel_freq(size_t index) { return index < FPV_CHANNELS ? s_channels[index / 8u][index % 8u].freq_mhz : 0; }
size_t rf_channel_index(void) { return s_index; }
const char *rf_band_name(unsigned band) { return band < FPV_BANDS ? s_band_names[band] : "?"; }
uint16_t rf_frequency_mhz(void) { return s_freq; }
int rf_offset_khz(void) { return s_offset_khz; }

/* Public 5 GHz centres. Off-grid FPV channels are first tuned to the
 * nearest one, then moved with phy_set_freq (zerowidth/C5VRX PR #3
 * decoded R8 and tuned E8 this way). */
#define C5_MIN_MHZ 5180u
#define C5_MAX_MHZ 5945u
static const struct { uint8_t channel; uint16_t mhz; } s_centres[] = {
    {132, 5660}, {136, 5680}, {140, 5700}, {144, 5720},
    {149, 5745}, {153, 5765}, {157, 5785}, {161, 5805},
    {165, 5825}, {169, 5845}, {173, 5865}, {177, 5885},
};

/* Under phy_begin(). The bandwidth mode goes first: the setup builds the
 * analog filter branch from it (see rf_set_filter). */
static esp_err_t channel_setup(uint16_t mhz)
{
    experimental_phy_prepare_restore();
    unsigned best = 0;
    for (unsigned k = 1; k < sizeof(s_centres) / sizeof(s_centres[0]); ++k)
        if (abs((int)mhz - (int)s_centres[k].mhz) < abs((int)mhz - (int)s_centres[best].mhz)) best = k;
    phy_wifi_fbw_sel(s_filter.bw20 ? 0u : 1u);
    esp_err_t err = esp_wifi_set_channel(s_centres[best].channel, WIFI_SECOND_CHAN_NONE);
    if (err == ESP_OK && mhz != s_centres[best].mhz) phy_set_freq(mhz, 0);
    if (err == ESP_OK) { s_setup_valid = true; s_setup_bw20 = s_filter.bw20; }
    return err;
}

esp_err_t rf_set_channel(size_t index)
{
    if (index >= FPV_CHANNELS) return ESP_ERR_INVALID_ARG;
    uint16_t mhz = s_channels[index / 8u][index % 8u].freq_mhz;
    if (mhz < C5_MIN_MHZ || mhz > C5_MAX_MHZ) return ESP_ERR_NOT_SUPPORTED;
    phy_begin();
    esp_err_t err = channel_setup(mhz);
    restore_lock();
    if (err == ESP_OK) {
        /* A retune may regenerate the vendor gain table. */
        capture_table();
        s_index = index;
        s_freq = mhz;
        s_offset_khz = 0;
    }
    phy_end();
    return err;
}

void rf_next_band(void)
{
    for (unsigned step = 1; step <= FPV_BANDS; ++step) {
        size_t next = ((s_index / 8u + step) % FPV_BANDS) * 8u + s_index % 8u;
        if (rf_set_channel(next) == ESP_OK) return;
    }
}

void rf_next_channel(void)
{
    size_t band = s_index / 8u;
    for (unsigned step = 1; step <= 8u; ++step) {
        size_t next = band * 8u + (s_index % 8u + step) % 8u;
        if (rf_set_channel(next) == ESP_OK) return;
    }
}

/* Clamped to +-2.5 MHz (was 1.5): cheap VTXs can be 2 MHz off frequency,
 * and a residual of -0.75 MHz already costs the picture at weak signal
 * (research/situations.py). FPV channels are >= 19 MHz apart, so the AFC
 * can never walk onto another channel. */
#define RF_OFFSET_MAX_KHZ 2500
void rf_set_offset_khz(int offset_khz)
{
    if (offset_khz < -RF_OFFSET_MAX_KHZ) offset_khz = -RF_OFFSET_MAX_KHZ;
    if (offset_khz > RF_OFFSET_MAX_KHZ) offset_khz = RF_OFFSET_MAX_KHZ;
    if (offset_khz == s_offset_khz) return;
    phy_begin();
    experimental_phy_prepare_restore();
    s_offset_khz = offset_khz;
    phy_chip_set_chan_offset(offset_khz);
    restore_lock();   /* the vendor helper re-enables BB AGC */
    phy_end();
}
