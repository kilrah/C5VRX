/* C5VRX-5 by Twotoz and the C5VRX contributors: USB serial console.
 * Polled, never paces the realtime path. */
#include "receiver.h"
#include "sense_lab.h"
#include <stdio.h>
#include <stdlib.h>
#include "esp_app_desc.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/queue.h"
#include "hal/usb_serial_jtag_ll.h"
#include "rf.h"
#include "snapshot.h"

extern volatile const char *s_boot_stage;

#ifndef C5VRX5_COMMIT
#define C5VRX5_COMMIT "unknown"
#endif

/* "<version>+<commit 12>": PR channels reuse one version string, so the
 * commit is what tells two pr-N builds apart in a saved log. */
const char *firmware_id(void)
{
    static char id[64];
    if (!id[0]) snprintf(id, sizeof(id), "%s+%.12s", esp_app_get_description()->version, C5VRX5_COMMIT);
    return id;
}

void firmware_print(void)
{
    const esp_app_desc_t *d = esp_app_get_description();
    char elf[65] = "";
    (void)esp_app_get_elf_sha256(elf, sizeof(elf));
    printf("FIRMWARE id=%s version=%s commit=%s elf_sha256=%s built=\"%s %s\" idf=%s cal_dump=1\n", firmware_id(), d->version,
           C5VRX5_COMMIT, elf[0] ? elf : "unknown", d->date, d->time, d->idf_ver);
}

static void status(void)
{
    firmware_print();
    const uint8_t gm = receiver_gain_mode();
    printf("C5VRX5 version=%s demod=%s lane=ultrafine gain_owner=%s "
           "uptime_s=%lld heap_internal=%u heap_largest=%u heap_min=%u heap_dma=%u heap_dma_largest=%u heap_lp=%u heap_psram=%u fw=%s\n",
           esp_app_get_description()->version, transport_program_name(),
           gm == GAIN_MODE_AUTO ? "keel1" : gm == GAIN_MODE_LEVEL ? "level" : gm == GAIN_MODE_RSSI ? "rssi" : "manual",
           esp_timer_get_time() / 1000000, (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),
           /* Lowest free since boot: the boot peak (splash chain plus the
            * receiver tasks) leaves ~21.9 KB with the PSRAM overlay (~2 KB
            * without), which a reading now cannot show. */
           (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA), (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_RTCRAM),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM), firmware_id());
    control_print();
    receiver_print();
    profile_print();
    calibration_print();
    transport_print();
#ifdef C5VRX_LCD
    lcd_print();
#endif
}

/* 8190 contiguous IQ bytes as hex lines (tools/capture_iq_snapshot.py). */
static void snapshot(void)
{
    if (s_menu_active || receiver_paused()) { printf("IQSNAP_REFUSED context_busy\n"); return; }
    uint8_t *raw = malloc(C5V4_LEVEL_SAMPLE_BYTES);
    if (!raw) { printf("IQSNAP_REFUSED memory\n"); return; }
    uint32_t epoch = s_gain_epoch;
    int64_t now = esp_timer_get_time();
    bool settled = now - s_last_gain_write_us >= 500 && s_gain.state != DG_SETTLE;
    if (!settled || !copy_snapshot(raw) || epoch != s_gain_epoch) {
        free(raw);
        printf("IQSNAP_REFUSED stale_or_settling\n");
        return;
    }
    uint32_t hash = 2166136261u;
    for (unsigned k = 0; k < C5V4_LEVEL_SAMPLE_BYTES; ++k) hash = (hash ^ raw[k]) * 16777619u;
    printf("IQSNAP_BEGIN bytes=%u rate_hz=%u format=Ihigh_Qlow gain=%u lane=2 freq_mhz=%u fnv1a=%08lx fw=%s\n",
           C5V4_LEVEL_SAMPLE_BYTES, IQ_RATE_HZ, s_current_gain, rf_frequency_mhz(), (unsigned long)hash, firmware_id());
    static const char hex[] = "0123456789abcdef";
    char line[513];
    for (unsigned offset = 0; offset < C5V4_LEVEL_SAMPLE_BYTES; offset += 256) {
        unsigned size = C5V4_LEVEL_SAMPLE_BYTES - offset > 256 ? 256 : C5V4_LEVEL_SAMPLE_BYTES - offset;
        for (unsigned k = 0; k < size; ++k) {
            line[2 * k] = hex[raw[offset + k] >> 4];
            line[2 * k + 1] = hex[raw[offset + k] & 15];
        }
        line[2 * size] = 0;
        printf("IQSNAP_DATA offset=%u %s\n", offset, line);
        vTaskDelay(1);
    }
    free(raw);
    printf("IQSNAP_END fnv1a=%08lx\n", (unsigned long)hash);
}

static void help(void)
{
    printf("C5VRX-5 keys: T status, C calibration data dump (read only), D range/health report, M gain map (for a preset), R per-second FIELD_DIAG lines on/off,\n"
           "  g OUTPUT 0.5x..5.0x in 0.5x steps (saved after application; restored at boot),\n"
           "  G RX gain LEVEL -> KEEL (KEEL-1) -> RSSI -> MAX -> fixed G78..G24 (models kept across reboots; MAX/fixed: next boot LEVEL),\n"
           "  A live VTX profile switch on/off (continuous matching every 30 s; off = report only),\n"
           "  m decoder U80P64-R1 <-> U85Q (verified restart, saved; video stops briefly),\n"
           "  I IQ snapshot, o menu open/close, n/x menu next/select, c next channel, b next band, s scan,\n"
           "  v video standard AUTO/NTSC/PAL, f analog filter AUTO/NARROW/NORMAL/WIDE (this boot; calibrated classes),\n"
           "  e close menu (web menu direct controls),\n"
           "  F calibrate current filter; B full RF optimization (ground only; video stops), L bench IQ lane comparison (video stops ~3 s),\n"
           "  N bench: noise floor with the video output on vs off (VTX off; video stops ~0.3 s x4),\n"
           "  Z DAC self-test (bench: test pattern, then one flat level per pin for a multimeter),\n"
           "  ! one bench RX sampling-phase slip (T shows phase status),\n"
           "  S + suite key: bench PHY sensitivity lab with the VTX ON (docs/PHY_SENSITIVITY_LAB.md),\n"
           "  ` reboot to USB download (flashing)\n");
}

/* The USB reader runs above every receiver task: the download key and the
 * heartbeat work however busy the CPU is (a starved console used to make
 * flashing need the BOOT button, 2026-10-11). It drains the FIFO, handles
 * the download key itself and queues the rest for the console task. */
static QueueHandle_t s_keys;

static void usb_reader_task(void *arg)
{
    (void)arg;
    for (;;) {
        for (unsigned k = 0; k < 64; ++k) {
            uint8_t c;
            if (usb_serial_jtag_ll_read_rxfifo(&c, 1) == 0) break;
            if (c == '`') rf_reboot_to_download();   /* never returns */
            (void)xQueueSend(s_keys, &c, 0);
        }
        /* Heartbeat: shows the board is alive whatever the console does. */
        static int64_t beat;
        if (esp_timer_get_time() - beat > 5000000 && !flightlog_dumping()) {
            beat = esp_timer_get_time();
            printf("HB t_s=%lld stage=%s gain=%u menu=%u startup=%u busy=%u\n", beat / 1000000, s_boot_stage,
                   s_current_gain, s_menu_active, calibration_in_startup(), menu_status_active());
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void console_task(void *arg)
{
    (void)arg;
    for (;;) {
        uint8_t c;
        static int64_t sense_until;   /* S waits 5 s for the suite key */
        while (xQueueReceive(s_keys, &c, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (sense_until && esp_timer_get_time() < sense_until && sense_lab_suite_valid((char)c)) {
                sense_until = 0;
                sense_lab_request((char)c);
                printf("SENSLAB queued suite=%c (runs in the calibration task; keep the VTX and antennas still)\n", c);
                continue;
            }
            sense_until = 0;
            switch (c) {
            case 'S': sense_lab_help(); sense_until = esp_timer_get_time() + 5000000; break;
            case 'T': status(); break;
            case 'I': snapshot(); break;
            case 'F': filter_request(); break;
            case 'B': filter_request_full(); break;
            case 'C': calibration_dump_request(); break;
            case 'D': diag_report(); break;
            case 'M': receiver_print_map(); break;
            case 'R': diag_toggle_stream(); break;
            case 'L': lane_compare_request(); break;
            case 'N': self_noise_request(); break;
            case '!': rx_clock_slip(1); printf("SLIP bench: one RX sampling-phase slip (T SPHASE shows the new excess)\n"); break;
            case 'Y': case 'W': case 'X': flightlog_console((char)c); break;
#ifdef C5VRX_LCD
            case 'V': lcd_dump(); break;
            case 'J': lcd_quiet_toggle(); break;
            case 'K': lcd_dc_toggle(); break;
#endif
            case 'h': case '?': help(); break;
            case 'o': case 'e': case 'v': case 'f': case 'n': case 'x': case 'c': case 'b': case 's': case 'g': case 'A': case 'U': case 'G': case 'Q': case 'Z': case 'm': {
                int command = c;
                if (s_commands) (void)xQueueSend(s_commands, &command, 0);
                break;
            }
            default: break;
            }
        }
        diag_poll();
        /* Bank-1 watch: prints only when a layer changes (rf_bank1_check). */
        static int64_t bank1_beat;
        if (esp_timer_get_time() - bank1_beat >= 5000000 && !calibration_in_startup()) {
            bank1_beat = esp_timer_get_time();
            rf_bank1_check(false);
        }
        /* Status beat for the web menu: it updates without a T request. */
        static int64_t status_beat;
        if (esp_timer_get_time() - status_beat >= 2000000) {
            status_beat = esp_timer_get_time();
            control_print_beat();
        }
    }
}

void console_start(void)
{
    s_keys = xQueueCreate(64, 1);
    ESP_ERROR_CHECK(s_keys ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(console_task, "console", 6144, NULL, 2, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(usb_reader_task, "usb_reader", 3072, NULL, 10, NULL) == pdPASS ? ESP_OK :
                    ESP_ERR_NO_MEM);
}

