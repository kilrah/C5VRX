/* C5VRX-5 by Twotoz and the C5VRX contributors: USB serial console.
 * Polled, never paces the realtime path. */
#include "receiver.h"
#include "sense_lab.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
#ifdef C5VRX_HWX
    static const int iq[8] = RF_IQ_GPIO_NUMS;
    printf("HWX id=%s name=%s iq_gpio=%d,%d,%d,%d,%d,%d,%d,%d dac=%u\n", C5VRX_HWX_ID, HWX_NAME,
           iq[0], iq[1], iq[2], iq[3], iq[4], iq[5], iq[6], iq[7],
#ifdef HWX_NO_DAC
           0u);
#else
           1u);
#endif
#endif
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
           /* Lowest free since boot. Receiver/control storage is reserved
            * before the optional splash; live free heap hides that peak. */
           (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA), (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_RTCRAM),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM), firmware_id());
    control_print();
    receiver_print();
    profile_print();
    calibration_print();
    transport_print();
    output_auto_print();
    receiver_pause_print();
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

/* Bench frame sweep (J, tools/frame_sweep.py): snapshots spread over one
 * video frame, each FRAME_SWEEP_STEP_NS later in the frame than the one
 * before, one per frame, until the whole frame is covered (~8 s for NTSC).
 * They are kept in PSRAM (the PSRAM build, sdkconfig.psram.defaults) and
 * printed afterwards, so the VTX can go off during the print. A refused copy
 * is retried on the next RX descriptor boundary. Each record carries its
 * boundary time; the host places it by that time and the fitted line period.
 * Live video keeps running. */
#define FRAME_SWEEP_STEP_NS 150000
#define FRAME_SWEEP_TRIES 4
static void frame_sweep(void)
{
    if (s_menu_active || receiver_paused()) { printf("IQSWEEP_REFUSED context_busy\n"); return; }
    const bool pal = s_detected_std_valid && s_detected_std == VIDEO_STD_PAL;
    const int64_t frame_ns = pal ? 40000000 : 33366667;   /* NTSC 1001/30 ms */
    const unsigned n = (unsigned)(frame_ns / FRAME_SWEEP_STEP_NS) + 3u;
    uint8_t *store = heap_caps_malloc((size_t)n * C5V4_LEVEL_SAMPLE_BYTES, MALLOC_CAP_SPIRAM);
    int64_t *when = heap_caps_calloc(n, sizeof(int64_t), MALLOC_CAP_SPIRAM);
    uint8_t *gain = heap_caps_calloc(n, 1, MALLOC_CAP_SPIRAM), *ok = heap_caps_calloc(n, 1, MALLOC_CAP_SPIRAM);
    uint8_t *tmp = malloc(C5V4_LEVEL_SAMPLE_BYTES);
    if (!store || !when || !gain || !ok || !tmp) {
        printf("IQSWEEP_REFUSED memory need=%u psram_free=%u (J needs the PSRAM build)\n", n * C5V4_LEVEL_SAMPLE_BYTES,
               (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        goto out;
    }
    printf("IQSWEEP_CAPTURE n=%u std=%s (keep the camera still)\n", n, pal ? "PAL" : "NTSC");
    const int64_t t0 = esp_timer_get_time() + 20000;
    unsigned good = 0, retries = 0, frames = 0;
    for (unsigned k = 0; k < n; ++k) {
        int64_t target;
        while ((target = t0 + ((int64_t)k * FRAME_SWEEP_STEP_NS + (int64_t)frames * frame_ns) / 1000) <
               esp_timer_get_time() + 1000)
            ++frames;   /* held up past this frame: the same place one frame later */
        ++frames;
        while (esp_timer_get_time() < target - 1500) vTaskDelay(1);
        while (esp_timer_get_time() < target) { }
        for (unsigned tr = 0; tr < FRAME_SWEEP_TRIES && !ok[k]; ++tr) {
            if (s_menu_active || receiver_paused()) break;
            gain[k] = (uint8_t)s_current_gain;
            if (copy_snapshot_at_boundary(tmp, &when[k])) {
                memcpy(store + (size_t)k * C5V4_LEVEL_SAMPLE_BYTES, tmp, C5V4_LEVEL_SAMPLE_BYTES);
                ok[k] = 1;
                ++good;
            } else {
                ++retries;
            }
        }
    }
    printf("IQSWEEP_BEGIN n=%u good=%u retries=%u bytes=%u rate_hz=%u std=%s frame_ns=%lld step_ns=%d freq_mhz=%u fw=%s "
           "(capture done: the VTX may go off)\n",
           n, good, retries, C5V4_LEVEL_SAMPLE_BYTES, IQ_RATE_HZ, pal ? "PAL" : "NTSC", (long long)frame_ns,
           FRAME_SWEEP_STEP_NS, rf_frequency_mhz(), firmware_id());
    static const char hex[] = "0123456789abcdef";
    char line[513];
    for (unsigned k = 0; k < n; ++k) {
        if (!ok[k]) { printf("IQSWEEP_SNAP k=%u ok=0\n", k); continue; }
        const uint8_t *raw = store + (size_t)k * C5V4_LEVEL_SAMPLE_BYTES;
        uint32_t hash = 2166136261u;
        for (unsigned j = 0; j < C5V4_LEVEL_SAMPLE_BYTES; ++j) hash = (hash ^ raw[j]) * 16777619u;
        printf("IQSWEEP_SNAP k=%u ok=1 frames=0 t_us=%lld gain=%u fnv1a=%08lx\n", k, (long long)when[k], gain[k],
               (unsigned long)hash);
        for (unsigned offset = 0; offset < C5V4_LEVEL_SAMPLE_BYTES; offset += 256) {
            unsigned size = C5V4_LEVEL_SAMPLE_BYTES - offset > 256 ? 256 : C5V4_LEVEL_SAMPLE_BYTES - offset;
            for (unsigned j = 0; j < size; ++j) {
                line[2 * j] = hex[raw[offset + j] >> 4];
                line[2 * j + 1] = hex[raw[offset + j] & 15];
            }
            line[2 * size] = 0;
            printf("IQSWEEP_DATA k=%u offset=%u %s\n", k, offset, line);
            vTaskDelay(1);
        }
    }
    printf("IQSWEEP_END n=%u good=%u\n", n, good);
out:
    free(tmp);
    heap_caps_free(ok);
    heap_caps_free(gain);
    heap_caps_free(when);
    heap_caps_free(store);
}

static void help(void)
{
    printf("C5VRX-5 keys: T status, C calibration data dump (read only), D range/health report, M gain map (for a preset), R per-second FIELD_DIAG lines on/off,\n"
           "  g OUTPUT 0.5x..5.0x in 0.5x steps (saved after application; restored at boot),\n"
           "  G RX gain LEVEL -> KEEL (KEEL-1) -> RSSI -> MAX -> fixed G78..G24 (models kept across reboots; MAX/fixed: next boot LEVEL),\n"
           "  A live VTX profile switch on/off (continuous matching every 30 s; off = report only),\n"
           "  m decoder U80P64-R1 <-> U85Q (verified restart, saved; video stops briefly),\n"
           "  I IQ snapshot, J bench frame sweep (one frame of IQ, still camera; tools/frame_sweep.py), o menu open/close, n/x menu next/select, c next channel, b next band, s scan,\n"
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
            case 'J': frame_sweep(); break;
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

