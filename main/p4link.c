/* C5VRX-5 by Twotoz and the C5VRX contributors: control link to an ESP32-P4
 * display (FireBeetle 2 ESP32-C5 P4-link build, docs/P4LINK.md).
 *
 * The raw IQ (decoder RAW) or the picture leaves on the TX pins with the TX
 * clock (transport.c);
 * this UART only carries text lines, 115200 8N1, C5 TX GPIO11 / RX GPIO12:
 *   C5 -> P4  "S key=value ..."  status, every 250 ms and on '?'
 *             "L idx name mhz"   one line per channel, answer to 'L'
 *   P4 -> C5  "K<c>"  a console key (control.c command set; F/B calibrate)
 *             "C<n>"  tune channel index n (band n/8, channel n%8)
 *             "A"     AFC on/off
 *             "?" / "L"
 * Lines that do not parse are ignored on both sides (the ROM boot log also
 * goes out on GPIO11). Nothing here touches the realtime path; commands go
 * through s_commands like the USB console's. */
#include "receiver.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_timer.h"

#define P4LINK_UART     UART_NUM_1
#define P4LINK_TX_GPIO  11
#define P4LINK_RX_GPIO  12
#define P4LINK_BAUD     115200
#define STATUS_PERIOD_US 250000

static void send_line(const char *s, int n)
{
    if (n > 0) (void)uart_write_bytes(P4LINK_UART, s, (size_t)n);
}

static void send_status(void)
{
    char line[320], gm[16], dc[12] = "";
    const int64_t now = esp_timer_get_time(), video_at = s_last_video_us;
    const cna_reading_t cn = receiver_cn_reading();
    (void)calibration_dc_needed(dc, sizeof(dc));
    const int n = snprintf(line, sizeof(line),
        "S idx=%u ch=%s f=%u lk=%u vid=%u std=%s det=%s sig=%d gm=%s g=%u menu=%u busy=%u scan=%u sp=%u "
        "prof=%s dem=%s out=%s cn=%d afc=%u cfo=%d dc=%s up=%lld\n",
        (unsigned)rf_channel_index(), rf_channel()->name, rf_frequency_mhz(), s_afc_locked,
        video_at && now - video_at < 500000,
        s_std_mode == STD_MODE_AUTO ? "AUTO" : s_std_mode == STD_MODE_PAL ? "PAL" : "NTSC",
        s_detected_std_valid ? (s_detected_std == VIDEO_STD_PAL ? "PAL" : "NTSC") : "-",
        s_signal_strength, receiver_gain_mode_name(receiver_gain_mode(), gm, sizeof(gm)), s_current_gain,
        s_menu_active, menu_status_active(), s_scan_active, s_scan_progress, profile_name(profile_active()),
        transport_program_name(), transport_output_name(), cn.x10, s_afc_enabled, s_cfo_khz, dc[0] ? dc : "-",
        now / 1000000);
    send_line(line, n < (int)sizeof(line) ? n : (int)sizeof(line) - 1);
}

static void send_channels(void)
{
    char line[40];
    for (unsigned i = 0; i < FPV_CHANNELS; ++i) {
        const int n = snprintf(line, sizeof(line), "L %u %s %u\n", i, rf_band_name(i / 8u), rf_channel_freq(i));
        send_line(line, n);
    }
}

static void queue(int command)
{
    if (s_commands) (void)xQueueSend(s_commands, &command, 0);
}

/* Whole lines only: the P4's ROM boot log also arrives here (its UART0 TX). */
static void handle_line(const char *s)
{
    if (s[0] == 'K' && (!s[1] || s[2])) return;
    if (s[0] != 'K' && s[0] != 'C' && s[1]) return;
    switch (s[0]) {
    case 'K':
        switch (s[1]) {
        case 'F': filter_request(); break;        /* CALIBRATE QUICK */
        case 'B': filter_request_full(); break;   /* CALIBRATE FULL */
        case 'o': case 'e': case 'v': case 'f': case 'n': case 'x': case 'c': case 'b': case 's': case 'g':
        case 'G': case 'm':
            queue(s[1]);
            break;
        default: break;
        }
        break;
    case 'C': {
        char *end;
        const unsigned long idx = strtoul(s + 1, &end, 10);
        if (end != s + 1 && !*end && idx < FPV_CHANNELS) queue(P4LINK_CMD_CHANNEL + (int)idx);
        break;
    }
    case 'A': queue(P4LINK_CMD_AFC); break;
    case '?': send_status(); break;
    case 'L': send_channels(); break;
    default: break;
    }
}

static void p4link_task(void *arg)
{
    (void)arg;
    char line[24];
    unsigned len = 0;
    int64_t next_status = 0;
    for (;;) {
        uint8_t buf[16];
        const int got = uart_read_bytes(P4LINK_UART, buf, sizeof(buf), pdMS_TO_TICKS(20));
        for (int k = 0; k < got; ++k) {
            const char c = (char)buf[k];
            if (c == '\n' || c == '\r') {
                line[len] = 0;
                if (len) handle_line(line);
                len = 0;
            } else if (len < sizeof(line) - 1u) {
                line[len++] = c;
            }
        }
        const int64_t now = esp_timer_get_time();
        if (now >= next_status) {
            next_status = now + STATUS_PERIOD_US;
            send_status();
        }
    }
}

void p4link_start(void)
{
    const uart_config_t cfg = {
        .baud_rate = P4LINK_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    /* Small RX ring, no TX ring (writes wait for FIFO room): internal SRAM
     * is the menu's descriptor heap on this board (no PSRAM). */
    esp_err_t err = uart_driver_install(P4LINK_UART, 256, 0, 0, NULL, 0);
    if (err == ESP_OK) err = uart_param_config(P4LINK_UART, &cfg);
    if (err == ESP_OK) err = uart_set_pin(P4LINK_UART, P4LINK_TX_GPIO, P4LINK_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err == ESP_OK && xTaskCreate(p4link_task, "p4link", 3072, NULL, 2, NULL) != pdPASS) err = ESP_ERR_NO_MEM;
    printf("P4LINK uart=%d tx=%d rx=%d baud=%d %s\n", P4LINK_UART, P4LINK_TX_GPIO, P4LINK_RX_GPIO, P4LINK_BAUD,
           err == ESP_OK ? "ready" : esp_err_to_name(err));
}
