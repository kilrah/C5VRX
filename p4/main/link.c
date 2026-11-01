/* C5VRX P4 display by Twotoz and the C5VRX contributors: UART to the C5
 * (115200 8N1, text lines; the C5 side is main/p4link.c). */
#include "link.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "board.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LINK_UART   UART_NUM_1
#define LINK_BAUD   115200

static const char *TAG = "link";
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static c5_status_t s_status;
static int64_t s_status_us;
static struct { char band; int mhz; } s_table[LINK_CHANNELS];
static bool s_table_ok;

static void send(const char *s) { (void)uart_write_bytes(LINK_UART, s, strlen(s)); }

void link_key(char key)
{
    char line[4] = {'K', key, '\n', 0};
    send(line);
}

void link_tune(int idx)
{
    char line[12];
    snprintf(line, sizeof(line), "C%d\n", (idx % LINK_CHANNELS + LINK_CHANNELS) % LINK_CHANNELS);
    send(line);
}

void link_afc_toggle(void) { send("A\n"); }

static void copy_str(char *dst, size_t n, const char *v)
{
    strncpy(dst, v, n - 1);
    dst[n - 1] = 0;
}

/* TEMP: the C5's last status lines (lk afc cfo sig g cn vid) for the log. */
static int s_hist_c5[16][7];
static unsigned s_hist_n;

void link_debug_print(void)
{
    printf("C5 lk/afc/cfo/sig/g/cn/vid:");
    for (unsigned i = 0; i < 8; ++i) {
        const int *h = s_hist_c5[(s_hist_n + 8 + i) & 15];
        printf(" %d/%d/%d/%d/%d/%d/%d", h[0], h[1], h[2], h[3], h[4], h[5], h[6]);
    }
    printf("\n");
}

static void parse_status(char *s)
{
    c5_status_t st = s_status;
    char *save = NULL;
    for (char *tok = strtok_r(s, " ", &save); tok; tok = strtok_r(NULL, " ", &save)) {
        char *eq = strchr(tok, '=');
        if (!eq) continue;
        *eq = 0;
        const char *k = tok, *v = eq + 1;
        const int i = atoi(v);
        if (!strcmp(k, "idx")) st.idx = i;
        else if (!strcmp(k, "ch")) copy_str(st.ch, sizeof(st.ch), v);
        else if (!strcmp(k, "f")) st.freq = i;
        else if (!strcmp(k, "lk")) st.afc_lock = i;
        else if (!strcmp(k, "vid")) st.video = i;
        else if (!strcmp(k, "std")) copy_str(st.std, sizeof(st.std), v);
        else if (!strcmp(k, "det")) copy_str(st.det, sizeof(st.det), v);
        else if (!strcmp(k, "sig")) st.sig = i;
        else if (!strcmp(k, "gm")) copy_str(st.gain_mode, sizeof(st.gain_mode), v);
        else if (!strcmp(k, "g")) st.gain = i;
        else if (!strcmp(k, "menu")) st.menu = i;
        else if (!strcmp(k, "busy")) st.busy = i;
        else if (!strcmp(k, "scan")) st.scan = i;
        else if (!strcmp(k, "sp")) st.scan_pct = i;
        else if (!strcmp(k, "prof")) copy_str(st.profile, sizeof(st.profile), v);
        else if (!strcmp(k, "dem")) copy_str(st.demod, sizeof(st.demod), v);
        else if (!strcmp(k, "out")) copy_str(st.output, sizeof(st.output), v);
        else if (!strcmp(k, "cn")) st.cn_x10 = i;
        else if (!strcmp(k, "afc")) st.afc = i;
        else if (!strcmp(k, "cfo")) st.cfo_khz = i;
        else if (!strcmp(k, "dc")) copy_str(st.dc, sizeof(st.dc), v);
    }
    ++st.seq;
    int *h = s_hist_c5[s_hist_n++ & 15];
    h[0] = st.afc_lock; h[1] = st.afc; h[2] = st.cfo_khz; h[3] = st.sig; h[4] = st.gain; h[5] = st.cn_x10;
    h[6] = st.video;
    portENTER_CRITICAL(&s_lock);
    s_status = st;
    s_status_us = esp_timer_get_time();
    portEXIT_CRITICAL(&s_lock);
}

static void parse_table(const char *s)
{
    unsigned idx, mhz;
    char band[4];
    if (sscanf(s, "L %u %3s %u", &idx, band, &mhz) != 3 || idx >= LINK_CHANNELS) return;
    s_table[idx].band = band[0];
    s_table[idx].mhz = (int)mhz;
    if (idx == LINK_CHANNELS - 1) s_table_ok = true;
}

static void link_task(void *arg)
{
    (void)arg;
    char line[360];
    unsigned len = 0;
    int64_t ask = 0;
    for (;;) {
        uint8_t buf[64];
        const int got = uart_read_bytes(LINK_UART, buf, sizeof(buf), pdMS_TO_TICKS(20));
        for (int k = 0; k < got; ++k) {
            const char c = (char)buf[k];
            if (c == '\n' || c == '\r') {
                line[len] = 0;
                if (len > 2 && line[0] == 'S' && line[1] == ' ') parse_status(line + 2);
                else if (len > 2 && line[0] == 'L' && line[1] == ' ') parse_table(line);
                len = 0;
            } else if (len < sizeof(line) - 1u) {
                line[len++] = c;
            }
        }
        /* Ask for the channel table until it arrived (the C5 may boot later). */
        const int64_t now = esp_timer_get_time();
        if (!s_table_ok && now >= ask && now - s_status_us < 1000000) {
            ask = now + 2000000;
            send("L\n");
        }
    }
}

void link_status(c5_status_t *out)
{
    portENTER_CRITICAL(&s_lock);
    *out = s_status;
    out->online = s_status_us && esp_timer_get_time() - s_status_us < 1000000;
    portEXIT_CRITICAL(&s_lock);
}

int link_raw_iq(void)
{
    portENTER_CRITICAL(&s_lock);
    const bool online = s_status_us && esp_timer_get_time() - s_status_us < 1000000;
    const bool raw = !strcmp(s_status.demod, "RAW");
    portEXIT_CRITICAL(&s_lock);
    return online ? raw : -1;
}

bool link_channel(int idx, char *band, int *mhz)
{
    if (!s_table_ok || idx < 0 || idx >= LINK_CHANNELS) return false;
    *band = s_table[idx].band;
    *mhz = s_table[idx].mhz;
    return true;
}

void link_start(void)
{
    const uart_config_t cfg = {
        .baud_rate = LINK_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(LINK_UART, 2048, 512, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LINK_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(LINK_UART, LINK_UART_TX_GPIO, LINK_UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    xTaskCreatePinnedToCore(link_task, "link", 4096, NULL, 4, NULL, 0);
    ESP_LOGI(TAG, "C5 link on UART1 tx=%d rx=%d", LINK_UART_TX_GPIO, LINK_UART_RX_GPIO);
}
