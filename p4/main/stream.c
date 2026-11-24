/* C5VRX P4 -> Meta Quest: the decoded lines as UDP over the Quest's USB
 * network link (CDC-NCM; the P4 is the USB host on its high-speed port,
 * behind the board's hub) or Ethernet. Wire format: stream.h.
 *
 * Rows wait in a ring in internal RAM (the decoder on core 1 fills it); the
 * sender on core 0 drains it every millisecond: 14,400 lines/s (PAL and
 * NTSC) x 644 B = ~75 Mbit/s with the headers.
 *
 * To the Quest over USB the rows go past lwIP: the sender writes the
 * Ethernet, IP and UDP headers itself (UDP checksum 0, allowed in IPv4),
 * puts frames of two rows into the NCM driver's pending NTB and sends the
 * burst as one USB transfer (ncm_host_flush). Through lwIP a send cost 70 us
 * idle and ~250-600 us next to the capture and decoder, and only half the
 * rows got out; past lwIP, all 14,400 rows/s with none dropped (2026-11-23).
 * To a client on the Ethernet subnet (the PC tools) the same frames go
 * straight to the EMAC driver (esp_eth_transmit, which copies them into its
 * DMA buffers): through lwIP, six rows a datagram (IP fragments it), only
 * ~10,400 of the 14,400 rows/s got out (2026-11-24). The EMAC's DMA buffers
 * hold one two-row frame each (sdkconfig.quest.defaults); when all are queued
 * the sender waits a tick, the ring holds 6.7 ms. Anywhere else (a client
 * behind a router) through lwIP. The status goes through lwIP either way,
 * which keeps the client's ARP entry fresh. */
#include "stream.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_net_stack.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "link.h"
#include "lwip/etharp.h"
#include "lwip/sockets.h"
#include "lwip/tcpip.h"
#include "ncm_host.h"
#include "usb/usb_host.h"

static const char *TAG = "stream";

#define RING            96      /* rows: 6.7 ms */
#define ROW_WIRE        (4 + CVBS_W + 2 * STREAM_CW)
#define HDR             12
#define ROWS_RAW        2       /* one Ethernet frame: 14 + 20 + 8 + 12 + 2 x 644 = 1342 B */
#define ROWS_LWIP       6       /* 3.9 KB, IP fragments it */
#define L2HDR           42      /* Ethernet + IPv4 + UDP */
#define PHY_RESET_GPIO  51      /* Waveshare ESP32-P4-Module-DEV-KIT: IP101 */

static stream_row_t s_ring[RING];
static volatile uint32_t s_head, s_tail;    /* decoder fills head, sender drains tail */
static volatile bool s_active;              /* a client said hello recently */
static int s_sock = -1;
static struct sockaddr_in s_client;
static int64_t s_hello_us;
static uint32_t s_seq;
static uint32_t s_rows_sent, s_dgrams, s_send_err, s_ring_full, s_cmds;
static uint32_t s_send_us, s_send_max_us, s_send_n;   /* send time since the last print */
static uint32_t s_raw_frames, s_raw_err;
/* Raw path: the client's MAC (from lwIP's ARP table), the link's own. */
typedef enum { RAW_NONE, RAW_USB, RAW_ETH } raw_t;
static raw_t s_raw;
static uint8_t s_peer_mac[6], s_own_mac[6];
static uint32_t s_own_ip;                   /* network order */
static esp_eth_handle_t s_eth;
static esp_netif_t *s_eth_netif;
static uint32_t s_eth_full;                 /* EMAC DMA buffers all queued: waited a tick */
static uint16_t s_ip_id;
static volatile bool s_led_test;            /* latency test running (below) */

/* ------------------------------------------------------------ decoder side */

stream_row_t *stream_row_get(void)
{
    if (!s_active) return NULL;
    if (s_head - __atomic_load_n(&s_tail, __ATOMIC_ACQUIRE) >= RING) { ++s_ring_full; return NULL; }
    return &s_ring[s_head % RING];
}

void stream_row_put(void)
{
    __atomic_store_n(&s_head, s_head + 1u, __ATOMIC_RELEASE);
}

/* ------------------------------------------------------------ sender */

static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v) { put16(p, (uint16_t)v); put16(p + 2, (uint16_t)(v >> 16)); }

static void header(uint8_t *b, uint8_t type, uint16_t field, uint16_t height)
{
    b[0] = 'C'; b[1] = '5'; b[2] = 1; b[3] = type;
    put32(b + 4, s_seq++);
    put16(b + 8, field);
    put16(b + 10, height);
}

static void send_dgram(const uint8_t *b, size_t n)
{
    struct sockaddr_in to = s_client;
    const int64_t t0 = esp_timer_get_time();
    const int r = sendto(s_sock, b, n, 0, (const struct sockaddr *)&to, sizeof(to));
    const uint32_t dt = (uint32_t)(esp_timer_get_time() - t0);
    s_send_us += dt;
    ++s_send_n;
    if (dt > s_send_max_us) s_send_max_us = dt;
    if (r < 0) ++s_send_err;
    else ++s_dgrams;
}

static void send_status(void)
{
    static uint8_t b[HDR + 400];
    c5_status_t st;
    cvbs_stats_t cs;
    link_status(&st);
    cvbs_get_stats(&cs);
    header(b, 2, 0, 0);
    const int n = snprintf((char *)b + HDR, sizeof(b) - HDR,
        "c5=%d idx=%d ch=%s f=%d sig=%d cn=%d.%d gain=%d gm=%s afc=%d cfo=%d vid=%d std=%s menu=%d scan=%d sp=%d dem=%s "
        "lock=%d pal=%d fields=%lu frames=%lu clock=%d raw=%d drop=%lu led=%d",
        st.online, st.idx, st.ch, st.freq, st.sig, st.cn_x10 / 10, st.cn_x10 % 10, st.gain, st.gain_mode, st.afc,
        st.cfo_khz, st.video, st.std, st.menu, st.scan, st.scan_pct, st.demod, cs.locked, cs.pal,
        (unsigned long)cs.fields_ps, (unsigned long)cs.frames_ps, cs.clock, cs.raw_iq, (unsigned long)s_ring_full,
        s_led_test);
    if (n > 0) send_dgram(b, HDR + (size_t)(n < (int)(sizeof(b) - HDR) ? n : (int)(sizeof(b) - HDR - 1)));
}

static uint16_t ip_csum(const uint8_t *h)
{
    uint32_t sum = 0;
    for (int i = 0; i < 20; i += 2) sum += (uint32_t)(h[i] << 8 | h[i + 1]);
    while (sum >> 16) sum = (sum & 0xffffu) + (sum >> 16);
    return (uint16_t)~sum;
}

/* Ethernet, IPv4 and UDP headers in front of a payload of n bytes at f + L2HDR. */
static void l2_headers(uint8_t *f, size_t n)
{
    memcpy(f, s_peer_mac, 6);
    memcpy(f + 6, s_own_mac, 6);
    f[12] = 0x08; f[13] = 0x00;
    uint8_t *ip = f + 14;
    const uint16_t tot = (uint16_t)(20 + 8 + n);
    ip[0] = 0x45; ip[1] = 0;
    ip[2] = (uint8_t)(tot >> 8); ip[3] = (uint8_t)tot;
    ip[4] = (uint8_t)(s_ip_id >> 8); ip[5] = (uint8_t)s_ip_id; ++s_ip_id;
    ip[6] = 0x40; ip[7] = 0;                  /* DF */
    ip[8] = 64; ip[9] = 17;                   /* TTL, UDP */
    ip[10] = ip[11] = 0;
    const uint32_t dst = s_client.sin_addr.s_addr;
    memcpy(ip + 12, &s_own_ip, 4);
    memcpy(ip + 16, &dst, 4);
    const uint16_t c = ip_csum(ip);
    ip[10] = (uint8_t)(c >> 8); ip[11] = (uint8_t)c;
    uint8_t *u = ip + 20;
    u[0] = STREAM_PORT >> 8; u[1] = STREAM_PORT & 0xff;
    memcpy(u + 2, &s_client.sin_port, 2);     /* network order already */
    const uint16_t ul = (uint16_t)(8 + n);
    u[4] = (uint8_t)(ul >> 8); u[5] = (uint8_t)ul;
    u[6] = u[7] = 0;                          /* no UDP checksum */
}

/* The client's MAC on the Ethernet subnet, if lwIP's ARP table has it. */
static bool eth_peer_mac(uint32_t ip)
{
    esp_netif_ip_info_t info;
    if (!s_eth_netif || !esp_netif_is_netif_up(s_eth_netif) || esp_netif_get_ip_info(s_eth_netif, &info) != ESP_OK ||
        !info.ip.addr || ((ip ^ info.ip.addr) & info.netmask.addr))
        return false;
    struct netif *nif = esp_netif_get_netif_impl(s_eth_netif);
    const ip4_addr_t a = {.addr = ip};
    struct eth_addr *ea = NULL;
    const ip4_addr_t *ia = NULL;
    LOCK_TCPIP_CORE();
    const ssize_t i = etharp_find_addr(nif, &a, &ea, &ia);
    if (i >= 0) memcpy(s_peer_mac, ea->addr, 6);
    UNLOCK_TCPIP_CORE();
    if (i < 0) return false;
    esp_netif_get_mac(s_eth_netif, s_own_mac);
    s_own_ip = info.ip.addr;
    return true;
}

/* The raw path applies to a client on the USB link or the Ethernet subnet
 * whose MAC lwIP knows. */
static void check_raw(void)
{
    const uint32_t net = esp_ip4addr_aton(NCM_HOST_IP) & 0x00ffffffu;   /* /24, network order */
    const uint32_t ip = s_client.sin_addr.s_addr;
    raw_t raw = RAW_NONE;
    if ((ip & 0x00ffffffu) == net) {
        if (ncm_host_peer_mac(ip, s_peer_mac, s_own_mac)) {
            s_own_ip = esp_ip4addr_aton(NCM_HOST_IP);
            raw = RAW_USB;
        }
    } else if (eth_peer_mac(ip)) {
        raw = RAW_ETH;
    }
    if (raw != s_raw)
        ESP_LOGI(TAG, "rows %s", raw == RAW_USB ? "past lwIP (USB link)" : raw == RAW_ETH ? "past lwIP (Ethernet)"
                                                                                       : "through lwIP");
    s_raw = raw;
}

/* One frame to the EMAC; all its DMA buffers queued: wait a tick, at most 5. */
static esp_err_t eth_send_frame(uint8_t *f, size_t n)
{
    for (int i = 0;; ++i) {
        const esp_err_t e = esp_eth_transmit(s_eth, f, n);
        if (e != ESP_ERR_NO_MEM || i == 5) return e;
        ++s_eth_full;
        vTaskDelay(1);
    }
}

/* ------------------------------------------------------------ latency test (LED)
 *
 * Command "T" toggles it. An LED on GPIO7 (low = on), held right in front of
 * the FPV camera's lens, blinks 250 ms on / 250 ms off. Every decoded row
 * carries its decode time (cvbs.c); the sender compares each row's mean luma
 * with that row's mean while the LED was dark, and the first row after a
 * switch-on that is LED_DELTA brighter gives the latency from the LED to the
 * decoded line: camera, VTX, C5 and P4 together. Each result also goes to the
 * client as a type 3 datagram right after the frame with that row, on the
 * same path, so the Quest can time the rest. */
#define LED_GPIO        7
#define LED_HALF_US     250000
#define LED_SETTLE_US   100000  /* baseline learnt from 100 ms after switch-off */
#define LED_DELTA       24      /* luma rise that counts as lit */
static esp_timer_handle_t s_led_timer;
static volatile bool s_led_lit;
static volatile uint32_t s_led_on_us, s_led_off_us;   /* low 32 bits of esp_timer */
static uint8_t s_base[CVBS_H_MAX];
static bool s_led_found, s_led_event;
static uint32_t s_lat_us, s_lat_n, s_lat_sum, s_lat_min, s_lat_max, s_lat_miss;
static uint16_t s_lat_row;

static void led_tick(void *arg)
{
    (void)arg;
    const uint32_t now = (uint32_t)esp_timer_get_time();
    if (!s_led_lit) {
        s_led_found = false;
        s_led_on_us = now;
        s_led_lit = true;
        gpio_set_level(LED_GPIO, 0);
    } else {
        if (!s_led_found) ++s_lat_miss;
        s_led_off_us = now;
        s_led_lit = false;
        gpio_set_level(LED_GPIO, 1);
    }
}

static void led_test(bool on)
{
    if (on == s_led_test) return;
    s_led_test = on;
    if (on) {
        s_lat_n = s_lat_sum = s_lat_max = s_lat_miss = 0;
        s_lat_min = UINT32_MAX;
        memset(s_base, 255, sizeof(s_base));   /* nothing is "brighter" until learnt */
        s_led_lit = false;
        s_led_off_us = (uint32_t)esp_timer_get_time();
        esp_timer_start_periodic(s_led_timer, LED_HALF_US);
    } else {
        esp_timer_stop(s_led_timer);
        s_led_lit = false;
        gpio_set_level(LED_GPIO, 1);
    }
    ESP_LOGI(TAG, "latency test %s", on ? "on (LED on GPIO7 in front of the lens)" : "off");
}

static void led_row(const stream_row_t *r)
{
    if (!s_led_test || r->row >= CVBS_H_MAX) return;
    uint32_t sum = 0;
    for (int i = 4; i < CVBS_W; i += 8) sum += r->y[i];
    const int m = (int)(sum / (CVBS_W / 8));
    const uint32_t on = s_led_on_us, off = s_led_off_us;
    const bool lit = s_led_lit && (int32_t)(r->t_us - on) >= 0;
    if (!s_led_lit && (int32_t)(r->t_us - (off + LED_SETTLE_US)) >= 0) {
        uint8_t *b = &s_base[r->row];
        *b = *b == 255 ? (uint8_t)m : (uint8_t)((*b * 3 + m) / 4);
    } else if (lit && !s_led_found && m > s_base[r->row] + LED_DELTA) {
        s_led_found = true;
        s_lat_us = r->t_us - on;
        s_lat_row = r->row;
        ++s_lat_n;
        s_lat_sum += s_lat_us;
        if (s_lat_us < s_lat_min) s_lat_min = s_lat_us;
        if (s_lat_us > s_lat_max) s_lat_max = s_lat_us;
        s_led_event = true;
    }
}

static void send_led_event(void)
{
    static uint8_t b[HDR + 120];
    header(b, 3, 0, 0);
    const int n = snprintf((char *)b + HDR, sizeof(b) - HDR, "led lat_us=%lu row=%u n=%lu avg_us=%lu min_us=%lu "
                           "max_us=%lu miss=%lu", (unsigned long)s_lat_us, s_lat_row, (unsigned long)s_lat_n,
                           (unsigned long)(s_lat_sum / s_lat_n), (unsigned long)s_lat_min,
                           (unsigned long)s_lat_max, (unsigned long)s_lat_miss);
    send_dgram(b, HDR + (size_t)n);
    ESP_LOGI(TAG, "LED -> decoded line %lu us (row %u), n=%lu avg %lu min %lu max %lu, missed %lu",
             (unsigned long)s_lat_us, s_lat_row, (unsigned long)s_lat_n, (unsigned long)(s_lat_sum / s_lat_n),
             (unsigned long)s_lat_min, (unsigned long)s_lat_max, (unsigned long)s_lat_miss);
}

static void sender_task(void *arg)
{
    (void)arg;
    static uint8_t f[L2HDR + HDR + ROWS_LWIP * ROW_WIRE];
    uint8_t *const b = f + L2HDR;
    int64_t next_status = 0;
    for (;;) {
        const int64_t now = esp_timer_get_time();
        if (s_active && now - s_hello_us > STREAM_HELLO_TIMEOUT_MS * 1000LL) {
            s_active = false;
            ESP_LOGI(TAG, "client %s silent: stream stopped", inet_ntoa(s_client.sin_addr));
        }
        if (!s_active) {
            __atomic_store_n(&s_tail, s_head, __ATOMIC_RELEASE);   /* drop what is left */
            s_raw = RAW_NONE;
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (now >= next_status) { send_status(); check_raw(); next_status = now + 250000; }
        uint32_t head = __atomic_load_n(&s_head, __ATOMIC_ACQUIRE);
        if (s_tail == head) { vTaskDelay(1); continue; }
        const int per = s_raw ? ROWS_RAW : ROWS_LWIP;
        while (s_tail != head) {
            const stream_row_t *r = &s_ring[s_tail % RING];
            header(b, 1, r->field, r->height);
            int n = 0;
            for (;;) {
                led_row(r);
                uint8_t *o = b + HDR + n * ROW_WIRE;
                put16(o, r->row);
                o[2] = r->flags;
                o[3] = 0;
                memcpy(o + 4, r->y, CVBS_W);
                memcpy(o + 4 + CVBS_W, r->cb, STREAM_CW);
                memcpy(o + 4 + CVBS_W + STREAM_CW, r->cr, STREAM_CW);
                ++n;
                __atomic_store_n(&s_tail, s_tail + 1u, __ATOMIC_RELEASE);
                if (n == per || s_tail == head) break;
                const stream_row_t *nx = &s_ring[s_tail % RING];
                if (nx->field != r->field || nx->height != r->height) break;
                r = nx;
            }
            const size_t len = HDR + (size_t)n * ROW_WIRE;
            if (s_raw) {
                const int64_t t0 = esp_timer_get_time();
                l2_headers(f, len);
                const esp_err_t e = s_raw == RAW_USB ? ncm_host_send_frame(f, L2HDR + len)
                                                     : eth_send_frame(f, L2HDR + len);
                if (e == ESP_OK) ++s_raw_frames;
                else ++s_raw_err;
                const uint32_t dt = (uint32_t)(esp_timer_get_time() - t0);
                s_send_us += dt;
                ++s_send_n;
                if (dt > s_send_max_us) s_send_max_us = dt;
            } else {
                send_dgram(b, len);
            }
            s_rows_sent += (uint32_t)n;
            if (s_led_event) { s_led_event = false; send_led_event(); }   /* after the row's frame */
        }
        if (s_raw == RAW_USB) ncm_host_flush();   /* the burst's frames in as few USB transfers as possible */
    }
}

/* Hellos and commands from the Quest. */
static void command_task(void *arg)
{
    (void)arg;
    char m[32];
    for (;;) {
        struct sockaddr_in from;
        socklen_t fl = sizeof(from);
        const int n = recvfrom(s_sock, m, sizeof(m) - 1, 0, (struct sockaddr *)&from, &fl);
        if (n <= 0) continue;
        m[n] = 0;
        if (m[0] == 'H') {
            if (!s_active || from.sin_addr.s_addr != s_client.sin_addr.s_addr || from.sin_port != s_client.sin_port) {
                s_client = from;
                ESP_LOGI(TAG, "streaming to %s:%u", inet_ntoa(from.sin_addr), ntohs(from.sin_port));
            }
            s_hello_us = esp_timer_get_time();
            s_active = true;
            continue;
        }
        ++s_cmds;
        if (m[0] == 'K' && m[1]) link_key(m[1]);
        else if (m[0] == 'C' && m[1] >= '0' && m[1] <= '9') link_tune(atoi(m + 1));
        else if (m[0] == 'A') link_afc_toggle();
        else if (m[0] == 'T') led_test(!s_led_test);
    }
}

void stream_link_down(void)
{
    s_hello_us = 0;   /* the sender stops at its next pass */
}

void stream_debug_print(void)
{
    printf("STREAM active=%d client=%s raw=%d rows=%lu dgrams=%lu raw_frames=%lu raw_err=%lu eth_full=%lu "
           "send_err=%lu ring_full=%lu cmds=%lu send_avg_us=%lu max_us=%lu\n", s_active,
           inet_ntoa(s_client.sin_addr), (int)s_raw, (unsigned long)s_rows_sent, (unsigned long)s_dgrams,
           (unsigned long)s_raw_frames, (unsigned long)s_raw_err, (unsigned long)s_eth_full, (unsigned long)s_send_err, (unsigned long)s_ring_full, (unsigned long)s_cmds,
           (unsigned long)(s_send_n ? s_send_us / s_send_n : 0), (unsigned long)s_send_max_us);
    s_send_us = s_send_max_us = s_send_n = 0;
    ncm_host_log_stats();
}

/* ------------------------------------------------------------ USB host: the Quest */

static usb_host_client_handle_t s_usb;
static QueueHandle_t s_usb_ev;
static SemaphoreHandle_t s_ctrl_done;
static usb_transfer_t *s_ctrl;
typedef struct { bool gone; uint8_t addr; usb_device_handle_t dev; } usb_ev_t;

static void usb_lib_task(void *arg)
{
    (void)arg;
    for (;;) {
        uint32_t flags;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) usb_host_device_free_all();
    }
}

static void usb_client_cb(const usb_host_client_event_msg_t *msg, void *arg)
{
    (void)arg;
    usb_ev_t ev = {0};
    if (msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) ev.addr = msg->new_dev.address;
    else { ev.gone = true; ev.dev = msg->dev_gone.dev_hdl; }
    xQueueSend(s_usb_ev, &ev, 0);
}

static void usb_client_task(void *arg)
{
    (void)arg;
    for (;;) usb_host_client_handle_events(s_usb, portMAX_DELAY);
}

static void ctrl_cb(usb_transfer_t *t) { (void)t; xSemaphoreGive(s_ctrl_done); }

static int control(usb_device_handle_t dev, uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue,
                   uint16_t wIndex, const void *out, uint16_t wLength, void *in)
{
    usb_setup_packet_t *setup = (usb_setup_packet_t *)s_ctrl->data_buffer;
    setup->bmRequestType = bmRequestType;
    setup->bRequest = bRequest;
    setup->wValue = wValue;
    setup->wIndex = wIndex;
    setup->wLength = wLength;
    if (out && wLength) memcpy(s_ctrl->data_buffer + sizeof(usb_setup_packet_t), out, wLength);
    s_ctrl->device_handle = dev;
    s_ctrl->bEndpointAddress = 0;
    s_ctrl->num_bytes = sizeof(usb_setup_packet_t) + wLength;
    s_ctrl->callback = ctrl_cb;
    s_ctrl->timeout_ms = 1000;
    if (usb_host_transfer_submit_control(s_usb, s_ctrl) != ESP_OK) return -1;
    if (xSemaphoreTake(s_ctrl_done, pdMS_TO_TICKS(2000)) != pdTRUE) return -1;
    if (s_ctrl->status != USB_TRANSFER_STATUS_COMPLETED) return -1;
    const int n = s_ctrl->actual_num_bytes - (int)sizeof(usb_setup_packet_t);
    if (in && n > 0) memcpy(in, s_ctrl->data_buffer + sizeof(usb_setup_packet_t), (size_t)n);
    return n;
}

static void usb_app_task(void *arg)
{
    (void)arg;
    usb_ev_t ev;
    for (;;) {
        if (xQueueReceive(s_usb_ev, &ev, portMAX_DELAY) != pdTRUE) continue;
        if (ev.gone) { (void)ncm_host_device_gone(ev.dev); continue; }
        usb_device_handle_t dev;
        if (usb_host_device_open(s_usb, ev.addr, &dev) != ESP_OK) continue;
        const usb_device_desc_t *d;
        ESP_ERROR_CHECK(usb_host_get_device_descriptor(dev, &d));
        ESP_LOGI(TAG, "USB device %04x:%04x class %u", d->idVendor, d->idProduct, d->bDeviceClass);
        if (d->bDeviceClass == USB_CLASS_HUB) { usb_host_device_close(s_usb, dev); continue; }
        if (ncm_host_attach(s_usb, dev, control)) continue;   /* the driver keeps the handle */
        ESP_LOGW(TAG, "no NCM function (Quest: adb shell svc usb setScreenUnlockedFunctions ncm)");
        usb_host_device_close(s_usb, dev);
    }
}

static void usb_start(void)
{
    s_usb_ev = xQueueCreate(8, sizeof(usb_ev_t));
    s_ctrl_done = xSemaphoreCreateBinary();
    const usb_host_config_t host_cfg = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LOWMED,
        .peripheral_map = BIT0,   /* high-speed controller -> the board's hub -> Type-A ports */
    };
    ESP_ERROR_CHECK(usb_host_install(&host_cfg));
    xTaskCreatePinnedToCore(usb_lib_task, "usb_lib", 4096, NULL, 6, NULL, 0);
    const usb_host_client_config_t client_cfg = {
        .is_synchronous = false,
        .max_num_event_msg = 8,
        .async = {.client_event_callback = usb_client_cb, .callback_arg = NULL},
    };
    ESP_ERROR_CHECK(usb_host_client_register(&client_cfg, &s_usb));
    ncm_host_init();
    ESP_ERROR_CHECK(usb_host_transfer_alloc(sizeof(usb_setup_packet_t) + 256, 0, &s_ctrl));
    xTaskCreatePinnedToCore(usb_client_task, "usb_client", 4096, NULL, 7, NULL, 0);
    xTaskCreatePinnedToCore(usb_app_task, "usb_app", 4096, NULL, 4, NULL, 0);
}

/* ------------------------------------------------------------ Ethernet (PC tools on the LAN) */

static void got_ip(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    const ip_event_got_ip_t *e = data;
    ESP_LOGI(TAG, "Ethernet " IPSTR ": stream on UDP port %d", IP2STR(&e->ip_info.ip), STREAM_PORT);
}

static void eth_start(void)
{
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    mac_config.flags |= ETH_MAC_FLAG_PIN_TO_CORE;   /* its RX task on core 0, away from the decoder */
    eth_esp32_emac_config_t emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_config, &mac_config);
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = ESP_ETH_PHY_ADDR_AUTO;
    phy_config.reset_gpio_num = PHY_RESET_GPIO;
    esp_eth_phy_t *phy = esp_eth_phy_new_ip101(&phy_config);
    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth;
    if (esp_eth_driver_install(&eth_config, &eth) != ESP_OK) { ESP_LOGE(TAG, "no Ethernet"); return; }
    esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *netif = esp_netif_new(&netif_config);
    esp_netif_set_hostname(netif, "c5vrx-p4");
    ESP_ERROR_CHECK(esp_netif_attach(netif, esp_eth_new_netif_glue(eth)));
    s_eth = eth;
    s_eth_netif = netif;
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, got_ip, NULL));
    ESP_ERROR_CHECK(esp_eth_start(eth));
}

void stream_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    eth_start();
    usb_start();
    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    const struct sockaddr_in addr = {.sin_family = AF_INET, .sin_port = htons(STREAM_PORT),
                                     .sin_addr.s_addr = htonl(INADDR_ANY)};
    if (s_sock < 0 || bind(s_sock, (const struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "cannot bind UDP port %d", STREAM_PORT);
        return;
    }
    int sz = 64 * 1024;
    setsockopt(s_sock, SOL_SOCKET, SO_SNDBUF, &sz, sizeof(sz));
    /* The sender on core 0 with the fill task: on core 1, below the decoder,
     * the decoder's bursts held it off long enough to overflow the ring (~5 %
     * of the rows dropped, 2026-11-23). Hellos and commands above it. lwIP
     * core locking: a sendto (status, the lwIP path) runs in the sender. */
    const gpio_config_t led = {.pin_bit_mask = 1ULL << LED_GPIO, .mode = GPIO_MODE_OUTPUT};
    gpio_set_level(LED_GPIO, 1);   /* off */
    gpio_config(&led);
    gpio_set_level(LED_GPIO, 1);
    const esp_timer_create_args_t ta = {.callback = led_tick, .name = "led"};
    ESP_ERROR_CHECK(esp_timer_create(&ta, &s_led_timer));
    xTaskCreatePinnedToCore(command_task, "stream_cmd", 3072, NULL, 12, NULL, 0);
    xTaskCreatePinnedToCore(sender_task, "stream", 4096, NULL, 10, NULL, 0);
    ESP_LOGI(TAG, "USB host on the Type-A ports (Quest, NCM: P4 %s), UDP port %d", NCM_HOST_IP, STREAM_PORT);
}
