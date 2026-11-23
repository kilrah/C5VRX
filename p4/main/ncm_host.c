/* USB host driver for a CDC-NCM device (the Quest with its USB function set to
 * NCM), taken from the DJI-on-Quest bridge (its p4-mitm/main/ncm_host.c). */
#include "ncm_host.h"

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_net_stack.h"
#include "lwip/etharp.h"
#include "lwip/tcpip.h"
#include "dhcpserver/dhcpserver.h"
#include "usb/usb_helpers.h"
#include "stream.h"

static const char *TAG = "ncm";

// CDC NCM (USB-IF "Network Control Model" 1.0)
#define NCM_SUBCLASS            0x0D
#define CDC_DATA_CLASS          0x0A
#define REQ_GET_NTB_PARAMETERS  0x80
#define REQ_SET_NTB_INPUT_SIZE  0x86
#define NTH16_SIGNATURE         0x484D434E // "NCMH"
#define NDP16_SIGNATURE_NCM0    0x304D434E // "NCM0" (no CRC)
#define NDP16_SIGNATURE_NCM1    0x314D434E // "NCM1" (CRC appended)

#define NTB_SIZE      16384 // our receive buffer; told to the device with SET_NTB_INPUT_SIZE
#define RX_XFERS      4
#define TX_XFERS      6
#define TX_XFER_SIZE  16384 // one NTB: up to TX_MAX_DGRAMS Ethernet frames (<= 1514 B each) plus headers
#define TX_MAX_DGRAMS 12    // datagrams per NTB (fewer if the device says so)
#define TX_INFLIGHT   2     // transfers on the bus before frames start to collect in the next NTB

typedef struct {
    esp_netif_driver_base_t base; // must be first: esp-netif hands this pointer back to us
    usb_host_client_handle_t client;
    usb_device_handle_t dev;
    uint8_t comm_if, data_if, ep_in, ep_out;
    uint16_t out_align, out_divisor;
    uint32_t out_max;            // device's largest NTB (dwNtbOutMaxSize), capped at TX_XFER_SIZE
    uint16_t out_max_dgrams;     // datagrams per NTB, at most TX_MAX_DGRAMS
    uint16_t tx_seq;
    bool up;
    usb_transfer_t *rx[RX_XFERS];
    usb_transfer_t *tx[TX_XFERS];
    QueueHandle_t tx_free;
    // Transmit batching (guarded by tx_lock): the NTB being filled, its datagram count and length
    SemaphoreHandle_t tx_lock;
    usb_transfer_t *pend;
    uint16_t pend_n, pend_len, ndp_off, data_off;
    int inflight;
} ncm_t;

static ncm_t s_ncm;
static esp_netif_t *s_netif;
static volatile uint32_t s_rx_ntb, s_rx_frames, s_rx_bad, s_tx_frames, s_tx_ntb, s_tx_err, s_tx_drop;

static inline uint16_t rd16(const uint8_t *p) { return p[0] | (p[1] << 8); }
static inline uint32_t rd32(const uint8_t *p) { return rd16(p) | ((uint32_t)rd16(p + 2) << 16); }
static inline void wr16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static inline void wr32(uint8_t *p, uint32_t v) { wr16(p, v); wr16(p + 2, v >> 16); }

// ------------------------------------------------------------------ receive: NTB16 -> Ethernet frames

static void free_rx(void *h, void *buffer)
{
    free(buffer);
}

static void parse_ntb(const uint8_t *p, size_t len)
{
    if (len < 12 || rd32(p) != NTH16_SIGNATURE) {
        s_rx_bad++;
        return;
    }
    s_rx_ntb++;
    uint16_t ndp = rd16(p + 10);
    int guard = 8;
    while (ndp && ndp + 8 <= len && guard--) {
        uint32_t sig = rd32(p + ndp);
        if (sig != NDP16_SIGNATURE_NCM0 && sig != NDP16_SIGNATURE_NCM1) break;
        uint16_t ndp_len = rd16(p + ndp + 4);
        for (size_t e = ndp + 8; e + 4 <= ndp + ndp_len && e + 4 <= len; e += 4) {
            uint16_t off = rd16(p + e), dlen = rd16(p + e + 2);
            if (!off || !dlen) break; // terminating entry
            if (off + dlen > len) break;
            uint8_t *frame = malloc(dlen);
            if (!frame) break;
            memcpy(frame, p + off, dlen);
            s_rx_frames++;
            if (s_netif) esp_netif_receive(s_netif, frame, dlen, frame);
            else free(frame);
        }
        ndp = rd16(p + ndp + 6); // next NDP
    }
}

static void rx_cb(usb_transfer_t *t)
{
    if (t->status == USB_TRANSFER_STATUS_COMPLETED) {
        if (t->actual_num_bytes) parse_ntb(t->data_buffer, t->actual_num_bytes);
    } else if (t->status == USB_TRANSFER_STATUS_NO_DEVICE || t->status == USB_TRANSFER_STATUS_CANCELED) {
        return;
    }
    if (s_ncm.up && t->device_handle == s_ncm.dev) usb_host_transfer_submit(t);
}

// ------------------------------------------------------------------ transmit: Ethernet frames -> NTB16
//
// Several frames share one NTB (one USB transfer): the host stack manages only ~6,500 transfers a
// second (P4 -> Quest, 2026-11-23: ~68 Mbit/s with one 1300-byte datagram per transfer). A frame
// goes out at once while fewer than TX_INFLIGHT transfers are on the bus; otherwise it waits in the
// pending NTB, which leaves when a transfer completes or when it is full. So a lone frame is not
// delayed, and under load each transfer carries what came in meanwhile.
//
// NTB layout: [NTH16 12 B][NDP16 at ndp_off: 8 B + (max datagrams + 1) x 4 B][datagrams, each aligned]

static inline uint16_t align_up(uint32_t v, uint16_t a) { return a ? (v + a - 1) / a * a : v; }

/** Start a new pending NTB in transfer t (lock held). */
static void ntb_open(ncm_t *n, usb_transfer_t *t)
{
    uint8_t *b = t->data_buffer;
    memset(b, 0, n->data_off);
    wr32(b, NTH16_SIGNATURE);
    wr16(b + 4, 12);
    wr16(b + 10, n->ndp_off);
    wr32(b + n->ndp_off, NDP16_SIGNATURE_NCM0);
    wr16(b + n->ndp_off + 4, 8 + 4 * (n->out_max_dgrams + 1));
    n->pend = t;
    n->pend_n = 0;
    n->pend_len = n->data_off;
}

static void tx_cb(usb_transfer_t *t);

/** Submit the pending NTB (lock held). */
static void ntb_submit(ncm_t *n)
{
    usb_transfer_t *t = n->pend;
    n->pend = NULL;
    uint8_t *b = t->data_buffer;
    wr16(b + 6, n->tx_seq++);
    wr16(b + 8, n->pend_len);
    t->device_handle = n->dev;
    t->bEndpointAddress = n->ep_out;
    t->num_bytes = n->pend_len;
    t->callback = tx_cb;
    t->flags = USB_TRANSFER_FLAG_ZERO_PACK; // end the NTB explicitly when it fills its last packet
    t->timeout_ms = 0;
    if (usb_host_transfer_submit(t) != ESP_OK) {
        s_tx_err++;
        s_tx_drop += n->pend_n;
        xQueueSend(n->tx_free, &t, 0);
        return;
    }
    n->inflight++;
    s_tx_ntb++;
    s_tx_frames += n->pend_n;
}

static void tx_cb(usb_transfer_t *t)
{
    ncm_t *n = &s_ncm;
    xSemaphoreTake(n->tx_lock, portMAX_DELAY);
    n->inflight--;
    xQueueSend(n->tx_free, &t, 0);
    if (n->pend && n->pend_n && n->up) ntb_submit(n); // what collected while the bus was busy
    xSemaphoreGive(n->tx_lock);
}

static inline bool ntb_fits(const ncm_t *n, size_t len)
{
    return n->pend_n < n->out_max_dgrams && align_up(n->pend_len, n->out_divisor) + len <= n->out_max;
}

/* defer: leave the frame in the pending NTB (ncm_host_flush() or a full NTB sends it). */
static esp_err_t transmit(ncm_t *n, const void *buffer, size_t len, bool defer)
{
    if (!n->up) return ESP_FAIL;
    if (n->data_off + len > n->out_max) return ESP_ERR_INVALID_SIZE;
    xSemaphoreTake(n->tx_lock, portMAX_DELAY);
    if (n->pend && !ntb_fits(n, len)) ntb_submit(n); // full: send it now
    if (!n->pend) {
        usb_transfer_t *t;
        if (xQueueReceive(n->tx_free, &t, 0) != pdTRUE) {
            // All transfers busy: wait for one without holding the lock (tx_cb needs it)
            xSemaphoreGive(n->tx_lock);
            if (xQueueReceive(n->tx_free, &t, pdMS_TO_TICKS(10)) != pdTRUE) {
                s_tx_drop++;
                return ESP_ERR_TIMEOUT;
            }
            xSemaphoreTake(n->tx_lock, portMAX_DELAY);
        }
        if (!n->pend) {
            ntb_open(n, t);
        } else { // another sender opened one meanwhile
            xQueueSend(n->tx_free, &t, 0);
            if (!ntb_fits(n, len)) {
                xSemaphoreGive(n->tx_lock);
                s_tx_drop++;
                return ESP_ERR_TIMEOUT;
            }
        }
    }
    uint8_t *b = n->pend->data_buffer;
    uint16_t off = align_up(n->pend_len, n->out_divisor);
    memset(b + n->pend_len, 0, off - n->pend_len);
    memcpy(b + off, buffer, len);
    uint8_t *e = b + n->ndp_off + 8 + 4 * n->pend_n; // the entry after it stays 0,0 (terminator)
    wr16(e, off);
    wr16(e + 2, len);
    n->pend_n++;
    n->pend_len = off + len;
    if (!defer && n->inflight < TX_INFLIGHT) ntb_submit(n);
    xSemaphoreGive(n->tx_lock);
    return ESP_OK;
}

static esp_err_t ncm_transmit(void *h, void *buffer, size_t len)
{
    return transmit(h, buffer, len, false);
}

static esp_err_t post_attach(esp_netif_t *netif, void *args)
{
    ncm_t *n = args;
    n->base.netif = netif;
    const esp_netif_driver_ifconfig_t io = {
        .handle = n,
        .transmit = ncm_transmit,
        .driver_free_rx_buffer = free_rx,
    };
    return esp_netif_set_driver_config(netif, &io);
}

// ------------------------------------------------------------------ setup

void ncm_host_init(void)
{
    static esp_netif_ip_info_t ip;
    ip.ip.addr = esp_ip4addr_aton(NCM_HOST_IP);
    ip.gw.addr = 0; // no gateway: the Quest keeps using Wi-Fi for everything else
    ip.netmask.addr = esp_ip4addr_aton("255.255.255.0");

    esp_netif_inherent_config_t base = ESP_NETIF_INHERENT_DEFAULT_ETH();
    base.flags = ESP_NETIF_DHCP_SERVER | ESP_NETIF_FLAG_AUTOUP;
    base.ip_info = &ip;
    base.get_ip_event = 0;
    base.lost_ip_event = 0;
    base.if_key = "USB_NCM";
    base.if_desc = "ncm";
    base.route_prio = 10;
    esp_netif_config_t cfg = {.base = &base, .driver = NULL, .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH};
    s_netif = esp_netif_new(&cfg);
    // Do not offer the P4 as router: the Quest must keep its default route (internet) on Wi-Fi
    esp_netif_dhcps_stop(s_netif);
    dhcps_offer_t no_router = 0;
    esp_netif_dhcps_option(s_netif, ESP_NETIF_OP_SET, ESP_NETIF_ROUTER_SOLICITATION_ADDRESS, &no_router,
                           sizeof(no_router));

    s_ncm.base.post_attach = post_attach;
    s_ncm.tx_free = xQueueCreate(TX_XFERS, sizeof(usb_transfer_t *));
    s_ncm.tx_lock = xSemaphoreCreateMutex();
    for (int i = 0; i < TX_XFERS; i++) {
        ESP_ERROR_CHECK(usb_host_transfer_alloc(TX_XFER_SIZE, 0, &s_ncm.tx[i]));
        xQueueSend(s_ncm.tx_free, &s_ncm.tx[i], 0);
    }
    for (int i = 0; i < RX_XFERS; i++) ESP_ERROR_CHECK(usb_host_transfer_alloc(NTB_SIZE, 0, &s_ncm.rx[i]));
    ESP_ERROR_CHECK(esp_netif_attach(s_netif, &s_ncm));

    uint8_t mac[6] = {0x02, 0x50, 0x34, 0x4d, 0x49, 0x01}; // locally administered
    esp_netif_set_mac(s_netif, mac);
    esp_netif_action_start(s_netif, NULL, 0, NULL);
}

/** Locate the NCM control interface and its data interface (alternate setting 1, bulk IN/OUT). */
static bool find_ncm(const usb_config_desc_t *cfg, ncm_t *n)
{
    int off = 0;
    const usb_standard_desc_t *d = (const usb_standard_desc_t *)cfg;
    bool have_comm = false;
    while ((d = usb_parse_next_descriptor_of_type(d, cfg->wTotalLength, USB_B_DESCRIPTOR_TYPE_INTERFACE, &off))) {
        const usb_intf_desc_t *intf = (const usb_intf_desc_t *)d;
        if (intf->bInterfaceClass == USB_CLASS_COMM && intf->bInterfaceSubClass == NCM_SUBCLASS) {
            n->comm_if = intf->bInterfaceNumber;
            have_comm = true;
        } else if (have_comm && intf->bInterfaceClass == CDC_DATA_CLASS && intf->bAlternateSetting == 1 &&
                   intf->bNumEndpoints == 2) {
            n->data_if = intf->bInterfaceNumber;
            n->ep_in = n->ep_out = 0;
            for (int i = 0; i < 2; i++) {
                int eoff = off;
                const usb_ep_desc_t *ep = usb_parse_endpoint_descriptor_by_index(intf, i, cfg->wTotalLength, &eoff);
                if (!ep) continue;
                if (USB_EP_DESC_GET_EP_DIR(ep)) n->ep_in = ep->bEndpointAddress;
                else n->ep_out = ep->bEndpointAddress;
            }
            return n->ep_in && n->ep_out;
        }
    }
    return false;
}

bool ncm_host_attach(usb_host_client_handle_t client, usb_device_handle_t dev, ncm_control_fn control)
{
    const usb_config_desc_t *cfg;
    if (usb_host_get_active_config_descriptor(dev, &cfg) != ESP_OK) return false;
    ncm_t *n = &s_ncm;
    if (n->up || !find_ncm(cfg, n)) return false;
    n->client = client;
    n->dev = dev;

    // NTB parameters: alignment rules for what we send
    uint8_t p[28] = {0};
    if (control(dev, 0xA1, REQ_GET_NTB_PARAMETERS, 0, n->comm_if, NULL, sizeof(p), p) == sizeof(p)) {
        uint32_t in_max = rd32(p + 4), out_max = rd32(p + 16);
        uint16_t out_dgrams = rd16(p + 26);
        n->out_divisor = rd16(p + 20);
        n->out_align = rd16(p + 24);
        n->out_max = out_max;
        n->out_max_dgrams = out_dgrams;
        ESP_LOGI(TAG, "NTB in max %u, out max %u (%u datagrams), out divisor %u, out alignment %u",
                 (unsigned)in_max, (unsigned)out_max, out_dgrams, n->out_divisor, n->out_align);
    }
    if (!n->out_max || n->out_max > TX_XFER_SIZE) n->out_max = TX_XFER_SIZE;
    if (!n->out_max_dgrams || n->out_max_dgrams > TX_MAX_DGRAMS) n->out_max_dgrams = TX_MAX_DGRAMS;
    if (!n->out_divisor) n->out_divisor = 4;
    n->ndp_off = align_up(12, n->out_align ? n->out_align : 4);
    n->data_off = n->ndp_off + 8 + 4 * (n->out_max_dgrams + 1);
    uint8_t in_size[4];
    wr32(in_size, NTB_SIZE);
    if (control(dev, 0x21, REQ_SET_NTB_INPUT_SIZE, 0, n->comm_if, in_size, 4, NULL) < 0)
        ESP_LOGW(TAG, "SET_NTB_INPUT_SIZE refused (device default must fit %d B)", NTB_SIZE);

    ESP_ERROR_CHECK(usb_host_interface_claim(client, dev, n->comm_if, 0));
    // Selecting the data interface's alternate setting 1 starts the link (carrier on for the device)
    if (control(dev, 0x01, USB_B_REQUEST_SET_INTERFACE, 1, n->data_if, NULL, 0, NULL) < 0) {
        ESP_LOGE(TAG, "SET_INTERFACE %u alt 1 failed", n->data_if);
        usb_host_interface_release(client, dev, n->comm_if);
        return false;
    }
    ESP_ERROR_CHECK(usb_host_interface_claim(client, dev, n->data_if, 1));
    n->up = true;
    for (int i = 0; i < RX_XFERS; i++) {
        usb_transfer_t *t = n->rx[i];
        t->device_handle = dev;
        t->bEndpointAddress = n->ep_in;
        t->num_bytes = NTB_SIZE;
        t->callback = rx_cb;
        t->timeout_ms = 0;
        ESP_ERROR_CHECK(usb_host_transfer_submit(t));
    }
    esp_netif_action_connected(s_netif, NULL, 0, NULL);
    esp_netif_dhcp_status_t dhcps = ESP_NETIF_DHCP_INIT;
    esp_netif_dhcps_get_status(s_netif, &dhcps);
    if (dhcps != ESP_NETIF_DHCP_STARTED) {
        esp_err_t err = esp_netif_dhcps_start(s_netif);
        ESP_LOGI(TAG, "DHCP server was %d, start: %s", dhcps, esp_err_to_name(err));
    }
    ESP_LOGI(TAG, "NCM link up (interfaces %u/%u, bulk IN 0x%02x OUT 0x%02x): P4 is %s, DHCP server running",
             n->comm_if, n->data_if, n->ep_in, n->ep_out, NCM_HOST_IP);
    return true;
}

void ncm_host_log_stats(void)
{
    if (!s_ncm.up) return;
    uint32_t a = s_rx_ntb, b = s_rx_frames, c = s_rx_bad, d = s_tx_ntb, f = s_tx_frames, e = s_tx_err,
             g = s_tx_drop;
    s_rx_ntb = s_rx_frames = s_rx_bad = s_tx_ntb = s_tx_frames = s_tx_err = s_tx_drop = 0;
    ESP_LOGI(TAG, "rx %" PRIu32 " NTB / %" PRIu32 " frames (bad %" PRIu32 ")  tx %" PRIu32 " NTB / %" PRIu32
             " frames (err %" PRIu32 ", dropped %" PRIu32 ")", a, b, c, d, f, e, g);
}

bool ncm_host_device_gone(usb_device_handle_t dev)
{
    ncm_t *n = &s_ncm;
    if (!n->up || dev != n->dev) return false;
    n->up = false;
    xSemaphoreTake(n->tx_lock, portMAX_DELAY);
    if (n->pend) { // never submitted: back to the free list (submitted ones come back through tx_cb)
        xQueueSend(n->tx_free, &n->pend, 0);
        n->pend = NULL;
    }
    xSemaphoreGive(n->tx_lock);
    stream_link_down(); // the Quest behind this link is gone
    esp_netif_action_disconnected(s_netif, NULL, 0, NULL);
    usb_host_interface_release(n->client, dev, n->data_if);
    usb_host_interface_release(n->client, dev, n->comm_if);
    usb_host_device_close(n->client, dev);
    n->dev = NULL;
    ESP_LOGI(TAG, "NCM link down");
    return true;
}

bool ncm_host_peer_mac(uint32_t ip, uint8_t mac[6], uint8_t own[6])
{
    if (!s_ncm.up || !s_netif) return false;
    struct netif *nif = esp_netif_get_netif_impl(s_netif);
    const ip4_addr_t a = {.addr = ip};
    struct eth_addr *ea = NULL;
    const ip4_addr_t *ia = NULL;
    LOCK_TCPIP_CORE();
    const ssize_t i = etharp_find_addr(nif, &a, &ea, &ia);
    if (i >= 0) memcpy(mac, ea->addr, 6);
    UNLOCK_TCPIP_CORE();
    esp_netif_get_mac(s_netif, own);
    return i >= 0;
}

esp_err_t ncm_host_send_frame(const void *frame, size_t len)
{
    if (!s_ncm.up) return ESP_FAIL;
    return transmit(&s_ncm, frame, len, true);
}

void ncm_host_flush(void)
{
    ncm_t *n = &s_ncm;
    if (!n->up) return;
    xSemaphoreTake(n->tx_lock, portMAX_DELAY);
    /* With transfers on the bus the pending NTB leaves when one completes (tx_cb). */
    if (n->pend && n->pend_n && n->inflight < TX_INFLIGHT) ntb_submit(n);
    xSemaphoreGive(n->tx_lock);
}
