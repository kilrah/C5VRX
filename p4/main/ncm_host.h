#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "usb/usb_host.h"

/**
 * USB host driver for a CDC-NCM network function (the Meta Quest with `cmd usb set-functions ncm`).
 *
 * The link appears in the P4's IP stack as interface "ncm" with the fixed address NCM_HOST_IP and a
 * DHCP server, so the device side (Quest: Android EthernetService on usb0, DHCP client) gets an
 * address automatically. Everything listening on all interfaces (e.g. the capture server on port
 * 30200) is then reachable from the Quest at NCM_HOST_IP.
 */
#define NCM_HOST_IP "192.168.7.1"

/** Control transfer helper provided by the caller: returns data-stage bytes or -1. */
typedef int (*ncm_control_fn)(usb_device_handle_t dev, uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue,
                              uint16_t wIndex, const void *out, uint16_t wLength, void *in);

/** Create the network interface (once, at startup). */
void ncm_host_init(void);

/**
 * Try to start NCM on a newly opened device. Returns true if the device has an NCM function and
 * the driver took ownership of the handle (released again by ncm_host_device_gone()).
 */
bool ncm_host_attach(usb_host_client_handle_t client, usb_device_handle_t dev, ncm_control_fn control);

/** Log and reset per-interval packet counters (no-op while the link is down). */
void ncm_host_log_stats(void);

/**
 * Raw sending past lwIP (the video stream): the peer's MAC from lwIP's ARP table (ip in network
 * order; false until lwIP has talked to it) and the link's own MAC; then whole Ethernet frames,
 * held in the pending NTB until ncm_host_flush(), a full NTB or a completing transfer.
 */
bool ncm_host_peer_mac(uint32_t ip, uint8_t mac[6], uint8_t own[6]);
esp_err_t ncm_host_send_frame(const void *frame, size_t len);
/** Send what ncm_host_send_frame() collected (or leave it to the next completing transfer). */
void ncm_host_flush(void);

/** Call when a device disappears. Returns true if it was the NCM device (handle closed). */
bool ncm_host_device_gone(usb_device_handle_t dev);
