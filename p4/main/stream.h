/* C5VRX P4 -> Meta Quest (Waveshare ESP32-P4-Module-DEV-KIT build): the
 * decoded picture leaves line by line as UDP, uncompressed, over the Quest's
 * USB network link (CDC-NCM, P4 192.168.7.1) or Ethernet. See docs/P4QUEST.md.
 *
 * Wire format (little endian), UDP from the P4's port STREAM_PORT to the
 * address that last sent a hello there:
 *   header, 12 bytes: 'C' '5' version(1) type, u32 sequence (all datagrams),
 *                     u16 field counter, u16 height
 *   type 1 (rows): flags byte at offset 3 of each row, then per row
 *     u16 row, u8 flags, u8 0, Y[512], Cb[64], Cr[64]
 *     row: frame row (interlaced: 0..height-1, field parity = row & 1;
 *     progressive: 0..height/2-1, each a field line); height: 576 PAL, 480 NTSC
 *   type 2 (status, 4/s): text "key=value ..." (the C5's status, decoder state)
 * Chroma: one Cb and one Cr per 8 pixels (the decoder's 64 windows across
 * the line; window i centred on pixel 8i + 4).
 * To the P4: "H" (hello, at least every STREAM_HELLO_TIMEOUT_MS), "K<key>",
 * "C<idx>", "A" as the C5 link's commands (link.h). */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "cvbs.h"

#define STREAM_PORT             30210
#define STREAM_HELLO_TIMEOUT_MS 3000
#define STREAM_CW               64      /* chroma values per line */

#define STREAM_ROW_PAL      0x01
#define STREAM_ROW_PROG     0x02
#define STREAM_ROW_COLOUR   0x04

typedef struct {
    uint16_t row, field, height;
    uint8_t flags;
    uint8_t pad[9];
    uint8_t y[CVBS_W];
    uint8_t cb[STREAM_CW], cr[STREAM_CW];
} __attribute__((aligned(16))) stream_row_t;

/* Networking (USB NCM to the Quest, Ethernet) and the sender task. */
void stream_start(void);
/* Decoder side (core 1): a free row slot, or NULL (nobody listening, or the
 * ring is full: the row is dropped); stream_row_put() queues it. */
stream_row_t *stream_row_get(void);
void stream_row_put(void);
/* The NCM device went away (ncm_host.c). */
void stream_link_down(void);
void stream_debug_print(void);
