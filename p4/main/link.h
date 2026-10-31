/* C5VRX P4 display by Twotoz and the C5VRX contributors: the UART to the
 * C5 receiver (protocol: main/p4link.c in the C5 firmware). */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define LINK_CHANNELS 48

typedef struct {
    bool online;                /* a status line within the last second */
    int idx, freq, sig, gain, scan_pct, cn_x10, cfo_khz;
    bool afc_lock, video, menu, busy, scan, afc;
    char ch[8], std[8], det[8], gain_mode[16], profile[16], demod[16], output[16], dc[12];
    uint32_t seq;               /* bumps with every status line */
} c5_status_t;

void link_start(void);
void link_status(c5_status_t *out);
void link_debug_print(void);   /* TEMP */
/* Channel table (band letter, MHz) once the C5 sent it; false until then. */
bool link_channel(int idx, char *band, int *mhz);

void link_key(char key);        /* a C5 console key, see p4link.c */
void link_tune(int idx);
void link_afc_toggle(void);
