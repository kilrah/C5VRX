/* C5VRX-5 by Twotoz and the C5VRX contributors: flight log format
 * (docs/FLIGHT_LOG.md; pure logic, host test test/test_flightlog.c; the web
 * decoder web/flight-log.js and tools/flight_log.py read the same layout).
 *
 * Partition "flightlog" (1.5 MiB, little-endian throughout):
 *   sector 0            header (magic C5LG, layout, format)
 *   sectors 1..255      record ring, one 4 KiB block per sector
 *   sectors 256..383    snapshot ring, 64 slots of 8 KiB
 *
 * Block: a 20-byte head written when the block is started, records appended
 * in place (NOR flash programs 1 -> 0 into the erased area, no erase), and
 * the CRC-32 of bytes 0..4091 in the last four bytes when it is closed. An
 * open block (power pulled) has no CRC; its records still read, up to the
 * first type 0xFF (erased) or a record that does not fit. Every boot starts
 * a new block, so a block never mixes sessions.
 *
 * Record: type (1 byte), length (1 byte), payload. Readers skip unknown types
 * by length; newer firmware only appends fields at the end of a payload.
 * FLOG_FORMAT changes only when that rule cannot be kept. */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "experimental.h"

#define FLOG_FORMAT          1u
#define FLOG_SECTOR          4096u
#define FLOG_RECORD_BLOCKS   255u
#define FLOG_SNAP_FIRST      256u          /* sector index */
#define FLOG_SNAP_SLOTS      64u
#define FLOG_SNAP_SECTORS    2u            /* per slot */
#define FLOG_SNAP_HEAD       64u
#define FLOG_SNAP_BYTES      (FLOG_SNAP_SECTORS * FLOG_SECTOR - FLOG_SNAP_HEAD)   /* 8128 IQ bytes */
#define FLOG_PARTITION_BYTES ((FLOG_SNAP_FIRST + FLOG_SNAP_SLOTS * FLOG_SNAP_SECTORS) * FLOG_SECTOR)

#define FLOG_MAGIC_HEADER    0x474c3543u   /* "C5LG" */
#define FLOG_MAGIC_BLOCK     0x424c3543u   /* "C5LB" */
#define FLOG_MAGIC_SNAP      0x534c3543u   /* "C5LS" */
#define FLOG_BLOCK_HEAD      20u
#define FLOG_BLOCK_CRC_AT    (FLOG_SECTOR - 4u)
#define FLOG_BLOCK_ROOM      (FLOG_BLOCK_CRC_AT - FLOG_BLOCK_HEAD)

enum {
    FLOG_T_EXP = 7,       /* full experimental state: version, count, values */
    FLOG_T_SESSION = 1,   /* identity, settings, reset reason (boot) */
    FLOG_T_CALIB = 2,     /* what the startup phase calibrated, filters, DC tables, sampling phase */
    FLOG_T_CARRIER = 3,   /* channel, AFC, deviation, profile, standard */
    FLOG_T_SECOND = 4,    /* per-second summary */
    FLOG_T_EVENT = 5,     /* rare state changes */
    FLOG_T_TRACE = 6,     /* dropout onset, 16 x 50 ms before or after */
};

enum {
    FLOG_E_CHANNEL = 1, FLOG_E_PROFILE = 2, FLOG_E_KP = 3, FLOG_E_CAL_START = 4, FLOG_E_CAL_END = 5,
    FLOG_E_DC_CAL_NEEDED = 6, FLOG_E_GAIN_MODE = 7, FLOG_E_MENU = 8, FLOG_E_FAULT = 9,
    FLOG_E_DROPOUT = 10, FLOG_E_RECOVERED = 11, FLOG_E_SNAPSHOT = 12, FLOG_E_DROPPED = 13,
    FLOG_E_ATTACK = 14,   /* the window that made the gain step down (receiver.c) */
    FLOG_E_EXP = 17,     /* item, old, new, readback OK */
    FLOG_E_OUTPUT_GAP = 15, FLOG_E_OUTPUT_UNLOCK = 16,
    FLOG_E_OUTPUT_AUTO = 18,   /* AUTO OUTPUT applied: gain x8, source tip code, sync depth x4 (control.c) */
    FLOG_E_PAUSE = 19,         /* receiver pause ended: duration ms (u32), first 8 characters of its owner tag */
};

enum { FLOG_SNAP_DROPOUT = 1, FLOG_SNAP_VQ_DROP = 2, FLOG_SNAP_FIRST_CARRIER = 3, FLOG_SNAP_PERIODIC = 4, FLOG_SNAP_OVERLOAD = 5,
       FLOG_SNAP_EDGE = 6, FLOG_SNAP_RECOVER = 7, FLOG_SNAP_OUTPUT_UNLOCK = 8 };

/* Verified program identity. The firmware/session SHA and kp level identify
 * the exact generated LUT; these IDs bind the instruction/model family. */
enum { FLOG_DEMOD_UNKNOWN = 0, FLOG_DEMOD_U85Q_1A09F5C092E8 = 1, FLOG_DEMOD_U85Q_LP_1A09F5C092E8 = 2, FLOG_DEMOD_P64_R1 = 3,
       FLOG_DEMOD_U50P64_R1 = 4, FLOG_DEMOD_D32_R1 = 5 };
/* Bits 1..7 were the sync flywheel's (removed 2026-10-25); kept so old
 * logs decode. Current firmware sets only DEMOD_VERIFIED. */
enum {
    FLOG_OUT_DEMOD_VERIFIED = 1u << 0, FLOG_OUT_FLYWHEEL_REQUESTED = 1u << 1,
    FLOG_OUT_FLYWHEEL_ACTIVE = 1u << 2, FLOG_OUT_OBSERVER_FRESH = 1u << 3,
    FLOG_OUT_H_TRUSTED = 1u << 4, FLOG_OUT_V_TRUSTED = 1u << 5,
    FLOG_OUT_CAMERA_PRESENT = 1u << 6, FLOG_OUT_FLAG_FORMAT = 1u << 7,
};

/* Fixed little-endian helpers (no struct packing assumptions). */
static inline void flog_put16(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline void flog_put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static inline uint32_t flog_get16(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8; }
static inline uint32_t flog_get32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static inline uint32_t flog_crc32(uint32_t crc, const uint8_t *p, size_t n)
{
    crc = ~crc;
    while (n--) {
        crc ^= *p++;
        for (int k = 0; k < 8; ++k) crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

/* Header sector. */
static inline void flog_header(uint8_t h[32])
{
    memset(h, 0xff, 32);
    flog_put32(h, FLOG_MAGIC_HEADER);
    h[4] = FLOG_FORMAT; h[5] = 0;
    flog_put16(h + 6, FLOG_SECTOR);
    flog_put16(h + 8, FLOG_RECORD_BLOCKS);
    flog_put16(h + 10, FLOG_SNAP_FIRST);
    flog_put16(h + 12, FLOG_SNAP_SLOTS);
    flog_put16(h + 14, FLOG_SNAP_BYTES);
    flog_put32(h + 16, flog_crc32(0, h, 16));
}

static inline bool flog_header_ok(const uint8_t h[32])
{
    uint8_t want[32];
    flog_header(want);
    return !memcmp(h, want, 20);
}

/* Block head: magic, format, flags, reserved, session, sequence, t0 (ms). */
static inline void flog_block_head(uint8_t b[FLOG_BLOCK_HEAD], uint32_t session, uint32_t seq, uint32_t t0_ms)
{
    flog_put32(b, FLOG_MAGIC_BLOCK);
    b[4] = FLOG_FORMAT; b[5] = 0; b[6] = 0; b[7] = 0;
    flog_put32(b + 8, session);
    flog_put32(b + 12, seq);
    flog_put32(b + 16, t0_ms);
}

typedef struct { bool valid; uint32_t session, seq, t0_ms; } flog_head_t;

static inline flog_head_t flog_parse_head(const uint8_t *b)
{
    flog_head_t h = {0};
    if (flog_get32(b) != FLOG_MAGIC_BLOCK || b[4] != FLOG_FORMAT) return h;
    h.valid = true;
    h.session = flog_get32(b + 8);
    h.seq = flog_get32(b + 12);
    h.t0_ms = flog_get32(b + 16);
    return h;
}

/* The block to write next and the next session, from every block head
 * (fed one at a time, no table): after the highest sequence, wrapping. */
typedef struct { uint32_t next_seq, next_session; unsigned next_block; uint32_t top, session; bool any; } flog_resume_t;

static inline void flog_resume_init(flog_resume_t *r) { *r = (flog_resume_t){1u, 1u, 0u, 0u, 0u, false}; }

static inline void flog_resume_add(flog_resume_t *r, unsigned block, unsigned blocks, flog_head_t h)
{
    if (!h.valid) return;
    if (!r->any || h.seq > r->top) { r->top = h.seq; r->next_block = (block + 1u) % blocks; r->next_seq = h.seq + 1u; }
    if (!r->any || h.session > r->session) { r->session = h.session; r->next_session = h.session + 1u; }
    r->any = true;
}

/* Walk the records of one block image; returns the bytes used. */
typedef bool (*flog_visit_t)(void *ctx, uint8_t type, const uint8_t *payload, uint8_t len);
static inline unsigned flog_walk(const uint8_t *block, flog_visit_t visit, void *ctx)
{
    unsigned at = FLOG_BLOCK_HEAD;
    while (at + 2u <= FLOG_BLOCK_CRC_AT) {
        const uint8_t type = block[at], len = block[at + 1u];
        if (type == 0xffu || type == 0u || at + 2u + len > FLOG_BLOCK_CRC_AT) break;
        if (visit && !visit(ctx, type, block + at + 2u, len)) break;
        at += 2u + len;
    }
    return at - FLOG_BLOCK_HEAD;
}

static inline bool flog_block_closed(const uint8_t *block)
{
    return flog_get32(block + FLOG_BLOCK_CRC_AT) == flog_crc32(0, block, FLOG_BLOCK_CRC_AT);
}

/* Per-second summary payload (FLOG_T_SECOND), append-only in format 1: 34
 * original bytes, 38 with received level, 42 gain reasons, 50 C/N-AUTO and
 * 69 with verified demod/output-observer telemetry. Readers take 34 and up. */
typedef struct {
    uint32_t t_s;
    int16_t vq_snr_x10;      /* VIDEO_Q, 0 = no line */
    uint16_t vq_lines_x100;
    int16_t cn_x10;          /* C/N estimate (-990 unknown) */
    uint8_t video_pct;       /* windows with sync */
    uint8_t dropout;         /* 0 none, else diag category + 1 */
    uint8_t gain_min, gain_mean, gain_max;
    uint8_t clip_q;          /* clip per mille / 4 */
    uint8_t gain_writes, overloads;           /* this second, saturating */
    int8_t dc_i, dc_q;       /* active row residual, 0.05 cells */
    int8_t off_i, off_q;     /* active row offset, codes */
    uint8_t servo_writes;    /* this second, saturating */
    int16_t afc_offset_khz, afc_residual_khz;
    uint8_t afc_corrections; /* this second */
    uint8_t demod;           /* profile (2 bits) | kp level (3 bits) << 2 | lp << 5 */
    uint8_t sphase;          /* state code */
    int8_t temp_c;
    uint16_t flags;          /* diag flags (low byte) | menu/scan/startup/paused/fault (high) */
    uint8_t rx_state;        /* rx_state.h: state + 1 | reason << 3; 0 = not recorded */
    int16_t level_dbm_x10;   /* carrier at the chip pin (rx_level.h), FLOG_LEVEL_UNKNOWN if none */
    int8_t noise_dbm;        /* the noise it is measured against, dBm */
    uint8_t level_flags;     /* bit 0 valid, 1 above the noise, 2 exact (G54+), 3 noise learned */
    uint8_t writes_attack;   /* gain writes down (folding or saturation), this second, saturating */
    uint8_t writes_release;  /* gain writes up */
    uint8_t sentinel;        /* fast-path overloads forwarded to the gain core */
    uint8_t overloads_after_write;   /* overloads within 5 ms of a gain write (transient suspects) */
    int16_t cn_power_x10, cn_m2m4_x10, cn_env_x10;   /* C/N-AUTO estimators (cn_auto.h), -990 unknown */
    uint8_t cn_conf_src;     /* confidence (2 bits) | source << 2 */
    uint8_t cn_flags;        /* CNA_F_* */
    uint8_t output_flags;    /* FLOG_OUT_*: current provenance/validity snapshot */
    uint8_t demod_id;        /* verified program/model enum above, else 0 */
    uint8_t output_standard; /* actual guard grid: 0 unknown, 1 NTSC, 2 PAL */
    uint8_t output_level;    /* verified output setting 0..9, 0xff unknown */
    uint16_t output_samples, output_fresh_samples; /* task-side 2-ms observations */
    uint8_t native_h_lock_pct, native_v_lock_pct;  /* of fresh samples; 0xff unknown */
    uint16_t native_h_lines; /* in-phase observed camera H pulses */
    uint16_t repaired_tips, suppressed_tips;       /* assisted vs native scheduled tips */
    uint16_t longest_h_unlock_ms;                  /* only during fresh observation */
    uint8_t output_observer_gaps;
    /* Picture measure (picture_sync.h, 71-byte records since 2026-10-28):
     * sync pulses found / expected in the decoded control windows, % (0xff
     * unknown), and how many windows. FIELD_DIAG video_pct stays the AFC's. */
    uint8_t picture_pct, picture_windows;
    uint32_t exp_mask;
    uint8_t exp_values[EXP_ITEMS];
    uint8_t exp_version, exp_mixed; /* setting changed during this second */
} flog_second_t;
#define FLOG_LEVEL_UNKNOWN (-32768)

#define FLOG_SECOND_LEN (77u + EXP_ITEMS)
#define FLOG_SECOND_LEN_V1 34u

static inline uint8_t flog_sat8(uint32_t v) { return (uint8_t)(v > 255u ? 255u : v); }
static inline int8_t flog_clamp8(int v) { return (int8_t)(v > 127 ? 127 : v < -127 ? -127 : v); }

static inline void flog_pack_second(const flog_second_t *s, uint8_t p[FLOG_SECOND_LEN])
{
    flog_put32(p, s->t_s);
    flog_put16(p + 4, (uint16_t)s->vq_snr_x10);
    flog_put16(p + 6, s->vq_lines_x100);
    flog_put16(p + 8, (uint16_t)s->cn_x10);
    p[10] = s->video_pct; p[11] = s->dropout;
    p[12] = s->gain_min; p[13] = s->gain_mean; p[14] = s->gain_max;
    p[15] = s->clip_q; p[16] = s->gain_writes; p[17] = s->overloads;
    p[18] = (uint8_t)s->dc_i; p[19] = (uint8_t)s->dc_q; p[20] = (uint8_t)s->off_i; p[21] = (uint8_t)s->off_q;
    p[22] = s->servo_writes;
    flog_put16(p + 23, (uint16_t)s->afc_offset_khz);
    flog_put16(p + 25, (uint16_t)s->afc_residual_khz);
    p[27] = s->afc_corrections; p[28] = s->demod; p[29] = s->sphase; p[30] = (uint8_t)s->temp_c;
    flog_put16(p + 31, s->flags);
    p[33] = s->rx_state;
    flog_put16(p + 34, (uint16_t)s->level_dbm_x10);
    p[36] = (uint8_t)s->noise_dbm; p[37] = s->level_flags;
    p[38] = s->writes_attack; p[39] = s->writes_release; p[40] = s->sentinel; p[41] = s->overloads_after_write;
    flog_put16(p + 42, (uint16_t)s->cn_power_x10); flog_put16(p + 44, (uint16_t)s->cn_m2m4_x10);
    flog_put16(p + 46, (uint16_t)s->cn_env_x10); p[48] = s->cn_conf_src; p[49] = s->cn_flags;
    p[50] = s->output_flags; p[51] = s->demod_id; p[52] = s->output_standard; p[53] = s->output_level;
    flog_put16(p + 54, s->output_samples); flog_put16(p + 56, s->output_fresh_samples);
    p[58] = s->native_h_lock_pct; p[59] = s->native_v_lock_pct;
    flog_put16(p + 60, s->native_h_lines); flog_put16(p + 62, s->repaired_tips);
    flog_put16(p + 64, s->suppressed_tips); flog_put16(p + 66, s->longest_h_unlock_ms);
    p[68] = s->output_observer_gaps;
    p[69] = s->picture_pct; p[70] = s->picture_windows;
    flog_put32(p + 71, s->exp_mask);
    p[75] = s->exp_version; p[76] = s->exp_mixed;
    memcpy(p + 77, s->exp_values, EXP_ITEMS);
}

static inline bool flog_unpack_second(const uint8_t *p, uint8_t len, flog_second_t *s)
{
    if (len < FLOG_SECOND_LEN_V1) return false;
    s->t_s = flog_get32(p);
    s->vq_snr_x10 = (int16_t)flog_get16(p + 4);
    s->vq_lines_x100 = (uint16_t)flog_get16(p + 6);
    s->cn_x10 = (int16_t)flog_get16(p + 8);
    s->video_pct = p[10]; s->dropout = p[11];
    s->gain_min = p[12]; s->gain_mean = p[13]; s->gain_max = p[14];
    s->clip_q = p[15]; s->gain_writes = p[16]; s->overloads = p[17];
    s->dc_i = (int8_t)p[18]; s->dc_q = (int8_t)p[19]; s->off_i = (int8_t)p[20]; s->off_q = (int8_t)p[21];
    s->servo_writes = p[22];
    s->afc_offset_khz = (int16_t)flog_get16(p + 23);
    s->afc_residual_khz = (int16_t)flog_get16(p + 25);
    s->afc_corrections = p[27]; s->demod = p[28]; s->sphase = p[29]; s->temp_c = (int8_t)p[30];
    s->flags = (uint16_t)flog_get16(p + 31);
    s->rx_state = p[33];
    s->level_dbm_x10 = len >= 38u ? (int16_t)flog_get16(p + 34) : (int16_t)FLOG_LEVEL_UNKNOWN;
    s->noise_dbm = len >= 38u ? (int8_t)p[36] : 0;
    s->level_flags = len >= 38u ? p[37] : 0;
    s->writes_attack = len >= 42u ? p[38] : 0;
    s->writes_release = len >= 42u ? p[39] : 0;
    s->sentinel = len >= 42u ? p[40] : 0;
    s->overloads_after_write = len >= 42u ? p[41] : 0;
    s->cn_power_x10 = len >= 50u ? (int16_t)flog_get16(p + 42) : -990;
    s->cn_m2m4_x10 = len >= 50u ? (int16_t)flog_get16(p + 44) : -990;
    s->cn_env_x10 = len >= 50u ? (int16_t)flog_get16(p + 46) : -990;
    s->cn_conf_src = len >= 50u ? p[48] : 0;
    s->cn_flags = len >= 50u ? p[49] : 0;
    s->output_flags = len >= 69u ? p[50] : 0;
    s->demod_id = len >= 69u ? p[51] : FLOG_DEMOD_UNKNOWN;
    s->output_standard = len >= 69u ? p[52] : 0;
    s->output_level = len >= 69u ? p[53] : 0xffu;
    s->output_samples = len >= 69u ? (uint16_t)flog_get16(p + 54) : 0;
    s->output_fresh_samples = len >= 69u ? (uint16_t)flog_get16(p + 56) : 0;
    s->native_h_lock_pct = len >= 69u ? p[58] : 0xffu;
    s->native_v_lock_pct = len >= 69u ? p[59] : 0xffu;
    s->native_h_lines = len >= 69u ? (uint16_t)flog_get16(p + 60) : 0;
    s->repaired_tips = len >= 69u ? (uint16_t)flog_get16(p + 62) : 0;
    s->suppressed_tips = len >= 69u ? (uint16_t)flog_get16(p + 64) : 0;
    s->longest_h_unlock_ms = len >= 69u ? (uint16_t)flog_get16(p + 66) : 0;
    s->output_observer_gaps = len >= 69u ? p[68] : 0;
    s->picture_pct = len >= 71u ? p[69] : 0xffu;
    s->picture_windows = len >= 71u ? p[70] : 0;
    s->exp_mask = len >= 77u ? flog_get32(p + 71) : 0;
    s->exp_version = len >= 77u ? p[75] : 0;
    s->exp_mixed = len >= 77u ? p[76] : 0;
    memset(s->exp_values, 0, sizeof(s->exp_values));
    if (len > 77u) {
        unsigned n = len - 77u;
        if (n > EXP_ITEMS) n = EXP_ITEMS;
        memcpy(s->exp_values, p + 77, n);
    }
    return true;
}

/* Which snapshot slot a new one takes when the ring is full: the moments
 * that went wrong outlive the routine ones. Priority dropout > picture drop
 * > first carrier > periodic; an empty slot first, else the oldest slot of
 * the lowest priority not above the new one; -1 when every slot holds a
 * more important snapshot (the new one is skipped). */
static inline unsigned flog_snap_priority(uint8_t trigger)
{
    /* The range edge is where the ground truth is needed (C/N estimator,
     * sensitivity model): EDGE and RECOVER rank with dropouts. */
    return trigger == FLOG_SNAP_DROPOUT || trigger == FLOG_SNAP_EDGE || trigger == FLOG_SNAP_RECOVER ||
           trigger == FLOG_SNAP_OUTPUT_UNLOCK ? 4u :
           trigger == FLOG_SNAP_VQ_DROP || trigger == FLOG_SNAP_OVERLOAD ? 3u :
           trigger == FLOG_SNAP_FIRST_CARRIER ? 2u : 1u;
}

/* The request still waiting for the snapshot gap when another arrives
 * (0 = none): a more important one replaces it, otherwise the first one
 * stays, so a dropout is not lost behind a pending periodic snapshot. */
static inline uint8_t flog_snap_request_merge(uint8_t pending, uint8_t trigger)
{
    return !pending || flog_snap_priority(trigger) > flog_snap_priority(pending) ? trigger : pending;
}

static inline int flog_snap_victim(const uint8_t *trigger, const uint32_t *seq, const bool *used, unsigned slots,
                                   uint8_t new_trigger)
{
    int best = -1;
    unsigned best_pri = 0;
    for (unsigned k = 0; k < slots; ++k) {
        if (!used[k]) return (int)k;
        const unsigned pri = flog_snap_priority(trigger[k]);
        if (pri > flog_snap_priority(new_trigger)) continue;
        if (best < 0 || pri < best_pri || (pri == best_pri && seq[k] < seq[best])) { best = (int)k; best_pri = pri; }
    }
    return best;
}

/* Snapshot slot head (64 bytes): magic, format, trigger, IQ bytes, session,
 * sequence, t_ms, frequency, gain, lane, filter key, profile, kp, lp, DC
 * offsets (codes) and residual (mcells) of the row, CRC-32 over the head's
 * first 60 bytes and the IQ bytes. */
typedef struct {
    uint8_t trigger, gain, lane, profile, kp, lp, demod_id, output_level, output_flags;
    uint16_t freq_mhz, filter_key;
    uint32_t session, seq, t_ms;
    int16_t off_i, off_q, dc_i_mcells, dc_q_mcells;
} flog_snap_t;

static inline void flog_snap_head(uint8_t h[FLOG_SNAP_HEAD], const flog_snap_t *s, const uint8_t *iq)
{
    memset(h, 0, FLOG_SNAP_HEAD);
    flog_put32(h, FLOG_MAGIC_SNAP);
    h[4] = FLOG_FORMAT; h[5] = s->trigger;
    flog_put16(h + 6, FLOG_SNAP_BYTES);
    flog_put32(h + 8, s->session); flog_put32(h + 12, s->seq); flog_put32(h + 16, s->t_ms);
    flog_put16(h + 20, s->freq_mhz); h[22] = s->gain; h[23] = s->lane;
    flog_put16(h + 24, s->filter_key); h[26] = s->profile; h[27] = s->kp; h[28] = s->lp;
    flog_put16(h + 30, (uint16_t)s->off_i); flog_put16(h + 32, (uint16_t)s->off_q);
    flog_put16(h + 34, (uint16_t)s->dc_i_mcells); flog_put16(h + 36, (uint16_t)s->dc_q_mcells);
    h[38] = s->demod_id; h[39] = s->output_level; h[40] = s->output_flags;
    flog_put32(h + 60, flog_crc32(flog_crc32(0, h, 60), iq, FLOG_SNAP_BYTES));
}

static inline bool flog_snap_ok(const uint8_t *h, const uint8_t *iq)
{
    return flog_get32(h) == FLOG_MAGIC_SNAP && h[4] == FLOG_FORMAT &&
           flog_get32(h + 60) == flog_crc32(flog_crc32(0, h, 60), iq, FLOG_SNAP_BYTES);
}
