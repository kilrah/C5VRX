/* C5VRX-5 by Twotoz and the C5VRX contributors: host test of the flight log
 * format (main/flightlog_format.h): CRC, headers, ring resume, record walk,
 * the per-second summary and snapshot heads.
 *
 *   cc -O2 -std=c11 -I../main test_flightlog.c -o t && ./t
 */
#include <stdio.h>
#include "flightlog_format.h"

static unsigned s_fail, s_run;
static void check(const char *name, int ok)
{
    ++s_run;
    printf("%-4s %s\n", ok ? "ok" : "FAIL", name);
    s_fail += !ok;
}

static unsigned s_count, s_types;
static bool visit(void *ctx, uint8_t type, const uint8_t *payload, uint8_t len)
{
    (void)ctx; (void)payload; (void)len;
    ++s_count;
    s_types |= 1u << type;
    return true;
}

int main(void)
{
    check("CRC-32 of \"123456789\" is the standard check value",
          flog_crc32(0, (const uint8_t *)"123456789", 9) == 0xcbf43926u);
    check("CRC-32 chains", flog_crc32(flog_crc32(0, (const uint8_t *)"1234", 4), (const uint8_t *)"56789", 5) ==
          0xcbf43926u);
    check("layout: 1.5 MiB, 64 snapshots of 8128 bytes", FLOG_PARTITION_BYTES == 0x180000u && FLOG_SNAP_BYTES == 8128u);

    uint8_t h[32];
    flog_header(h);
    check("header round trip", flog_header_ok(h));
    h[4] ^= 1;
    check("header of another format rejected", !flog_header_ok(h));

    /* Ring resume: blocks 0..9 written with seq 300..309, then the ring
     * wrapped into 0..2 with 310..312; sessions up to 9. */
    flog_resume_t r;
    flog_resume_init(&r);
    for (unsigned k = 0; k < FLOG_RECORD_BLOCKS; ++k) {
        flog_head_t head = {0};
        if (k < 10u) head = (flog_head_t){true, k < 3u ? 9u : 8u, k < 3u ? 310u + k : 300u + k, 0};
        flog_resume_add(&r, k, FLOG_RECORD_BLOCKS, head);
    }
    check("resume after the newest block across the wrap", r.next_block == 3u && r.next_seq == 313u &&
          r.next_session == 10u);
    flog_resume_init(&r);
    check("empty log starts at block 0, session 1", r.next_block == 0u && r.next_seq == 1u && r.next_session == 1u);
    flog_resume_add(&r, FLOG_RECORD_BLOCKS - 1u, FLOG_RECORD_BLOCKS, (flog_head_t){true, 4u, 77u, 0});
    check("newest in the last block wraps to block 0", r.next_block == 0u && r.next_seq == 78u);

    /* A block: head, three records, then erased flash; closed with its CRC. */
    static uint8_t b[FLOG_SECTOR];
    memset(b, 0xff, sizeof(b));
    flog_block_head(b, 5u, 42u, 1234u);
    const flog_head_t ph = flog_parse_head(b);
    check("block head round trip", ph.valid && ph.session == 5u && ph.seq == 42u && ph.t0_ms == 1234u);
    flog_second_t s = {.exp_version = 1, .exp_mask = 0x1234, .exp_values = {[9] = 2}, .exp_mixed = 1, .t_s = 77, .vq_snr_x10 = -45, .vq_lines_x100 = 160, .cn_x10 = -990, .video_pct = 88,
                       .dropout = 3, .gain_min = 70, .gain_mean = 75, .gain_max = 83, .clip_q = 9, .gain_writes = 255,
                       .overloads = 2, .dc_i = -20, .dc_q = 13, .off_i = -96, .off_q = 96, .servo_writes = 4,
                       .afc_offset_khz = -1250, .afc_residual_khz = 310, .afc_corrections = 1, .demod = 0x2d,
                       .sphase = 4, .temp_c = -12, .flags = 0x0a05,
                       .output_flags = FLOG_OUT_DEMOD_VERIFIED | FLOG_OUT_OBSERVER_FRESH | FLOG_OUT_H_TRUSTED,
                       .demod_id = FLOG_DEMOD_U85Q_LP_1A09F5C092E8, .output_standard = 2, .output_level = 6,
                       .output_samples = 500, .output_fresh_samples = 480, .native_h_lock_pct = 97,
                       .native_v_lock_pct = 95, .native_h_lines = 15625, .repaired_tips = 12,
                       .suppressed_tips = 3, .longest_h_unlock_ms = 106, .output_observer_gaps = 2,
                       .picture_pct = 87, .picture_windows = 14};
    unsigned at = FLOG_BLOCK_HEAD;
    b[at++] = FLOG_T_SECOND; b[at++] = FLOG_SECOND_LEN;
    flog_pack_second(&s, b + at); at += FLOG_SECOND_LEN;
    b[at++] = FLOG_T_EVENT; b[at++] = 5; memset(b + at, 1, 5); at += 5;
    b[at++] = 99; b[at++] = 3; memset(b + at, 2, 3); at += 3;           /* a type from a newer firmware */
    s_count = 0; s_types = 0;
    const unsigned used = flog_walk(b, visit, NULL);
    check("walk: every record, unknown types skipped by length", s_count == 3u && used == at - FLOG_BLOCK_HEAD &&
          (s_types & (1u << FLOG_T_SECOND)) && (s_types & (1u << FLOG_T_EVENT)));
    check("an open block (no CRC yet) is not closed", !flog_block_closed(b));
    flog_put32(b + FLOG_BLOCK_CRC_AT, flog_crc32(0, b, FLOG_BLOCK_CRC_AT));
    check("closed with its CRC", flog_block_closed(b));
    b[FLOG_BLOCK_HEAD + 5] ^= 1;
    check("a changed byte breaks the CRC", !flog_block_closed(b));
    b[FLOG_BLOCK_HEAD + 5] ^= 1;

    flog_second_t u;
    uint8_t again[FLOG_SECOND_LEN];
    const bool unpacked = flog_unpack_second(b + FLOG_BLOCK_HEAD + 2, FLOG_SECOND_LEN, &u);
    flog_pack_second(&u, again);   /* compare the packed bytes, not struct padding */
    check("summary round trip, every field", unpacked && !memcmp(again, b + FLOG_BLOCK_HEAD + 2, FLOG_SECOND_LEN) &&
          u.t_s == 77u && u.vq_snr_x10 == -45 && u.dc_i == -20 && u.off_q == 96 && u.afc_offset_khz == -1250 &&
          u.temp_c == -12 && u.flags == 0x0a05u && u.demod_id == FLOG_DEMOD_U85Q_LP_1A09F5C092E8 &&
          u.output_level == 6u && u.output_samples == 500u && u.output_fresh_samples == 480u &&
          u.native_h_lock_pct == 97u && u.native_h_lines == 15625u && u.longest_h_unlock_ms == 106u &&
          u.picture_pct == 87u && u.picture_windows == 14u);
    check("old experiment summary keeps fields and defaults D10 off",
          flog_unpack_second(b + FLOG_BLOCK_HEAD + 2, FLOG_SECOND_LEN - 1u, &u) &&
          u.exp_mask == s.exp_mask && u.exp_values[9] == 2 && !u.exp_values[EXP_CLOCK_D10]);
    check("a short summary is refused", !flog_unpack_second(b + FLOG_BLOCK_HEAD + 2, FLOG_SECOND_LEN_V1 - 1u, &u));
    {   /* format-1 records (34 bytes) still read; the level is unknown */
        flog_second_t old;
        check("a 34-byte summary still reads, level unknown",
              flog_unpack_second(b + FLOG_BLOCK_HEAD + 2, FLOG_SECOND_LEN_V1, &old) &&
              old.level_dbm_x10 == FLOG_LEVEL_UNKNOWN && old.level_flags == 0 &&
              old.demod_id == FLOG_DEMOD_UNKNOWN && old.output_level == 0xffu &&
              old.native_h_lock_pct == 0xffu && old.native_v_lock_pct == 0xffu && old.picture_pct == 0xffu);
    }
    {   /* the immediately previous 50-byte records remain valid too */
        flog_second_t old;
        check("a 50-byte summary keeps output telemetry unknown",
              flog_unpack_second(b + FLOG_BLOCK_HEAD + 2, 50u, &old) &&
              old.demod_id == FLOG_DEMOD_UNKNOWN && old.output_level == 0xffu &&
              old.output_samples == 0u && old.native_h_lock_pct == 0xffu);
    }

    /* A record that would cross the CRC word ends the walk. */
    memset(b + FLOG_BLOCK_HEAD, 0xff, FLOG_BLOCK_ROOM);
    b[FLOG_BLOCK_CRC_AT - 3] = FLOG_T_EVENT; b[FLOG_BLOCK_CRC_AT - 2] = 8;
    s_count = 0;
    (void)flog_walk(b, visit, NULL);
    check("record over the end is not read", s_count == 0u);

    static uint8_t iq[FLOG_SNAP_BYTES];
    for (unsigned k = 0; k < sizeof(iq); ++k) iq[k] = (uint8_t)(k * 7u);
    const flog_snap_t sn = {.trigger = FLOG_SNAP_DROPOUT, .gain = 83, .freq_mhz = 5865, .session = 5, .seq = 9, .t_ms = 1,
                            .demod_id = FLOG_DEMOD_U85Q_1A09F5C092E8, .output_level = 4,
                            .output_flags = FLOG_OUT_DEMOD_VERIFIED | FLOG_OUT_H_TRUSTED};
    uint8_t sh[FLOG_SNAP_HEAD];
    flog_snap_head(sh, &sn, iq);
    check("snapshot head and IQ verify", flog_snap_ok(sh, iq));
    check("snapshot binds verified output identity", sh[38] == FLOG_DEMOD_U85Q_1A09F5C092E8 &&
          sh[39] == 4u && sh[40] == (FLOG_OUT_DEMOD_VERIFIED | FLOG_OUT_H_TRUSTED));
    iq[4000] ^= 0x10;
    check("a changed IQ byte is caught", !flog_snap_ok(sh, iq));

    /* Snapshot ring: priorities when full. */
    {
        uint8_t trig[4] = {FLOG_SNAP_DROPOUT, FLOG_SNAP_PERIODIC, FLOG_SNAP_PERIODIC, FLOG_SNAP_VQ_DROP};
        uint32_t seq[4] = {1, 5, 2, 3};
        bool used[4] = {true, true, false, true};
        check("an empty slot first", flog_snap_victim(trig, seq, used, 4, FLOG_SNAP_PERIODIC) == 2);
        used[2] = true;
        check("full: the oldest periodic goes first", flog_snap_victim(trig, seq, used, 4, FLOG_SNAP_DROPOUT) == 2);
        check("a periodic snapshot replaces only a periodic one", flog_snap_victim(trig, seq, used, 4, FLOG_SNAP_PERIODIC) == 2);
        uint8_t important[4] = {FLOG_SNAP_DROPOUT, FLOG_SNAP_DROPOUT, FLOG_SNAP_VQ_DROP, FLOG_SNAP_DROPOUT};
        check("all more important: a periodic one is skipped", flog_snap_victim(important, seq, used, 4, FLOG_SNAP_PERIODIC) < 0);
        check("a dropout replaces the least important, oldest", flog_snap_victim(important, seq, used, 4, FLOG_SNAP_DROPOUT) == 2);
        uint8_t drops[4] = {FLOG_SNAP_DROPOUT, FLOG_SNAP_DROPOUT, FLOG_SNAP_DROPOUT, FLOG_SNAP_DROPOUT};
        check("all dropouts: the oldest dropout goes", flog_snap_victim(drops, seq, used, 4, FLOG_SNAP_DROPOUT) == 0);
        check("an edge snapshot ranks with a dropout, above a picture drop",
              flog_snap_priority(FLOG_SNAP_EDGE) == flog_snap_priority(FLOG_SNAP_DROPOUT) &&
              flog_snap_priority(FLOG_SNAP_EDGE) > flog_snap_priority(FLOG_SNAP_VQ_DROP) &&
              flog_snap_priority(FLOG_SNAP_RECOVER) == flog_snap_priority(FLOG_SNAP_DROPOUT) &&
              flog_snap_priority(FLOG_SNAP_OUTPUT_UNLOCK) == flog_snap_priority(FLOG_SNAP_DROPOUT));
    }

    /* A request waiting for the snapshot gap: a more important one replaces it. */
    check("no pending request: the new one waits", flog_snap_request_merge(0, FLOG_SNAP_PERIODIC) == FLOG_SNAP_PERIODIC);
    check("a dropout replaces a pending periodic request",
          flog_snap_request_merge(FLOG_SNAP_PERIODIC, FLOG_SNAP_DROPOUT) == FLOG_SNAP_DROPOUT);
    check("a dropout replaces a pending picture drop",
          flog_snap_request_merge(FLOG_SNAP_VQ_DROP, FLOG_SNAP_DROPOUT) == FLOG_SNAP_DROPOUT);
    check("a periodic request does not replace a pending dropout",
          flog_snap_request_merge(FLOG_SNAP_DROPOUT, FLOG_SNAP_PERIODIC) == FLOG_SNAP_DROPOUT);
    check("equal priority: the first request stays",
          flog_snap_request_merge(FLOG_SNAP_DROPOUT, FLOG_SNAP_EDGE) == FLOG_SNAP_DROPOUT);
    {
        /* Snapshot at t=0, picture drop at t=3, dropout at t=5: the dropout is taken at t=10. */
        uint8_t pending = 0;
        pending = flog_snap_request_merge(pending, FLOG_SNAP_VQ_DROP);
        pending = flog_snap_request_merge(pending, FLOG_SNAP_DROPOUT);
        pending = flog_snap_request_merge(pending, FLOG_SNAP_PERIODIC);
        check("picture drop, dropout, periodic while waiting: the dropout is taken", pending == FLOG_SNAP_DROPOUT);
    }
    printf("%u/%u flight log checks passed\n", s_run - s_fail, s_run);
    return s_fail != 0;
}
