#!/usr/bin/env python3
"""C5VRX-5 flight log reader (C5VRX by Twotoz and the C5VRX contributors).

Format: main/flightlog_format.h, docs/FLIGHT_LOG.md. The web flasher's Logs
page reads the same dump in the browser (web/flight-log.js).

    python tools/flight_log.py dump COM10 flight.c5log      # read the module (console W)
    python tools/flight_log.py decode flight.c5log out_dir  # CSV, events, snapshots
"""
import argparse
import re
import csv
import json
import struct
import sys
import time
import zlib
from pathlib import Path

SECTOR = 4096
RECORD_BLOCKS = 255
SNAP_FIRST = 256
SNAP_SLOTS = 64
SNAP_SECTORS = 2
SNAP_HEAD = 64
SNAP_BYTES = SNAP_SECTORS * SECTOR - SNAP_HEAD
PARTITION = (SNAP_FIRST + SNAP_SLOTS * SNAP_SECTORS) * SECTOR
BLOCK_HEAD = 20
CRC_AT = SECTOR - 4
MAGIC_BLOCK, MAGIC_SNAP = 0x424C3543, 0x534C3543

EVENTS = {1: 'channel', 2: 'profile', 3: 'kp', 4: 'cal_start', 5: 'cal_end', 6: 'dc_cal_needed', 7: 'gain_mode',
          8: 'menu', 9: 'fault', 10: 'dropout', 11: 'recovered', 12: 'snapshot', 13: 'dropped', 14: 'attack',
          15: 'output_gap', 16: 'output_unlock', 17: 'exp', 18: 'output_auto', 19: 'pause'}
DROPOUT = ['CALIBRATION_OR_PHY', 'FAST_EVIDENCE_LOSS', 'AT_RF_FLOOR', 'DEMOD_CLIFF', 'UNKNOWN']
SPHASE = ['unverified', 'settled', 'drifted', 'unsettled', 'noise_limited', 'probe_failed']
# 2: kp0.5 NARROW until 2026-10-23, the menu's XNARROW filter since 2026-10-26.
PROFILES = ['DEFAULT', 'MATCHED', 'XNARROW', 'WIDE']
RX_STATES = ['', 'ACQUIRE', 'TRACK', 'EDGE', 'RECOVER', 'FAULT', 'OWNED']
RX_REASONS = ['none', 'no_carrier', 'weak_signal', 'dc_clipping', 'overload', 'dc_residual', 'sync_loss_with_rf',
              'owned', 'fault', 'low_picture_snr']
TRIGGERS = {1: 'dropout', 2: 'vq_drop', 3: 'first_carrier', 4: 'periodic', 5: 'overload', 6: 'edge', 7: 'recover',
            8: 'output_unlock'}
OUTPUT_DEMODS = {1: 'U85Q 1a09f5c092e8', 2: 'U85Q-LP 1a09f5c092e8', 3: 'U80P64-R1', 4: 'U50P64-R1', 5: 'D32-R2'}
EXP_NAMES = ['GAIN', 'PHY FILT', 'AFC', 'DEMOD', 'IQ CORR', 'PHASE REPAIR', 'AUTO PROFILE',
             'FAST GAIN', 'FOLD RADIUS', 'FOLD MEMORY', 'LEVEL TARGET', 'LEVEL RELEASE', 'GAIN RATE',
             'GAIN CAP', 'LOG CADENCE', 'LNA BIAS', 'CURRENT', 'LOOP CAP', 'SPUR 0', 'IQ LANE', 'CLOCK D10', 'AUTO OUTPUT',
             'STRONG LANE', 'LEVEL TOP', 'LOW DC']
# Experiment schema versions (main/experimental.h EXP_VERSION). Version 2
# (2026-11-14) swapped AUTO OUTPUT to OFF/ON with OFF the baseline; values are
# reported in the version-2 meaning (AUTO OUTPUT: 0 = OFF, 1 = ON).
EXP_VERSIONS = (1, 2)
EXP_AUTO_OUTPUT = EXP_NAMES.index('AUTO OUTPUT')


def exp_values(version, values):
    """Experiment values of a record in the current (version-2) meaning."""
    values = list(values)
    if version == 1 and len(values) > EXP_AUTO_OUTPUT and values[EXP_AUTO_OUTPUT] in (0, 1):
        values[EXP_AUTO_OUTPUT] ^= 1   # v1: 0 = ON, 1 = OFF
    return values


OUTPUT_STANDARDS = {1: 'NTSC', 2: 'PAL'}


def crc32(data, crc=0):
    return zlib.crc32(data, crc) & 0xffffffff


LINE = re.compile(r'FLOG_D (\d+) ([0-9a-f]+) ([0-9a-f]{8})$')
LINE_BYTES = 256


def unit(offset):
    """The dump unit holding `offset` (header, a record block, or a snapshot
    slot): its start and the line offsets a complete dump of it sends."""
    if offset < SECTOR:
        return 0, [0]
    sector = offset // SECTOR
    if sector >= SNAP_FIRST:
        start = (SNAP_FIRST + (sector - SNAP_FIRST) // SNAP_SECTORS * SNAP_SECTORS) * SECTOR
        length = SNAP_SECTORS * SECTOR
    else:
        start, length = sector * SECTOR, SECTOR
    return start, list(range(start, start + length, LINE_BYTES))


def complete(got):
    """Starts of the units whose every line arrived intact."""
    return {start for start, lines in {unit(o) [0]: unit(o)[1] for o in got}.items() if all(o in got for o in lines)}


def dump(port, out, passes=4):
    """Console W, repeated for any line that did not arrive intact (other
    tasks print too); every FLOG_D line carries its CRC-32."""
    import serial
    s = serial.Serial()
    s.port, s.baudrate, s.timeout, s.dtr, s.rts = port, 115200, .2, False, False
    s.open()
    image = bytearray(bytes([0xff]) * PARTITION)
    got, bad, want = set(), 0, None
    for attempt in range(passes):
        s.reset_input_buffer()
        s.write(b'W')
        buf, end, finished = b'', time.monotonic() + 30, False
        while time.monotonic() < end and not finished:
            data = s.read(65536)
            if data:
                end = time.monotonic() + 30
            buf += data
            lines = buf.split(b'\n')
            buf = lines.pop()
            for raw in lines:
                line = raw.decode(errors='replace').strip()
                at = line.find('FLOG')
                line = line[at:] if at >= 0 else line
                if line.startswith('FLOG partition=missing'):
                    sys.exit(line)
                if line.startswith('FLOG_BEGIN'):
                    info = dict(kv.split('=', 1) for kv in line.split()[1:] if '=' in kv)
                    want = (int(info.get('blocks', 0)), int(info.get('snapshots', 0)))
                elif line.startswith('FLOG_D '):
                    m = LINE.match(line)
                    chunk = bytes.fromhex(m[2]) if m and len(m[2]) % 2 == 0 else None
                    if chunk is None or zlib.crc32(chunk) != int(m[3], 16):
                        bad += 1
                        continue
                    o = int(m[1])
                    image[o:o + len(chunk)] = chunk
                    got.add(o)
                elif line.startswith('FLOG_END'):
                    finished = True
        done = complete(got)
        blocks = sum(1 for start in done if 0 < start < SNAP_FIRST * SECTOR)
        snaps = sum(1 for start in done if start >= SNAP_FIRST * SECTOR)
        print(f'pass {attempt + 1}: {blocks} blocks, {snaps} snapshots complete; {bad} damaged lines so far')
        if want and blocks >= want[0] and snaps >= want[1] and 0 in got:
            break
    Path(out).write_bytes(bytes(image))
    print(f'wrote {out}')


def records(block):
    at = BLOCK_HEAD
    while at + 2 <= CRC_AT:
        t, n = block[at], block[at + 1]
        if t in (0, 0xff) or at + 2 + n > CRC_AT:
            break
        yield t, block[at + 2:at + 2 + n]
        at += 2 + n


def second(p):
    v = struct.unpack_from('<IhHhBBBBBBBBbbbbBhhBBBbH', p)
    keys = ('t_s vq_snr_x10 vq_lines_x100 cn_x10 video_pct dropout gain_min gain_mean gain_max clip_q gain_writes '
            'overloads dc_i dc_q off_i off_q servo_writes afc_offset_khz afc_residual_khz afc_corrections demod sphase '
            'temp_c flags').split()
    d = dict(zip(keys, v))
    d['vq_snr_db'] = d['vq_snr_x10'] / 10
    d['cn_db'] = None if d['cn_x10'] <= -990 else d['cn_x10'] / 10
    d['dc_i_cells'], d['dc_q_cells'] = d['dc_i'] * .05, d['dc_q'] * .05
    d['profile'] = PROFILES[d['demod'] & 3]
    d['kp'] = (d['demod'] >> 2) & 7
    d['lp'] = (d['demod'] >> 5) & 1
    d['sphase_state'] = SPHASE[d['sphase']] if d['sphase'] < len(SPHASE) else 'unknown'
    d['dropout_category'] = DROPOUT[d['dropout'] - 1] if 0 < d['dropout'] <= len(DROPOUT) else ''
    rx = p[33] if len(p) > 33 else 0
    d['rx_state'] = RX_STATES[rx & 7] if (rx & 7) < len(RX_STATES) else '?'
    d['rx_reason'] = (RX_REASONS[rx >> 3] if (rx >> 3) < len(RX_REASONS) else '?') if rx else ''
    # Received level at the chip pin (rx_level.h), 38-byte records.
    lv, noise, flags = struct.unpack_from('<hbB', p, 34) if len(p) >= 38 else (-32768, 0, 0)
    d['level_dbm'] = lv / 10 if lv != -32768 and flags & 1 else None
    d['noise_dbm'] = noise if flags & 1 else None
    d['level_above_noise'] = int(bool(flags & 2))
    d['level_exact'] = int(bool(flags & 4))
    # Why the gain moved (42-byte records).
    d['writes_attack'], d['writes_release'], d['sentinel'], d['overloads_after_write'] = (p[38], p[39], p[40], p[41]) if len(p) >= 42 else (None, None, None, None)
    # C/N-AUTO (50-byte records): power, M2M4, envelope C/N, confidence, source, flags
    d['cn_power_db'] = d['cn_m2m4_db'] = d['cn_env_db'] = None
    d['cn_confidence'] = d['cn_source'] = d['cn_flags'] = None
    if len(p) >= 50:
        cp, cm, ce = struct.unpack_from('<hhh', p, 42)
        d['cn_power_db'] = None if cp <= -990 else cp / 10
        d['cn_m2m4_db'] = None if cm <= -990 else cm / 10
        d['cn_env_db'] = None if ce <= -990 else ce / 10
        d['cn_confidence'], d['cn_source'], d['cn_flags'] = p[48] & 3, p[48] >> 2, p[49]
    # Verified output identity and native/assisted sync observer (69-byte records).
    # Defaults are explicit UNKNOWN so old and new records can share one CSV.
    d.update(output_demod=None, output_level=None, output_auto_gain=None, output_standard=None, output_verified=0,
             flywheel_requested=0, flywheel_active=0, output_observer_fresh=0,
             native_h_trusted=0, native_v_trusted=0, camera_present=0, output_flag_format=0,
             output_samples=None, output_fresh_samples=None, output_coverage_pct=None,
             native_h_lock_pct=None, native_v_lock_pct=None, native_h_lines=None,
             repaired_tips=None, suppressed_tips=None, longest_h_unlock_ms=None,
             output_observer_gaps=None)
    if len(p) >= 69:
        of = p[50]
        d.update(output_verified=int(bool(of & 1)), flywheel_requested=int(bool(of & 2)),
                 flywheel_active=int(bool(of & 4)), output_observer_fresh=int(bool(of & 8)),
                 native_h_trusted=int(bool(of & 16)), native_v_trusted=int(bool(of & 32)),
                 camera_present=int(bool(of & 64)), output_flag_format=int(bool(of & 128)))
        d['output_demod'] = OUTPUT_DEMODS.get(p[51], f'unknown ({p[51]})') if d['output_verified'] else None
        d['output_standard'] = OUTPUT_STANDARDS.get(p[52])
        d['output_level'] = None if p[53] == 0xff else p[53]
        # 0x80 | gain x8: AUTO OUTPUT (output_auto.h) instead of a manual level
        if p[53] != 0xff and p[53] & 0x80:
            d['output_auto_gain'] = (p[53] & 63) / 8
        d['output_samples'], d['output_fresh_samples'] = struct.unpack_from('<HH', p, 54)
        d['output_coverage_pct'] = (round(100 * d['output_fresh_samples'] / d['output_samples'])
                                    if d['output_samples'] else None)
        d['native_h_lock_pct'] = None if p[58] == 0xff else p[58]
        d['native_v_lock_pct'] = None if p[59] == 0xff else p[59]
        (d['native_h_lines'], d['repaired_tips'], d['suppressed_tips'],
         d['longest_h_unlock_ms']) = struct.unpack_from('<HHHH', p, 60)
        d['output_observer_gaps'] = p[68]
    # Picture measure (71-byte records): sync pulses found / expected in the
    # decoded windows (main/picture_sync.h), apart from the AFC-gated video_pct.
    d['picture_pct'] = (None if p[69] == 0xff else p[69]) if len(p) >= 71 else None
    d['picture_windows'] = p[70] if len(p) >= 71 else None
    d['exp_mask'] = struct.unpack_from('<I', p, 71)[0] if len(p) >= 77 else None
    d['exp_version'] = p[75] if len(p) >= 77 else None
    d['exp_mixed'] = p[76] if len(p) >= 77 else None
    values = exp_values(d['exp_version'], p[77:77 + len(EXP_NAMES)]) if d['exp_version'] in EXP_VERSIONS else []
    for i, name in enumerate(EXP_NAMES):
        d['exp_' + name.lower().replace(' ', '_')] = values[i] if i < len(values) else None
    return d


def experimental_groups(seconds):
    """Exact combinations, excluding owned/transition seconds and unknown schema.
    Counts are observations, not seconds of exposure at a reduced log cadence.
    No event replay is needed: every observation carries its complete state.
    """
    groups = {}
    for row in seconds:
        if row.get('exp_version') not in EXP_VERSIONS or row.get('exp_mixed') or row['flags'] & 0x0f00:
            continue
        key = tuple(row['exp_' + n.lower().replace(' ', '_')] for n in EXP_NAMES)
        groups.setdefault(key, []).append(row)
    result = []
    for key, rows in sorted(groups.items()):
        out = dict(zip(EXP_NAMES, key))
        out['observations'] = len(rows)
        for field in ['picture_pct', 'video_pct', 'level_dbm', 'gain_mean']:
            values = [r[field] for r in rows if r.get(field) is not None]
            out[field] = sum(values) / len(values) if values else None
        result.append(out)
    return result


def decode(image):
    blocks = []
    for k in range(RECORD_BLOCKS):
        b = image[(1 + k) * SECTOR:(2 + k) * SECTOR]
        magic, fmt = struct.unpack_from('<IB', b)
        if magic != MAGIC_BLOCK or fmt != 1:
            continue
        session, seq, t0 = struct.unpack_from('<III', b, 8)
        closed = struct.unpack_from('<I', b, CRC_AT)[0] == crc32(b[:CRC_AT])
        blocks.append((seq, session, t0, closed, b))
    blocks.sort()
    sessions = {}
    for seq, session, t0, closed, b in blocks:
        s = sessions.setdefault(session, {'session': session, 'seconds': [], 'events': [], 'traces': [],
                                          'carrier': [], 'blocks': 0, 'open_blocks': 0})
        s['blocks'] += 1
        s['open_blocks'] += not closed
        for t, p in records(b):
            if t == 1 and len(p) >= 48:
                s['boot'] = {'reset_reason': p[5], 'firmware': p[6:30].split(b'\0')[0].decode(errors='replace'),
                             'cal_generation': struct.unpack_from('<H', p, 30)[0], 'chip_rev': p[32],
                             'board_id': '%08x' % struct.unpack_from('<I', p, 33)[0], 'gain_max': p[37],
                             'channel_index': p[40], 'gain_mode': p[43], 'flywheel': p[44], 'profile_live': p[45],
                             'freq_mhz': struct.unpack_from('<H', p, 46)[0]}
            elif t == 7 and len(p) >= 2 and p[0] in EXP_VERSIONS and len(p) >= 2 + p[1]:
                s.setdefault('experimental_states', []).append(dict(zip(EXP_NAMES, exp_values(p[0], p[2:2 + p[1]]))))
            elif t == 2 and len(p) >= 56:
                filt = []
                for c in range(3):
                    e = p[6 + c * 8:14 + c * 8]
                    filt.append(None if not e[0] else {'bw20': e[1], 'code': struct.unpack('b', e[2:3])[0],
                                                      'skirt': e[3], 'width_khz': struct.unpack_from('<H', e, 4)[0],
                                                      'nbw_khz': struct.unpack_from('<H', e, 6)[0]})
                s['calib'] = {'manual': p[0], 'filter_result': ['', 'loaded', 'stored', 'other'][min(p[1], 3)],
                              'duration_ms': struct.unpack_from('<I', p, 2)[0], 'filters': dict(zip(['NORMAL', 'NARROW', 'WIDE'], filt)),
                              'dc_rows': p[45], 'dc_missing': p[46], 'dc_template': p[47], 'repair_flags': p[48],
                              'sphase': SPHASE[p[49]] if p[49] < len(SPHASE) else 'unknown',
                              'boot_dc_mcells': struct.unpack_from('<hh', p, 50), 'temp_c': struct.unpack('b', p[54:55])[0]}
            elif t == 3 and len(p) >= 17:
                t_ms, freq, off, cfo, dev = struct.unpack_from('<IHhih', p)
                s['carrier'].append({'t_ms': t_ms, 'freq_mhz': freq, 'afc_offset_khz': off, 'porch_khz': cfo,
                                     'deviation': dev / 100, 'profile': PROFILES[p[14] & 3],
                                     'standard': ['', 'NTSC', 'PAL'][min(p[15], 2)], 'channel_index': p[16]})
            elif t == 4:
                s['seconds'].append(second(p))
            elif t == 5 and len(p) >= 5:
                t_ms, kind = struct.unpack_from('<IB', p)
                event = {'t_ms': t_ms, 'event': EVENTS.get(kind, str(kind)), 'data': p[5:].hex()}
                if kind == 19 and len(p) >= 9:      # who paused the receiver, and how long
                    event['pause_ms'] = struct.unpack_from('<I', p, 5)[0]
                    event['owner'] = bytes(p[9:17]).split(b'\0')[0].decode('ascii', 'replace')
                elif kind == 18 and len(p) >= 8:    # AUTO OUTPUT applied
                    event.update(output_gain=p[5] / 8, source_tip_code=p[6], sync_depth_codes=p[7] / 4)
                s['events'].append(event)
            elif t == 6 and len(p) >= 6:
                onset, part, n = struct.unpack_from('<IBB', p)
                s['traces'].append({'onset_ms': onset, 'part': part, 'entries': [
                    dict(zip(('dt_ms', 'video_pct', 'gain', 'cn_half_db', 'clip_q', 'kp_lp', 'flags'),
                             struct.unpack_from('<HBBbBBB', p, 6 + 8 * k))) for k in range(n)]})
    snaps = []
    for k in range(SNAP_SLOTS):
        at = (SNAP_FIRST + k * SNAP_SECTORS) * SECTOR
        h, iq = image[at:at + SNAP_HEAD], image[at + SNAP_HEAD:at + SNAP_SECTORS * SECTOR]
        if struct.unpack_from('<I', h)[0] != MAGIC_SNAP:
            continue
        ok = struct.unpack_from('<I', h, 60)[0] == crc32(iq, crc32(h[:60]))
        session, seq, t_ms = struct.unpack_from('<III', h, 8)
        freq, gain, lane, key, profile, kp, lp = struct.unpack_from('<HBBHBBB', h, 20)
        off_i, off_q, dc_i, dc_q = struct.unpack_from('<hhhh', h, 30)
        snaps.append({'slot': k, 'ok': ok, 'trigger': TRIGGERS.get(h[5], str(h[5])), 'session': session, 'seq': seq,
                      't_ms': t_ms, 'freq_mhz': freq, 'gain': gain, 'lane': lane, 'filter_key': '%04x' % key,
                      'profile': PROFILES[profile & 3], 'kp': kp, 'lp': lp, 'offset_codes': [off_i, off_q],
                      'dc_mcells': [dc_i, dc_q],
                      'output_demod': OUTPUT_DEMODS.get(h[38], f'unknown ({h[38]})') if h[40] & 1 else None,
                      'output_level': h[39] if h[40] & 1 and h[39] != 0xff else None,
                      'output_flags': h[40], 'iq': bytes(iq)})
    return [sessions[k] for k in sorted(sessions)], sorted(snaps, key=lambda x: x['seq'])


def write_out(image, out):
    out.mkdir(parents=True, exist_ok=True)
    sessions, snaps = decode(image)
    for s in sessions:
        name = out / f"session_{s['session']:04d}"
        if s['seconds']:
            with open(f'{name}_seconds.csv', 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(s['seconds'][0].keys()))
                w.writeheader()
                w.writerows(s['seconds'])
        groups = experimental_groups(s['seconds'])
        if groups:
            with open(f'{name}_experiments.csv', 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(groups[0]))
                w.writeheader()
                w.writerows(groups)
        meta = {k: v for k, v in s.items() if k != 'seconds'}
        Path(f'{name}.json').write_text(json.dumps(meta, indent=1, default=list))
        print(f"session {s['session']}: {len(s['seconds'])} s, {len(s['events'])} events, "
              f"{len(s['carrier'])} carrier records, {s['blocks']} blocks ({s['open_blocks']} open)")
    for sn in snaps:
        base = out / f"snap_s{sn['session']:04d}_{sn['seq']:05d}"
        Path(f'{base}.bin').write_bytes(sn['iq'])
        meta = {k: v for k, v in sn.items() if k != 'iq'}
        meta.update(bytes=len(sn['iq']), rate_hz=40000000, format='Ihigh_Qlow')
        Path(f'{base}.json').write_text(json.dumps(meta, indent=1))
    print(f'{len(snaps)} snapshots')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    d = sub.add_parser('dump')
    d.add_argument('port')
    d.add_argument('out')
    e = sub.add_parser('decode')
    e.add_argument('image')
    e.add_argument('out', type=Path)
    a = ap.parse_args()
    if a.cmd == 'dump':
        dump(a.port, a.out)
    else:
        write_out(Path(a.image).read_bytes(), a.out)


if __name__ == '__main__':
    main()
