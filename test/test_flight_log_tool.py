#!/usr/bin/env python3
"""Host checks for append-only output telemetry in tools/flight_log.py."""
import struct
import importlib.util
import unittest
from pathlib import Path

SPEC = importlib.util.spec_from_file_location('flight_log', Path(__file__).parents[1] / 'tools' / 'flight_log.py')
flight_log = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(flight_log)


def summary(length=71):
    p = bytearray(length)
    struct.pack_into('<IhHh', p, 0, 7, 45, 120, 80)
    p[28] = 1 | (3 << 2)
    if length >= 38:
        struct.pack_into('<h', p, 34, -885)
        p[36], p[37] = 0xa1, 1 | 2 | 4
    if length >= 50:
        struct.pack_into('<hhh', p, 42, 120, 110, 100)
    if length >= 69:
        p[50:54] = bytes((0xff, 2, 2, 6))
        struct.pack_into('<HH', p, 54, 500, 480)
        p[58:60] = bytes((97, 95))
        struct.pack_into('<HHHH', p, 60, 15625, 12, 3, 106)
        p[68] = 2
    if length >= 71:
        p[69:71] = bytes((87, 14))
    return p


class FlightLogToolTest(unittest.TestCase):
    def test_experiment_append_keeps_old_records(self):
        old = summary(97)
        struct.pack_into('<I', old, 71, 0x1234)
        old[75] = 1
        old[86] = 2
        row = flight_log.second(old)
        self.assertEqual(row['exp_mask'], 0x1234)
        self.assertEqual(row['exp_fold_memory'], 2)
        self.assertIsNone(row['exp_clock_d10'])
        new = old + bytes((1,))
        self.assertEqual(flight_log.second(new)['exp_clock_d10'], 1)

    def test_auto_output_reads_the_same_in_both_schema_versions(self):
        """EXP_VERSION 2 swapped AUTO OUTPUT to OFF/ON (OFF the baseline);
        rows report the version-2 meaning: 0 = OFF, 1 = ON."""
        k = flight_log.EXP_AUTO_OUTPUT

        def row(version, value):
            p = summary(77 + len(flight_log.EXP_NAMES))
            p[75:77] = bytes((version, 0))
            p[77 + k] = value
            return flight_log.second(p)
        self.assertEqual(row(1, 0)['exp_auto_output'], 1)   # v1 ON
        self.assertEqual(row(1, 1)['exp_auto_output'], 0)   # v1 OFF
        self.assertEqual(row(2, 0)['exp_auto_output'], 0)   # v2 OFF
        self.assertEqual(row(2, 1)['exp_auto_output'], 1)   # v2 ON
        self.assertIsNone(row(3, 1)['exp_auto_output'])     # unknown schema
        groups = flight_log.experimental_groups([row(1, 1), row(2, 0), row(1, 0), row(2, 1)])
        self.assertEqual([(g['AUTO OUTPUT'], g['observations']) for g in groups], [(0, 2), (1, 2)])

    def test_output_telemetry_decodes_with_validity(self):
        row = flight_log.second(summary())
        self.assertEqual(row['output_demod'], 'U85Q-LP 1a09f5c092e8')
        self.assertEqual(row['output_level'], 6)
        self.assertEqual(row['output_standard'], 'PAL')
        self.assertEqual(row['output_coverage_pct'], 96)
        self.assertEqual(row['native_h_lock_pct'], 97)
        self.assertEqual(row['native_h_lines'], 15625)
        self.assertEqual(row['repaired_tips'], 12)
        self.assertEqual(row['longest_h_unlock_ms'], 106)
        self.assertEqual(row['output_observer_gaps'], 2)
        self.assertEqual(row['picture_pct'], 87)
        self.assertEqual(row['picture_windows'], 14)

    def test_old_record_keeps_output_unknown_and_same_columns(self):
        old = flight_log.second(summary(34))
        new = flight_log.second(summary())
        self.assertIsNone(old['output_demod'])
        self.assertIsNone(old['native_h_lock_pct'])
        self.assertIsNone(old['picture_pct'])
        self.assertIsNone(flight_log.second(summary(69))['picture_pct'])
        self.assertEqual(old.keys(), new.keys())

    def test_experiments_survive_lost_events_and_share_no_mixed_seconds(self):
        def row(value=0, mixed=0, flags=0):
            p = summary(77 + len(flight_log.EXP_NAMES))
            struct.pack_into('<I', p, 71, 1 << 9 if value else 0)
            p[75:77] = bytes((1, mixed))
            p[77:81] = bytes((254, 12, 0, 0))
            p[77 + 9] = value
            struct.pack_into('<H', p, 31, flags)
            return flight_log.second(p)
        a, b, c = row(1), row(1), row(2)
        b['picture_pct'] = 43
        groups = flight_log.experimental_groups([a, b, c, row(1, mixed=1), row(1, flags=0x100),
                                                  flight_log.second(summary())])
        self.assertEqual(len(groups), 2)  # same bitmask, different actual settings
        self.assertEqual(groups[0]['observations'], 2)
        self.assertEqual(groups[0]['picture_pct'], 65)
        self.assertEqual(groups[1]['FOLD MEMORY'], 2)
        self.assertEqual(groups[1]['observations'], 1)
        self.assertEqual(a.keys(), flight_log.second(summary(34)).keys())


if __name__ == '__main__':
    unittest.main()
