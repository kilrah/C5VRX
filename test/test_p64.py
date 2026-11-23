"""C5VRX by Twotoz/contributors: PR #250 P64 table and assembled datapath.

Uses the ESP-IDF assembler/parser and existing BitScrambler emulator. Host
evidence only: physical throughput, picture quality and range remain untested.
"""
import json
import hashlib
import struct
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "research"))
import p64_model
import u85q_lp
import sim


def reference(raw, table):
    """Independent two-lookups-per-pair reference; reset state/output zero."""
    output = np.zeros(len(raw), dtype=np.uint8)
    state = 0
    positions = (4, 5, 6, 7, 0, 1, 2, 3, 15, 11)
    for pair in range(len(raw) // 2 - 1):
        word = int(raw[2 * pair]) | (int(raw[2 * pair + 1]) << 8)
        address = sum(((word >> bit) & 1) << index for index, bit in enumerate(positions))
        token = int(table[address]) >> 12
        value = int(table[state * 16 + token])
        state = (value >> 6) & 63
        output[2 * pair + 2:2 * pair + 4] = value & 63
    return output


def check_u85q():
    """main/u85q.bsasm is the archived U85Q the host models score, its LUT is
    main/u85q_table.h, the loader accepts its image and the assembled program
    decodes as the reference (5-bit tokens)."""
    program = ROOT / "main/u85q.bsasm"
    assert program.read_bytes() == (ROOT / "research/legacy_demod/u85q.bsasm").read_bytes()
    instructions, meta, lut = u85q_lp.assemble(program)
    header = (ROOT / "main/u85q_table.h").read_text()
    table = np.array([int(v) for v in header.split("u85q_table[1024] = {", 1)[1].split("};", 1)[0].replace(",", " ").split()],
                     dtype=np.uint16)
    assert len(table) == 1024 and np.array_equal(lut, table) and np.array_equal(table, sim.u85q_lut())
    assert hashlib.sha256(struct.pack("<1024H", *table)).hexdigest() in header
    binary = u85q_lp._bsasm().insts_to_binary(instructions, meta, lut)
    assert binary[:7] == bytes((1, 0, 3, 8, 0, 2, 1)) and len(binary) == 300 + 2048   # transport.c program_words()
    raw = np.random.default_rng(85).integers(0, 256, 12000, dtype=np.uint8)
    got = np.frombuffer(u85q_lp.emulate(program, raw.tobytes(), len(raw) - 8), dtype=np.uint8)
    expected = sim._pair_codes(raw, table, sim.PAIR_BITS, 5)[:len(got)]
    assert np.array_equal(got & 63, expected), "assembled U85Q differs from the pair/state reference"
    for name in ("transport.c", "control.c", "receiver.h"):
        source = (ROOT / "main" / name).read_text()
        assert "kp_tables" not in source and "u85q_lp" not in source, name


def check_d32():
    """D32-R2 (research/nl_tracker.py): the U85Q program, instruction for
    instruction, with its own table; header and program current."""
    import nl_tracker
    table = nl_tracker.table()
    assert nl_tracker.HEADER.read_text() == nl_tracker.header_text(table)
    assert nl_tracker.PROGRAM.read_text() == nl_tracker.program_text(table)
    nl_tracker.gates(table)
    instructions, meta, lut = u85q_lp.assemble(nl_tracker.PROGRAM)
    reference_instructions, reference_meta, _ = u85q_lp.assemble(ROOT / "main/u85q.bsasm")
    binary = u85q_lp._bsasm().insts_to_binary(instructions, meta, lut)
    reference = u85q_lp._bsasm().insts_to_binary(reference_instructions, reference_meta, lut)
    assert binary[:300] == reference[:300] and len(binary) == 300 + 2048   # same program, own LUT
    assert np.array_equal(lut, table)
    assert struct.unpack_from("<1024H", binary, 300) == tuple(int(v) for v in table)


def check_u50p64():
    """U50P64-R1 (research/u50p64.py): the U80P64-R1 program, instruction for
    instruction, with the loop-gain 0.50 table; header and program current."""
    import u50p64
    table = u50p64.table()
    assert u50p64.HEADER.read_text() == u50p64.header_text(table)
    assert u50p64.PROGRAM.read_text() == u50p64.program_text(table)
    u50p64.gates(table)
    instructions, meta, lut = u85q_lp.assemble(u50p64.PROGRAM)
    reference_instructions, reference_meta, _ = u85q_lp.assemble(ROOT / "main/p64.bsasm")
    assert len(instructions) == 8 and np.array_equal(lut, table)
    binary = u85q_lp._bsasm().insts_to_binary(instructions, meta, lut)
    reference = u85q_lp._bsasm().insts_to_binary(reference_instructions, reference_meta, lut)
    assert binary == reference and binary[:7] == bytes((1, 0, 3, 8, 0, 2, 1)) and len(binary) == 300 + 2048
    assert struct.unpack_from("<1024H", binary, 300) == tuple(int(v) for v in table)


def main():
    # Fixed decoders (operator decision 2026-10-28): P64 and the untrained
    # U85Q, no kp ladder or LP; the retired policies stay research.
    build = (ROOT / "main/CMakeLists.txt").read_text()
    assert build.count('target_bitscrambler_add_src(') == 4
    assert 'target_bitscrambler_add_src("u50p64.bsasm")' in build
    check_u50p64()
    assert 'target_bitscrambler_add_src("d32.bsasm")' in build
    check_d32()
    assert 'target_bitscrambler_add_src("p64.bsasm")' in build
    assert 'target_bitscrambler_add_src("u85q.bsasm")' in build
    check_u85q()
    for name in ("transport.c", "control.c", "receiver.h"):
        source = (ROOT / "main" / name).read_text()
        for retired in ("transport_set_p64", "transport_kp_set", "kp_tables.h", "kp_ladder.h", 'blob_load("kp_auto"', 'blob_load("lp_cn"'):
            assert retired not in source, (name, retired)
    table = p64_model.checked_in_table()
    assert len(table) == 1024
    assert np.array_equal(table, p64_model.repaired_table())
    table_digest = hashlib.sha256(struct.pack("<1024H", *table)).hexdigest()
    assert table_digest in (ROOT / "main/p64_tables.h").read_text(), "diagnostic LUT identity is stale"
    repair = json.loads((ROOT / "research/data/p64_repair.json").read_text())
    params = repair["params"]
    # Preserve the original model as a reproducible negative reference.
    original = p64_model.champion_table()
    assert hashlib.sha256(struct.pack("<1024H", *original)).hexdigest() == repair["original_lut_sha256"]
    n_sync = int(params["n_sync"])
    assert n_sync == 14 and 360 / (64 - n_sync) == 7.2
    reachable = {0}
    while True:
        expanded = reachable | {(int(table[s * 16 + t]) >> 6) & 63 for s in reachable for t in range(16)}
        if expanded == reachable:
            break
        reachable = expanded
    assert len(reachable) == 64
    assert params["token_sectors"] == 16 and params["phase_consistent_output"]

    # Independent physical transfer check: 588 clean tones spanning the
    # measured video band, radii and starting phases. Optimizer scores or
    # state reachability alone did not catch a porch stuck at sync code 15.
    sample = np.arange(8000)
    worst_error = 0.0
    for radius in (2.0, 4.0, 6.0):
        for phase in (0.27, 1.1, 2.7, -2.1):
            for mhz in np.arange(-5.5, 6.51, 0.25):
                z = radius * np.exp(2j * np.pi * mhz * 1e6 * sample / 40e6 + 1j * phase)
                i, q = np.floor(z.real).astype(np.int64), np.floor(z.imag).astype(np.int64)
                raw = (((i & 15) << 4) | (q & 15)).astype(np.uint8)
                codes = sim._pair_codes(raw, table, sim.PAIR_BITS, 4)[1000:]
                expected = (mhz + 6.0) / 15.0 * 63.0
                error = abs(float(codes.mean()) - expected)
                worst_error = max(worst_error, error)
                assert error < 0.5, (radius, phase, mhz, float(codes.mean()), expected)
    # Show that the test rejects the imported optimizer champion.
    z = 4 * np.exp(-2j * np.pi * 1e6 * sample / 40e6 + 0.27j)
    raw = (((np.floor(z.real).astype(np.int64) & 15) << 4) |
           (np.floor(z.imag).astype(np.int64) & 15)).astype(np.uint8)
    assert abs(float(sim._pair_codes(raw, original, sim.PAIR_BITS, 4)[1000:].mean()) - 21.0) > 5

    # A real sync-to-porch transition must escape; no constant sync clamp.
    freq = np.r_[np.full(2000, -3.3e6), np.full(4000, -1e6)]
    for radius in (2.0, 4.0, 6.0):
        for phase in (0.27, 1.1, 2.7, -2.1):
            z = radius * np.exp(2j * np.pi * np.cumsum(freq) / 40e6 + 1j * phase)
            raw = (((np.floor(z.real).astype(np.int64) & 15) << 4) |
                   (np.floor(z.imag).astype(np.int64) & 15)).astype(np.uint8)
            # After 1 us, every 0.8-us average is already at the porch:
            # waiting 5 us would miss the colour burst this must preserve.
            porch = sim._pair_codes(raw, table, sim.PAIR_BITS, 4)[2040:-8]
            mean = np.convolve(porch, np.ones(32) / 32, mode="valid")
            assert np.max(np.abs(mean - 21)) < 0.5
            assert np.convolve((porch < 19).astype(int), np.ones(80, dtype=int), mode="valid").max() < 80

    program = ROOT / "main/p64.bsasm"
    instructions, meta, lut = u85q_lp.assemble(program)
    assert len(instructions) == 8 and np.array_equal(lut, table)
    binary = u85q_lp._bsasm().insts_to_binary(instructions, meta, lut)
    # Exact production program_words() acceptance, including full LUT size.
    assert binary[:7] == bytes((1, 0, 3, 8, 0, 2, 1))
    assert len(binary) == 300 + 2048
    assert struct.unpack_from("<1024H", binary, 300) == tuple(int(v) for v in table)
    rng = np.random.default_rng(250)
    raw = rng.integers(0, 256, 12000, dtype=np.uint8)
    got = np.frombuffer(u85q_lp.emulate(program, raw.tobytes(), len(raw) - 8), dtype=np.uint8)
    expected = reference(raw, table)[:len(got)]
    assert np.array_equal(got & 63, expected), "assembled P64 differs from pair/state reference"
    assert np.array_equal(got[::2] & 63, got[1::2] & 63), "DAC20 must duplicate at TX40"
    print(f"PASS: U80P64-R1 transfer (588 tones, max error {worst_error:.3f} code); sync-to-porch escape; original regression rejected; full loader image; assembled DAC20/TX40")


if __name__ == "__main__":
    main()
