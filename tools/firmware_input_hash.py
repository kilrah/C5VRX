#!/usr/bin/env python3
"""Fingerprint tracked C5VRX-5 firmware inputs, excluding website and docs.

Identical inputs keep an existing release (c5vrx5.yml publish steps), so a
docs- or website-only change never mints new firmware. legacy/ is archived
and never built. With an experiment id, the hash also covers
hardware_experiments/<id>/ (the hardware experiment build of that id); the
production hash never depends on hardware_experiments/.
"""
import hashlib
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
INPUT_DIRS = ('main/',)
INPUT_FILES = ('CMakeLists.txt', 'partitions.csv', 'sdkconfig.defaults', 'sdkconfig.psram.defaults', 'dependencies.lock',
               '.github/workflows/c5vrx5.yml', 'tools/firmware_input_hash.py')


def firmware_input(path, experiment=None):
    if path.endswith('.md'):
        return False
    if experiment and path.startswith(f'hardware_experiments/{experiment}/'):
        return True
    return path.startswith(INPUT_DIRS) or path in INPUT_FILES


def main():
    # The generation argument of older workflows is accepted and ignored.
    if len(sys.argv) > 1 and sys.argv[1] != '5':
        raise SystemExit('Only C5VRX-5 is built; legacy generations are archived')
    experiment = sys.argv[2] if len(sys.argv) > 2 else None
    if experiment is not None and (not re.fullmatch(r'[a-z0-9_]+', experiment) or
                                   not (ROOT / 'hardware_experiments' / experiment / 'hwx.h').is_file()):
        raise SystemExit(f'Unknown hardware experiment {experiment!r}')
    paths = subprocess.check_output(
        ['git', '-c', f'safe.directory={ROOT}', 'ls-files', '-z'], cwd=ROOT
    ).decode().split('\0')
    digest = hashlib.sha256()
    for path in sorted(p for p in paths if p and firmware_input(p, experiment)):
        digest.update(path.encode() + b'\0')
        digest.update(hashlib.sha256((ROOT / path).read_bytes()).digest())
    print(digest.hexdigest())


if __name__ == '__main__':
    main()
