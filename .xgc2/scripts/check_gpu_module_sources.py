#!/usr/bin/env python3
"""Verify the pinned GPU inputs and reproduce the memory-entry patch offline."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile


def main():
    root = Path(__file__).resolve().parents[2] / 'src/sensing-gpu'
    pin = json.loads((root / 'source-pin.json').read_text())
    for item in pin['files']:
        source = root / ('LICENSE' if item['path'] == 'LICENSE' else 'upstream/' + item['path'])
        actual = hashlib.sha256(source.read_bytes()).hexdigest()
        if actual != item['sha256']:
            raise SystemExit('pinned source hash mismatch: ' + item['path'])
    with tempfile.TemporaryDirectory(prefix='xsim-gpu-pin-') as directory:
        work = Path(directory)
        shutil.copytree(root / 'upstream/local_sensing', work / 'local_sensing')
        subprocess.run(['patch', '--batch', '--fuzz=0', '-p1', '-i', str(root / 'marsim-memory-entry.patch')],
                       cwd=work, check=True)
        for name in ('opengl_sim.hpp', '360camera.vs', 'camera.fs'):
            path = Path('local_sensing/include') / name
            if (work / path).read_bytes() != (root / path).read_bytes():
                raise SystemExit('GPU memory-entry patch does not reproduce: ' + name)
    print('PASS: %d pinned inputs and exact patch reproduction' % len(pin['files']))


if __name__ == '__main__':
    main()
