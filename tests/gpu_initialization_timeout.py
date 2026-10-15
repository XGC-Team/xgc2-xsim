"""A hung foreign create ends the process within its supplied initialization budget."""
import subprocess
import sys
import time

for mode in ('hang', 'hang-stop'):
    started = time.monotonic()
    result = subprocess.run([sys.argv[1], mode, sys.argv[2]], capture_output=True, text=True, timeout=3)
    assert result.returncode == 1, result
    assert 'GPU initialization deadline exceeded after 100 ms' in result.stderr, result.stderr
    assert time.monotonic() - started < 2, 'hung GPU create exceeded the process bound'
print('PASS: hung GPU initialization and stop terminate nonzero within the supplied budget')
