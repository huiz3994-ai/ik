#!/usr/bin/env python3
"""Protocol-only deterministic server for the C++ process-lifecycle test."""
import sys

print("READY", flush=True)
for line in sys.stdin:
    if line.strip() == "QUIT":
        break
    values = line.split()
    seed = values[16:23]
    print("SUCCESS 0.1 1 0 0 1 " + " ".join(seed) + " fake_success", flush=True)
