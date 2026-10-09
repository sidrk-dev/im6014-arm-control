"""Per-cycle tracker cost at capacity 256 vs 65536 (list.pop(0) windows + snapshot copies)."""
import time
import common  # noqa: F401
from station_timing import StationTimingTracker
for cap in (256, 65536):
    tr = StationTimingTracker(0.005, capacity=cap)
    t = 0.0
    for _ in range(cap + 10):
        tr.start_cycle(t); tr.end_cycle(t + 0.001); t += 0.005
    a = time.perf_counter()
    for _ in range(200):
        tr.start_cycle(t); tr.end_cycle(t + 0.001); t += 0.005
    b = time.perf_counter()
    for _ in range(20):
        tr.snapshot()
    c = time.perf_counter()
    print(f"capacity={cap}: start+end_cycle {(b-a)/200*1e6:.1f} us/cycle; snapshot() {(c-b)/20*1e3:.3f} ms/call (check_step calls snapshot() once per round)")
