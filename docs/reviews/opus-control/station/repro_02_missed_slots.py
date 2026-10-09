"""run_steps counts a 'missed slot skipped' for every wake-up that is late by any
amount, although the late slot is executed (not skipped) and the next slot is
re-anchored to the actual start. Deterministic and real-clock variants.
"""
import math
import time

import common
from common import ScriptedBus, armed


class Clock:
    def __init__(self, t=100.0):
        self.t = t

    def __call__(self):
        return self.t


print("== Deterministic: nominal 10 ms, every sleep oversleeps by 50 us, idle cycles take 1 ms ==")
clk = Clock()
core, owner, tracker, coord = armed(ScriptedBus(), clk, nominal=0.010)
orig_tick = core.tick


def tick():
    clk.t += 0.001
    return orig_tick()


core.tick = tick
recs = coord.run_steps(10, sleep_fn=lambda d: setattr(clk, "t", clk.t + d + 50e-6))
for r in recs:
    print(f"  cycle {r.cycle_index}: actual_period={r.actual_period_s and round(r.actual_period_s*1e3,3)} ms "
          f"exec={r.execution_duration_s*1e3:.3f} ms overrun={r.overrun} missed_slots_skipped={r.missed_slots_skipped}")
snap = tracker.snapshot()
print(f"  counters: cycles={snap.counters.cycles} overruns={snap.counters.overruns} "
      f"missed_slots_skipped={snap.counters.missed_slots_skipped}  (true skipped slots: 0)")

print()
print("== Real clock + time.sleep: nominal 5 ms, 100 idle cycles ==")
clk2 = time.monotonic
core2, owner2, tracker2, coord2 = armed(ScriptedBus(), clk2, nominal=0.005)
recs2 = coord2.run_steps(100)
lates = []
for prev, cur in zip(recs2, recs2[1:]):
    lates.append(cur.started_s - (prev.started_s + 0.005))
true_missed = sum(max(0, math.floor(l / 0.005)) for l in lates)
snap2 = tracker2.snapshot()
print(f"  counters: cycles={snap2.counters.cycles} overruns={snap2.counters.overruns} "
      f"missed_slots_skipped={snap2.counters.missed_slots_skipped}")
print(f"  start lateness vs anchored slot: min={min(lates)*1e6:.1f} us median={sorted(lates)[len(lates)//2]*1e6:.1f} us "
      f"max={max(lates)*1e6:.1f} us; cycles late by >= 1 full period: {sum(1 for l in lates if l >= 0.005)}; "
      f"whole periods actually elapsed past slots: {true_missed}")
print(f"  mean actual period {snap2.mean_actual_period_s*1e3:.3f} ms vs nominal 5.000 ms")
