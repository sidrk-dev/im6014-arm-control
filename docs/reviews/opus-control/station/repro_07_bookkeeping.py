"""(A) check_step() labels its Stop-query transactions with the index of the NEXT
loop cycle, so they collide with that cycle's mode-1 transactions.
(B) check_step() ignores JointCore.receive()'s Decision: rejected observations are
recorded as outcome OK.
(C) TimingSnapshot min/max/mean are over the last `capacity` samples while counters
are cumulative; an earlier stall disappears from max_actual_period_s.
"""
import time

import common
from common import ScriptedBus, armed, submit
from station_timing import StationTimingTracker

print("[A] cycle_index of check_step records vs loop cycles")
bus = ScriptedBus()
core, owner, tracker, coord = armed(bus, time.monotonic)
submit(core, owner, 1)
coord.step()
rows = [(t.cycle_index, t.joint_name, t.plan_id, t.outcome) for t in tracker.snapshot().recent_transactions]
for r in rows:
    print("   cycle_index=%d %s plan_id=%s outcome=%s" % r)
print("   distinct cycle indices:", sorted({r[0] for r in rows}), "for 1 executed cycle + 1 check round")

print()
print("[B] check_step after core.disconnect(): core rejects every observation")
core.stop()
core.disconnect()
recs = coord.check_step()
print("   ", [(r.joint_name, r.outcome, r.feedback_received) for r in recs])
from station_step import mapped_sample, mapped_health  # noqa: E402
from im6014_codec import feedback  # noqa: E402
reply = feedback(common.make_feedback_frame(0), 0)
d = core.receive(0, mapped_sample(core.joint_specs[0].mapping, reply), mapped_health(reply))
print(f"    direct core.receive -> accepted={d.accepted} code={d.code}")

print()
print("[C] capacity=256: one 60 ms period then 300 periods of 5 ms")
tr = StationTimingTracker(0.005, capacity=256)
t = 0.0
tr.start_cycle(t); tr.end_cycle(t + 0.001)
t += 0.060
tr.start_cycle(t); tr.end_cycle(t + 0.001)
for _ in range(300):
    t += 0.005
    tr.start_cycle(t); tr.end_cycle(t + 0.001)
snap = tr.snapshot()
print(f"   cycles={snap.counters.cycles} max_actual_period_s={snap.max_actual_period_s:.4f} "
      f"(true max 0.0600) mean={snap.mean_actual_period_s:.4f}")
