"""R5: max_reference_gap_s couples two incompatible roles.

(a) The Stop anchor is stamped with the CHECK observation time and is never
    refreshed in READY/ARMED, so with a cadence-sized gap (50 ms) any
    check->arm->submit latency above the gap Stops on the first refresh and
    revokes the owner (no write ever happens).
(b) With a gap large enough for (a) (1.0 s, used by every repo test), a station
    hiccup mid-motion lets ONE packet jump the commanded position by
    min(V*h, lead) relative to the previous emitted packet while every
    samplewise V/A certificate still passes.
"""
from fractions import Fraction
from harness import Rig, spec, JointTarget, JointSample
from im6014_codec import decode_command

# (a)
s = spec(v_max=1.0, a_max=10.0, lead=1.0)
rig = Rig((s,), (0.1,), gap=0.05, t0=10.0)
anchor = rig.snap().confirmed_references[0]
print(f"(a) anchor sample_time_s={anchor.sample_time_s} (check observation)")
rig.clock.value += 0.20  # operator/client latency between check and first command
rig.feed()               # fresh healthy observations keep arriving in ARMED
d = rig.submit((JointTarget(0.1, 0.0, 0.0, 5.0, 0.2),))
print(f"    submit at t={rig.clock.value}: {d.code.value}")
plan = rig.step(0.01)
snap = rig.snap()
print(f"    first prepare_refresh -> {plan}; state={snap.state.value}; current_owner={snap.current_owner}; "
      f"fault_latched={snap.fault_latched}; anchor still t={snap.confirmed_references[0].sample_time_s}")

# (b)
s = spec(v_max=1.0, a_max=10.0, lead=1.0, q_max=2.0)
rig = Rig((s,), (0.0,), gap=1.0, t0=10.0)
goal = (JointTarget(0.9, 0.0, 0.0, 5.0, 0.2),)
rig.submit(goal)
for _ in range(3):
    rig.step(0.01)
before = rig.snap().confirmed_references[0]
rig.clock.value += 0.80          # station hiccup: no refresh for 0.8 s
rig.feed()                       # first fresh observation after the hiccup
rig.submit(goal)                 # client re-submits same goal (fresh window)
plan = rig.step(0.0)
after = rig.snap().confirmed_references[0]
q_prev_pkt = before.q_rad
q_new_pkt = after.q_rad
h = after.sample_time_s - before.sample_time_s
print(f"(b) previous emitted q={q_prev_pkt:.6f}; next emitted q={q_new_pkt:.6f}; "
      f"jump={q_new_pkt - q_prev_pkt:.4f} rad in one packet; h={h:.3f}s; "
      f"v_step={float(after.position_step_velocity_rad_s):.4f} <= V; "
      f"|dv|={float(abs(after.position_step_velocity_rad_s - before.position_step_velocity_rad_s)):.4f} <= A*h={10*h:.2f}; "
      f"state={rig.snap().state.value}")
