"""R3: diagnose which rule rejects in R1/R2 using the pure shaper.

Drives joint_reference.shape_reference_joint directly (no core), ideal tracking,
and at the first rejection recomputes the feasible interval exactly to show
whether it was empty or whether a non-empty interval only contained the
previous grid point (line 615 'v_desired != 0 and q unchanged' rule).
"""
from fractions import Fraction
import random
from harness import spec, JointTarget, JointSample
from joint_contract import JointTrajectoryReference
from joint_reference import shape_reference_joint
from im6014_codec import raw_to_position

GRID = Fraction(raw_to_position(1))


def diagnose(sp, goal, ref, q_meas, h):
    hq = Fraction(h)
    qp = Fraction(ref.q_rad)
    vp = ref.position_step_velocity_rad_s
    V, A, L = map(Fraction, (sp.v_max_rad_s, sp.a_max_rad_s2, sp.max_lead_rad))
    lo = max(-V, (Fraction(sp.q_min_rad) - qp) / hq, vp - A * hq, (Fraction(q_meas) - L - qp) / hq)
    hi = min(V, (Fraction(sp.q_max_rad) - qp) / hq, vp + A * hq, (Fraction(q_meas) + L - qp) / hq)
    vd = (Fraction(goal.q_rad) - qp) / hq + Fraction(goal.dq_rad_s)
    if lo > hi:
        return f"EMPTY interval: v_lo={float(lo):.6g} > v_hi={float(hi):.6g}"
    width_grid = (hi - lo) * hq / GRID
    return (f"interval non-empty [{float(lo):.4g}, {float(hi):.4g}] rad/s; reachable q-width = "
            f"{float(width_grid):.3f} grid steps; v_desired={float(vd):.4g} != 0; "
            f"projected step {float(max(lo, min(hi, vd)) * hq / GRID):+.3f} grid -> rounds to 'no move' -> stall rule")


def run(goal_q, *, q0=0.0, a=10.0, v=1.0, h_fn=lambda k: 0.01, steps=2000, label=""):
    sp = spec(a_max=a, v_max=v)
    goal = JointTarget(goal_q, 0.0, 0.0, 5.0, 0.2)
    ref = JointTrajectoryReference(q0, Fraction(0), 0.0, 10.0, False)
    t = 10.0
    for k in range(1, steps + 1):
        h = h_fn(k)
        t_new = t + h
        q_meas = ref.q_rad
        code, tgt, new_ref, pkt = shape_reference_joint(sp, goal, ref, JointSample(q_meas, 0, 0), t_new, 1.0)
        if code is not None:
            print(f"{label} goal={goal_q} a={a}: step {k} -> {code.value}; q_prev={ref.q_rad:.6f} "
                  f"v_prev={float(ref.position_step_velocity_rad_s):+.5f}")
            print("   ", diagnose(sp, goal, ref, q_meas, t_new - t))
            return
        ref, t = new_ref, t_new
    print(f"{label} goal={goal_q} a={a}: no rejection in {steps} steps, q={ref.q_rad:.6f}")


run(0.1, q0=0.1, label="[hold-at-measured]")
run(0.3, label="[h=10ms]")
for a in (2.0, 3.7, 25.0, 100.0):
    run(0.3, a=a, label="[h=10ms]")
rng = random.Random(1)
run(0.3, h_fn=lambda k: 0.01 + rng.uniform(-0.0005, 0.0005), label="[h=10ms +-0.5ms jitter]")
rng2 = random.Random(2)
run(0.3, a=50.0, v=2.0, h_fn=lambda k: 0.002 + rng2.uniform(-1e-4, 1e-4), label="[h=2ms +-0.1ms jitter]")

# Control: a grid-exact goal (what the repo's hold tests use) still faults at the velocity reversal.
from im6014_codec import position_to_raw
g = raw_to_position(position_to_raw(0.3))
run(g, label="[grid-exact goal, h=10ms]")
for a in (2.0, 25.0):
    run(g, a=a, label="[grid-exact goal, h=10ms]")
