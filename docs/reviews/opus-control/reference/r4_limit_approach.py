"""R4: no braking look-ahead -> approaching an inclusive limit (or a goal near it)
at speed produces an EMPTY feasible interval and a constraint fault.

Uses the pure shaper with ideal tracking. Also continues the repo's own
'test_feasible_approach_near_q_limit_stays_inside_exact_bounds' scenario
(spec V=2, A=4, anchor 1.95, goal 2.0 == q_max, h=0.1) past its single step.
"""
from fractions import Fraction
from harness import spec, JointTarget, JointSample
from joint_contract import JointTrajectoryReference
from joint_reference import shape_reference_joint


def interval(sp, ref, q_meas, h):
    hq = Fraction(h)
    qp = Fraction(ref.q_rad)
    vp = ref.position_step_velocity_rad_s
    V, A, L = map(Fraction, (sp.v_max_rad_s, sp.a_max_rad_s2, sp.max_lead_rad))
    parts = {
        "speed": (-V, V),
        "accel": (vp - A * hq, vp + A * hq),
        "pos": ((Fraction(sp.q_min_rad) - qp) / hq, (Fraction(sp.q_max_rad) - qp) / hq),
        "lead": ((Fraction(q_meas) - L - qp) / hq, (Fraction(q_meas) + L - qp) / hq),
    }
    lo = max(p[0] for p in parts.values())
    hi = min(p[1] for p in parts.values())
    return lo, hi, parts


def run(label, sp, q0, goal_q, h, steps=500):
    goal = JointTarget(goal_q, 0.0, 0.0, 1.0, 0.1)
    ref = JointTrajectoryReference(q0, Fraction(0), 0.0, 10.0, False)
    t = 10.0
    print(f"{label}: q0={q0} goal={goal_q} q_max={sp.q_max_rad} V={sp.v_max_rad_s} A={sp.a_max_rad_s2} h={h}")
    for k in range(1, steps + 1):
        code, tgt, new, pkt = shape_reference_joint(sp, goal, ref, JointSample(ref.q_rad, 0, 0), t + h, 1.0)
        if code is not None:
            lo, hi, parts = interval(sp, ref, ref.q_rad, (t + h) - t)
            print(f"  step {k}: {code.value}; q_prev={ref.q_rad:.6f} v_prev={float(ref.position_step_velocity_rad_s):+.4f}")
            print(f"    v_lo={float(lo):+.4f} (accel lower {float(parts['accel'][0]):+.4f}) "
                  f"v_hi={float(hi):+.4f} (position upper {float(parts['pos'][1]):+.4f}) -> "
                  f"{'EMPTY' if lo > hi else 'non-empty'}")
            return
        if k <= 2:
            print(f"  step {k}: q={new.q_rad:.6f} v={float(new.position_step_velocity_rad_s):+.4f}")
        ref, t = new, t + h
    print(f"  no rejection in {steps} steps")


run("[repo near-limit test continued]", spec(q_min=-2.0, q_max=2.0, v_max=2.0, a_max=4.0, lead=100.0), 1.95, 2.0, 0.1)
run("[goal == q_max from 0.0]", spec(q_max=0.5, v_max=1.0, a_max=10.0), 0.0, 0.5, 0.01)
run("[goal 0.45 with q_max 0.5]", spec(q_max=0.5, v_max=1.0, a_max=10.0), 0.0, 0.45, 0.01)
