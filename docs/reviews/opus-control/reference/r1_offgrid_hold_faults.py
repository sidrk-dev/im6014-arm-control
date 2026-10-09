"""R1: a goal that is not exactly on the emitted position grid cannot be held.

Scenario A: 'hold where you are' (goal q == checked measured q == 0.1, dq=0).
Scenario B: move to an ordinary goal (0.3) from rest; once the reference settles
            next to the goal it faults instead of holding.
Measurement follows the confirmed reference exactly (ideal tracking).
"""
from fractions import Fraction
from harness import Rig, spec, JointTarget, State
from im6014_codec import raw_to_position, RATIO
import math

GRID = raw_to_position(1)
print(f"output position grid = {GRID!r} rad (joint grid for reduction 1)")


def run(label, q0, goal, steps=400, dt=0.01, **kw):
    s = spec(**kw)
    rig = Rig((s,), (q0,))
    d = rig.submit((JointTarget(goal, 0.0, 0.0, 5.0, 0.2),))
    print(f"\n[{label}] q0={q0} goal={goal} submit={d.code.value}")
    prev = rig.snap().confirmed_references[0]
    for k in range(1, steps + 1):
        if k % 50 == 0:  # keep the window alive with a fresh sequence
            rig.submit((JointTarget(goal, 0.0, 0.0, 5.0, 0.2),))
        plan = rig.step(dt)
        snap = rig.snap()
        ref = snap.confirmed_references[0]
        if plan is None:
            f = snap.faults[0] if snap.faults else None
            print(f"  step {k}: prepare_refresh -> None; state={snap.state.value}; "
                  f"constraint_fault={getattr(f, 'constraint_fault', None)}")
            print(f"  last confirmed q={prev.q_rad!r} (goal-q={goal - prev.q_rad:.3e}, "
                  f"|goal-q|/grid={abs(goal - prev.q_rad) / GRID:.3f}), "
                  f"v_step={float(prev.position_step_velocity_rad_s):.3e}")
            return k, snap.state
        if k <= 3 or ref.q_rad != prev.q_rad and k % 25 == 0:
            print(f"  step {k}: q_ref={ref.q_rad!r} v_step={float(ref.position_step_velocity_rad_s):.4g}")
        prev = ref
    print(f"  survived {steps} steps; final q={prev.q_rad!r}")
    return steps, rig.snap().state


run("A hold-at-measured", 0.1, 0.1)
run("B move-and-hold", 0.0, 0.3, a_max=10.0, v_max=1.0)
run("C grid-exact hold control", 0.0, 0.0)
