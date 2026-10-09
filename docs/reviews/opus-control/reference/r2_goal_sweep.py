"""R2: sweep ordinary point-to-point goals from rest; classify the outcome.

Ideal tracking (measured == confirmed reference). Single joint, reduction 1,
limits [-2, 2], V=1 rad/s, A=10 rad/s^2, lead 1 rad, h=10 ms, goal dq=0.
Outcome per goal: FAULT step / settled / still moving, plus peak overshoot.
"""
from harness import Rig, spec, JointTarget
from im6014_codec import raw_to_position

GRID = raw_to_position(1)


def run(goal, *, q0=0.0, dt=0.01, steps=600, tracking=True, **kw):
    s = spec(**kw)
    rig = Rig((s,), (q0,), tracking=tracking)
    jt = (JointTarget(goal, 0.0, 0.0, 5.0, 0.2),)
    d = rig.submit(jt)
    if not d.accepted:
        return f"submit rejected {d.code.value}"
    peak = q0
    prev = rig.snap().confirmed_references[0]
    for k in range(1, steps + 1):
        if k % 50 == 0:
            d = rig.submit(jt)
            if not d.accepted:
                return f"resubmit rejected at step {k}: {d.code.value}"
        plan = rig.step(dt)
        snap = rig.snap()
        ref = snap.confirmed_references[0]
        if plan is None:
            f = snap.faults[0] if snap.faults else None
            kind = [n for n in ("constraint_fault", "position_limit", "speed_limit", "effort_limit") if f and getattr(f, n)]
            return (f"FAULT at step {k} ({kind}); q_ref={prev.q_rad:.6f}; "
                    f"overshoot={(peak - goal) if goal >= q0 else (goal - peak):+.4f} rad")
        if (goal >= q0 and ref.q_rad > peak) or (goal < q0 and ref.q_rad < peak):
            peak = ref.q_rad
        prev = ref
    return f"no fault in {steps} steps; q_ref={prev.q_rad:.6f}; overshoot={(peak-goal):+.4f}"


print("ideal tracking, h=10ms, V=1, A=10")
for goal in (0.05, 0.1, 0.2, 0.25, 0.3, 0.5, 0.75, -0.3):
    print(f"  goal={goal:+.3f}: {run(goal)}")
print("goal at the inclusive upper limit (q_max=0.5), from 0.0, lead 1")
print(f"  goal=+0.500: {run(0.5, q_max=0.5)}")
print("static measurement (tracking=False), lead 1")
for goal in (0.2, 0.5):
    print(f"  goal={goal:+.3f}: {run(goal, tracking=False)}")
