"""R7: 3-joint arm, 'hold current pose' right after arm -> whole-arm constraint
fault on the 2nd refresh; contrast with the same request snapped to the
emitted grid (which holds for 500 cycles). 100 Hz cadence, ideal tracking.
"""
from harness import Rig, spec, JointTarget
from joint_mapping import JointMap

specs = (
    spec("shoulder", 1, mapping=JointMap(1.0, 1, 0.0), q_min=-1.5, q_max=1.5, v_max=1.0, a_max=10.0, lead=0.3),
    spec("elbow", 2, mapping=JointMap(1.0, -1, 0.2), q_min=-2.0, q_max=0.5, v_max=1.0, a_max=10.0, lead=0.3),
    spec("wrist", 3, mapping=JointMap(1.0, 1, 0.0), q_min=-2.5, q_max=2.5, v_max=2.0, a_max=20.0, lead=0.3),
)
pose = (0.1, -0.35, 1.2)  # measured joint angles at check (rad)


def hold(goal_pose, label, steps=500):
    rig = Rig(specs, pose)
    goals = tuple(JointTarget(q, 0.0, 0.0, 10.0, 0.5) for q in goal_pose)
    print(f"[{label}] submit: {rig.submit(goals).code.value}")
    for k in range(1, steps + 1):
        if k % 50 == 0:
            rig.submit(goals)
        if rig.step(0.01) is None:
            snap = rig.snap()
            print(f"  refresh {k}: no plan; state={snap.state.value}; faulted joints="
                  f"{[(f.name, 'constraint' if f.constraint_fault else '?') for f in snap.faults]}; "
                  f"current_owner={snap.current_owner}")
            return rig
        if k == 1:
            print("  refresh 1 emitted q =", [r.q_rad for r in rig.snap().confirmed_references])
    print(f"  held for {steps} refreshes; state={rig.snap().state.value}")
    return rig


rig = hold(pose, "goal = measured pose")
snapped = tuple(r.q_rad for r in Rig(specs, pose).snap().confirmed_references)
# snap each goal to the grid point the first refresh emits (what a client cannot know a priori)
probe = Rig(specs, pose)
probe.submit(tuple(JointTarget(q, 0.0, 0.0, 10.0, 0.5) for q in pose))
probe.step(0.01)
grid_pose = tuple(r.q_rad for r in probe.snap().confirmed_references)
hold(grid_pose, "goal = first emitted grid pose (client-side workaround)")
