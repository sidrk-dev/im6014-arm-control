"""R9: the parent's retained exact-certificate scenario
(evidence/control-limits/units/C15/C14-exact-oracle.py: V=0.2, A=4, lead=0.125,
goal=+-0.075 with feedforward +-0.05, measured 0.0, h alternating 1/32,1/64)
stops at 16 steps. Continue the identical inputs until rejection (max 400).
"""
import sys
from fractions import Fraction
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[4]))
from joint_contract import JointSpec, JointTrajectoryReference
from joint_protection import ProtectionPolicy
from joint_mapping import JointMap, JointTarget, JointSample
from joint_reference import shape_reference_joint

summary = {}
for reduction in (.5, 1, 3, 100):
    for direction in (-1, 1):
        for zero in (-.1, 0, .25):
            mapping = JointMap(reduction, direction, zero)
            joint = JointSpec('fixture', 0, mapping, -1, 1, .2, 4.0, .125, 1000.0, 'oracle fixture',
                              ProtectionPolicy(85, 70, 105, 90, 500, 1, .25))
            for sign in (1, -1):
                ref = JointTrajectoryReference(0, 0, 0, 0, False)
                goal = JointTarget(sign * .075, sign * .05, 0, 1, .1)
                result = "no rejection in 400 steps"
                for step in range(400):
                    now = ref.sample_time_s + (1 / 64 if step % 2 else 1 / 32)
                    code, target, nxt, pkt = shape_reference_joint(joint, goal, ref, JointSample(0.0, -.05, 0.0), now, .25)
                    if code is not None:
                        result = f"{code.value} at step {step} (q_prev={ref.q_rad:+.6f}, goal={goal.q_rad:+.3f})"
                        break
                    ref = nxt
                summary.setdefault(result.split(" at")[0], []).append((reduction, direction, zero, sign, result))

for k, rows in summary.items():
    print(f"{k}: {len(rows)} of 48 cases")
    for r in rows[:4]:
        print("   ", r)

# Trajectory character for one case (reduction 1, direction +1, zero 0, sign +1)
mapping = JointMap(1, 1, 0)
joint = JointSpec('fixture', 0, mapping, -1, 1, .2, 4.0, .125, 1000.0, 'oracle fixture',
                  ProtectionPolicy(85, 70, 105, 90, 500, 1, .25))
ref = JointTrajectoryReference(0, 0, 0, 0, False)
goal = JointTarget(.075, .05, 0, 1, .1)
qs = []
for step in range(400):
    now = ref.sample_time_s + (1 / 64 if step % 2 else 1 / 32)
    code, target, ref, pkt = shape_reference_joint(joint, goal, ref, JointSample(0.0, -.05, 0.0), now, .25)
    qs.append(ref.q_rad)
tail = qs[200:]
print(f"case (1,+1,0,+1): steps 200-399 q_ref min={min(tail):.5f} max={max(tail):.5f} "
      f"(goal 0.075); sign changes of dq in tail="
      f"{sum(1 for a, b, c in zip(tail, tail[1:], tail[2:]) if (b - a) * (c - b) < 0)}")
