"""Shared offline harness for slice-reference reproductions (fake I/O only).

Imports the reviewed worktree modules; never opens a serial port.
"""
import sys

WORKTREE = str(__import__("pathlib").Path(__file__).resolve().parents[4])
if WORKTREE not in sys.path:
    sys.path.insert(0, WORKTREE)

from joint_contract import (  # noqa: E402
    JointSpec, MotorHealth, SourceIdentity, State, Target, WriteResult,
)
from joint_controller import JointCore  # noqa: E402
from joint_mapping import JointMap, JointSample, JointTarget  # noqa: E402
from joint_protection import ProtectionPolicy  # noqa: E402

PROTECTION = ProtectionPolicy(
    driver_trip_c=85.0, driver_reset_c=70.0, winding_trip_c=105.0,
    winding_reset_c=90.0, continuous_effort_nm=0.5, continuous_dwell_s=1.0,
    recovery_dwell_s=0.5,
)
HEALTHY_STOP = MotorHealth(0, 0, 0, False, 25.0, 30.0)
HEALTHY_FOC = MotorHealth(1, 0, 0, False, 25.0, 30.0)


class Clock:
    def __init__(self, value=0.0):
        self.value = value

    def __call__(self):
        return self.value


def spec(name="J1", motor_id=0, *, mapping=None, q_min=-2.0, q_max=2.0,
         v_max=1.0, a_max=10.0, lead=1.0, effort=100.0):
    return JointSpec(
        name, motor_id, mapping or JointMap(1.0, 1, 0.0), q_min, q_max,
        v_max, a_max, lead, effort, "review fixture", PROTECTION,
    )


class Rig:
    """Minimal fake station: perfect-tracking (or static) measurement model."""

    def __init__(self, specs, q0, *, gap=1.0, t0=10.0, tracking=True,
                 feedback_max_age_s=0.5):
        self.clock = Clock(t0)
        self.core = JointCore(specs, clock=self.clock, max_reference_gap_s=gap,
                              feedback_max_age_s=feedback_max_age_s)
        self.specs = specs
        self.q_meas = list(q0)
        self.tracking = tracking
        self.seq = 0
        c = self.core
        assert c.begin_connect().accepted
        assert c.connected(SourceIdentity("fixture", "review", 1)).accepted
        self.owner = c.claim_owner("review")
        assert c.begin_check().accepted
        for s, q in zip(specs, q0):
            assert c.receive(s.motor_id, JointSample(q, 0.0, 0.0), HEALTHY_STOP).accepted
        d = c.checked()
        assert d.accepted, d
        d = c.arm(self.owner)
        assert d.accepted, d

    def feed(self, mode_health=HEALTHY_FOC):
        for s, q in zip(self.specs, self.q_meas):
            self.core.receive(s.motor_id, JointSample(q, 0.0, 0.0), mode_health)

    def submit(self, joints, valid_for=1.0):
        self.seq += 1
        w = self.core.issue_window(valid_for)
        return self.core.submit_target(Target(self.seq, tuple(joints), w, self.owner))

    def step(self, dt):
        """Advance time, feed measurement, prepare, claim+dispatch every joint."""
        self.clock.value += dt
        self.feed()
        plan = self.core.prepare_refresh()
        if plan is None:
            return None
        for i in range(len(plan.packets)):
            if not self.core.claim_dispatch(plan.plan_id, i):
                return plan
            self.core.dispatch_result(plan.plan_id, i, WriteResult.COMPLETE)
        if self.tracking:
            refs = self.core.snapshot().confirmed_references
            self.q_meas = [r.q_rad for r in refs]
        return plan

    def snap(self):
        return self.core.snapshot()
