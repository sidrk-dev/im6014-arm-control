"""Shared fixture for slice-controller reproductions (fake I/O only, no serial)."""
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


class Clock:
    def __init__(self, value=0.0):
        self.value = value

    def __call__(self):
        return self.value


POLICY = ProtectionPolicy(
    driver_trip_c=85.0, driver_reset_c=70.0,
    winding_trip_c=105.0, winding_reset_c=90.0,
    continuous_effort_nm=4.0,      # joint Nm
    continuous_dwell_s=0.5,
    recovery_dwell_s=0.25,
)


def arm_specs():
    # Three joints, external reduction 1 so joint Nm == actuator output Nm.
    return tuple(
        JointSpec(
            name=f"J{i + 1}", motor_id=i, mapping=JointMap(1.0, 1, 0.0),
            q_min_rad=-2.0, q_max_rad=2.0, v_max_rad_s=5.0, a_max_rad_s2=50.0,
            max_lead_rad=0.5, max_effort_nm=8.0,
            limits_provenance="review fixture", protection=POLICY,
        )
        for i in range(3)
    )


def health(mode=0, driver_c=25.0, winding_c=30.0, errors=0):
    return MotorHealth(mode=mode, errors=errors, warnings=0, timeout=False,
                       driver_c=driver_c, winding_c=winding_c)


def make_core(clock, **kw):
    kw.setdefault("max_reference_gap_s", 0.5)
    kw.setdefault("feedback_max_age_s", 0.5)
    return JointCore(arm_specs(), clock=clock, **kw)


def feed_all(core, tau=0.0, mode=0, driver_c=25.0, winding_c=30.0, q=0.0):
    out = []
    for spec in core.joint_specs:
        out.append(core.receive(spec.motor_id, JointSample(q, 0.0, tau),
                                health(mode, driver_c, winding_c)))
    return out


def connect_check_arm(core, clock, owner_name="client", ack=False, dwell_steps=0, step=0.1,
                      driver_c=25.0, winding_c=30.0):
    if core.snapshot().state is State.DISCONNECTED:
        assert core.begin_connect().accepted
        assert core.connected(SourceIdentity("fixture", "review", 1)).accepted
    owner = core.claim_owner(owner_name)
    d = core.begin_check(acknowledge_fault=ack)
    assert d.accepted, d
    feed_all(core, driver_c=driver_c, winding_c=winding_c)
    for _ in range(dwell_steps):
        clock.value += step
        feed_all(core, driver_c=driver_c, winding_c=winding_c)
    d = core.checked()
    if not d.accepted:
        return owner, d
    d = core.arm(owner)
    return owner, d
