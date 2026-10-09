"""Shared offline helpers for slice-station reproductions (no physical I/O)."""
import math
import sys
from pathlib import Path

REPO = Path(str(__import__("pathlib").Path(__file__).resolve().parents[4]))
for p in (REPO, REPO / "scripts"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from benchmark_station_timing import make_feedback_frame  # noqa: E402
from joint_contract import (  # noqa: E402
    JointSpec, ProtectionPolicy, SourceIdentity, Target,
)
from joint_controller import JointCore  # noqa: E402
from joint_mapping import JointMap, JointTarget  # noqa: E402
from station_step import StationStepCoordinator  # noqa: E402
from station_timing import StationTimingConfig, StationTimingTracker  # noqa: E402


def three_joint_specs():
    policy = ProtectionPolicy(
        driver_trip_c=85.0, driver_reset_c=70.0, winding_trip_c=105.0,
        winding_reset_c=90.0, continuous_effort_nm=10.0,
        continuous_dwell_s=0.5, recovery_dwell_s=0.25,
    )
    return tuple(
        JointSpec(
            name=f"J{i + 1}", motor_id=i, mapping=JointMap(1.0, 1, 0.0),
            q_min_rad=-3.14, q_max_rad=3.14, v_max_rad_s=10.0,
            a_max_rad_s2=50.0, max_lead_rad=1.0, max_effort_nm=20.0,
            limits_provenance=f"review fixture J{i + 1}", protection=policy,
        )
        for i in range(3)
    )


class ScriptedBus:
    """In-memory bus: each write appends the next scripted reply (or nothing)."""

    def __init__(self, replies_by_motor=None):
        # replies_by_motor: dict motor_id -> list of frames (b'' = no reply)
        self.replies = {k: list(v) for k, v in (replies_by_motor or {}).items()}
        self.timeout = 0.005
        self.write_timeout = 0.05
        self._buf = bytearray()
        self.writes = []

    @property
    def in_waiting(self):
        return len(self._buf)

    def reset_input_buffer(self):
        self._buf.clear()

    def write(self, data):
        data = bytes(data)
        self.writes.append(data)
        motor_id = data[2] & 0x0F
        mode = (data[2] >> 4) & 7
        queue = self.replies.get(motor_id)
        if queue:
            frame = queue.pop(0)
        else:
            frame = make_feedback_frame(motor_id, mode=mode)
        self._buf.extend(frame)
        return len(data)

    def read(self, size):
        chunk = bytes(self._buf[:size])
        del self._buf[:size]
        return chunk


def armed(bus, clock, *, nominal=0.005, deadline_s=0.05, specs=None):
    specs = specs or three_joint_specs()
    core = JointCore(specs, clock=clock, max_reference_gap_s=1.0)
    core.begin_connect()
    core.connected(SourceIdentity("fixture", "review", 1))
    owner = core.claim_owner("review-client")
    core.begin_check()
    tracker = StationTimingTracker(StationTimingConfig(nominal_period_s=nominal))
    coord = StationStepCoordinator(core, bus, tracker, clock=clock, deadline_s=deadline_s)
    setup = coord.check_step()
    assert all(r.outcome == "OK" for r in setup), [r.outcome for r in setup]
    assert core.checked().accepted
    assert core.arm(owner).accepted
    return core, owner, tracker, coord


def submit(core, owner, seq, n=3):
    window = core.issue_window(valid_for_s=0.5)
    targets = tuple(
        JointTarget(q_rad=0.01 * math.sin(seq * 0.1), dq_rad_s=0.0, tau_nm=0.0, kp=5.0, kd=0.5)
        for _ in range(n)
    )
    d = core.submit_target(Target(sequence=seq, joints=targets, window=window, owner=owner))
    assert d.accepted, d
