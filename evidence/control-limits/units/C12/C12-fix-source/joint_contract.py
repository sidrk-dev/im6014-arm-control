"""Frozen domain records and status types for joint control.

All records are immutable (frozen) data contracts with no transport or clock ownership.
"""

from dataclasses import dataclass
from enum import Enum
from fractions import Fraction
import math
from typing import Literal

from im6014_codec import decode_command
from joint_mapping import JointMap, JointSample, JointTarget


class State(Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    CHECKING = "checking"
    READY = "ready"
    ARMED = "armed"
    MOVING = "moving"
    STOPPED = "stopped"
    FAULT = "fault"


class DecisionCode(str, Enum):
    ACCEPTED = "accepted"
    INVALID_STATE = "invalid_state"
    INVALID_ARGUMENT = "invalid_argument"
    INVALID_OWNER = "invalid_owner"
    INVALID_SOURCE = "invalid_source"
    UNKNOWN_JOINT = "unknown_joint"
    INVALID_OBSERVATION = "invalid_observation"
    FAULT_ACK_REQUIRED = "fault_ack_required"
    FAULT_LATCHED = "fault_latched"
    OBSERVATIONS_INCOMPLETE = "observations_incomplete"
    STALE_FEEDBACK = "stale_feedback"
    UNHEALTHY = "unhealthy"
    NOT_STOPPED = "not_stopped"
    DUPLICATE_OR_OLDER = "duplicate_or_older"
    EXPIRED = "expired"
    POSITION_OUT_OF_BOUNDS = "position_out_of_bounds"
    SPEED_OUT_OF_BOUNDS = "speed_out_of_bounds"
    LEAD_OUT_OF_BOUNDS = "lead_out_of_bounds"
    REFERENCE_GAP_EXCEEDED = "reference_gap_exceeded"
    CONSTRAINT_VIOLATION = "constraint_violation"


class CommunicationFailure(str, Enum):
    TIMEOUT = "timeout"
    INVALID_FEEDBACK = "invalid_feedback"
    DRIVER_FAILURE = "driver_failure"
    CLOCK_FAILURE = "clock_failure"
    PREWRITE_DEADLINE = "prewrite_deadline"


class WriteResult(str, Enum):
    COMPLETE = "complete"
    PARTIAL = "partial"
    FAILED = "failed"
    PREWRITE_DEADLINE = "prewrite_deadline"


@dataclass(frozen=True)
class Decision:
    accepted: bool
    code: DecisionCode

    def __post_init__(self) -> None:
        if type(self.accepted) is not bool:
            raise ValueError("accepted must be a bool")
        if not isinstance(self.code, DecisionCode):
            raise ValueError("code must be a DecisionCode")
        if self.accepted != (self.code is DecisionCode.ACCEPTED):
            raise ValueError("accepted must match the decision code")

    def __bool__(self) -> bool:
        return self.accepted


@dataclass(frozen=True)
class JointSpec:
    name: str
    motor_id: int
    mapping: JointMap
    q_min_rad: float
    q_max_rad: float
    v_max_rad_s: float
    a_max_rad_s2: float
    max_lead_rad: float
    limits_provenance: str

    def __post_init__(self) -> None:
        if type(self.name) is not str or not self.name.strip():
            raise ValueError("joint name must be a non-empty string")
        if type(self.motor_id) is not int or not (0 <= self.motor_id <= 14):
            raise ValueError("motor_id must be an integer from 0 to 14")
        if not isinstance(self.mapping, JointMap):
            raise ValueError("mapping must be a JointMap")

        if isinstance(self.q_min_rad, bool) or not isinstance(self.q_min_rad, (int, float)):
            raise ValueError("q_min_rad must be a finite real number")
        if isinstance(self.q_max_rad, bool) or not isinstance(self.q_max_rad, (int, float)):
            raise ValueError("q_max_rad must be a finite real number")
        try:
            min_val = float(self.q_min_rad)
            max_val = float(self.q_max_rad)
        except (OverflowError, TypeError, ValueError) as exc:
            raise ValueError("position limits must be finite") from exc
        if not math.isfinite(min_val) or not math.isfinite(max_val):
            raise ValueError("position limits must be finite")
        if min_val > max_val:
            raise ValueError("q_min_rad must be less than or equal to q_max_rad")

        if isinstance(self.v_max_rad_s, bool) or not isinstance(self.v_max_rad_s, (int, float)):
            raise ValueError("v_max_rad_s must be a finite positive real number")
        try:
            v_max_val = float(self.v_max_rad_s)
        except (OverflowError, TypeError, ValueError) as exc:
            raise ValueError("speed limit must be finite") from exc
        if not math.isfinite(v_max_val) or v_max_val <= 0.0:
            raise ValueError("v_max_rad_s must be finite and positive")

        if isinstance(self.a_max_rad_s2, bool) or not isinstance(self.a_max_rad_s2, (int, float)):
            raise ValueError("a_max_rad_s2 must be a finite positive real number")
        try:
            a_max_val = float(self.a_max_rad_s2)
        except (OverflowError, TypeError, ValueError) as exc:
            raise ValueError("acceleration limit must be finite") from exc
        if not math.isfinite(a_max_val) or a_max_val <= 0.0:
            raise ValueError("a_max_rad_s2 must be finite and positive")

        if isinstance(self.max_lead_rad, bool) or not isinstance(self.max_lead_rad, (int, float)):
            raise ValueError("max_lead_rad must be a finite positive real number")
        try:
            max_lead_val = float(self.max_lead_rad)
        except (OverflowError, TypeError, ValueError) as exc:
            raise ValueError("max_lead_rad must be finite") from exc
        if not math.isfinite(max_lead_val) or max_lead_val <= 0.0:
            raise ValueError("max_lead_rad must be finite and positive")

        if (
            type(self.limits_provenance) is not str
            or not self.limits_provenance.strip()
            or len(self.limits_provenance) > 256
        ):
            raise ValueError("limits_provenance must be a bounded non-empty string")

        object.__setattr__(self, "q_min_rad", min_val)
        object.__setattr__(self, "q_max_rad", max_val)
        object.__setattr__(self, "v_max_rad_s", v_max_val)
        object.__setattr__(self, "a_max_rad_s2", a_max_val)
        object.__setattr__(self, "max_lead_rad", max_lead_val)

@dataclass(frozen=True)
class JointFault:
    name: str
    motor_id: int
    errors: int = 0
    warnings: int = 0
    timeout: bool = False
    communication_error: CommunicationFailure | None = None
    stale: bool = False
    communication_errors: tuple[CommunicationFailure, ...] = ()
    position_limit: bool = False
    speed_limit: bool = False
    constraint_fault: bool = False

    def __post_init__(self) -> None:
        if type(self.name) is not str or not self.name.strip():
            raise ValueError("joint name must be a non-empty string")
        if type(self.motor_id) is not int or not (0 <= self.motor_id <= 14):
            raise ValueError("motor_id must be an integer from 0 to 14")
        if type(self.errors) is not int or not (0 <= self.errors <= 0xFFFFFFFF):
            raise ValueError("errors must be a uint32 integer")
        if type(self.warnings) is not int or not (0 <= self.warnings <= 7):
            raise ValueError("warnings must be a uint3 integer")
        if type(self.timeout) is not bool:
            raise ValueError("timeout must be a bool")
        if self.communication_error is not None and not isinstance(self.communication_error, CommunicationFailure):
            raise ValueError("communication_error must be a CommunicationFailure or None")
        if type(self.stale) is not bool:
            raise ValueError("stale must be a bool")
        if type(self.communication_errors) is not tuple or any(
            not isinstance(c, CommunicationFailure) for c in self.communication_errors
        ):
            raise ValueError("communication_errors must be a tuple of CommunicationFailure")
        if type(self.position_limit) is not bool:
            raise ValueError("position_limit must be a bool")
        if type(self.speed_limit) is not bool:
            raise ValueError("speed_limit must be a bool")
        if type(self.constraint_fault) is not bool:
            raise ValueError("constraint_fault must be a bool")

        if len(self.communication_errors) != len(set(self.communication_errors)):
            raise ValueError("communication_errors must contain unique codes")
        if self.communication_error is not None and self.communication_errors and (
            self.communication_error not in self.communication_errors
        ):
            raise ValueError("communication_error must belong to communication_errors")

        if self.communication_error is not None and not self.communication_errors:
            object.__setattr__(self, "communication_errors", (self.communication_error,))
        elif self.communication_errors and self.communication_error is None:
            object.__setattr__(self, "communication_error", self.communication_errors[-1])


@dataclass(frozen=True)
class MotorHealth:
    mode: int
    errors: int
    warnings: int
    timeout: bool

    def __post_init__(self) -> None:
        if type(self.mode) is not int or self.mode not in (0, 1):
            raise ValueError("mode must be the integer 0 or 1")
        if type(self.errors) is not int or not (0 <= self.errors <= 0xFFFFFFFF):
            raise ValueError("errors must be a uint32 integer")
        if type(self.warnings) is not int or not (0 <= self.warnings <= 7):
            raise ValueError("warnings must be a uint3 integer")
        if type(self.timeout) is not bool:
            raise ValueError("timeout must be a bool")

    @property
    def healthy(self) -> bool:
        return self.errors == 0 and self.warnings == 0 and not self.timeout


@dataclass(frozen=True)
class SourceIdentity:
    backend: Literal["physical", "demo", "fixture"]
    identity: str
    connection: int

    def __post_init__(self) -> None:
        if type(self.backend) is not str or self.backend not in ("physical", "demo", "fixture"):
            raise ValueError("backend must be physical, demo, or fixture")
        if type(self.identity) is not str or not self.identity.strip():
            raise ValueError("identity must be a non-empty string")
        if type(self.connection) is not int or self.connection < 0:
            raise ValueError("connection must be a nonnegative integer")


@dataclass(frozen=True)
class JointStatus:
    sample: JointSample | None = None
    health: MotorHealth | None = None
    received_at: float | None = None
    communication_error: CommunicationFailure | None = None

    def __post_init__(self) -> None:
        if (self.sample is None) != (self.health is None):
            raise ValueError("sample and health must both be present or both be absent")
        if self.sample is not None and not isinstance(self.sample, JointSample):
            raise ValueError("sample must be a JointSample")
        if self.health is not None and not isinstance(self.health, MotorHealth):
            raise ValueError("health must be MotorHealth")
        if self.communication_error is not None and not isinstance(self.communication_error, CommunicationFailure):
            raise ValueError("communication_error must be a CommunicationFailure or None")
        if self.valid and self.received_at is None:
            raise ValueError("a valid observation requires received_at")
        if self.received_at is not None:
            if isinstance(self.received_at, bool) or not isinstance(self.received_at, (int, float)):
                raise ValueError("received_at must be a finite real number or None")
            try:
                received_at = float(self.received_at)
            except (OverflowError, TypeError, ValueError) as exc:
                raise ValueError("received_at must be finite") from exc
            if not math.isfinite(received_at):
                raise ValueError("received_at must be finite")
            if not self.valid:
                raise ValueError("received_at requires a valid observation")
            object.__setattr__(self, "received_at", received_at)

    @property
    def valid(self) -> bool:
        return self.sample is not None and self.health is not None

    @property
    def healthy(self) -> bool:
        return self.valid and self.health.healthy and self.communication_error is None

    @property
    def errors(self) -> int:
        return self.health.errors if self.health is not None else 0

    @property
    def warnings(self) -> int:
        return self.health.warnings if self.health is not None else 0

    @property
    def timeout(self) -> bool:
        return self.health.timeout if self.health is not None else False


@dataclass(frozen=True)
class OwnerToken:
    epoch: str
    generation: int
    connection_id: str

    def __post_init__(self) -> None:
        if type(self.epoch) is not str or not self.epoch.strip() or len(self.epoch) > 64:
            raise ValueError("epoch must be a bounded non-empty string")
        if type(self.generation) is not int or not (1 <= self.generation <= (1 << 64) - 1):
            raise ValueError("generation must be an integer from 1 to 2**64 - 1")
        if type(self.connection_id) is not str or not self.connection_id.strip() or len(self.connection_id) > 64:
            raise ValueError("connection_id must be a bounded non-empty string")


def owner_tokens_match(candidate: object, current: object) -> bool:
    return (
        type(candidate) is OwnerToken
        and type(current) is OwnerToken
        and candidate.epoch == current.epoch
        and candidate.generation == current.generation
        and candidate.connection_id == current.connection_id
    )


@dataclass(frozen=True)
class IntentWindow:
    id: int
    created_at_s: float
    expires_at_s: float

    def __post_init__(self) -> None:
        if type(self.id) is not int or not (0 <= self.id <= (1 << 64) - 1):
            raise ValueError("window ID must be an integer from 0 to 2**64 - 1")
        if isinstance(self.created_at_s, bool) or not isinstance(self.created_at_s, (int, float)):
            raise ValueError("created_at_s must be a finite real number")
        if isinstance(self.expires_at_s, bool) or not isinstance(self.expires_at_s, (int, float)):
            raise ValueError("expires_at_s must be a finite real number")
        try:
            created = float(self.created_at_s)
            expires = float(self.expires_at_s)
        except (OverflowError, TypeError, ValueError) as exc:
            raise ValueError("window timestamps must be finite") from exc
        if not math.isfinite(created) or not math.isfinite(expires):
            raise ValueError("window timestamps must be finite")
        if expires <= created:
            raise ValueError("expires_at_s must be strictly greater than created_at_s")
        object.__setattr__(self, "created_at_s", created)
        object.__setattr__(self, "expires_at_s", expires)


@dataclass(frozen=True)
class Target:
    sequence: int
    joints: tuple[JointTarget, ...]
    window: IntentWindow
    owner: OwnerToken

    def __post_init__(self) -> None:
        if type(self.sequence) is not int or not (0 <= self.sequence <= (1 << 64) - 1):
            raise ValueError("sequence must be an integer from 0 to 2**64 - 1")
        if type(self.joints) is not tuple:
            raise ValueError("joints must be a tuple of JointTarget")
        if not self.joints:
            raise ValueError("joints must be a non-empty tuple")
        if any(not isinstance(joint, JointTarget) for joint in self.joints):
            raise ValueError("every joint target must be a JointTarget")
        if not isinstance(self.window, IntentWindow):
            raise ValueError("window must be an IntentWindow")
        if type(self.owner) is not OwnerToken:
            raise ValueError("owner must be an OwnerToken")


@dataclass(frozen=True)
class JointTrajectoryReference:
    q_rad: float
    position_step_velocity_rad_s: Fraction
    encoded_velocity_rad_s: float
    sample_time_s: float
    is_emitted: bool = False
    requested_goal: JointTarget | None = None
    emitted_target: JointTarget | None = None
    packet: bytes | None = None

    def __post_init__(self) -> None:
        for name, val in (("q_rad", self.q_rad),
                          ("encoded_velocity_rad_s", self.encoded_velocity_rad_s),
                          ("sample_time_s", self.sample_time_s)):
            if isinstance(val, bool) or not isinstance(val, (int, float)):
                raise ValueError(f"{name} must be a finite real number")
            try:
                v = float(val)
            except (OverflowError, TypeError, ValueError) as exc:
                raise ValueError(f"{name} must be finite") from exc
            if not math.isfinite(v):
                raise ValueError(f"{name} must be finite")
            object.__setattr__(self, name, v)

        derivative = self.position_step_velocity_rad_s
        if isinstance(derivative, bool) or not isinstance(derivative, (int, float, Fraction)):
            raise ValueError("position_step_velocity_rad_s must be an exact finite rational")
        if isinstance(derivative, Fraction):
            exact_derivative = derivative
        else:
            try:
                as_float = float(derivative)
            except (OverflowError, TypeError, ValueError) as exc:
                raise ValueError("position_step_velocity_rad_s must be finite") from exc
            if not math.isfinite(as_float):
                raise ValueError("position_step_velocity_rad_s must be finite")
            exact_derivative = Fraction(as_float)
        object.__setattr__(self, "position_step_velocity_rad_s", exact_derivative)

        if type(self.is_emitted) is not bool:
            raise ValueError("is_emitted must be a bool")
        if self.is_emitted:
            if not isinstance(self.requested_goal, JointTarget):
                raise ValueError("an emitted reference requires its requested goal")
            if not isinstance(self.emitted_target, JointTarget):
                raise ValueError("an emitted reference requires its decoded target")
            if type(self.packet) is not bytes:
                raise ValueError("an emitted reference requires immutable packet bytes")
            try:
                decoded = decode_command(self.packet)
            except ValueError as exc:
                raise ValueError("an emitted reference requires a canonical command") from exc
            if decoded["mode"] != 1 or not decoded["watchdog"]:
                raise ValueError("an emitted reference requires a watchdog-enabled FOC command")
            if self.q_rad != self.emitted_target.q_rad:
                raise ValueError("reference position must match its decoded target")
            if self.encoded_velocity_rad_s != self.emitted_target.dq_rad_s:
                raise ValueError("encoded velocity must match its decoded target")
        else:
            if (
                self.requested_goal is not None
                or self.emitted_target is not None
                or self.packet is not None
            ):
                raise ValueError("an unemitted Stop anchor cannot contain an emitted command")
            if exact_derivative != 0 or self.encoded_velocity_rad_s != 0.0:
                raise ValueError("an unemitted Stop anchor must have zero velocity history")


@dataclass(frozen=True)
class RefreshPlan:
    plan_id: int
    target: Target
    prepared_at_s: float
    joint_targets: tuple[JointTarget, ...]
    packets: tuple[bytes, ...]
    proposed_references: tuple[JointTrajectoryReference, ...]

    def __post_init__(self) -> None:
        if type(self.plan_id) is not int or not (0 <= self.plan_id <= (1 << 64) - 1):
            raise ValueError("plan_id must be an integer from 0 to 2**64 - 1")
        if not isinstance(self.target, Target):
            raise ValueError("target must be a Target")
        if isinstance(self.prepared_at_s, bool) or not isinstance(self.prepared_at_s, (int, float)):
            raise ValueError("prepared_at_s must be a finite real number")
        try:
            prepared = float(self.prepared_at_s)
        except (OverflowError, TypeError, ValueError) as exc:
            raise ValueError("prepared_at_s must be finite") from exc
        if not math.isfinite(prepared):
            raise ValueError("prepared_at_s must be finite")
        object.__setattr__(self, "prepared_at_s", prepared)
        if type(self.joint_targets) is not tuple or not self.joint_targets:
            raise ValueError("joint_targets must be a non-empty tuple")
        if any(not isinstance(jt, JointTarget) for jt in self.joint_targets):
            raise ValueError("every joint target must be a JointTarget")
        if len(self.target.joints) != len(self.joint_targets):
            raise ValueError("joint_targets must match the complete target vector")
        if type(self.packets) is not tuple or len(self.packets) != len(self.joint_targets):
            raise ValueError("packets must be a tuple matching joint_targets")
        for packet in self.packets:
            if type(packet) is not bytes:
                raise ValueError("packets must contain immutable bytes")
            try:
                decoded = decode_command(packet)
            except ValueError as exc:
                raise ValueError("packets must contain canonical commands") from exc
            if decoded["mode"] != 1 or not decoded["watchdog"]:
                raise ValueError("refresh packets must be watchdog-enabled FOC commands")
        if (
            type(self.proposed_references) is not tuple
            or len(self.proposed_references) != len(self.joint_targets)
        ):
            raise ValueError("proposed_references must match joint_targets length")
        for goal, target, packet, reference in zip(
            self.target.joints,
            self.joint_targets,
            self.packets,
            self.proposed_references,
        ):
            if not isinstance(reference, JointTrajectoryReference) or not reference.is_emitted:
                raise ValueError("every plan requires a complete emitted reference")
            if reference.requested_goal != goal:
                raise ValueError("reference goal must match the admitted joint goal")
            if reference.emitted_target != target or reference.packet != packet:
                raise ValueError("reference target and packet must match the prepared command")
            if reference.sample_time_s > prepared:
                raise ValueError("reference sample time cannot follow plan preparation")

    @property
    def window(self) -> IntentWindow:
        return self.target.window

    @property
    def owner(self) -> OwnerToken:
        return self.target.owner


@dataclass(frozen=True)
class JointDispatchedRecord:
    sequence: int
    target: JointTarget
    dispatched_at_s: float
    owner: OwnerToken

    def __post_init__(self) -> None:
        if type(self.sequence) is not int or not (0 <= self.sequence <= (1 << 64) - 1):
            raise ValueError("sequence must be an integer from 0 to 2**64 - 1")
        if not isinstance(self.target, JointTarget):
            raise ValueError("target must be a JointTarget")
        if type(self.owner) is not OwnerToken:
            raise ValueError("owner must be an OwnerToken")
        if isinstance(self.dispatched_at_s, bool) or not isinstance(self.dispatched_at_s, (int, float)):
            raise ValueError("dispatched_at_s must be a finite real number")
        try:
            dispatched = float(self.dispatched_at_s)
        except (OverflowError, TypeError, ValueError) as exc:
            raise ValueError("dispatched_at_s must be finite") from exc
        if not math.isfinite(dispatched):
            raise ValueError("dispatched_at_s must be finite")
        object.__setattr__(self, "dispatched_at_s", dispatched)


@dataclass(frozen=True)
class JointDispatchAttempt:
    plan_id: int
    sequence: int
    target: JointTarget
    result: WriteResult
    started_at_s: float
    owner: OwnerToken

    def __post_init__(self) -> None:
        if type(self.plan_id) is not int or not (0 <= self.plan_id <= (1 << 64) - 1):
            raise ValueError("plan_id must be an integer from 0 to 2**64 - 1")
        if type(self.sequence) is not int or not (0 <= self.sequence <= (1 << 64) - 1):
            raise ValueError("sequence must be an integer from 0 to 2**64 - 1")
        if not isinstance(self.target, JointTarget):
            raise ValueError("target must be a JointTarget")
        if not isinstance(self.result, WriteResult):
            raise ValueError("result must be a WriteResult")
        if type(self.owner) is not OwnerToken:
            raise ValueError("owner must be an OwnerToken")
        if isinstance(self.started_at_s, bool) or not isinstance(self.started_at_s, (int, float)):
            raise ValueError("started_at_s must be a finite real number")
        try:
            started = float(self.started_at_s)
        except (OverflowError, TypeError, ValueError) as exc:
            raise ValueError("started_at_s must be finite") from exc
        if not math.isfinite(started):
            raise ValueError("started_at_s must be finite")
        object.__setattr__(self, "started_at_s", started)


@dataclass(frozen=True)
class Snapshot:
    state: State
    joint_names: tuple[str, ...]
    joints: tuple[JointStatus, ...]
    source: SourceIdentity | None
    fault_latched: bool
    feedback_ages_s: tuple[float | None, ...]
    faults: tuple[JointFault, ...] = ()
    admitted_target: Target | None = None
    prepared_plan: RefreshPlan | None = None
    dispatched_records: tuple[JointDispatchedRecord | None, ...] = ()
    last_sequence: int = -1
    dispatch_attempts: tuple[JointDispatchAttempt | None, ...] = ()
    latest_owner: OwnerToken | None = None
    current_owner: OwnerToken | None = None
    confirmed_references: tuple[JointTrajectoryReference | None, ...] = ()

    def __post_init__(self) -> None:
        if not isinstance(self.state, State):
            raise ValueError("state must be a State")
        if type(self.joint_names) is not tuple or any(type(name) is not str for name in self.joint_names):
            raise ValueError("joint_names must be a tuple of strings")
        if type(self.joints) is not tuple or any(not isinstance(joint, JointStatus) for joint in self.joints):
            raise ValueError("joints must be a tuple of JointStatus")
        if type(self.feedback_ages_s) is not tuple:
            raise ValueError("feedback_ages_s must be a tuple")
        if len(self.joint_names) != len(self.joints):
            raise ValueError("joint names and statuses must have equal lengths")
        if len(self.feedback_ages_s) != len(self.joints):
            raise ValueError("feedback ages and joint statuses must have equal lengths")
        normalized_ages: list[float | None] = []
        for joint, age in zip(self.joints, self.feedback_ages_s):
            if joint.received_at is None:
                if age is not None:
                    raise ValueError("feedback age must be None before a joint is received")
                normalized_ages.append(None)
                continue
            if isinstance(age, bool) or not isinstance(age, (int, float)):
                raise ValueError("feedback age must be a finite nonnegative number")
            try:
                age_value = float(age)
            except (OverflowError, TypeError, ValueError) as exc:
                raise ValueError("feedback age must be finite") from exc
            if not math.isfinite(age_value) or age_value < 0.0:
                raise ValueError("feedback age must be finite and nonnegative")
            normalized_ages.append(age_value)
        object.__setattr__(self, "feedback_ages_s", tuple(normalized_ages))
        if self.source is not None and not isinstance(self.source, SourceIdentity):
            raise ValueError("source must be a SourceIdentity or None")
        if type(self.fault_latched) is not bool:
            raise ValueError("fault_latched must be a bool")
        if type(self.faults) is not tuple or any(not isinstance(f, JointFault) for f in self.faults):
            raise ValueError("faults must be a tuple of JointFault")
        if self.admitted_target is not None and not isinstance(self.admitted_target, Target):
            raise ValueError("admitted_target must be a Target or None")
        if self.prepared_plan is not None and not isinstance(self.prepared_plan, RefreshPlan):
            raise ValueError("prepared_plan must be a RefreshPlan or None")
        if not self.dispatched_records:
            object.__setattr__(self, "dispatched_records", tuple(None for _ in self.joints))
        else:
            if type(self.dispatched_records) is not tuple or len(self.dispatched_records) != len(self.joints):
                raise ValueError("dispatched_records must be a tuple with length matching joints")
            if any(r is not None and not isinstance(r, JointDispatchedRecord) for r in self.dispatched_records):
                raise ValueError("every dispatched record must be a JointDispatchedRecord or None")
        if not self.dispatch_attempts:
            object.__setattr__(self, "dispatch_attempts", tuple(None for _ in self.joints))
        else:
            if type(self.dispatch_attempts) is not tuple or len(self.dispatch_attempts) != len(self.joints):
                raise ValueError("dispatch_attempts must be a tuple with length matching joints")
            if any(r is not None and not isinstance(r, JointDispatchAttempt) for r in self.dispatch_attempts):
                raise ValueError("every dispatch attempt must be a JointDispatchAttempt or None")
        if type(self.last_sequence) is not int or not (-1 <= self.last_sequence <= (1 << 64) - 1):
            raise ValueError("last_sequence must be an integer from -1 to 2**64 - 1")
        if self.latest_owner is not None and type(self.latest_owner) is not OwnerToken:
            raise ValueError("latest_owner must be an OwnerToken or None")
        if self.current_owner is not None and type(self.current_owner) is not OwnerToken:
            raise ValueError("current_owner must be an OwnerToken or None")
        if not self.confirmed_references:
            object.__setattr__(self, "confirmed_references", tuple(None for _ in self.joints))
        else:
            if type(self.confirmed_references) is not tuple or len(self.confirmed_references) != len(self.joints):
                raise ValueError("confirmed_references must be a tuple with length matching joints")
            if any(r is not None and not isinstance(r, JointTrajectoryReference) for r in self.confirmed_references):
                raise ValueError("every confirmed reference must be a JointTrajectoryReference or None")

    @property
    def affected_joints(self) -> tuple[str, ...]:
        return tuple(fault.name for fault in self.faults)
