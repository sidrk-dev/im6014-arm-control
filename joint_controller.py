"""Transport-free lifecycle and health state for an ordered joint set.

All methods on a JointCore belong to one calling thread. The core owns no
transport and all externally visible records are immutable snapshots.
"""

from collections.abc import Iterable
from dataclasses import dataclass
from enum import Enum
import math
import time
from typing import Literal

from im6014_codec import command
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


class CommunicationFailure(str, Enum):
    TIMEOUT = "timeout"
    INVALID_FEEDBACK = "invalid_feedback"
    DRIVER_FAILURE = "driver_failure"


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

    def __post_init__(self) -> None:
        if type(self.name) is not str or not self.name.strip():
            raise ValueError("joint name must be a non-empty string")
        if type(self.motor_id) is not int or not 0 <= self.motor_id <= 14:
            raise ValueError("motor_id must be an integer from 0 to 14")
        if not isinstance(self.mapping, JointMap):
            raise ValueError("mapping must be a JointMap")


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
class Target:
    sequence: int
    joints: tuple[JointTarget, ...]

    def __post_init__(self) -> None:
        if type(self.sequence) is not int or not (0 <= self.sequence <= (1 << 64) - 1):
            raise ValueError("sequence must be an integer from 0 to 2**64 - 1")
        if type(self.joints) is not tuple:
            raise ValueError("joints must be a tuple of JointTarget")
        if not self.joints:
            raise ValueError("joints must be a non-empty tuple")
        if any(not isinstance(joint, JointTarget) for joint in self.joints):
            raise ValueError("every joint target must be a JointTarget")


@dataclass(frozen=True)
class Snapshot:
    state: State
    joint_names: tuple[str, ...]
    joints: tuple[JointStatus, ...]
    source: SourceIdentity | None
    fault_latched: bool
    feedback_ages_s: tuple[float | None, ...]
    faults: tuple[JointFault, ...] = ()
    pending_target: Target | None = None
    last_sequence: int = -1

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
        if self.pending_target is not None and not isinstance(self.pending_target, Target):
            raise ValueError("pending_target must be a Target or None")
        if type(self.last_sequence) is not int or not (-1 <= self.last_sequence <= (1 << 64) - 1):
            raise ValueError("last_sequence must be an integer from -1 to 2**64 - 1")

    @property
    def affected_joints(self) -> tuple[str, ...]:
        return tuple(fault.name for fault in self.faults)


class JointCore:
    """Thread-confined state machine for one fixed, ordered set of joints."""

    def __init__(
        self,
        joints: Iterable[JointSpec],
        *,
        feedback_max_age_s: float = 0.5,
        clock=None,
    ):
        if isinstance(feedback_max_age_s, bool) or not isinstance(feedback_max_age_s, (int, float)):
            raise ValueError("feedback_max_age_s must be a finite positive real number")
        try:
            feedback_max_age_s = float(feedback_max_age_s)
        except (OverflowError, TypeError, ValueError) as exc:
            raise ValueError("feedback_max_age_s must be finite") from exc
        if not math.isfinite(feedback_max_age_s) or feedback_max_age_s <= 0.0:
            raise ValueError("feedback_max_age_s must be finite and positive")
        if clock is None:
            clock = time.monotonic
        if not callable(clock):
            raise ValueError("clock must be callable")

        try:
            specs = tuple(joints)
        except TypeError as exc:
            raise ValueError("joints must be an iterable of JointSpec") from exc
        if not specs:
            raise ValueError("at least one active joint is required")
        if any(not isinstance(spec, JointSpec) for spec in specs):
            raise ValueError("every joint must be a JointSpec")
        if len({spec.name for spec in specs}) != len(specs):
            raise ValueError("joint names must be unique")
        if len({spec.motor_id for spec in specs}) != len(specs):
            raise ValueError("motor IDs must be unique")

        self._specs: tuple[JointSpec, ...] = specs
        self._id_to_index: dict[int, int] = {
            spec.motor_id: index for index, spec in enumerate(specs)
        }
        self._state: State = State.DISCONNECTED
        self._observations: list[JointStatus] = [JointStatus() for _ in specs]
        self._source: SourceIdentity | None = None
        self._feedback_max_age_s = feedback_max_age_s
        self._clock = clock
        self._faults: dict[str, JointFault] = {}
        self._pending_target: Target | None = None
        self._last_sequence: int = -1

    @property
    def joint_specs(self) -> tuple[JointSpec, ...]:
        return self._specs

    @staticmethod
    def _accepted() -> Decision:
        return Decision(True, DecisionCode.ACCEPTED)

    @staticmethod
    def _rejected(code: DecisionCode) -> Decision:
        return Decision(False, code)

    def _clear_observations(self) -> None:
        self._observations = [JointStatus() for _ in self._specs]

    def _latch_joint_fault(
        self,
        name: str,
        motor_id: int,
        *,
        errors: int = 0,
        warnings: int = 0,
        timeout: bool = False,
        communication_error: CommunicationFailure | None = None,
        stale: bool = False,
    ) -> None:
        existing = self._faults.get(name) or JointFault(name, motor_id)
        communication_errors = existing.communication_errors
        if communication_error is not None and communication_error not in communication_errors:
            communication_errors += (communication_error,)
        fault = JointFault(
            name, motor_id,
            errors=existing.errors | errors,
            warnings=existing.warnings | warnings,
            timeout=existing.timeout or timeout,
            stale=existing.stale or stale,
            communication_error=communication_error or existing.communication_error,
            communication_errors=communication_errors,
        )
        self._faults[name] = fault
        self._state = State.FAULT
        self._pending_target = None

    def _now(self) -> float:
        value = self._clock()
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError("clock must return a finite real number")
        try:
            result = float(value)
        except (OverflowError, TypeError, ValueError) as exc:
            raise ValueError("clock must return a finite real number") from exc
        if not math.isfinite(result):
            raise ValueError("clock must return a finite real number")
        return result

    @staticmethod
    def _age(received_at: float, now: float) -> float:
        age = now - received_at
        if not math.isfinite(age) or age < 0.0:
            raise ValueError("clock must be monotonic and feedback age finite")
        return age

    def _all_fresh(self, now: float) -> bool:
        return all(
            joint.valid
            and joint.received_at is not None
            and self._age(joint.received_at, now) <= self._feedback_max_age_s
            for joint in self._observations
        )

    def begin_connect(self) -> Decision:
        if self._state is not State.DISCONNECTED:
            return self._rejected(DecisionCode.INVALID_STATE)
        self._state = State.CONNECTING
        return self._accepted()

    def connected(self, source: SourceIdentity) -> Decision:
        if self._state is not State.CONNECTING:
            return self._rejected(DecisionCode.INVALID_STATE)
        if not isinstance(source, SourceIdentity):
            return self._rejected(DecisionCode.INVALID_SOURCE)
        self._clear_observations()
        self._source = source
        self._state = State.CONNECTED
        return self._accepted()

    def begin_check(self, *, acknowledge_fault: bool = False) -> Decision:
        if type(acknowledge_fault) is not bool:
            return self._rejected(DecisionCode.INVALID_ARGUMENT)
        if self._state not in (State.CONNECTED, State.STOPPED, State.FAULT):
            return self._rejected(DecisionCode.INVALID_STATE)
        if bool(self._faults) and not acknowledge_fault:
            return self._rejected(DecisionCode.FAULT_ACK_REQUIRED)
        self._clear_observations()
        self._pending_target = None
        self._state = State.CHECKING
        return self._accepted()

    def receive(
        self, motor_id: int, sample: JointSample, status: MotorHealth
    ) -> Decision:
        if self._state not in (
            State.CONNECTED,
            State.CHECKING,
            State.READY,
            State.ARMED,
            State.MOVING,
            State.STOPPED,
            State.FAULT,
        ):
            return self._rejected(DecisionCode.INVALID_STATE)
        if type(motor_id) is not int:
            return self._rejected(DecisionCode.INVALID_ARGUMENT)
        index = self._id_to_index.get(motor_id)
        if index is None:
            return self._rejected(DecisionCode.UNKNOWN_JOINT)
        if not isinstance(sample, JointSample) or not isinstance(status, MotorHealth):
            return self._rejected(DecisionCode.INVALID_OBSERVATION)

        self._observations[index] = JointStatus(sample, status, self._now())
        if not status.healthy:
            self._latch_joint_fault(
                self._specs[index].name,
                motor_id,
                errors=status.errors,
                warnings=status.warnings,
                timeout=status.timeout,
            )
        return self._accepted()

    def communication_failed(
        self, motor_id: int, code: CommunicationFailure
    ) -> Decision:
        if self._state in (State.DISCONNECTED, State.CONNECTING):
            return self._rejected(DecisionCode.INVALID_STATE)
        if type(motor_id) is not int:
            return self._rejected(DecisionCode.INVALID_ARGUMENT)
        index = self._id_to_index.get(motor_id)
        if index is None:
            return self._rejected(DecisionCode.UNKNOWN_JOINT)

        if not isinstance(code, CommunicationFailure):
            return self._rejected(DecisionCode.INVALID_ARGUMENT)
        failure_code = code

        spec = self._specs[index]
        self._latch_joint_fault(
            spec.name,
            motor_id,
            communication_error=failure_code,
        )
        joint = self._observations[index]
        self._observations[index] = JointStatus(
            joint.sample, joint.health, joint.received_at, failure_code)
        return self._accepted()

    def checked(self) -> Decision:
        if self._state is not State.CHECKING:
            return self._rejected(DecisionCode.INVALID_STATE)
        if any(not joint.valid for joint in self._observations):
            return self._rejected(DecisionCode.OBSERVATIONS_INCOMPLETE)
        now = self._now()
        if not self._all_fresh(now):
            return self._rejected(DecisionCode.STALE_FEEDBACK)
        if any(not joint.healthy for joint in self._observations):
            return self._rejected(DecisionCode.UNHEALTHY)
        if any(joint.health.mode != 0 for joint in self._observations):
            return self._rejected(DecisionCode.NOT_STOPPED)
        self._state = State.READY
        self._faults.clear()
        return self._accepted()

    def arm(self) -> Decision:
        if self._state is not State.READY:
            return self._rejected(DecisionCode.INVALID_STATE)
        if bool(self._faults):
            return self._rejected(DecisionCode.FAULT_LATCHED)
        if any(not joint.valid for joint in self._observations):
            return self._rejected(DecisionCode.OBSERVATIONS_INCOMPLETE)
        now = self._now()
        if not self._all_fresh(now):
            return self._rejected(DecisionCode.STALE_FEEDBACK)
        if any(not joint.healthy for joint in self._observations):
            return self._rejected(DecisionCode.UNHEALTHY)
        self._state = State.ARMED
        return self._accepted()

    def tick(self) -> Decision:
        if self._state in (State.DISCONNECTED, State.CONNECTING):
            return self._rejected(DecisionCode.INVALID_STATE)
        if self._state in (State.READY, State.ARMED, State.MOVING):
            now = self._now()
            stale_specs = []
            for index, spec in enumerate(self._specs):
                joint = self._observations[index]
                if (
                    not joint.valid
                    or joint.received_at is None
                    or self._age(joint.received_at, now) > self._feedback_max_age_s
                ):
                    stale_specs.append(spec)
            if stale_specs:
                for spec in stale_specs:
                    self._latch_joint_fault(
                        spec.name,
                        spec.motor_id,
                        stale=True,
                    )
        return self._accepted()

    def submit_target(self, target: Target) -> Decision:
        if self._state not in (State.ARMED, State.MOVING):
            return self._rejected(DecisionCode.INVALID_STATE)
        if any(not joint.valid for joint in self._observations):
            return self._rejected(DecisionCode.OBSERVATIONS_INCOMPLETE)
        now = self._now()
        if not self._all_fresh(now):
            return self._rejected(DecisionCode.STALE_FEEDBACK)
        if any(not joint.healthy for joint in self._observations):
            return self._rejected(DecisionCode.UNHEALTHY)

        if (
            not isinstance(target, Target)
            or len(target.joints) != len(self._specs)
        ):
            return self._rejected(DecisionCode.INVALID_ARGUMENT)

        try:
            for spec, joint_target in zip(self._specs, target.joints):
                output = spec.mapping.to_output(joint_target)
                command(
                    spec.motor_id,
                    mode=1,
                    rpm=output.rpm,
                    kd=output.kd,
                    torque=output.tau_nm,
                    watchdog=True,
                    position=output.q_rad,
                    kp=output.kp,
                )
        except ValueError:
            return self._rejected(DecisionCode.INVALID_ARGUMENT)

        if target.sequence <= self._last_sequence:
            return self._rejected(DecisionCode.DUPLICATE_OR_OLDER)

        self._pending_target = target
        self._last_sequence = target.sequence
        return self._accepted()

    def consume_target(self) -> Target | None:
        if self._state not in (State.ARMED, State.MOVING):
            return None
        self.tick()
        if self._state is State.FAULT or any(not joint.healthy for joint in self._observations):
            return None
        target = self._pending_target
        self._pending_target = None
        if target is not None:
            self._state = State.MOVING
        return target

    def stop(self) -> Decision:
        if self._state in (State.DISCONNECTED, State.CONNECTING):
            return self._rejected(DecisionCode.INVALID_STATE)
        self._pending_target = None
        if self._state is State.FAULT or bool(self._faults):
            self._state = State.FAULT
        else:
            self._state = State.STOPPED
        return self._accepted()

    def disconnect(self) -> Decision:
        self._state = State.DISCONNECTED
        self._source = None
        self._clear_observations()
        self._pending_target = None
        return self._accepted()

    def snapshot(self) -> Snapshot:
        received = [joint.received_at for joint in self._observations]
        now = self._now() if any(value is not None for value in received) else None
        ages = tuple(
            None if value is None else self._age(value, now)
            for value in received
        )
        faults = tuple(self._faults[spec.name] for spec in self._specs
                       if spec.name in self._faults)
        return Snapshot(
            state=self._state,
            joint_names=tuple(spec.name for spec in self._specs),
            joints=tuple(self._observations),
            source=self._source,
            fault_latched=bool(self._faults),
            feedback_ages_s=ages,
            faults=faults,
            pending_target=self._pending_target,
            last_sequence=self._last_sequence,
        )
