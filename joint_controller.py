"""Transport-free lifecycle and health state for an ordered joint set.

All methods on a JointCore belong to one calling thread. The core owns no
transport and all externally visible records are immutable snapshots.
"""

from collections.abc import Iterable
from dataclasses import dataclass
from enum import Enum
from typing import Literal

from joint_mapping import JointMap, JointSample


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
    UNHEALTHY = "unhealthy"
    NOT_STOPPED = "not_stopped"


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
class MotorHealth:
    mode: int
    errors: int
    warnings: int
    timeout: bool

    def __post_init__(self) -> None:
        if type(self.mode) is not int or self.mode not in (0, 1):
            raise ValueError("mode must be the integer 0 or 1")
        if type(self.errors) is not int or self.errors < 0:
            raise ValueError("errors must be a nonnegative integer")
        if type(self.warnings) is not int or self.warnings < 0:
            raise ValueError("warnings must be a nonnegative integer")
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

    def __post_init__(self) -> None:
        if (self.sample is None) != (self.health is None):
            raise ValueError("sample and health must both be present or both be absent")
        if self.sample is not None and not isinstance(self.sample, JointSample):
            raise ValueError("sample must be a JointSample")
        if self.health is not None and not isinstance(self.health, MotorHealth):
            raise ValueError("health must be MotorHealth")

    @property
    def valid(self) -> bool:
        return self.sample is not None and self.health is not None

    @property
    def healthy(self) -> bool:
        return self.valid and self.health.healthy


@dataclass(frozen=True)
class Snapshot:
    state: State
    joint_names: tuple[str, ...]
    joints: tuple[JointStatus, ...]
    source: SourceIdentity | None
    fault_latched: bool

    def __post_init__(self) -> None:
        if not isinstance(self.state, State):
            raise ValueError("state must be a State")
        if type(self.joint_names) is not tuple or any(type(name) is not str for name in self.joint_names):
            raise ValueError("joint_names must be a tuple of strings")
        if type(self.joints) is not tuple or any(not isinstance(joint, JointStatus) for joint in self.joints):
            raise ValueError("joints must be a tuple of JointStatus")
        if len(self.joint_names) != len(self.joints):
            raise ValueError("joint names and statuses must have equal lengths")
        if self.source is not None and not isinstance(self.source, SourceIdentity):
            raise ValueError("source must be a SourceIdentity or None")
        if type(self.fault_latched) is not bool:
            raise ValueError("fault_latched must be a bool")


class JointCore:
    """Thread-confined state machine for one fixed, ordered set of joints."""

    def __init__(self, joints: Iterable[JointSpec]):
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
        self._fault_latched: bool = False

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
        if self._fault_latched and not acknowledge_fault:
            return self._rejected(DecisionCode.FAULT_ACK_REQUIRED)
        self._clear_observations()
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

        self._observations[index] = JointStatus(sample, status)
        if not status.healthy:
            self._fault_latched = True
            self._state = State.FAULT
        return self._accepted()

    def checked(self) -> Decision:
        if self._state is not State.CHECKING:
            return self._rejected(DecisionCode.INVALID_STATE)
        if any(not joint.valid for joint in self._observations):
            return self._rejected(DecisionCode.OBSERVATIONS_INCOMPLETE)
        if any(not joint.healthy for joint in self._observations):
            return self._rejected(DecisionCode.UNHEALTHY)
        if any(joint.health.mode != 0 for joint in self._observations):
            return self._rejected(DecisionCode.NOT_STOPPED)
        self._state = State.READY
        self._fault_latched = False
        return self._accepted()

    def arm(self) -> Decision:
        if self._state is not State.READY:
            return self._rejected(DecisionCode.INVALID_STATE)
        if self._fault_latched:
            return self._rejected(DecisionCode.FAULT_LATCHED)
        if any(not joint.valid for joint in self._observations):
            return self._rejected(DecisionCode.OBSERVATIONS_INCOMPLETE)
        if any(not joint.healthy for joint in self._observations):
            return self._rejected(DecisionCode.UNHEALTHY)
        self._state = State.ARMED
        return self._accepted()

    def stop(self) -> Decision:
        if self._state in (State.DISCONNECTED, State.CONNECTING):
            return self._rejected(DecisionCode.INVALID_STATE)
        if self._state is State.FAULT or self._fault_latched:
            self._state = State.FAULT
        else:
            self._state = State.STOPPED
        return self._accepted()

    def disconnect(self) -> Decision:
        self._state = State.DISCONNECTED
        self._source = None
        self._clear_observations()
        return self._accepted()

    def snapshot(self) -> Snapshot:
        return Snapshot(
            state=self._state,
            joint_names=tuple(spec.name for spec in self._specs),
            joints=tuple(self._observations),
            source=self._source,
            fault_latched=self._fault_latched,
        )
