"""Transport-free lifecycle and health state for an ordered joint set.

All methods on a JointCore belong to one calling thread. The core owns no
transport and all externally visible records are immutable snapshots.
"""

from collections.abc import Iterable
from dataclasses import dataclass
import math
import time

from im6014_codec import (
    command,
    decode_command,
    project_position_raw,
    project_speed_raw,
    raw_to_position,
    raw_to_rpm,
)
from joint_mapping import JointSample, JointTarget, OutputSample
from joint_records import (
    ArmDecision,
    CommunicationFailure,
    Decision,
    DecisionCode,
    JointFault,
    JointSpec,
    JointStatus,
    MotorHealth,
    Ownership,
    Reference,
    Snapshot,
    SourceIdentity,
    State,
    Target,
)


@dataclass(frozen=True)
class _PendingTarget:
    target: Target
    accepted_at_s: float


class JointCore:
    """Thread-confined state machine for one fixed, ordered set of joints."""

    def __init__(
        self,
        joints: Iterable[JointSpec],
        *,
        feedback_max_age_s: float = 0.5,
        clock=None,
        max_reference_step_s: float,
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
        if isinstance(max_reference_step_s, bool) or not isinstance(max_reference_step_s, (int, float)):
            raise ValueError("max_reference_step_s must be finite and positive")
        try:
            max_reference_step_s = float(max_reference_step_s)
        except (OverflowError, TypeError, ValueError) as exc:
            raise ValueError("max_reference_step_s must be finite and positive") from exc
        if not math.isfinite(max_reference_step_s) or max_reference_step_s <= 0.0:
            raise ValueError("max_reference_step_s must be finite and positive")

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
        self._max_reference_step_s = max_reference_step_s
        self._clock = clock
        self._last_now: float | None = None
        self._faults: dict[str, JointFault] = {}
        self._pending_target: _PendingTarget | None = None
        self._reference_positions: tuple[float, ...] | None = None
        self._last_reference_at_s: float | None = None
        self._last_sequence: int = -1
        self._ownership: Ownership | None = None
        self._ownership_generation = 0

    @property
    def joint_specs(self) -> tuple[JointSpec, ...]:
        return self._specs

    @staticmethod
    def _accepted() -> Decision:
        return Decision(True, DecisionCode.ACCEPTED)

    @staticmethod
    def _rejected(code: DecisionCode) -> Decision:
        return Decision(False, code)

    @staticmethod
    def _arm_rejected(code: DecisionCode) -> ArmDecision:
        return ArmDecision(False, code, None)

    def _revoke_ownership(self) -> None:
        if self._ownership is not None:
            self._ownership = None
            self._ownership_generation += 1

    def _clear_observations(self) -> None:
        self._observations = [JointStatus() for _ in self._specs]

    def _clear_reference(self) -> None:
        self._reference_positions = None
        self._last_reference_at_s = None

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
        self._clear_reference()
        self._revoke_ownership()

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
        if self._last_now is not None and result < self._last_now:
            raise ValueError("clock must be monotonic")
        self._last_now = result
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
        self._clear_reference()
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

    def arm(self, *, owner: str) -> ArmDecision:
        if self._state is not State.READY:
            return self._arm_rejected(DecisionCode.INVALID_STATE)
        if type(owner) is not str or not owner.strip():
            return self._arm_rejected(DecisionCode.INVALID_ARGUMENT)
        if bool(self._faults):
            return self._arm_rejected(DecisionCode.FAULT_LATCHED)
        if any(not joint.valid for joint in self._observations):
            return self._arm_rejected(DecisionCode.OBSERVATIONS_INCOMPLETE)
        now = self._now()
        if not self._all_fresh(now):
            return self._arm_rejected(DecisionCode.STALE_FEEDBACK)
        if any(not joint.healthy for joint in self._observations):
            return self._arm_rejected(DecisionCode.UNHEALTHY)
        if any(joint.health.mode != 0 for joint in self._observations):
            return self._arm_rejected(DecisionCode.NOT_STOPPED)
        if any(
            not spec.position_limits.minimum_rad <= joint.sample.q_rad <= spec.position_limits.maximum_rad
            for spec, joint in zip(self._specs, self._observations)
        ):
            return self._arm_rejected(DecisionCode.POSITION_LIMIT)
        try:
            for spec, joint in zip(self._specs, self._observations):
                seed = JointTarget(joint.sample.q_rad, 0.0, 0.0, 0.0, 0.0)
                output = spec.mapping.to_output(seed)
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
                self._project_joint_target(
                    spec,
                    seed,
                    spec.position_limits.minimum_rad,
                    spec.position_limits.maximum_rad,
                )
        except (ArithmeticError, ValueError):
            return self._arm_rejected(DecisionCode.INVALID_ARGUMENT)
        self._state = State.ARMED
        self._ownership_generation += 1
        self._ownership = Ownership(owner, self._ownership_generation)
        self._reference_positions = tuple(joint.sample.q_rad for joint in self._observations)
        self._last_reference_at_s = now
        return ArmDecision(True, DecisionCode.ACCEPTED, self._ownership)

    def tick(self) -> Decision:
        if self._state in (State.DISCONNECTED, State.CONNECTING):
            return self._rejected(DecisionCode.INVALID_STATE)
        if self._state not in (State.READY, State.ARMED, State.MOVING):
            return self._accepted()
        return self._tick_at(self._now())

    def _tick_at(self, now: float) -> Decision:
        if self._state in (State.READY, State.ARMED, State.MOVING):
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
        if target.created_at_s > now:
            return self._rejected(DecisionCode.INVALID_ARGUMENT)
        if now >= target.expires_at_s:
            return self._rejected(DecisionCode.EXPIRED)
        if self._ownership is None or target.ownership is not self._ownership:
            return self._rejected(DecisionCode.INVALID_OWNERSHIP)
        if any(
            not spec.position_limits.minimum_rad <= joint_target.q_rad <= spec.position_limits.maximum_rad
            for spec, joint_target in zip(self._specs, target.joints)
        ):
            return self._rejected(DecisionCode.POSITION_LIMIT)

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

        self._pending_target = _PendingTarget(target, now)
        self._last_sequence = target.sequence
        return self._accepted()

    def _project_joint_target(
        self,
        spec: JointSpec,
        target: JointTarget,
        q_min: float,
        q_max: float,
    ) -> JointTarget:
        mapping = spec.mapping
        direction = mapping.direction
        velocity = spec.max_velocity_rad_s
        wanted_speed = max(-velocity, min(velocity, target.dq_rad_s))

        def position_to_joint(output_rad: float) -> float:
            return mapping.from_output(OutputSample(output_rad, 0.0, 0.0)).q_rad

        def speed_to_joint(output_rpm: float) -> float:
            return mapping.from_output(OutputSample(0.0, output_rpm, 0.0)).dq_rad_s

        raw_position = project_position_raw(
            target.q_rad, q_min, q_max, position_to_joint, direction
        )
        raw_speed = project_speed_raw(
            wanted_speed, -velocity, velocity, speed_to_joint, direction
        )
        if raw_position is None or raw_speed is None:
            raise ValueError("no representable reference fits the configured bounds")

        safe_target = JointTarget(
            position_to_joint(raw_to_position(raw_position)),
            speed_to_joint(raw_to_rpm(raw_speed)),
            target.tau_nm,
            target.kp,
            target.kd,
        )
        output = mapping.to_output(safe_target)
        frame = command(
            spec.motor_id,
            mode=1,
            rpm=output.rpm,
            kd=output.kd,
            torque=output.tau_nm,
            watchdog=True,
            position=output.q_rad,
            kp=output.kp,
        )
        decoded = decode_command(frame)
        if decoded["raw_position"] != raw_position or decoded["raw_rpm"] != raw_speed:
            raise ValueError("normal command encoding changed a projected grid code")
        if not q_min <= safe_target.q_rad <= q_max:
            raise ValueError("projected position violates its closed bounds")
        if not -velocity <= safe_target.dq_rad_s <= velocity:
            raise ValueError("projected speed violates its closed bounds")
        return safe_target

    def consume_target(self) -> Reference | None:
        if self._state not in (State.ARMED, State.MOVING):
            return None
        now = self._now()
        self._tick_at(now)
        if self._state is State.FAULT or any(not joint.healthy for joint in self._observations):
            return None
        pending = self._pending_target
        if pending is None:
            return None
        target = pending.target
        if (
            self._ownership is None
            or target.ownership is not self._ownership
            or target.ownership.generation != self._ownership_generation
        ):
            self._pending_target = None
            return None
        if now >= target.expires_at_s:
            self._pending_target = None
            return None
        if self._reference_positions is None or self._last_reference_at_s is None:
            return None

        elapsed = now - self._last_reference_at_s
        step_time = min(elapsed, self._max_reference_step_s)
        try:
            candidate_joints = []
            for spec, requested, q_reference in zip(
                self._specs, target.joints, self._reference_positions
            ):
                step = spec.max_velocity_rad_s * step_time
                lower = max(spec.position_limits.minimum_rad, q_reference - step)
                upper = min(spec.position_limits.maximum_rad, q_reference + step)
                candidate_joints.append(
                    self._project_joint_target(spec, requested, lower, upper)
                )
        except (ArithmeticError, ValueError):
            return None

        release_now = self._now()
        self._tick_at(release_now)
        if (
            self._state not in (State.ARMED, State.MOVING)
            or any(not joint.healthy for joint in self._observations)
        ):
            return None
        if self._pending_target is not pending:
            return None
        if (
            self._ownership is None
            or target.ownership is not self._ownership
            or target.ownership.generation != self._ownership_generation
        ):
            self._pending_target = None
            return None
        if release_now >= target.expires_at_s:
            self._pending_target = None
            return None

        reference = Reference(
            target,
            tuple(candidate_joints),
            pending.accepted_at_s,
            release_now,
        )
        self._reference_positions = tuple(joint.q_rad for joint in reference.joints)
        self._last_reference_at_s = release_now
        self._pending_target = None
        self._state = State.MOVING
        return reference

    def stop(self) -> Decision:
        if self._state in (State.DISCONNECTED, State.CONNECTING):
            return self._rejected(DecisionCode.INVALID_STATE)
        self._pending_target = None
        self._clear_reference()
        self._revoke_ownership()
        if self._state is State.FAULT or bool(self._faults):
            self._state = State.FAULT
        else:
            self._state = State.STOPPED
        return self._accepted()

    def disconnect(self) -> Decision:
        self._revoke_ownership()
        self._state = State.DISCONNECTED
        self._source = None
        self._clear_observations()
        self._pending_target = None
        self._clear_reference()
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
            ownership=self._ownership,
            ownership_generation=self._ownership_generation,
            faults=faults,
            pending_target=None if self._pending_target is None else self._pending_target.target,
            last_sequence=self._last_sequence,
        )
