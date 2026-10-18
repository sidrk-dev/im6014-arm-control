"""Transport-free lifecycle and health state for an ordered joint set.

All methods on a JointCore belong to one calling thread. The core owns no
transport and all externally visible records are immutable snapshots.
"""

from collections.abc import Iterable
from dataclasses import dataclass
from fractions import Fraction
import math
import time
from typing import Literal

from im6014_codec import command
from joint_mapping import JointSample, JointTarget
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
from joint_reference import (
    compute_predicted_effort,
    plan_reference,
    position_rates_at_release,
    project_joint_target,
)


@dataclass(frozen=True)
class _PendingTarget:
    target: Target
    accepted_at_s: float


@dataclass(frozen=True)
class _ReferenceHistory:
    joints: tuple[JointTarget, ...]
    issued_at_s: float
    kind: Literal["seed", "issued"]
    position_rates_rad_s: tuple[float, ...]


@dataclass(frozen=True)
class _ProtectionHistory:
    received_at_s: float | None = None
    overloaded: bool = False
    recovering: bool = False
    overload_s: Fraction = Fraction(0)
    recovery_s: Fraction = Fraction(0)


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
        self._protection = [_ProtectionHistory() for _ in specs]
        self._pending_target: _PendingTarget | None = None
        self._history: _ReferenceHistory | None = None
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

    def _break_protection_continuity(self, index: int | None = None) -> None:
        indices = range(len(self._specs)) if index is None else (index,)
        for i in indices:
            prior = self._protection[i]
            self._protection[i] = _ProtectionHistory(overload_s=prior.overload_s)

    def _observe_protection(
        self, index: int, sample: JointSample, health: MotorHealth, now: float
    ) -> tuple[bool, bool, bool]:
        policy = self._specs[index].protection
        prior = self._protection[index]
        unknown = health.driver_c is None or health.winding_c is None
        hot = ((health.driver_c is not None and health.driver_c >= policy.driver_limit_c)
               or (health.winding_c is not None and health.winding_c >= policy.winding_limit_c))
        credible = health.healthy and not hot
        overloaded = credible and abs(sample.tau_nm) > policy.sustained_effort_nm
        recovering = (credible and not overloaded and (
            not self._faults or (self._state is State.CHECKING and health.mode == 0)
        ))
        dt = Fraction(0)
        contiguous = False
        if prior.received_at_s is not None:
            gap = Fraction(now) - Fraction(prior.received_at_s)
            if gap <= Fraction(min(self._feedback_max_age_s, policy.max_sample_gap_s)):
                dt = gap
                contiguous = True
        overload_s = prior.overload_s
        if prior.overloaded and overloaded:
            overload_s = min(Fraction(policy.overload_duration_s), overload_s + dt)
        recovery_s = Fraction(0)
        if contiguous and prior.recovering and recovering:
            recovery_s = min(Fraction(policy.recovery_duration_s), prior.recovery_s + dt)
        if recovering and recovery_s >= Fraction(policy.recovery_duration_s):
            overload_s = Fraction(0)
        self._protection[index] = _ProtectionHistory(
            now, overloaded, recovering, overload_s, recovery_s,
        )
        return hot, overloaded and overload_s >= Fraction(policy.overload_duration_s), unknown

    def _protection_healthy(self, index: int) -> bool:
        joint = self._observations[index]
        policy = self._specs[index].protection
        return (joint.healthy
                and joint.health.driver_c < policy.driver_limit_c
                and joint.health.winding_c < policy.winding_limit_c
                and abs(joint.sample.tau_nm) <= policy.sustained_effort_nm)

    def _clear_reference(self) -> None:
        self._history = None

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
        over_temperature: bool = False,
        persistent_effort: bool = False,
        protection_unavailable: bool = False,
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
            over_temperature=existing.over_temperature or over_temperature,
            persistent_effort=existing.persistent_effort or persistent_effort,
            protection_unavailable=existing.protection_unavailable or protection_unavailable,
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
        self._break_protection_continuity()
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
        self._protection = [
            _ProtectionHistory(h.received_at_s, h.overloaded, overload_s=h.overload_s)
            for h in self._protection
        ]
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

        now = self._now()
        self._observations[index] = JointStatus(sample, status, now)
        hot, overloaded, unknown = self._observe_protection(index, sample, status, now)
        if not status.healthy or hot or overloaded:
            self._latch_joint_fault(
                self._specs[index].name,
                motor_id,
                errors=status.errors,
                warnings=status.warnings,
                timeout=status.timeout,
                over_temperature=hot,
                persistent_effort=overloaded,
                protection_unavailable=unknown,
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
        self._break_protection_continuity(index)
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
        if any(not self._protection_healthy(i) for i in range(len(self._specs))):
            return self._rejected(DecisionCode.UNHEALTHY)
        for i, spec in enumerate(self._specs):
            fault = self._faults.get(spec.name)
            if (fault is not None
                    and (fault.over_temperature or fault.persistent_effort or fault.protection_unavailable)
                    and self._protection[i].recovery_s < Fraction(spec.protection.recovery_duration_s)):
                return self._rejected(DecisionCode.RECOVERY_PENDING)
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
        if any(not self._protection_healthy(i) for i in range(len(self._specs))):
            return self._arm_rejected(DecisionCode.UNHEALTHY)
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
                project_joint_target(
                    spec,
                    seed,
                    spec.position_limits.minimum_rad,
                    spec.position_limits.maximum_rad,
                    -spec.max_velocity_rad_s,
                    spec.max_velocity_rad_s,
                )
        except (ArithmeticError, ValueError):
            return self._arm_rejected(DecisionCode.INVALID_ARGUMENT)
        self._state = State.ARMED
        self._ownership_generation += 1
        self._ownership = Ownership(owner, self._ownership_generation)
        self._history = _ReferenceHistory(
            joints=tuple(JointTarget(joint.sample.q_rad, 0.0, 0.0, 0.0, 0.0) for joint in self._observations),
            issued_at_s=now,
            kind="seed",
            position_rates_rad_s=tuple(0.0 for _ in self._specs),
        )
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

        if any(
            abs(joint_target.tau_nm) > spec.max_effort_nm
            for spec, joint_target in zip(self._specs, target.joints)
        ):
            return self._rejected(DecisionCode.EFFORT_LIMIT)

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
        if self._history is None:
            return None

        elapsed = now - self._history.issued_at_s
        measured_positions = tuple(joint.sample.q_rad for joint in self._observations)
        measured_velocities = tuple(joint.sample.dq_rad_s for joint in self._observations)
        candidate_joints = plan_reference(
            specs=self._specs,
            requested=target.joints,
            previous=self._history.joints,
            previous_rates=self._history.position_rates_rad_s,
            measured_positions=measured_positions,
            elapsed_s=elapsed,
            max_step_s=self._max_reference_step_s,
            allow_bootstrap=(self._history.kind == "seed"),
            measured_velocities=measured_velocities,
        )
        if candidate_joints is None:
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

        release_elapsed = release_now - self._history.issued_at_s
        rates = position_rates_at_release(
            specs=self._specs,
            previous=self._history.joints,
            previous_rates=self._history.position_rates_rad_s,
            candidate=candidate_joints,
            elapsed_s=release_elapsed,
        )
        if rates is None:
            return None

        for spec, cand, obs in zip(self._specs, candidate_joints, self._observations):
            if (
                obs.sample is None
                or not math.isfinite(obs.sample.q_rad)
                or not math.isfinite(obs.sample.dq_rad_s)
            ):
                return None
            if abs(cand.q_rad - obs.sample.q_rad) > spec.max_measured_lead_rad:
                return None
            tau_pred = compute_predicted_effort(spec, cand, obs.sample.q_rad, obs.sample.dq_rad_s)
            if tau_pred is None or abs(tau_pred) > spec.max_effort_nm:
                return None

        reference = Reference(
            target,
            candidate_joints,
            pending.accepted_at_s,
            release_now,
        )
        self._history = _ReferenceHistory(
            joints=reference.joints,
            issued_at_s=release_now,
            kind="issued",
            position_rates_rad_s=rates,
        )
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
        self._break_protection_continuity()
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
