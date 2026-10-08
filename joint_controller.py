"""Transport-free lifecycle and health state for an ordered joint set.

All methods on a JointCore belong to one calling thread. The core owns no
transport and all externally visible records are immutable snapshots.
"""

from collections.abc import Iterable
from dataclasses import dataclass
import math
import secrets
import time

from im6014_codec import command
from joint_contract import (
    CommunicationFailure,
    Decision,
    DecisionCode,
    IntentWindow,
    JointDispatchAttempt,
    JointDispatchedRecord,
    JointFault,
    JointSpec,
    JointStatus,
    MotorHealth,
    OwnerToken,
    RefreshPlan,
    Snapshot,
    SourceIdentity,
    State,
    Target,
    WriteResult,
)
from joint_mapping import JointSample

__all__ = ("JointCore",)


@dataclass(frozen=True)
class _DispatchClaim:
    plan: RefreshPlan
    index: int
    started_at_s: float


class JointCore:
    """Thread-confined state machine for one fixed, ordered set of joints.

    The core public API assumes trusted station callers; C16 server binds its
    actual connection identity rather than trusting client-supplied JSON fields.
    No authentication claims are made beyond that boundary.
    """

    def __init__(
        self,
        joints: Iterable[JointSpec],
        *,
        feedback_max_age_s: float = 0.5,
        max_window_s: float = 1.0,
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

        if isinstance(max_window_s, bool) or not isinstance(max_window_s, (int, float)):
            raise ValueError("max_window_s must be a finite positive real number")
        try:
            max_window_s = float(max_window_s)
        except (OverflowError, TypeError, ValueError) as exc:
            raise ValueError("max_window_s must be finite") from exc
        if not math.isfinite(max_window_s) or max_window_s <= 0.0:
            raise ValueError("max_window_s must be finite and positive")

        if clock is None:
            clock = time.monotonic
        if not callable(clock):
            raise ValueError("clock must be callable")

        self._epoch = secrets.token_hex(16)

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
        self._max_window_s = max_window_s
        self._clock = clock
        self._now_high_water_s: float | None = None
        self._next_window_id: int = 0
        self._offered_window: IntentWindow | None = None
        self._next_generation: int = 1
        self._latest_owner: OwnerToken | None = None
        self._current_owner: OwnerToken | None = None
        self._admitted_target: Target | None = None
        self._prepared_plan: RefreshPlan | None = None
        self._next_plan_id: int = 0
        self._next_dispatch_index: int = 0
        self._dispatch_claim: _DispatchClaim | None = None
        self._dispatched_records: list[JointDispatchedRecord | None] = [None for _ in specs]
        self._dispatch_attempts: list[JointDispatchAttempt | None] = [None for _ in specs]
        self._faults: dict[str, JointFault] = {}
        self._last_sequence: int = -1

    @property
    def joint_specs(self) -> tuple[JointSpec, ...]:
        return self._specs

    @property
    def admitted_target(self) -> Target | None:
        return self._admitted_target

    @staticmethod
    def _accepted() -> Decision:
        return Decision(True, DecisionCode.ACCEPTED)

    @staticmethod
    def _rejected(code: DecisionCode) -> Decision:
        return Decision(False, code)

    def _clear_observations(self) -> None:
        self._observations = [JointStatus() for _ in self._specs]

    def _invalidate_plan(self) -> None:
        self._prepared_plan = None
        self._next_dispatch_index = 0

    def _revoke_authority(self) -> None:
        self._current_owner = None
        self._offered_window = None
        self._admitted_target = None
        self._invalidate_plan()

    def claim_owner(self, connection_id: str) -> OwnerToken:
        """Issue and claim owner authority for a trusted station caller.

        The core public API assumes trusted station callers; C16 server binds its
        actual connection identity rather than trusting client-supplied JSON fields.
        No authentication claims are made beyond that boundary.
        """
        if (
            type(connection_id) is not str
            or not connection_id.strip()
            or len(connection_id) > 64
        ):
            raise ValueError("connection_id must be a bounded non-empty string")
        was_owned = self._current_owner is not None
        stop_for_claim = self._state in (
            State.CHECKING,
            State.READY,
            State.ARMED,
            State.MOVING,
        ) or (was_owned and self._state not in (State.DISCONNECTED, State.CONNECTING))
        self._revoke_authority()
        if self._state in (State.DISCONNECTED, State.CONNECTING):
            pass
        elif self._state is State.FAULT or self._faults:
            self._state = State.FAULT
        elif stop_for_claim:
            self._state = State.STOPPED

        if self._next_generation > (1 << 64) - 1:
            raise OverflowError("owner generation sequence exhausted")

        generation = self._next_generation
        self._next_generation += 1
        token = OwnerToken(
            epoch=self._epoch,
            generation=generation,
            connection_id=connection_id,
        )
        self._current_owner = token
        self._latest_owner = token
        return token

    def _record_dispatch_failure(
        self, index: int, cause: CommunicationFailure
    ) -> None:
        spec = self._specs[index]
        self._latch_joint_fault(
            spec.name,
            spec.motor_id,
            communication_error=cause,
        )
        joint = self._observations[index]
        self._observations[index] = JointStatus(
            joint.sample,
            joint.health,
            joint.received_at,
            cause,
        )

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
        self._revoke_authority()

    def _withdraw_expired_goal(self) -> None:
        self._revoke_authority()
        if self._faults:
            self._state = State.FAULT
        elif self._state in (State.ARMED, State.MOVING):
            self._state = State.STOPPED

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
        if self._now_high_water_s is not None and result < self._now_high_water_s:
            raise ValueError(
                f"clock regressed from {self._now_high_water_s} to {result}"
            )
        self._now_high_water_s = result
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

    def _housekeeping(self, now: float) -> DecisionCode | None:
        if self._state in (State.READY, State.ARMED, State.MOVING):
            stale_specs = [
                spec
                for spec, joint in zip(self._specs, self._observations)
                if (
                    not joint.valid
                    or joint.received_at is None
                    or self._age(joint.received_at, now) > self._feedback_max_age_s
                )
            ]
            if stale_specs:
                for spec in stale_specs:
                    self._latch_joint_fault(spec.name, spec.motor_id, stale=True)
                return DecisionCode.STALE_FEEDBACK
        if (
            self._admitted_target is not None
            and now >= self._admitted_target.window.expires_at_s
        ):
            self._withdraw_expired_goal()
            return DecisionCode.EXPIRED
        return None

    def issue_window(self, valid_for_s: float) -> IntentWindow:
        if isinstance(valid_for_s, bool) or not isinstance(valid_for_s, (int, float)):
            raise ValueError("valid_for_s must be a finite positive real number")
        try:
            valid_for_s = float(valid_for_s)
        except (OverflowError, TypeError, ValueError) as exc:
            raise ValueError("valid_for_s must be finite") from exc
        if not math.isfinite(valid_for_s) or valid_for_s <= 0.0:
            raise ValueError("valid_for_s must be finite and positive")
        if valid_for_s > self._max_window_s:
            raise ValueError(
                f"valid_for_s ({valid_for_s}) exceeds configured maximum window ({self._max_window_s})"
            )
        now = self._now()
        window_id = self._next_window_id
        if window_id > (1 << 64) - 1:
            raise OverflowError("window ID sequence exhausted")
        self._next_window_id += 1
        window = IntentWindow(
            id=window_id,
            created_at_s=now,
            expires_at_s=now + valid_for_s,
        )
        self._offered_window = window
        return window

    def begin_connect(self) -> Decision:
        if self._state is not State.DISCONNECTED or self._dispatch_claim is not None:
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
        if self._state not in (State.CONNECTED, State.STOPPED, State.FAULT) or self._dispatch_claim is not None:
            return self._rejected(DecisionCode.INVALID_STATE)
        if bool(self._faults) and not acknowledge_fault:
            return self._rejected(DecisionCode.FAULT_ACK_REQUIRED)
        self._clear_observations()
        self._admitted_target = None
        self._invalidate_plan()
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
            joint.sample, joint.health, joint.received_at, failure_code
        )
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

    def arm(self, owner: OwnerToken) -> Decision:
        if self._state is not State.READY:
            return self._rejected(DecisionCode.INVALID_STATE)
        if (
            not isinstance(owner, OwnerToken)
            or self._current_owner is None
            or owner != self._current_owner
        ):
            return self._rejected(DecisionCode.INVALID_OWNER)
        if bool(self._faults):
            return self._rejected(DecisionCode.FAULT_LATCHED)
        if any(not joint.valid for joint in self._observations):
            return self._rejected(DecisionCode.OBSERVATIONS_INCOMPLETE)
        now = self._now()
        if not self._all_fresh(now):
            return self._rejected(DecisionCode.STALE_FEEDBACK)
        if any(not joint.healthy for joint in self._observations):
            return self._rejected(DecisionCode.UNHEALTHY)
        if any(joint.health.mode != 0 for joint in self._observations):
            return self._rejected(DecisionCode.NOT_STOPPED)
        self._state = State.ARMED
        return self._accepted()

    def tick(self) -> Decision:
        if self._state in (State.DISCONNECTED, State.CONNECTING):
            return self._rejected(DecisionCode.INVALID_STATE)
        now = self._now()
        self._housekeeping(now)
        return self._accepted()

    def submit_target(self, target: Target) -> Decision:
        if self._state not in (State.ARMED, State.MOVING):
            return self._rejected(DecisionCode.INVALID_STATE)
        now = self._now()
        housekeeping = self._housekeeping(now)
        if housekeeping is not None:
            return self._rejected(housekeeping)
        if self._state not in (State.ARMED, State.MOVING):
            return self._rejected(DecisionCode.INVALID_STATE)
        if any(not joint.healthy for joint in self._observations):
            return self._rejected(DecisionCode.UNHEALTHY)

        if not isinstance(target, Target) or len(target.joints) != len(self._specs):
            return self._rejected(DecisionCode.INVALID_ARGUMENT)

        if (
            not isinstance(target.owner, OwnerToken)
            or self._current_owner is None
            or target.owner != self._current_owner
        ):
            return self._rejected(DecisionCode.INVALID_OWNER)

        if not isinstance(target.window, IntentWindow):
            return self._rejected(DecisionCode.INVALID_ARGUMENT)
        if self._offered_window is None:
            return self._rejected(DecisionCode.INVALID_ARGUMENT)
        if target.window.id < self._offered_window.id:
            return self._rejected(DecisionCode.EXPIRED)
        if target.window.id != self._offered_window.id:
            return self._rejected(DecisionCode.INVALID_ARGUMENT)
        if (
            target.window.created_at_s != self._offered_window.created_at_s
            or target.window.expires_at_s != self._offered_window.expires_at_s
        ):
            return self._rejected(DecisionCode.INVALID_ARGUMENT)
        if now >= target.window.expires_at_s:
            return self._rejected(DecisionCode.EXPIRED)

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

        self._admitted_target = target
        self._last_sequence = target.sequence
        self._invalidate_plan()
        return self._accepted()

    def prepare_refresh(self) -> RefreshPlan | None:
        if self._dispatch_claim is not None:
            return None
        if self._state not in (State.ARMED, State.MOVING):
            return None
        if self._state is State.FAULT or bool(self._faults):
            return None
        now = self._now()
        if self._housekeeping(now) is not None:
            return None
        if any(not joint.healthy for joint in self._observations):
            return None
        if self._admitted_target is None:
            return None
        if self._current_owner is None or self._admitted_target.owner != self._current_owner:
            return None

        try:
            for spec, joint_target in zip(self._specs, self._admitted_target.joints):
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
            return None

        plan_id = self._next_plan_id
        if plan_id > (1 << 64) - 1:
            raise OverflowError("plan ID sequence exhausted")
        self._next_plan_id += 1

        plan = RefreshPlan(
            plan_id=plan_id,
            target=self._admitted_target,
            prepared_at_s=now,
            joint_targets=self._admitted_target.joints,
        )
        self._prepared_plan = plan
        self._next_dispatch_index = 0
        return plan

    def claim_dispatch(self, plan_id: int, index: int) -> bool:
        if (
            type(plan_id) is not int
            or type(index) is not int
        ):
            return False
        if self._dispatch_claim is not None:
            return False
        if self._prepared_plan is None or self._prepared_plan.plan_id != plan_id:
            return False
        if self._state not in (State.ARMED, State.MOVING):
            return False
        if self._state is State.FAULT or bool(self._faults):
            return False
        if index < 0 or index >= len(self._specs):
            return False
        if index != self._next_dispatch_index:
            return False

        now = self._now()
        if self._housekeeping(now) is not None:
            return False
        if self._admitted_target is None or self._prepared_plan is None:
            return False
        if self._current_owner is None or self._prepared_plan.target.owner != self._current_owner:
            return False
        if any(not joint.healthy for joint in self._observations):
            return False

        self._dispatch_claim = _DispatchClaim(self._prepared_plan, index, now)
        return True

    def dispatch_result(
        self, plan_id: int, index: int, result: WriteResult
    ) -> Decision:
        if (
            type(plan_id) is not int
            or type(index) is not int
        ):
            return self._rejected(DecisionCode.INVALID_ARGUMENT)
        if not isinstance(result, WriteResult):
            return self._rejected(DecisionCode.INVALID_ARGUMENT)
        claim = self._dispatch_claim
        if claim is None or claim.plan.plan_id != plan_id or claim.index != index:
            return self._rejected(DecisionCode.INVALID_ARGUMENT)
        if index < 0 or index >= len(self._specs):
            return self._rejected(DecisionCode.INVALID_ARGUMENT)

        self._dispatch_claim = None
        plan = claim.plan
        joint_target = plan.joint_targets[index]
        self._dispatch_attempts[index] = JointDispatchAttempt(
            plan_id=plan.plan_id,
            sequence=plan.target.sequence,
            target=joint_target,
            result=result,
            started_at_s=claim.started_at_s,
            owner=plan.target.owner,
        )
        if result is WriteResult.COMPLETE:
            record = JointDispatchedRecord(
                sequence=plan.target.sequence,
                target=joint_target,
                dispatched_at_s=claim.started_at_s,
                owner=plan.target.owner,
            )
            self._dispatched_records[index] = record
            try:
                now = self._now()
            except Exception:
                self._record_dispatch_failure(index, CommunicationFailure.CLOCK_FAILURE)
                return self._rejected(DecisionCode.FAULT_LATCHED)

            housekeeping = self._housekeeping(now)
            still_current = (
                self._prepared_plan is plan
                and self._admitted_target is plan.target
                and self._state in (State.ARMED, State.MOVING)
                and not self._faults
                and self._current_owner is not None
                and self._current_owner == plan.target.owner
            )
            if still_current and housekeeping is None:
                self._next_dispatch_index = index + 1
                self._state = State.MOVING
            return self._accepted()

        cause = (
            CommunicationFailure.PREWRITE_DEADLINE
            if result is WriteResult.PREWRITE_DEADLINE
            else CommunicationFailure.DRIVER_FAILURE
        )
        self._record_dispatch_failure(index, cause)
        return self._rejected(DecisionCode.FAULT_LATCHED)

    def stop(self) -> Decision:
        if self._state in (State.DISCONNECTED, State.CONNECTING):
            return self._rejected(DecisionCode.INVALID_STATE)
        self._revoke_authority()
        if self._state is State.FAULT or bool(self._faults):
            self._state = State.FAULT
        else:
            self._state = State.STOPPED
        return self._accepted()

    def disconnect(self) -> Decision:
        self._state = State.DISCONNECTED
        self._source = None
        self._clear_observations()
        self._revoke_authority()
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
            admitted_target=self._admitted_target,
            prepared_plan=self._prepared_plan,
            dispatched_records=tuple(self._dispatched_records),
            dispatch_attempts=tuple(self._dispatch_attempts),
            last_sequence=self._last_sequence,
            latest_owner=self._latest_owner,
            current_owner=self._current_owner,
        )
