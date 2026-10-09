"""Single-executor station boundary for a three-joint fixture arm.

Ingress records bounded requests and capabilities. One manual or background
executor owns Core, timing, packet exchange, Stop confirmation, and transport
close. Public snapshots are immutable values published by that executor.
"""

from collections import OrderedDict, deque
from collections.abc import Callable
from dataclasses import dataclass, replace
import math
import secrets
import threading
import time

from arm_fixture import (
    ArmPacketFixture,
    J1_MAX_RAD,
    J1_MIN_RAD,
    SYNTHETIC_KD,
    SYNTHETIC_KP,
    create_synthetic_arm_specs,
)
from im6014_codec import command
from im6014_port import PortConfig
from im6014_serial import InvalidFeedbackError, TransactionTimeoutError, exchange
from joint_contract import (
    JointFault,
    JointTrajectoryReference,
    MotorHealth,
    OwnerToken,
    Snapshot as CoreSnapshot,
    SourceIdentity,
    State,
    Target,
    WriteResult,
)
from joint_controller import JointCore
from joint_mapping import JointSample, JointTarget
from joint_protection import ProtectionPolicy
from station_step import StationStepCoordinator, mapped_health, mapped_sample
from station_timing import (
    LoopStepRecord,
    StationTimingConfig,
    StationTimingTracker,
    TimingCounters,
    TimingSnapshot,
    validate_finite_time,
)

__all__ = (
    "DEFAULT_EXCHANGE_DEADLINE_S",
    "DEFAULT_FEEDBACK_MAX_AGE_S",
    "DEFAULT_INPUT_LEASE_S",
    "DEFAULT_MAX_REFERENCE_GAP_S",
    "DEFAULT_MAX_WINDOW_S",
    "DEFAULT_SLOT_PERIOD_S",
    "DEFAULT_SPEED_LIMIT_RAD_S",
    "MAX_SESSIONS",
    "ArmAction",
    "ArmStation",
    "ArmStationSnapshot",
    "CheckAction",
    "ConnectAction",
    "DispatchAuthority",
    "DispatchRecordView",
    "DispatchAttemptView",
    "DisconnectAction",
    "InputTicket",
    "JogIntent",
    "PhysicalDraft",
    "PhysicalJointDraft",
    "ReceiptLookup",
    "RequestReceipt",
    "SessionHandle",
    "SetSpeedAction",
    "StopAttemptView",
    "StopAction",
    "compute_zero_output",
    "validate_physical_draft",
)

DEFAULT_SLOT_PERIOD_S = 0.020
DEFAULT_INPUT_LEASE_S = 0.150
DEFAULT_EXCHANGE_DEADLINE_S = 0.050
DEFAULT_MAX_WINDOW_S = 0.200
DEFAULT_MAX_REFERENCE_GAP_S = 0.500
DEFAULT_FEEDBACK_MAX_AGE_S = 0.500
DEFAULT_SPEED_LIMIT_RAD_S = 0.500
MAX_SESSIONS = 16
MAX_LIFECYCLE_REQUESTS = 16
MAX_RECEIPTS = 64
MAX_TERMINAL_LIFECYCLE_RECEIPTS = 64
BACKGROUND_CLOSE_WAIT_S = 1.0


@dataclass(frozen=True)
class SessionHandle:
    session_id: str
    created_at_s: float

    def __post_init__(self) -> None:
        if type(self.session_id) is not str or not self.session_id:
            raise ValueError("session_id must be a non-empty string")
        validate_finite_time(self.created_at_s, "created_at_s")


@dataclass(frozen=True)
class InputTicket:
    ticket_id: str
    session_id: str
    owner_generation: int
    created_at_s: float
    expires_at_s: float
    bound_sequence: int

    def __post_init__(self) -> None:
        if type(self.ticket_id) is not str or not self.ticket_id:
            raise ValueError("ticket_id must be a non-empty string")
        if type(self.session_id) is not str or not self.session_id:
            raise ValueError("session_id must be a non-empty string")
        if type(self.owner_generation) is not int or self.owner_generation < 1:
            raise ValueError("owner_generation must be a positive integer")
        validate_finite_time(self.created_at_s, "created_at_s")
        validate_finite_time(self.expires_at_s, "expires_at_s")
        if self.expires_at_s <= self.created_at_s:
            raise ValueError("expires_at_s must follow created_at_s")
        if type(self.bound_sequence) is not int or self.bound_sequence < 0:
            raise ValueError("bound_sequence must be a non-negative integer")


@dataclass(frozen=True)
class ConnectAction:
    source_kind: str = "fixture"
    draft: "PhysicalDraft | None" = None

    def __post_init__(self) -> None:
        if type(self.source_kind) is not str or self.source_kind not in ("fixture", "physical"):
            raise ValueError("source_kind must be 'fixture' or 'physical'")


@dataclass(frozen=True)
class CheckAction:
    acknowledge_fault: bool = False

    def __post_init__(self) -> None:
        if type(self.acknowledge_fault) is not bool:
            raise ValueError("acknowledge_fault must be bool")


@dataclass(frozen=True)
class ArmAction:
    support_acknowledgment: bool = True

    def __post_init__(self) -> None:
        if type(self.support_acknowledgment) is not bool:
            raise ValueError("support_acknowledgment must be bool")


@dataclass(frozen=True)
class JogIntent:
    sequence: int
    axes: tuple[float, float, float]
    deadman: bool
    focus: bool
    device_epoch: str
    ticket: InputTicket

    def __post_init__(self) -> None:
        if type(self.sequence) is not int or self.sequence < 0:
            raise ValueError("sequence must be a non-negative integer")
        if type(self.axes) is not tuple or len(self.axes) != 3:
            raise ValueError("axes must be a 3-element tuple")
        for axis in self.axes:
            if isinstance(axis, bool) or not isinstance(axis, (int, float)):
                raise ValueError("each axis must be a finite real number")
            try:
                value = float(axis)
            except (OverflowError, TypeError, ValueError) as exc:
                raise ValueError("each axis must be a finite real number") from exc
            if not math.isfinite(value) or not -1.0 <= value <= 1.0:
                raise ValueError("each axis must be in [-1.0, 1.0]")
        if type(self.deadman) is not bool or type(self.focus) is not bool:
            raise ValueError("deadman and focus must be bool values")
        if type(self.device_epoch) is not str or not self.device_epoch:
            raise ValueError("device_epoch must be a non-empty string")
        if type(self.ticket) is not InputTicket:
            raise ValueError("ticket must be an InputTicket")


@dataclass(frozen=True)
class StopAction:
    reason: str = "operator_stop"

    def __post_init__(self) -> None:
        if type(self.reason) is not str or not self.reason or len(self.reason) > 64:
            raise ValueError("reason must be a bounded non-empty string")


@dataclass(frozen=True)
class DisconnectAction:
    pass


@dataclass(frozen=True)
class SetSpeedAction:
    speed_rad_s: float

    def __post_init__(self) -> None:
        speed = validate_finite_time(self.speed_rad_s, "speed_rad_s")
        if speed <= 0.0:
            raise ValueError("speed_rad_s must be positive")


@dataclass(frozen=True)
class RequestReceipt:
    request_id: int
    action_name: str
    status: str
    code: str
    message: str
    timestamp_s: float


@dataclass(frozen=True)
class ReceiptLookup:
    request_id: int
    status: str
    receipt: RequestReceipt | None

    def __post_init__(self) -> None:
        if type(self.request_id) is not int or self.request_id < 1:
            raise ValueError("request_id must be a positive integer")
        if self.status not in ("present", "expired", "unknown"):
            raise ValueError("status must be present, expired, or unknown")
        if (self.status == "present") != (self.receipt is not None):
            raise ValueError("present receipt status must match receipt presence")


@dataclass(frozen=True)
class StopAttemptView:
    motor_id: int
    attempted: bool
    outcome: str
    write_result: WriteResult | None
    feedback_mode: int | None
    feedback_received: bool
    confirmed: bool


@dataclass(frozen=True)
class PhysicalJointDraft:
    motor_id: int
    name: str
    direction: int
    reference_joint_rad: float
    zero_output_rad: float | None = None
    q_min_rad: float | None = None
    q_max_rad: float | None = None
    v_max_rad_s: float | None = None
    a_max_rad_s2: float | None = None
    max_lead_rad: float | None = None
    max_effort_nm: float | None = None
    kp: float | None = None
    kd: float | None = None
    protection: ProtectionPolicy | None = None
    limits_provenance: str = ""


@dataclass(frozen=True)
class PhysicalDraft:
    port_config: PortConfig | None = None
    joints: tuple[PhysicalJointDraft, ...] = ()
    support_acknowledged: bool = False
    timeout_facts_acknowledged: bool = False
    allow_hardware_opt_in: bool = False


def _finite_number(value: object) -> bool:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(float(value))
    except (OverflowError, TypeError, ValueError):
        return False


def validate_physical_draft(draft: PhysicalDraft | None) -> tuple[bool, list[str]]:
    """Purely validate a proposed commissioning draft; never acquire hardware."""
    if not isinstance(draft, PhysicalDraft):
        return False, ["A physical commissioning draft is required"]

    errors: list[str] = []
    if type(draft.allow_hardware_opt_in) is not bool or not draft.allow_hardware_opt_in:
        errors.append("Hardware opt-in is required")
    if not isinstance(draft.port_config, PortConfig):
        errors.append("Valid PortConfig is required")
    if type(draft.support_acknowledged) is not bool or not draft.support_acknowledged:
        errors.append("Support acknowledgment is required")
    if type(draft.timeout_facts_acknowledged) is not bool or not draft.timeout_facts_acknowledged:
        errors.append("Timeout facts acknowledgment is required")
    if type(draft.joints) is not tuple or len(draft.joints) != 3:
        errors.append("Physical arm requires exactly 3 joint drafts")
        return False, errors

    if any(not isinstance(joint, PhysicalJointDraft) for joint in draft.joints):
        errors.append("Every joint draft must be a PhysicalJointDraft")
        return False, errors
    if tuple(j.motor_id for j in draft.joints) != (0, 1, 2):
        errors.append("Joint motor IDs must be [0, 1, 2]")

    for joint in draft.joints:
        label = joint.name if type(joint.name) is str and joint.name else str(joint.motor_id)
        if type(joint.motor_id) is not int or not 0 <= joint.motor_id <= 2:
            errors.append(f"Joint {label} motor ID is invalid")
        if type(joint.direction) is not int or joint.direction not in (-1, 1):
            errors.append(f"Joint {label} direction must be integer -1 or 1")
        if not _finite_number(joint.reference_joint_rad):
            errors.append(f"Joint {label} reference must be finite")
        if not _finite_number(joint.zero_output_rad):
            errors.append(f"Joint {label} zero_output_rad must be confirmed and finite")
        if not _finite_number(joint.q_min_rad) or not _finite_number(joint.q_max_rad):
            errors.append(f"Joint {label} requires explicit finite position limits")
        elif joint.q_min_rad >= joint.q_max_rad:
            errors.append(f"Joint {label} position limits must be ordered")
        elif joint.motor_id == 0 and (
            joint.q_min_rad < J1_MIN_RAD or joint.q_max_rad > J1_MAX_RAD
        ):
            errors.append(f"J1 limits must stay within the canonical model bounds")

        for field_name in (
            "v_max_rad_s",
            "a_max_rad_s2",
            "max_lead_rad",
            "max_effort_nm",
        ):
            value = getattr(joint, field_name)
            if not _finite_number(value) or value <= 0:
                errors.append(f"Joint {label} requires positive finite {field_name}")
        for field_name in ("kp", "kd"):
            value = getattr(joint, field_name)
            if not _finite_number(value) or value < 0:
                errors.append(f"Joint {label} requires non-negative finite {field_name}")
        if not isinstance(joint.protection, ProtectionPolicy):
            errors.append(f"Joint {label} requires ProtectionPolicy")
        if (
            type(joint.limits_provenance) is not str
            or not joint.limits_provenance.strip()
            or any(word in joint.limits_provenance.lower() for word in ("guessed", "default", "assumed"))
        ):
            errors.append(f"Joint {label} requires explicit limits provenance")

    return not errors, errors


def compute_zero_output(
    measured_output_rad: float,
    direction: int,
    reference_joint_rad: float,
) -> float:
    """Return measured_output - direction * reference_joint after validation."""
    if not _finite_number(measured_output_rad) or not _finite_number(reference_joint_rad):
        raise ValueError("measured output and reference must be finite numbers")
    if type(direction) is not int or direction not in (-1, 1):
        raise ValueError("direction must be integer -1 or 1")
    return float(measured_output_rad) - direction * float(reference_joint_rad)


@dataclass(frozen=True)
class DispatchAuthority:
    owner_generation: int
    expires_at_s: float | None
    revoked: bool
    stop_pending: bool


@dataclass(frozen=True)
class DispatchRecordView:
    sequence: int
    target: JointTarget
    dispatched_at_s: float


@dataclass(frozen=True)
class DispatchAttemptView:
    sequence: int
    target: JointTarget
    result: WriteResult
    started_at_s: float


@dataclass(frozen=True)
class ArmStationSnapshot:
    lifecycle_state: str
    connection_phase: str
    source: SourceIdentity | None
    requested_axes: tuple[float, float, float] | None
    admitted_goal: tuple[JointTarget, ...] | None
    dispatched_reference: tuple[JointTarget | None, ...] | None
    dispatch_records: tuple[DispatchRecordView | None, ...]
    dispatch_attempts: tuple[DispatchAttemptView | None, ...]
    confirmed_reference: tuple[JointTrajectoryReference | None, ...] | None
    measured_sample: tuple[JointSample | None, ...]
    measured_health: tuple[MotorHealth | None, ...]
    feedback_ages_s: tuple[float | None, ...]
    faults: tuple[JointFault, ...]
    fault_latched: bool
    owner_session_id: str | None
    owner_generation: int
    input_phase: str
    hold_energized: bool
    stop_logical: bool
    stop_confirmed: bool
    stop_pending: bool
    per_joint_stop_confirmed: tuple[bool, ...]
    stop_attempts: tuple[StopAttemptView | None, ...]
    close_attempted: bool
    close_confirmed: bool
    close_unconfirmed: bool
    executor_failed: bool
    timing_counters: TimingCounters | None
    timing_snapshot: TimingSnapshot | None
    latest_receipt: RequestReceipt | None
    device_epoch: str | None
    selected_speed_rad_s: float
    commissioning_blocked: bool
    commissioning_reasons: tuple[str, ...]


class ArmStation:
    """One Core and one transport executor behind bounded ingress."""

    def __init__(
        self,
        *,
        clock: Callable[[], float] | None = None,
        fixture: ArmPacketFixture | None = None,
        slot_period_s: float = DEFAULT_SLOT_PERIOD_S,
        input_lease_s: float = DEFAULT_INPUT_LEASE_S,
        exchange_deadline_s: float = DEFAULT_EXCHANGE_DEADLINE_S,
        max_reference_gap_s: float = DEFAULT_MAX_REFERENCE_GAP_S,
        feedback_max_age_s: float = DEFAULT_FEEDBACK_MAX_AGE_S,
        max_window_s: float = DEFAULT_MAX_WINDOW_S,
        jog_speed_rad_s: float = DEFAULT_SPEED_LIMIT_RAD_S,
    ) -> None:
        self._clock = time.monotonic if clock is None else clock
        if not callable(self._clock):
            raise ValueError("clock must be callable")
        self._last_time_s = 0.0
        self.slot_period_s = validate_finite_time(slot_period_s, "slot_period_s")
        self.input_lease_s = validate_finite_time(input_lease_s, "input_lease_s")
        self.exchange_deadline_s = validate_finite_time(
            exchange_deadline_s, "exchange_deadline_s"
        )
        self.max_reference_gap_s = validate_finite_time(
            max_reference_gap_s, "max_reference_gap_s"
        )
        self.feedback_max_age_s = validate_finite_time(
            feedback_max_age_s, "feedback_max_age_s"
        )
        self.max_window_s = validate_finite_time(max_window_s, "max_window_s")
        self._selected_speed_rad_s = validate_finite_time(
            jog_speed_rad_s, "jog_speed_rad_s"
        )
        if min(
            self.slot_period_s,
            self.input_lease_s,
            self.exchange_deadline_s,
            self.max_reference_gap_s,
            self.feedback_max_age_s,
            self.max_window_s,
            self._selected_speed_rad_s,
        ) <= 0.0:
            raise ValueError("station timing, lease, window, and speed limits must be positive")
        if self._selected_speed_rad_s > 2.0:
            raise ValueError("jog_speed_rad_s must not exceed 2.0")

        self._specs = create_synthetic_arm_specs()
        self._bus = fixture if fixture is not None else ArmPacketFixture(clock=self._clock)
        self._core: JointCore | None = None
        self._timing: StationTimingTracker | None = None
        self._coordinator: StationStepCoordinator | None = None
        self._transport_closed = False

        self._condition = threading.Condition(threading.Lock())
        self._sessions: dict[str, SessionHandle] = {}
        self._owner_session_id: str | None = None
        self._owner_generation = 0
        self._ticket: InputTicket | None = None
        self._pending_intent: tuple[int, JogIntent, InputTicket] | None = None
        self._latest_intent: JogIntent | None = None
        self._latest_intent_expiry_s: float | None = None
        self._last_accepted_intent_sequence = -1
        self._owner_deadline_s: float | None = None
        self._input_enabled = False
        self._deadman_active = False
        self._new_deadman_edge_pending = False
        self._neutral_event_pending = False
        self._release_capture_pending = False
        self._release_neutral_pending = False
        self._arm_transition = False
        self._device_epoch: str | None = None
        self._priority_stop_pending = False
        self._priority_stop_reason: str | None = None
        self._stop_in_progress = False
        self._lifecycle_in_progress = False
        self._lifecycle_queue: deque[tuple[int, object]] = deque()
        self._receipts: OrderedDict[int, RequestReceipt] = OrderedDict()
        self._active_lifecycle_receipts: OrderedDict[int, RequestReceipt] = OrderedDict()
        self._terminal_lifecycle_receipts: OrderedDict[int, RequestReceipt] = OrderedDict()
        self._latest_receipt: RequestReceipt | None = None
        self._next_request_id = 1

        self._current_core_owner: OwnerToken | None = None
        self._next_core_sequence = 0
        self._needs_neutral_and_edge = True
        self._neutral_seen = False
        self._hold_targets: tuple[JointTarget, ...] | None = None
        self._holding_position = False
        self._hold_energized = False
        self._input_phase = "disconnected"
        self._connection_phase = "disconnected"
        self._per_joint_stop_confirmed = {motor_id: False for motor_id in (0, 1, 2)}
        self._stop_attempts: dict[int, StopAttemptView | None] = {
            motor_id: None for motor_id in (0, 1, 2)
        }
        self._close_attempted = False
        self._close_confirmed = False
        self._close_unconfirmed = False
        self._close_request_id = 0
        self._close_completed_id = 0
        self._executor_failed = False
        self._commissioning_blocked = False
        self._commissioning_reasons: tuple[str, ...] = ()

        self._execution_mode: str | None = None
        self._executor_thread_id: int | None = None
        self._loop_thread: threading.Thread | None = None
        self._loop_running = threading.Event()
        self._loop_stop_requested = False

        self._published = self._empty_snapshot()

    def _now(self) -> float:
        now = validate_finite_time(self._clock(), "clock")
        self._last_time_s = now
        return now

    def _empty_snapshot(self) -> ArmStationSnapshot:
        return ArmStationSnapshot(
            lifecycle_state=State.DISCONNECTED.value,
            connection_phase="disconnected",
            source=None,
            requested_axes=None,
            admitted_goal=None,
            dispatched_reference=(None, None, None),
            dispatch_records=(None, None, None),
            dispatch_attempts=(None, None, None),
            confirmed_reference=(None, None, None),
            measured_sample=(None, None, None),
            measured_health=(None, None, None),
            feedback_ages_s=(None, None, None),
            faults=(),
            fault_latched=False,
            owner_session_id=None,
            owner_generation=0,
            input_phase="disconnected",
            hold_energized=False,
            stop_logical=True,
            stop_confirmed=False,
            stop_pending=False,
            per_joint_stop_confirmed=(False, False, False),
            stop_attempts=(None, None, None),
            close_attempted=False,
            close_confirmed=False,
            close_unconfirmed=False,
            executor_failed=False,
            timing_counters=None,
            timing_snapshot=None,
            latest_receipt=None,
            device_epoch=None,
            selected_speed_rad_s=self._selected_speed_rad_s,
            commissioning_blocked=False,
            commissioning_reasons=(),
        )

    @staticmethod
    def _action_name(action: object) -> str:
        return type(action).__name__

    def _new_receipt_locked(
        self,
        request_id: int,
        action_name: str,
        status: str,
        code: str,
        message: str,
        timestamp_s: float,
    ) -> RequestReceipt:
        receipt = RequestReceipt(
            request_id, action_name, status, code, message, timestamp_s
        )
        self._receipts[request_id] = receipt
        self._receipts.move_to_end(request_id)
        while len(self._receipts) > MAX_RECEIPTS:
            self._receipts.popitem(last=False)
        if self._latest_receipt is None or request_id >= self._latest_receipt.request_id:
            self._latest_receipt = receipt
        return receipt

    def _finish_receipt_locked(
        self,
        request_id: int,
        action_name: str,
        status: str,
        code: str,
        message: str,
        timestamp_s: float,
    ) -> RequestReceipt | None:
        if (
            request_id not in self._receipts
            and request_id not in self._active_lifecycle_receipts
        ):
            return None
        receipt = self._new_receipt_locked(
            request_id, action_name, status, code, message, timestamp_s
        )
        if self._active_lifecycle_receipts.pop(request_id, None) is not None:
            self._terminal_lifecycle_receipts[request_id] = receipt
            self._terminal_lifecycle_receipts.move_to_end(request_id)
            while len(self._terminal_lifecycle_receipts) > MAX_TERMINAL_LIFECYCLE_RECEIPTS:
                self._terminal_lifecycle_receipts.popitem(last=False)
        if self._latest_receipt is not None and request_id < self._latest_receipt.request_id:
            self._latest_receipt = max(
                (self._latest_receipt, receipt), key=lambda item: item.request_id
            )
        return receipt

    def _finish_receipt(
        self,
        request_id: int,
        action_name: str,
        status: str,
        code: str,
        message: str,
    ) -> None:
        now = self._now()
        with self._condition:
            self._finish_receipt_locked(
                request_id, action_name, status, code, message, now
            )
            self._publish_ingress_locked()

    def receipt(self, request_id: int) -> ReceiptLookup:
        if type(request_id) is not int or request_id < 1:
            raise ValueError("request_id must be a positive integer")
        with self._condition:
            receipt = (
                self._receipts.get(request_id)
                or self._active_lifecycle_receipts.get(request_id)
                or self._terminal_lifecycle_receipts.get(request_id)
            )
            if receipt is not None:
                return ReceiptLookup(request_id, "present", receipt)
            status = "expired" if request_id < self._next_request_id else "unknown"
            return ReceiptLookup(request_id, status, None)

    def _publish_ingress_locked(self) -> None:
        view = self._published
        self._published = replace(
            view,
            requested_axes=(
                self._latest_intent.axes if self._latest_intent is not None else None
            ),
            owner_session_id=self._owner_session_id,
            owner_generation=self._owner_generation,
            stop_pending=self._priority_stop_pending or self._stop_in_progress,
            latest_receipt=self._latest_receipt,
            device_epoch=self._device_epoch,
            selected_speed_rad_s=self._selected_speed_rad_s,
            close_attempted=self._close_attempted,
            close_confirmed=self._close_confirmed,
            close_unconfirmed=self._close_unconfirmed,
        )

    def register_session(self, client_info: str = "") -> SessionHandle:
        now = self._now()
        with self._condition:
            if len(self._sessions) >= MAX_SESSIONS:
                raise RuntimeError("Maximum registered sessions exceeded")
            handle = SessionHandle(secrets.token_hex(16), now)
            self._sessions[handle.session_id] = handle
            self._publish_ingress_locked()
            return handle

    def _registered_session_locked(self, handle: object) -> bool:
        return (
            type(handle) is SessionHandle
            and self._sessions.get(handle.session_id) == handle
        )

    def _request_stop_locked(self, reason: str, timestamp_s: float) -> None:
        if not self._priority_stop_pending:
            self._owner_generation += 1
        self._priority_stop_pending = True
        self._priority_stop_reason = reason
        self._revoke_ingress_locked(
            timestamp_s,
            "STOP_PREEMPTED",
            "Station authority was revoked by Stop",
        )

    def _revoke_ingress_locked(
        self, timestamp_s: float, code: str, message: str
    ) -> None:
        self._input_enabled = False
        self._owner_deadline_s = None
        self._ticket = None
        self._latest_intent = None
        self._latest_intent_expiry_s = None
        if self._pending_intent is not None:
            request_id, intent, _ = self._pending_intent
            self._new_receipt_locked(
                request_id,
                "JogIntent",
                "rejected",
                code,
                message,
                timestamp_s,
            )
            self._pending_intent = None
        self._deadman_active = False
        self._new_deadman_edge_pending = False
        self._neutral_event_pending = False
        self._release_capture_pending = False
        self._release_neutral_pending = False
        while self._lifecycle_queue:
            request_id, action = self._lifecycle_queue.popleft()
            self._finish_receipt_locked(
                request_id,
                self._action_name(action),
                "rejected",
                code,
                message,
                timestamp_s,
            )
        self._publish_ingress_locked()

    def _release_owner_locked(
        self, timestamp_s: float, code: str, message: str
    ) -> None:
        if self._owner_session_id is None:
            return
        self._owner_generation += 1
        self._revoke_ingress_locked(timestamp_s, code, message)
        self._owner_session_id = None
        self._last_accepted_intent_sequence = -1
        self._device_epoch = None
        self._publish_ingress_locked()

    def issue_ticket(self, session_handle: SessionHandle, sequence: int) -> InputTicket:
        now = self._now()
        if type(sequence) is not int or sequence < 0:
            raise ValueError("sequence must be a non-negative integer")
        with self._condition:
            if not self._registered_session_locked(session_handle):
                raise ValueError("Session handle is not registered")
            if session_handle.session_id != self._owner_session_id:
                raise ValueError("Session is not the current station owner")
            if not self._input_enabled or self._priority_stop_pending or self._stop_in_progress:
                raise RuntimeError("Input tickets require an armed station with live authority")
            if (
                self._input_enabled
                and self._owner_deadline_s is not None
                and now >= self._owner_deadline_s
            ):
                self._request_stop_locked("owner_lease_expired", now)
                raise RuntimeError("Owner input lease expired; Stop was queued")
            ticket = InputTicket(
                ticket_id=secrets.token_hex(16),
                session_id=session_handle.session_id,
                owner_generation=self._owner_generation,
                created_at_s=now,
                expires_at_s=now + self.input_lease_s,
                bound_sequence=sequence,
            )
            self._ticket = ticket
            return ticket

    def request(self, action: object, session_handle: SessionHandle) -> RequestReceipt:
        now = self._now()
        with self._condition:
            request_id = self._next_request_id
            self._next_request_id += 1
            name = self._action_name(action)

            if not self._registered_session_locked(session_handle):
                receipt = self._new_receipt_locked(
                    request_id, name, "rejected", "UNKNOWN_SESSION",
                    "Session handle is not registered", now
                )
                self._publish_ingress_locked()
                return receipt

            if isinstance(action, StopAction):
                self._request_stop_locked(action.reason, now)
                self._condition.notify_all()
                receipt = self._new_receipt_locked(
                    request_id, "Stop", "accepted", "STOP_QUEUED",
                    "Priority Stop queued", now
                )
                self._publish_ingress_locked()
                return receipt

            if isinstance(action, JogIntent):
                if session_handle.session_id != self._owner_session_id:
                    code, message = "NOT_OWNER", "Action requires the station owner"
                elif self._priority_stop_pending or self._stop_in_progress:
                    code, message = "STOP_PENDING", "Input authority is revoked"
                elif self._arm_transition or self._lifecycle_in_progress:
                    code, message = "LIFECYCLE_BUSY", "Input is unavailable during a lifecycle transition"
                elif (
                    self._input_enabled
                    and self._owner_deadline_s is not None
                    and now >= self._owner_deadline_s
                ):
                    self._request_stop_locked("owner_lease_expired", now)
                    code, message = "OWNER_EXPIRED", "Owner lease expired; Stop was queued"
                else:
                    canonical = self._ticket
                    if canonical is None or action.ticket.ticket_id != canonical.ticket_id:
                        code, message = "INVALID_TICKET", "Ticket not found or already consumed"
                    else:
                        self._ticket = None
                        if action.ticket != canonical:
                            code, message = "INVALID_TICKET", "Ticket does not match its server record"
                        elif canonical.session_id != self._owner_session_id:
                            code, message = "INVALID_TICKET", "Ticket belongs to another session"
                        elif canonical.owner_generation != self._owner_generation:
                            code, message = "EXPIRED_OWNER_GENERATION", "Ticket owner generation expired"
                        elif now >= canonical.expires_at_s:
                            code, message = "TICKET_EXPIRED", "Ticket expired before receipt"
                        elif action.sequence != canonical.bound_sequence:
                            code, message = "SEQUENCE_MISMATCH", "Sequence differs from server ticket"
                        elif action.focus is False:
                            self._request_stop_locked("focus_lost", now)
                            code, message = "FOCUS_LOST", "Focus loss revoked input authority"
                        elif self._device_epoch is not None and self._device_epoch != action.device_epoch:
                            self._device_epoch = action.device_epoch
                            self._request_stop_locked("device_epoch_changed", now)
                            code, message = "DEVICE_EPOCH_CHANGED", "Device change revoked input authority"
                        elif action.sequence <= self._last_accepted_intent_sequence:
                            code, message = "DUPLICATE_OR_OLDER", "Input sequence is duplicate or older"
                        else:
                            if self._device_epoch is None:
                                self._device_epoch = action.device_epoch
                            if self._pending_intent is not None:
                                old_id, _, _ = self._pending_intent
                                self._new_receipt_locked(
                                    old_id, "JogIntent", "superseded", "SUPERSEDED",
                                    "A newer unsent input replaced this one", now
                                )
                            self._last_accepted_intent_sequence = action.sequence
                            self._latest_intent = action
                            self._latest_intent_expiry_s = canonical.expires_at_s
                            self._pending_intent = (request_id, action, canonical)
                            was_deadman_active = self._deadman_active
                            if action.deadman:
                                if not self._deadman_active:
                                    self._new_deadman_edge_pending = True
                                self._deadman_active = True
                            else:
                                self._deadman_active = False
                                if self._input_enabled and all(axis == 0.0 for axis in action.axes):
                                    self._neutral_event_pending = True
                                if was_deadman_active and self._input_enabled:
                                    self._release_capture_pending = True
                                    self._release_neutral_pending = all(
                                        axis == 0.0 for axis in action.axes
                                    )
                            if self._input_enabled:
                                self._owner_deadline_s = canonical.expires_at_s
                            receipt = self._new_receipt_locked(
                                request_id, "JogIntent", "accepted", "ACCEPTED",
                                "Jog input admitted", now
                            )
                            self._publish_ingress_locked()
                            return receipt
                receipt = self._new_receipt_locked(
                    request_id, "JogIntent", "rejected", code, message, now
                )
                self._publish_ingress_locked()
                return receipt

            if isinstance(action, SetSpeedAction):
                if session_handle.session_id != self._owner_session_id:
                    receipt = self._new_receipt_locked(
                        request_id, name, "rejected", "NOT_OWNER",
                        "Action requires the station owner", now
                    )
                elif action.speed_rad_s > 2.0:
                    receipt = self._new_receipt_locked(
                        request_id, name, "rejected", "INVALID_SPEED",
                        "Requested speed exceeds the fixture policy", now
                    )
                else:
                    self._selected_speed_rad_s = action.speed_rad_s
                    receipt = self._new_receipt_locked(
                        request_id, name, "accepted", "ACCEPTED",
                        "Selected speed updated", now
                    )
                self._publish_ingress_locked()
                return receipt

            if isinstance(action, (ConnectAction, CheckAction, ArmAction, DisconnectAction)):
                if self._close_attempted:
                    receipt = self._new_receipt_locked(
                        request_id, name, "rejected", "CLOSE_IN_PROGRESS",
                        "Station close has begun", now
                    )
                elif self._priority_stop_pending or self._stop_in_progress:
                    receipt = self._new_receipt_locked(
                        request_id, name, "rejected", "STOP_PENDING",
                        "Lifecycle change must wait for Stop confirmation", now
                    )
                elif self._lifecycle_in_progress:
                    receipt = self._new_receipt_locked(
                        request_id, name, "rejected", "LIFECYCLE_BUSY",
                        "A lifecycle transition is already in progress", now
                    )
                elif self._owner_session_id is not None and (
                    self._owner_session_id != session_handle.session_id
                ):
                    receipt = self._new_receipt_locked(
                        request_id, name, "rejected", "NOT_OWNER",
                        "Another session owns the station", now
                    )
                elif (
                    len(self._lifecycle_queue) >= MAX_LIFECYCLE_REQUESTS
                    or len(self._active_lifecycle_receipts) >= MAX_LIFECYCLE_REQUESTS
                ):
                    receipt = self._new_receipt_locked(
                        request_id, name, "rejected", "QUEUE_FULL",
                        "Lifecycle queue is full", now
                    )
                else:
                    if self._owner_session_id is None:
                        self._owner_session_id = session_handle.session_id
                        self._owner_generation += 1
                    self._lifecycle_queue.append((request_id, action))
                    receipt = self._new_receipt_locked(
                        request_id, name, "queued", "QUEUED",
                        "Lifecycle request queued", now
                    )
                    self._active_lifecycle_receipts[request_id] = receipt
                self._publish_ingress_locked()
                return receipt

            receipt = self._new_receipt_locked(
                request_id, name, "rejected", "INVALID_ACTION",
                "Unrecognized station action", now
            )
            self._publish_ingress_locked()
            return receipt

    def dispatch_authority(self) -> DispatchAuthority:
        with self._condition:
            return DispatchAuthority(
                owner_generation=self._owner_generation,
                expires_at_s=self._owner_deadline_s,
                revoked=(
                    not self._input_enabled
                    or self._priority_stop_pending
                    or self._stop_in_progress
                    or self._lifecycle_in_progress
                ),
                stop_pending=self._priority_stop_pending or self._stop_in_progress,
            )

    def _ensure_core(self) -> None:
        if self._core is not None:
            return
        self._core = JointCore(
            self._specs,
            max_reference_gap_s=self.max_reference_gap_s,
            feedback_max_age_s=self.feedback_max_age_s,
            max_window_s=self.max_window_s,
            clock=self._clock,
        )
        self._timing = StationTimingTracker(
            StationTimingConfig(nominal_period_s=self.slot_period_s)
        )
        self._coordinator = StationStepCoordinator(
            self._core,
            self._bus,
            self._timing,
            clock=self._clock,
            deadline_s=self.exchange_deadline_s,
        )

    def _bind_executor(self, mode: str) -> None:
        ident = threading.get_ident()
        with self._condition:
            if self._execution_mode is None:
                self._execution_mode = mode
                self._executor_thread_id = ident
            elif self._execution_mode != mode or self._executor_thread_id != ident:
                raise RuntimeError("Core execution is confined to its bound executor")

    def _assert_executor(self) -> None:
        if self._executor_thread_id != threading.get_ident():
            raise RuntimeError("Core operation attempted outside the bound executor")

    def step(self, *, missed_slots: int = 0) -> LoopStepRecord:
        """Run one deterministic manual step on the thread that first owns it."""
        self._bind_executor("manual")
        if type(missed_slots) is not int or missed_slots < 0:
            raise ValueError("missed_slots must be a non-negative integer")
        try:
            return self._step_once(missed_slots=missed_slots)
        except Exception:
            self._record_executor_failure()
            raise

    def _take_priority_stop(self) -> str | None:
        with self._condition:
            if not self._priority_stop_pending:
                return None
            reason = self._priority_stop_reason or "operator_stop"
            self._priority_stop_pending = False
            self._priority_stop_reason = None
            self._stop_in_progress = True
            self._publish_ingress_locked()
            return reason

    def _take_lifecycle(self) -> tuple[int, object] | None:
        with self._condition:
            if not self._lifecycle_queue:
                return None
            self._lifecycle_in_progress = True
            return self._lifecycle_queue.popleft()

    def _take_input(self) -> tuple[
        tuple[int, JogIntent, InputTicket] | None, bool, bool, bool, bool
    ]:
        with self._condition:
            pending = self._pending_intent
            self._pending_intent = None
            neutral = self._neutral_event_pending
            self._neutral_event_pending = False
            release = self._release_capture_pending
            release_neutral = self._release_neutral_pending
            self._release_capture_pending = False
            self._release_neutral_pending = False
            edge = self._new_deadman_edge_pending
            self._new_deadman_edge_pending = False
            return pending, neutral, release, release_neutral, edge

    def _set_stop_confirmation(self, records: tuple, started_at_s: float) -> None:
        assert self._core is not None
        core_snapshot = self._core.snapshot()
        checked_at_s = self._now()
        for motor_id in self._per_joint_stop_confirmed:
            self._per_joint_stop_confirmed[motor_id] = False
        index_by_id = {spec.motor_id: index for index, spec in enumerate(self._specs)}
        for record in records:
            index = index_by_id.get(record.motor_id)
            if index is None:
                continue
            status = core_snapshot.joints[index]
            timestamp = record.feedback_timestamp_s
            self._per_joint_stop_confirmed[record.motor_id] = bool(
                record.outcome == "OK"
                and record.feedback_received
                and timestamp is not None
                and timestamp >= started_at_s
                and timestamp <= checked_at_s
                and checked_at_s - timestamp <= self.feedback_max_age_s
                and status.valid
                and status.received_at == timestamp
                and status.health is not None
                and status.health.mode == 0
            )

    def _record_stop_transactions(self, records: tuple) -> None:
        assert self._core is not None
        try:
            snapshot = self._core.snapshot()
        except Exception:
            snapshot = None
        index_by_id = {spec.motor_id: index for index, spec in enumerate(self._specs)}
        for record in records:
            index = index_by_id.get(record.motor_id)
            health = (
                snapshot.joints[index].health
                if snapshot is not None and index is not None
                else None
            )
            self._stop_attempts[record.motor_id] = StopAttemptView(
                motor_id=record.motor_id,
                attempted=True,
                outcome=record.outcome,
                write_result=record.write_result,
                feedback_mode=health.mode if health is not None else None,
                feedback_received=record.feedback_received,
                confirmed=self._per_joint_stop_confirmed[record.motor_id],
            )

    def _attempt_mode0_all(self) -> None:
        started_at_s: float | None
        try:
            started_at_s = self._now()
        except Exception:
            started_at_s = None
        self._per_joint_stop_confirmed = {motor_id: False for motor_id in (0, 1, 2)}
        self._stop_attempts = {motor_id: None for motor_id in (0, 1, 2)}
        index_by_id = {spec.motor_id: index for index, spec in enumerate(self._specs)}
        for spec in self._specs:
            motor_id = spec.motor_id
            write_result: WriteResult | None = None
            feedback_received = False
            feedback_mode: int | None = None
            confirmed = False
            outcome = "DRIVER_FAILURE"

            def on_write(result: WriteResult) -> None:
                nonlocal write_result
                write_result = result

            try:
                reply = exchange(
                    self._bus,
                    command(motor_id, mode=0),
                    motor_id,
                    deadline_s=self.exchange_deadline_s,
                    on_write=on_write,
                )
                feedback_received = True
                feedback_mode = reply.get("mode")
                if feedback_mode == 0:
                    outcome = "OK"
                    if started_at_s is not None and self._core is not None:
                        sample = mapped_sample(spec.mapping, reply)
                        health = mapped_health(reply)
                        try:
                            self._core.receive(motor_id, sample, health)
                            current = self._core.snapshot().joints[index_by_id[motor_id]]
                            checked_at_s = self._now()
                            confirmed = bool(
                                current.sample is sample
                                and current.health is health
                                and current.valid
                                and current.received_at is not None
                                and started_at_s <= current.received_at <= checked_at_s
                                and current.health.mode == 0
                            )
                        except Exception:
                            confirmed = False
                else:
                    outcome = "MODE_MISMATCH"
            except InvalidFeedbackError:
                outcome = "INVALID_FEEDBACK"
            except TransactionTimeoutError:
                outcome = "TIMEOUT"
            except Exception:
                outcome = "DRIVER_FAILURE"

            self._per_joint_stop_confirmed[motor_id] = confirmed
            self._stop_attempts[motor_id] = StopAttemptView(
                motor_id=motor_id,
                attempted=True,
                outcome=outcome,
                write_result=write_result,
                feedback_mode=feedback_mode,
                feedback_received=feedback_received,
                confirmed=confirmed,
            )

    def _execute_stop(self, reason: str) -> None:
        assert self._core is not None and self._coordinator is not None
        with self._condition:
            self._stop_in_progress = True
            self._input_enabled = False
            self._owner_deadline_s = None
            self._ticket = None
            self._latest_intent = None
            self._latest_intent_expiry_s = None
            self._pending_intent = None
            self._publish_ingress_locked()
        self._hold_energized = False
        self._holding_position = False
        self._hold_targets = None
        self._current_core_owner = None
        self._needs_neutral_and_edge = True
        self._neutral_seen = False
        self._per_joint_stop_confirmed = {motor_id: False for motor_id in (0, 1, 2)}
        self._stop_attempts = {motor_id: None for motor_id in (0, 1, 2)}
        logical_stop = False
        try:
            if self._connection_phase in ("disconnected", "physical_commissioning"):
                self._input_phase = "disconnected"
                logical_stop = True
                return
            logical_stop = self._core.stop().accepted
            self._input_phase = "stopped"
            started = self._now()
            records = self._coordinator.check_step()
            self._set_stop_confirmation(records, started)
            self._record_stop_transactions(records)
            if self._core.snapshot().state is State.FAULT:
                self._input_phase = "fault"
        except Exception:
            try:
                logical_stop = self._core.snapshot().state in (
                    State.STOPPED, State.FAULT, State.DISCONNECTED
                )
            except Exception:
                pass
            self._attempt_mode0_all()
        finally:
            with self._condition:
                self._stop_in_progress = False
                self._publish_ingress_locked()
            try:
                self._publish()
            except Exception:
                with self._condition:
                    self._published = replace(
                        self._published,
                        lifecycle_state=(
                            State.FAULT.value
                            if self._published.fault_latched
                            else (
                                State.STOPPED.value
                                if logical_stop
                                else self._published.lifecycle_state
                            )
                        ),
                        input_phase=self._input_phase,
                        hold_energized=False,
                        stop_logical=logical_stop,
                        stop_confirmed=all(self._per_joint_stop_confirmed.values()),
                        per_joint_stop_confirmed=tuple(
                            self._per_joint_stop_confirmed[motor_id]
                            for motor_id in (0, 1, 2)
                        ),
                        stop_attempts=tuple(
                            self._stop_attempts[motor_id] for motor_id in (0, 1, 2)
                        ),
                    )

    def _execute_close(self, request_id: int) -> bool:
        with self._condition:
            self._close_attempted = True
        try:
            close = getattr(self._bus, "close", None)
            if close is None:
                closed = getattr(self._bus, "is_open", None) is False
            else:
                close_result = close()
                closed = (
                    close_result is not False
                    and getattr(self._bus, "is_open", None) is False
                )
        except Exception:
            closed = False
        if closed:
            self._close_confirmed = True
            self._close_unconfirmed = False
            self._transport_closed = True
            if self._core is not None:
                self._core.disconnect()
            self._current_core_owner = None
            with self._condition:
                can_release_owner = (
                    self._connection_phase in ("disconnected", "physical_commissioning")
                    or all(self._per_joint_stop_confirmed.values())
                )
                if can_release_owner:
                    self._release_owner_locked(
                        self._last_time_s,
                        "STATION_CLOSED",
                        "Queued lifecycle request was revoked after close",
                    )
                self._input_phase = (
                    "executor_failed" if self._executor_failed else "disconnected"
                )
                self._connection_phase = "disconnected"
        else:
            self._close_confirmed = False
            self._close_unconfirmed = True
        with self._condition:
            self._close_completed_id = max(self._close_completed_id, request_id)
            self._condition.notify_all()
        try:
            self._publish()
        except Exception:
            with self._condition:
                self._published = replace(
                    self._published,
                    lifecycle_state=(
                        State.DISCONNECTED.value if closed else self._published.lifecycle_state
                    ),
                    connection_phase=self._connection_phase,
                    source=None if closed else self._published.source,
                    owner_session_id=self._owner_session_id,
                    owner_generation=self._owner_generation,
                    input_phase=self._input_phase,
                    hold_energized=False,
                    stop_logical=(True if closed else self._published.stop_logical),
                    stop_confirmed=all(self._per_joint_stop_confirmed.values()),
                    stop_pending=self._priority_stop_pending or self._stop_in_progress,
                    per_joint_stop_confirmed=tuple(
                        self._per_joint_stop_confirmed[motor_id]
                        for motor_id in (0, 1, 2)
                    ),
                    stop_attempts=tuple(
                        self._stop_attempts[motor_id] for motor_id in (0, 1, 2)
                    ),
                    close_attempted=self._close_attempted,
                    close_confirmed=self._close_confirmed,
                    close_unconfirmed=self._close_unconfirmed,
                    executor_failed=self._executor_failed,
                    latest_receipt=self._latest_receipt,
                )
        return closed

    def _pending_close_id(self) -> int | None:
        with self._condition:
            if self._close_request_id > self._close_completed_id:
                return self._close_request_id
            return None

    def _step_once(self, *, missed_slots: int = 0) -> LoopStepRecord:
        self._assert_executor()
        if type(missed_slots) is not int or missed_slots < 0:
            raise ValueError("missed_slots must be a non-negative integer")
        self._ensure_core()
        assert self._core is not None and self._coordinator is not None

        now = self._now()
        with self._condition:
            expired = (
                self._input_enabled
                and self._owner_deadline_s is not None
                and now >= self._owner_deadline_s
            )
            if expired:
                self._request_stop_locked("owner_lease_expired", now)
        reason = self._take_priority_stop()
        close_id = self._pending_close_id()
        if reason is not None or close_id is not None:
            self._execute_stop(reason or "station_close")
            if close_id is not None:
                self._execute_close(close_id)
            return self._idle_step(missed_slots)

        lifecycle = self._take_lifecycle()
        armed_now = False
        if lifecycle is not None:
            request_id, action = lifecycle
            try:
                armed_now = self._handle_lifecycle_action(request_id, action)
            except Exception:
                with self._condition:
                    self._finish_receipt_locked(
                        request_id,
                        self._action_name(action),
                        "rejected",
                        "EXECUTOR_FAILED",
                        "Lifecycle execution failed before a terminal receipt was recorded",
                        self._last_time_s,
                    )
                    self._publish_ingress_locked()
                raise
            finally:
                with self._condition:
                    self._lifecycle_in_progress = False
                    self._publish_ingress_locked()

        now = self._now()
        with self._condition:
            expired = (
                self._input_enabled
                and self._owner_deadline_s is not None
                and now >= self._owner_deadline_s
            )
            if expired:
                self._request_stop_locked("owner_lease_expired", now)
        reason = self._take_priority_stop()
        close_id = self._pending_close_id()
        if reason is not None or close_id is not None:
            self._execute_stop(reason or "station_close")
            if close_id is not None:
                self._execute_close(close_id)
            return self._idle_step(missed_slots)

        pending, neutral_event, release_event, release_neutral, edge_event = self._take_input()
        if neutral_event:
            self._neutral_seen = True
        if release_event:
            self._capture_release_hold()
            if release_neutral:
                self._neutral_seen = True
        if pending is not None and not armed_now:
            self._apply_intent(pending, edge_event)
        elif pending is None and not armed_now:
            self._refresh_active_goal()

        now = self._now()
        with self._condition:
            expired = (
                self._input_enabled
                and self._owner_deadline_s is not None
                and now >= self._owner_deadline_s
            )
            if expired:
                self._request_stop_locked("owner_lease_expired", now)
        reason = self._take_priority_stop()
        close_id = self._pending_close_id()
        if reason is not None or close_id is not None:
            self._execute_stop(reason or "station_close")
            if close_id is not None:
                self._execute_close(close_id)
            return self._idle_step(missed_slots)

        core_snapshot = self._core.snapshot()
        core_state = core_snapshot.state
        if core_state is State.READY:
            self._coordinator.check_step()
            record = self._idle_step(missed_slots)
        elif core_state in (State.ARMED, State.MOVING):
            refs = core_snapshot.confirmed_references
            if not refs or any(ref is None or now <= ref.sample_time_s for ref in refs):
                record = self._idle_step(missed_slots)
            else:
                record = self._coordinator.step(missed_slots=missed_slots)
        else:
            record = self._coordinator.step(missed_slots=missed_slots)

        if any(tx.plan_id is not None for tx in record.transactions):
            self._per_joint_stop_confirmed = {motor_id: False for motor_id in (0, 1, 2)}
        if self._core.snapshot().state is State.FAULT:
            self._hold_energized = False
            self._holding_position = False
            self._input_phase = "fault"
            with self._condition:
                self._request_stop_locked("core_fault", now)
        reason = self._take_priority_stop()
        close_id = self._pending_close_id()
        if reason is not None or close_id is not None:
            self._execute_stop(reason or "station_close")
            if close_id is not None:
                self._execute_close(close_id)
        self._publish()
        return record

    def _idle_step(self, missed_slots: int) -> LoopStepRecord:
        assert self._core is not None and self._timing is not None
        started = self._now()
        self._timing.start_cycle(started)
        self._core.tick()
        ended = self._now()
        ages = self._core.snapshot().feedback_ages_s
        return self._timing.end_cycle(
            ended,
            feedback_ages=ages,
            feedback_timestamps=(),
            transactions=(),
            missed_slots=missed_slots,
        )

    def _ready_anchor_is_stale(self, snapshot: CoreSnapshot, now: float) -> bool:
        refs = snapshot.confirmed_references
        if not refs or any(ref is None for ref in refs):
            return True
        if any(
            age is None or age >= self.feedback_max_age_s
            for age in snapshot.feedback_ages_s
        ):
            return True
        return any(
            now - ref.sample_time_s >= self.max_reference_gap_s
            for ref in refs
            if ref is not None
        )

    def _refresh_ready_anchor(self):
        assert self._core is not None and self._coordinator is not None
        stopped = self._core.stop()
        if not stopped.accepted:
            return stopped
        with self._condition:
            connection_id = self._owner_session_id or "fixture"
        self._current_core_owner = self._core.claim_owner(connection_id)
        checking = self._core.begin_check()
        if not checking.accepted:
            return checking
        started = self._now()
        records = self._coordinator.check_step()
        self._set_stop_confirmation(records, started)
        checked = self._core.checked()
        self._input_phase = "ready" if checked.accepted else "check_failed"
        return checked

    def _handle_lifecycle_action(self, request_id: int, action: object) -> bool:
        assert self._core is not None and self._coordinator is not None
        now = self._now()
        name = self._action_name(action)
        if isinstance(action, ConnectAction):
            connected = False
            release_reservation = False
            if action.source_kind == "physical":
                if self._core.snapshot().state is not State.DISCONNECTED:
                    self._finish_receipt(request_id, name, "rejected", "INVALID_STATE", "Disconnect the fixture before physical commissioning")
                else:
                    release_reservation = True
                    valid, errors = validate_physical_draft(action.draft)
                    self._commissioning_blocked = True
                    self._connection_phase = "physical_commissioning"
                    self._commissioning_reasons = (
                        tuple(errors)
                        if not valid
                        else ("Physical connection is not implemented in this phase",)
                    )
                    self._finish_receipt(
                        request_id,
                        name,
                        "rejected",
                        "COMMISSIONING_BLOCKED" if not valid else "COMMISSIONING_REQUIRED",
                        "Physical connection remains commissioning blocked",
                    )
            elif action.source_kind != "fixture":
                self._finish_receipt(request_id, name, "rejected", "INVALID_SOURCE", "Unknown source kind")
            elif self._transport_closed or self._core.snapshot().state is not State.DISCONNECTED:
                self._finish_receipt(request_id, name, "rejected", "INVALID_STATE", "Fixture cannot connect in this state")
            else:
                release_reservation = True
                self._core.begin_connect()
                decision = self._core.connected(SourceIdentity("fixture", "arm-fixture", 1))
                if decision.accepted:
                    connected = True
                    self._current_core_owner = self._core.claim_owner(self._owner_session_id or "fixture")
                    self._per_joint_stop_confirmed = {
                        motor_id: False for motor_id in (0, 1, 2)
                    }
                    self._connection_phase = "fixture"
                    self._input_phase = "connected"
                    self._commissioning_blocked = False
                    self._commissioning_reasons = ()
                    self._finish_receipt(request_id, name, "accepted", "CONNECTED", "Connected to fixture")
                else:
                    self._finish_receipt(request_id, name, "rejected", decision.code.value, "Fixture connect rejected")
            if release_reservation and not connected:
                with self._condition:
                    self._release_owner_locked(
                        now,
                        "CONNECT_FAILED",
                        "Queued lifecycle request was revoked after Connect failed",
                    )
        elif isinstance(action, CheckAction):
            state = self._core.snapshot().state
            if state not in (State.CONNECTED, State.STOPPED, State.FAULT):
                self._finish_receipt(request_id, name, "rejected", "INVALID_STATE", "Core is not checkable")
            else:
                self._current_core_owner = self._core.claim_owner(self._owner_session_id or "fixture")
                decision = self._core.begin_check(acknowledge_fault=action.acknowledge_fault)
                if not decision.accepted:
                    self._finish_receipt(request_id, name, "rejected", decision.code.value, "Check rejected")
                else:
                    started = self._now()
                    records = self._coordinator.check_step()
                    self._set_stop_confirmation(records, started)
                    checked = self._core.checked()
                    if checked.accepted:
                        self._input_phase = "ready"
                        self._finish_receipt(request_id, name, "accepted", "READY", "Fresh mode 0 check completed")
                    else:
                        self._input_phase = "check_failed"
                        self._finish_receipt(request_id, name, "rejected", checked.code.value, "Check did not satisfy Core")
        elif isinstance(action, ArmAction):
            with self._condition:
                self._arm_transition = True
            try:
                snap = self._core.snapshot()
                if snap.state is not State.READY:
                    code, message = "NOT_READY", "Arm requires READY state"
                elif not action.support_acknowledgment:
                    code, message = "SUPPORT_ACK_REQUIRED", "Arm requires support acknowledgment"
                elif self._ready_anchor_is_stale(snap, now):
                    refreshed = self._refresh_ready_anchor()
                    if not refreshed.accepted:
                        code, message = refreshed.code.value, "Fresh Stop/check anchor was rejected"
                        snap = self._core.snapshot()
                    else:
                        snap = self._core.snapshot()
                        code, message = "", ""
                else:
                    code, message = "", ""
                if not code and any(not joint.healthy for joint in snap.joints):
                    code, message = "UNHEALTHY", "Arm requires healthy feedback"
                elif not code:
                    arm = self._core.arm(self._current_core_owner)
                    if not arm.accepted:
                        code, message = arm.code.value, "Arm rejected by Core"
                    else:
                        measured = tuple(
                            JointTarget(
                                q_rad=joint.sample.q_rad,
                                dq_rad_s=0.0,
                                tau_nm=0.0,
                                kp=SYNTHETIC_KP,
                                kd=SYNTHETIC_KD,
                            )
                            for joint in self._core.snapshot().joints
                            if joint.sample is not None
                        )
                        if len(measured) != len(self._specs):
                            code, message = "OBSERVATIONS_INCOMPLETE", "Measured hold is incomplete"
                        else:
                            decision = self._submit_targets(measured)
                            if not decision.accepted:
                                code, message = decision.code.value, "Measured hold was rejected by Core"
                            else:
                                armed_at = self._now()
                                with self._condition:
                                    stop_won = self._priority_stop_pending or self._stop_in_progress
                                    if not stop_won:
                                        self._hold_targets = measured
                                        self._holding_position = True
                                        self._hold_energized = True
                                        self._input_phase = "awaiting_neutral"
                                        self._needs_neutral_and_edge = True
                                        self._neutral_seen = False
                                        self._owner_generation += 1
                                        self._ticket = None
                                        self._pending_intent = None
                                        self._latest_intent = None
                                        self._latest_intent_expiry_s = None
                                        self._deadman_active = False
                                        self._new_deadman_edge_pending = False
                                        self._neutral_event_pending = False
                                        self._release_capture_pending = False
                                        self._release_neutral_pending = False
                                        self._input_enabled = True
                                        self._owner_deadline_s = armed_at + self.input_lease_s
                                        self._publish_ingress_locked()
                                if stop_won:
                                    code, message = "STOP_PREEMPTED", "Stop preempted Arm before input authority was granted"
                                else:
                                    self._finish_receipt(request_id, name, "accepted", "ARMED", "Measured hold admitted")
                                    return True
                if code in ("OBSERVATIONS_INCOMPLETE", "INVALID_OWNER") or (
                    code not in ("NOT_READY", "SUPPORT_ACK_REQUIRED", "UNHEALTHY", "STOP_PREEMPTED")
                    and self._core.snapshot().state in (State.ARMED, State.MOVING)
                ):
                    self._core.stop()
                    stop_now = self._now()
                    with self._condition:
                        self._request_stop_locked("arm_target_rejected", stop_now)
                self._finish_receipt(request_id, name, "rejected", code, message)
            except Exception:
                with self._condition:
                    self._request_stop_locked("arm_exception", self._last_time_s)
                raise
            finally:
                with self._condition:
                    self._arm_transition = False
                    self._publish_ingress_locked()
        elif isinstance(action, DisconnectAction):
            state = self._core.snapshot().state
            released = state is State.DISCONNECTED
            if not released:
                self._core.stop()
                started = self._now()
                records = self._coordinator.check_step()
                self._set_stop_confirmation(records, started)
                self._record_stop_transactions(records)
                released = all(self._per_joint_stop_confirmed.values())
                if released:
                    released = self._core.disconnect().accepted
            if released:
                disconnected_at = self._now()
                with self._condition:
                    self._release_owner_locked(
                        disconnected_at,
                        "DISCONNECTED",
                        "Lifecycle request was revoked by Disconnect",
                    )
                self._current_core_owner = None
                self._hold_targets = None
                self._hold_energized = False
                self._holding_position = False
                self._input_phase = "disconnected"
                self._connection_phase = "disconnected"
                self._finish_receipt(request_id, name, "accepted", "DISCONNECTED", "Disconnected from fixture")
            else:
                self._input_phase = "stop_unconfirmed"
                self._finish_receipt(
                    request_id,
                    name,
                    "rejected",
                    "STOP_UNCONFIRMED",
                    "Disconnect retained ownership because mode 0 was not confirmed for every motor",
                )
        else:
            self._finish_receipt(request_id, name, "rejected", "INVALID_ACTION", "Unsupported lifecycle request")
        return False

    def _submit_targets(self, targets: tuple[JointTarget, ...]):
        assert self._core is not None
        if self._current_core_owner is None:
            from joint_contract import Decision, DecisionCode

            return Decision(False, DecisionCode.INVALID_OWNER)
        window = self._core.issue_window(valid_for_s=self.max_window_s)
        self._next_core_sequence += 1
        target = Target(
            sequence=self._next_core_sequence,
            joints=targets,
            window=window,
            owner=self._current_core_owner,
        )
        return self._core.submit_target(target)

    def _capture_release_hold(self) -> None:
        assert self._core is not None
        now = self._now()
        refs = self._core.snapshot().confirmed_references
        if not refs or any(ref is None for ref in refs):
            with self._condition:
                self._request_stop_locked("release_without_complete_reference", now)
            return
        targets = tuple(
            JointTarget(
                q_rad=ref.q_rad,
                dq_rad_s=0.0,
                tau_nm=0.0,
                kp=SYNTHETIC_KP,
                kd=SYNTHETIC_KD,
            )
            for ref in refs
        )
        self._hold_targets = targets
        self._holding_position = True
        self._hold_energized = True
        self._input_phase = "holding"

    def _apply_intent(
        self,
        pending: tuple[int, JogIntent, InputTicket],
        fresh_edge: bool,
    ) -> None:
        request_id, intent, ticket = pending
        assert self._core is not None
        now = self._now()
        if now >= ticket.expires_at_s:
            self._finish_receipt(request_id, "JogIntent", "rejected", "TICKET_EXPIRED", "Input expired before execution")
            return
        state = self._core.snapshot().state
        if state not in (State.ARMED, State.MOVING):
            self._finish_receipt(
                request_id,
                "JogIntent",
                "rejected",
                "NOT_ARMED",
                "Station is not armed for input",
            )
            return
        if not intent.deadman:
            if all(axis == 0.0 for axis in intent.axes):
                self._neutral_seen = True
            if self._hold_targets is not None:
                self._input_phase = "holding" if self._hold_energized else "awaiting_neutral"
                decision = self._submit_targets(self._hold_targets)
                if not decision.accepted:
                    self._reject_target_and_stop(request_id, decision.code.value)
                    return
            else:
                self._input_phase = "hold_unavailable"
                self._finish_receipt(
                    request_id,
                    "JogIntent",
                    "rejected",
                    "NO_HOLD_TARGET",
                    "Released input has no confirmed hold target",
                )
                return
            self._finish_receipt(request_id, "JogIntent", "accepted", "HOLDING", "Released input keeps the confirmed goal")
            return

        if self._needs_neutral_and_edge:
            if not (self._neutral_seen and fresh_edge):
                self._input_phase = "awaiting_neutral"
                if self._hold_targets is not None:
                    self._submit_targets(self._hold_targets)
                self._finish_receipt(request_id, "JogIntent", "rejected", "NEUTRAL_REQUIRED", "A fresh neutral release and deadman edge are required")
                return
            self._needs_neutral_and_edge = False
            self._neutral_seen = False

        snapshot = self._core.snapshot()
        refs = snapshot.confirmed_references
        if not refs or any(ref is None for ref in refs):
            self._reject_target_and_stop(request_id, "REFERENCE_INCOMPLETE")
            return
        goals: list[JointTarget] = []
        with self._condition:
            speed_rad_s = self._selected_speed_rad_s
        for ref, axis in zip(refs, intent.axes):
            dt = now - ref.sample_time_s
            if dt < 0.0:
                self._reject_target_and_stop(request_id, "CLOCK_REGRESSION")
                return
            if dt == 0.0:
                self._input_phase = "awaiting_elapsed"
                self._finish_receipt(
                    request_id,
                    "JogIntent",
                    "rejected",
                    "NO_ELAPSED_TIME",
                    "Every confirmed reference needs positive elapsed time",
                )
                return
            goals.append(
                JointTarget(
                    q_rad=ref.q_rad + float(axis) * speed_rad_s * dt,
                    dq_rad_s=0.0,
                    tau_nm=0.0,
                    kp=SYNTHETIC_KP,
                    kd=SYNTHETIC_KD,
                )
            )
        targets = tuple(goals)
        decision = self._submit_targets(targets)
        if not decision.accepted:
            self._reject_target_and_stop(request_id, decision.code.value)
            return
        moving = any(axis != 0.0 for axis in intent.axes)
        self._holding_position = not moving
        if not moving:
            self._hold_targets = targets
        self._hold_energized = True
        self._input_phase = "jogging" if moving else "holding"
        self._finish_receipt(request_id, "JogIntent", "accepted", "ACCEPTED", "Core accepted the bounded input goal")

    def _reject_target_and_stop(self, request_id: int, code: str) -> None:
        assert self._core is not None
        self._finish_receipt(request_id, "JogIntent", "rejected", code, "Core rejected the requested goal")
        now = self._now()
        with self._condition:
            self._request_stop_locked("goal_rejected", now)

    def _refresh_active_goal(self) -> None:
        assert self._core is not None
        if self._core.snapshot().state not in (State.ARMED, State.MOVING):
            return
        if self._holding_position and self._hold_targets is not None:
            targets = self._hold_targets
        else:
            admitted = self._core.snapshot().admitted_target
            if admitted is None:
                return
            targets = admitted.joints
        decision = self._submit_targets(targets)
        if not decision.accepted:
            now = self._now()
            with self._condition:
                self._request_stop_locked("goal_refresh_rejected", now)

    def _publish(self) -> None:
        if self._core is None or self._timing is None:
            with self._condition:
                self._published = replace(
                    self._published,
                    input_phase=(
                        "executor_failed"
                        if self._executor_failed
                        else self._published.input_phase
                    ),
                    hold_energized=(
                        False if self._executor_failed else self._published.hold_energized
                    ),
                    executor_failed=self._executor_failed,
                    stop_pending=self._priority_stop_pending or self._stop_in_progress,
                    owner_session_id=self._owner_session_id,
                    owner_generation=self._owner_generation,
                    latest_receipt=self._latest_receipt,
                    device_epoch=self._device_epoch,
                )
            return
        core = self._core.snapshot()
        timing = self._timing.snapshot()
        if core.state is State.DISCONNECTED:
            samples = (None, None, None)
            healths = (None, None, None)
            ages = (None, None, None)
        else:
            samples = tuple(joint.sample for joint in core.joints)
            healths = tuple(joint.health for joint in core.joints)
            ages = core.feedback_ages_s
        admitted = core.admitted_target.joints if core.admitted_target is not None else None
        records = tuple(
            None if record is None else DispatchRecordView(
                record.sequence, record.target, record.dispatched_at_s
            )
            for record in core.dispatched_records
        )
        attempts = tuple(
            None if attempt is None else DispatchAttemptView(
                attempt.sequence, attempt.target, attempt.result, attempt.started_at_s
            )
            for attempt in core.dispatch_attempts
        )
        stop_logical = core.state in (
            State.STOPPED, State.FAULT, State.DISCONNECTED
        )
        with self._condition:
            self._published = ArmStationSnapshot(
                lifecycle_state=core.state.value,
                connection_phase=self._connection_phase,
                source=core.source,
                requested_axes=self._latest_intent.axes if self._latest_intent else None,
                admitted_goal=admitted,
                dispatched_reference=tuple(
                    None if record is None else record.target for record in records
                ),
                dispatch_records=records,
                dispatch_attempts=attempts,
                confirmed_reference=core.confirmed_references,
                measured_sample=samples,
                measured_health=healths,
                feedback_ages_s=ages,
                faults=core.faults,
                fault_latched=core.fault_latched,
                owner_session_id=self._owner_session_id,
                owner_generation=self._owner_generation,
                input_phase=self._input_phase,
                hold_energized=self._hold_energized,
                stop_logical=stop_logical,
                stop_confirmed=all(self._per_joint_stop_confirmed.values()),
                stop_pending=self._priority_stop_pending or self._stop_in_progress,
                per_joint_stop_confirmed=tuple(
                    self._per_joint_stop_confirmed[motor_id] for motor_id in (0, 1, 2)
                ),
                stop_attempts=tuple(
                    self._stop_attempts[motor_id] for motor_id in (0, 1, 2)
                ),
                close_attempted=self._close_attempted,
                close_confirmed=self._close_confirmed,
                close_unconfirmed=self._close_unconfirmed,
                executor_failed=self._executor_failed,
                timing_counters=timing.counters,
                timing_snapshot=timing,
                latest_receipt=self._latest_receipt,
                device_epoch=self._device_epoch,
                selected_speed_rad_s=self._selected_speed_rad_s,
                commissioning_blocked=self._commissioning_blocked,
                commissioning_reasons=self._commissioning_reasons,
            )

    def snapshot(self) -> ArmStationSnapshot:
        """Return the last immutable published view without reading Core."""
        with self._condition:
            return self._published

    def run_steps(self, count: int) -> list[LoopStepRecord]:
        if type(count) is not int or count < 1:
            raise ValueError("count must be a positive integer")
        return [self.step() for _ in range(count)]

    def start_loop(self) -> None:
        with self._condition:
            if self._execution_mode is not None or self._loop_thread is not None:
                raise RuntimeError("Station already has an executor")
            self._execution_mode = "background"
            self._loop_running.set()
            self._loop_thread = threading.Thread(
                target=self._loop_worker, daemon=True, name="arm-station-executor"
            )
            self._loop_thread.start()

    def _loop_worker(self) -> None:
        with self._condition:
            self._executor_thread_id = threading.get_ident()
        nominal = self.slot_period_s
        try:
            next_slot = self._now()
            while self._loop_running.is_set():
                now = self._now()
                if now < next_slot:
                    time.sleep(next_slot - now)
                    now = self._now()
                missed = 0
                if now > next_slot:
                    assert self._timing is not None or self._core is None
                    if self._timing is not None:
                        missed, _ = self._timing.calculate_slot_skip(now, next_slot)
                record = self._step_once(missed_slots=missed)
                if self._close_confirmed:
                    break
                if self._close_unconfirmed:
                    self._wait_for_close_retry(self._close_completed_id)
                    continue
                if self._loop_stop_requested:
                    break
                next_slot = record.started_s + nominal
        except Exception:
            self._record_executor_failure()
            self._retry_failed_close()
        finally:
            self._loop_running.clear()
            if not self._executor_failed:
                self._publish()

    def _wait_for_close_retry(self, completed_id: int) -> None:
        with self._condition:
            self._condition.wait_for(
                lambda: self._close_request_id > completed_id or self._close_confirmed,
                timeout=None,
            )

    def _record_executor_failure(self) -> None:
        self._executor_failed = True
        self._input_phase = "executor_failed"
        self._hold_energized = False
        self._holding_position = False
        with self._condition:
            fallback_at = self._last_time_s
            self._request_stop_locked("executor_failure", fallback_at)
            self._arm_transition = False
            self._lifecycle_in_progress = False
            self._priority_stop_pending = False
            self._priority_stop_reason = None
            self._stop_in_progress = True
            self._close_attempted = True
            self._close_request_id += 1
            close_request_id = self._close_request_id
            cached = self._published
            self._publish_ingress_locked()
        stop_accepted = False
        if self._core is not None:
            try:
                stop_accepted = self._core.stop().accepted
            except Exception:
                pass
        if (
            not self._transport_closed
            and self._connection_phase not in ("disconnected", "physical_commissioning")
        ):
            self._attempt_mode0_all()
        else:
            self._per_joint_stop_confirmed = {motor_id: False for motor_id in (0, 1, 2)}
            self._stop_attempts = {motor_id: None for motor_id in (0, 1, 2)}
        next_state = cached.lifecycle_state
        if stop_accepted and cached.lifecycle_state != State.DISCONNECTED.value:
            if cached.fault_latched:
                next_state = State.FAULT.value
            else:
                next_state = State.STOPPED.value
        with self._condition:
            self._stop_in_progress = False
            self._priority_stop_pending = False
            self._published = replace(
                self._published,
                lifecycle_state=next_state,
                input_phase="executor_failed",
                hold_energized=False,
                stop_logical=(
                    next_state in (State.STOPPED.value, State.FAULT.value, State.DISCONNECTED.value)
                ),
                stop_confirmed=all(self._per_joint_stop_confirmed.values()),
                per_joint_stop_confirmed=tuple(
                    self._per_joint_stop_confirmed[motor_id]
                    for motor_id in (0, 1, 2)
                ),
                stop_attempts=tuple(
                    self._stop_attempts[motor_id] for motor_id in (0, 1, 2)
                ),
                executor_failed=True,
                stop_pending=False,
                owner_session_id=self._owner_session_id,
                owner_generation=self._owner_generation,
                latest_receipt=self._latest_receipt,
            )
        try:
            self._execute_close(close_request_id)
        except Exception:
            with self._condition:
                self._close_confirmed = False
                self._close_unconfirmed = True
                self._close_completed_id = max(
                    self._close_completed_id, close_request_id
                )
                self._condition.notify_all()

    def _retry_failed_close(self) -> None:
        while self._close_unconfirmed and not self._close_confirmed:
            completed_id = self._close_completed_id
            self._wait_for_close_retry(completed_id)
            request_id = self._pending_close_id()
            if request_id is None:
                continue
            with self._condition:
                self._stop_in_progress = True
            if (
                not self._transport_closed
                and self._connection_phase not in ("disconnected", "physical_commissioning")
            ):
                if self._core is not None:
                    try:
                        self._core.stop()
                    except Exception:
                        pass
                self._attempt_mode0_all()
            with self._condition:
                self._stop_in_progress = False
                self._priority_stop_pending = False
                self._priority_stop_reason = None
                self._publish_ingress_locked()
            self._execute_close(request_id)

    def stop_loop(self) -> None:
        try:
            now = self._now()
        except Exception:
            now = self._last_time_s
        with self._condition:
            if self._execution_mode != "background":
                return
            if self._close_confirmed:
                return
            if not self._close_attempted:
                self._loop_stop_requested = True
                self._close_attempted = True
                self._close_request_id += 1
                self._request_stop_locked("loop_stop", now)
            self._condition.notify_all()
            thread = self._loop_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=BACKGROUND_CLOSE_WAIT_S)

    def close(self) -> bool:
        """Queue Stop and close on the executor; retain ownership on failure."""
        with self._condition:
            if self._close_confirmed:
                return True
        try:
            now = self._now()
        except Exception:
            now = self._last_time_s
        with self._condition:
            if self._close_confirmed:
                return True
            self._close_attempted = True
            self._close_request_id += 1
            request_id = self._close_request_id
            self._request_stop_locked("station_close", now)
            mode = self._execution_mode
            if mode == "background":
                self._condition.notify_all()
                thread = self._loop_thread
            else:
                thread = None
        if mode == "background":
            if thread is None:
                return False
            with self._condition:
                self._condition.wait_for(
                    lambda: self._close_completed_id >= request_id,
                    timeout=BACKGROUND_CLOSE_WAIT_S,
                )
                return self._close_completed_id >= request_id and self._close_confirmed
        if mode is None:
            self._bind_executor("manual")
        elif self._executor_thread_id != threading.get_ident():
            return False
        self._ensure_core()
        reason = self._take_priority_stop()
        self._execute_stop(reason or "station_close")
        self._execute_close(request_id)
        return self._close_confirmed
