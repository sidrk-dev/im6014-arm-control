"""Station timing instrumentation and bounded measurement tracking.

Tracks actual loop start-to-start period, nominal configuration separately,
per-joint transaction start/end duration/result, completion overrun, missed
slots skipped (never burst), and per-joint observed feedback age and sampling skew.

Exact finite monotonic checks; no rounded nominal numbers reported as actuals;
fixed counter saturation behavior.
"""

from collections import deque
from collections.abc import Iterable
from dataclasses import dataclass, field
import math
import time

from joint_contract import OwnerToken, WriteResult

__all__ = (
    "DEFAULT_CAPACITY",
    "UINT64_MAX",
    "VALID_OUTCOMES",
    "JointTransactionRecord",
    "LoopStepRecord",
    "StationTimingConfig",
    "StationTimingTracker",
    "TimingCounters",
    "TimingSnapshot",
    "saturate_add",
    "validate_finite_time",
    "validate_monotonic_time",
)

UINT64_MAX = (1 << 64) - 1
DEFAULT_CAPACITY = 256

VALID_OUTCOMES = frozenset((
    "OK",
    "TIMEOUT",
    "INVALID_FEEDBACK",
    "WRITE_CANCELLED",
    "PREWRITE_DEADLINE",
    "DRIVER_FAILURE",
    "SKIPPED",
    "CLOCK_FAILURE",
))


def saturate_add(current: int, delta: int) -> int:
    """Add delta to current saturating at UINT64_MAX without wrapping."""
    if type(current) is not int or current < 0:
        raise ValueError("current counter must be a non-negative integer")
    if type(delta) is not int or delta < 0:
        raise ValueError("counter delta must be a non-negative integer")
    if current >= UINT64_MAX or delta == 0:
        return UINT64_MAX if current >= UINT64_MAX else current
    result = current + delta
    return UINT64_MAX if result > UINT64_MAX else result


def validate_finite_time(value: object, name: str = "time") -> float:
    """Validate that value is a finite, non-negative real int or float."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite real number")
    try:
        result = float(value)
    except (OverflowError, TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite real number") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite real number")
    if result < 0.0:
        raise ValueError(f"{name} must be non-negative")
    return result


def validate_monotonic_time(value: object, last_time: float | None = None, name: str = "time") -> float:
    """Validate finite non-negative time and monotonic advance against last_time."""
    result = validate_finite_time(value, name)
    if last_time is not None:
        if result < last_time:
            raise ValueError(f"{name} regressed from {last_time} to {result}")
    return result


@dataclass(frozen=True)
class StationTimingConfig:
    """Nominal station timing configuration, kept separate from actual measurements."""

    nominal_period_s: float
    capacity: int = DEFAULT_CAPACITY

    def __post_init__(self) -> None:
        if isinstance(self.nominal_period_s, bool) or not isinstance(self.nominal_period_s, (int, float)):
            raise ValueError("nominal_period_s must be a finite positive number")
        try:
            period = float(self.nominal_period_s)
        except (OverflowError, TypeError, ValueError) as exc:
            raise ValueError("nominal_period_s must be finite and positive") from exc
        if not math.isfinite(period) or period <= 0.0:
            raise ValueError("nominal_period_s must be finite and positive")
        object.__setattr__(self, "nominal_period_s", period)

        if type(self.capacity) is not int or not (1 <= self.capacity <= 65536):
            raise ValueError("capacity must be an integer between 1 and 65536")


@dataclass(frozen=True)
class JointTransactionRecord:
    """Per-joint transaction start/end duration and outcome correlation."""

    cycle_index: int
    joint_index: int
    joint_name: str
    motor_id: int
    plan_id: int | None
    sequence: int | None
    owner: OwnerToken | None
    write_result: WriteResult | None
    outcome: str
    started_s: float
    ended_s: float
    duration_s: float
    feedback_received: bool
    feedback_timestamp_s: float | None = None
    feedback_age_s: float | None = None

    def __post_init__(self) -> None:
        if type(self.cycle_index) is not int or self.cycle_index < 0:
            raise ValueError("cycle_index must be a non-negative integer")
        if type(self.joint_index) is not int or self.joint_index < 0:
            raise ValueError("joint_index must be a non-negative integer")
        if type(self.joint_name) is not str or not self.joint_name.strip():
            raise ValueError("joint_name must be a non-empty string")
        if type(self.motor_id) is not int or not (0 <= self.motor_id <= 14):
            raise ValueError("motor_id must be an integer from 0 to 14")
        if self.plan_id is not None and (type(self.plan_id) is not int or self.plan_id < 0):
            raise ValueError("plan_id must be None or a non-negative integer")
        if self.sequence is not None and (type(self.sequence) is not int or self.sequence < 0):
            raise ValueError("sequence must be None or a non-negative integer")
        if self.owner is not None and not isinstance(self.owner, OwnerToken):
            raise ValueError("owner must be None or an OwnerToken")
        if self.write_result is not None and not isinstance(self.write_result, WriteResult):
            raise ValueError("write_result must be None or a WriteResult")
        if type(self.outcome) is not str or not self.outcome.strip():
            raise ValueError("outcome must be a non-empty string")
        if self.outcome not in VALID_OUTCOMES:
            raise ValueError(f"outcome {self.outcome!r} is not in VALID_OUTCOMES")

        started = validate_finite_time(self.started_s, "started_s")
        ended = validate_finite_time(self.ended_s, "ended_s")
        if ended < started:
            raise ValueError("ended_s must be greater than or equal to started_s")
        object.__setattr__(self, "started_s", started)
        object.__setattr__(self, "ended_s", ended)

        duration = validate_finite_time(self.duration_s, "duration_s")
        expected_duration = ended - started
        if not math.isclose(duration, expected_duration, rel_tol=1e-9, abs_tol=1e-9):
            raise ValueError(f"duration_s {duration} does not match ended_s - started_s {expected_duration}")
        object.__setattr__(self, "duration_s", duration)

        if type(self.feedback_received) is not bool:
            raise ValueError("feedback_received must be a bool")

        if self.feedback_timestamp_s is not None:
            fb_ts = validate_finite_time(self.feedback_timestamp_s, "feedback_timestamp_s")
            object.__setattr__(self, "feedback_timestamp_s", fb_ts)

        if self.feedback_age_s is not None:
            fb_age = validate_finite_time(self.feedback_age_s, "feedback_age_s")
            object.__setattr__(self, "feedback_age_s", fb_age)


@dataclass(frozen=True)
class LoopStepRecord:
    """Actual loop iteration start-to-start timing, overrun, and joint records."""

    cycle_index: int
    started_s: float
    ended_s: float
    execution_duration_s: float
    actual_period_s: float | None
    overrun: bool
    overrun_s: float
    overrun_joints: tuple[str, ...]
    missed_slots_skipped: int
    feedback_ages_s: tuple[float | None, ...]
    sampling_skew_s: float | None
    transactions: tuple[JointTransactionRecord, ...]

    def __post_init__(self) -> None:
        if type(self.cycle_index) is not int or self.cycle_index < 0:
            raise ValueError("cycle_index must be a non-negative integer")

        started = validate_finite_time(self.started_s, "started_s")
        ended = validate_finite_time(self.ended_s, "ended_s")
        if ended < started:
            raise ValueError("ended_s must be greater than or equal to started_s")
        object.__setattr__(self, "started_s", started)
        object.__setattr__(self, "ended_s", ended)

        exec_dur = validate_finite_time(self.execution_duration_s, "execution_duration_s")
        expected_dur = ended - started
        if not math.isclose(exec_dur, expected_dur, rel_tol=1e-9, abs_tol=1e-9):
            raise ValueError("execution_duration_s must equal ended_s - started_s")
        object.__setattr__(self, "execution_duration_s", exec_dur)

        if self.actual_period_s is not None:
            period = validate_finite_time(self.actual_period_s, "actual_period_s")
            object.__setattr__(self, "actual_period_s", period)

        if type(self.overrun) is not bool:
            raise ValueError("overrun must be a bool")

        overrun_dur = validate_finite_time(self.overrun_s, "overrun_s")
        object.__setattr__(self, "overrun_s", overrun_dur)

        if type(self.overrun_joints) is not tuple or any(type(j) is not str for j in self.overrun_joints):
            raise ValueError("overrun_joints must be a tuple of joint name strings")

        if type(self.missed_slots_skipped) is not int or self.missed_slots_skipped < 0:
            raise ValueError("missed_slots_skipped must be a non-negative integer")

        if type(self.feedback_ages_s) is not tuple:
            raise ValueError("feedback_ages_s must be a tuple")
        for age in self.feedback_ages_s:
            if age is not None:
                validate_finite_time(age, "feedback_age")

        if self.sampling_skew_s is not None:
            skew = validate_finite_time(self.sampling_skew_s, "sampling_skew_s")
            object.__setattr__(self, "sampling_skew_s", skew)

        if type(self.transactions) is not tuple or any(not isinstance(t, JointTransactionRecord) for t in self.transactions):
            raise ValueError("transactions must be a tuple of JointTransactionRecord")


@dataclass(frozen=True)
class TimingCounters:
    """Fixed-capacity saturated cumulative event counters."""

    cycles: int
    transactions: int
    successful_transactions: int
    failed_transactions: int
    timeout_transactions: int
    invalid_feedback_transactions: int
    write_cancelled_transactions: int
    prewrite_deadline_transactions: int
    driver_failure_transactions: int
    skipped_transactions: int
    clock_failure_transactions: int
    overruns: int
    missed_slots_skipped: int

    def __post_init__(self) -> None:
        for name in (
            "cycles",
            "transactions",
            "successful_transactions",
            "failed_transactions",
            "timeout_transactions",
            "invalid_feedback_transactions",
            "write_cancelled_transactions",
            "prewrite_deadline_transactions",
            "driver_failure_transactions",
            "skipped_transactions",
            "clock_failure_transactions",
            "overruns",
            "missed_slots_skipped",
        ):
            val = getattr(self, name)
            if type(val) is not int or not (0 <= val <= UINT64_MAX):
                raise ValueError(f"{name} must be a uint64 integer")


@dataclass(frozen=True)
class TimingSnapshot:
    """Immutable copy-out snapshot of timing configuration, counters, and statistics."""

    config: StationTimingConfig
    counters: TimingCounters
    total_overrun_s: float
    last_cycle: LoopStepRecord | None
    recent_cycles: tuple[LoopStepRecord, ...]
    recent_transactions: tuple[JointTransactionRecord, ...]
    min_actual_period_s: float | None
    max_actual_period_s: float | None
    mean_actual_period_s: float | None
    min_execution_duration_s: float | None
    max_execution_duration_s: float | None
    mean_execution_duration_s: float | None
    min_transaction_duration_s: float | None
    max_transaction_duration_s: float | None
    mean_transaction_duration_s: float | None
    latest_feedback_ages_s: tuple[float | None, ...]
    latest_sampling_skew_s: float | None


class StationTimingTracker:
    """Thread-confined timing tracker with bounded records and exact monotonicity.

    Calculates actual start-to-start loop period, separate nominal configuration,
    transaction start/end duration, completion overrun attribution, missed slot
    skipping without bursting, feedback age and sampling skew.
    """

    def __init__(self, config: StationTimingConfig | float, capacity: int = DEFAULT_CAPACITY) -> None:
        if isinstance(config, (int, float)) and not isinstance(config, bool):
            self._config = StationTimingConfig(nominal_period_s=float(config), capacity=capacity)
        elif isinstance(config, StationTimingConfig):
            self._config = config
        else:
            raise ValueError("config must be StationTimingConfig or nominal period float")

        self._recent_cycles: deque[LoopStepRecord] = deque(maxlen=self._config.capacity)
        self._recent_transactions: deque[JointTransactionRecord] = deque(maxlen=self._config.capacity)

        # High-water mark of monotonic time
        self._high_water_s: float | None = None
        self._last_cycle_start_s: float | None = None
        self._current_cycle_start_s: float | None = None
        self._current_cycle_index: int = 0
        self._cycle_in_progress: bool = False

        # Cumulative counters saturating at UINT64_MAX
        self._counter_cycles: int = 0
        self._counter_transactions: int = 0
        self._counter_successful: int = 0
        self._counter_failed: int = 0
        self._counter_timeout: int = 0
        self._counter_invalid_feedback: int = 0
        self._counter_write_cancelled: int = 0
        self._counter_prewrite_deadline: int = 0
        self._counter_driver_failure: int = 0
        self._counter_skipped: int = 0
        self._counter_clock_failure: int = 0
        self._counter_overruns: int = 0
        self._counter_missed_slots: int = 0

        self._total_overrun_s: float = 0.0

        # Cached statistics over history
        self._actual_periods: list[float] = []
        self._execution_durations: list[float] = []
        self._transaction_durations: list[float] = []
        self._latest_feedback_ages: tuple[float | None, ...] = ()
        self._latest_sampling_skew: float | None = None

    @property
    def config(self) -> StationTimingConfig:
        return self._config

    def start_cycle(self, now_s: float) -> int:
        """Start a new loop cycle iteration, verifying clock monotonicity.

        Returns the assigned cycle_index.
        """
        now = validate_monotonic_time(now_s, self._high_water_s, "loop cycle start")
        self._high_water_s = now
        self._current_cycle_start_s = now
        self._cycle_in_progress = True
        return self._current_cycle_index

    def record_transaction(self, record: JointTransactionRecord) -> None:
        """Record one per-joint transaction outcome and duration."""
        if not isinstance(record, JointTransactionRecord):
            raise ValueError("record must be a JointTransactionRecord")

        # Update high-water mark if transaction end is later
        if self._high_water_s is None or record.ended_s > self._high_water_s:
            self._high_water_s = record.ended_s

        self._counter_transactions = saturate_add(self._counter_transactions, 1)

        outcome = record.outcome
        if outcome == "OK":
            self._counter_successful = saturate_add(self._counter_successful, 1)
        else:
            self._counter_failed = saturate_add(self._counter_failed, 1)

        if outcome == "TIMEOUT":
            self._counter_timeout = saturate_add(self._counter_timeout, 1)
        elif outcome == "INVALID_FEEDBACK":
            self._counter_invalid_feedback = saturate_add(self._counter_invalid_feedback, 1)
        elif outcome == "WRITE_CANCELLED":
            self._counter_write_cancelled = saturate_add(self._counter_write_cancelled, 1)
        elif outcome == "PREWRITE_DEADLINE":
            self._counter_prewrite_deadline = saturate_add(self._counter_prewrite_deadline, 1)
        elif outcome == "DRIVER_FAILURE":
            self._counter_driver_failure = saturate_add(self._counter_driver_failure, 1)
        elif outcome == "SKIPPED":
            self._counter_skipped = saturate_add(self._counter_skipped, 1)
        elif outcome == "CLOCK_FAILURE":
            self._counter_clock_failure = saturate_add(self._counter_clock_failure, 1)

        self._transaction_durations.append(record.duration_s)
        if len(self._transaction_durations) > self._config.capacity:
            self._transaction_durations.pop(0)

        self._recent_transactions.append(record)

    def calculate_overrun_attribution(
        self,
        transactions: tuple[JointTransactionRecord, ...],
        duration_s: float,
    ) -> tuple[str, ...]:
        """Attribute completion overrun to the joints exceeding their nominal time slice."""
        if duration_s <= self._config.nominal_period_s or not transactions:
            return ()
        fair_share = self._config.nominal_period_s / len(transactions)
        exceeded = [tx.joint_name for tx in transactions if tx.duration_s > fair_share]
        if exceeded:
            return tuple(exceeded)
        # If no single joint strictly exceeded fair_share, attribute to the joint(s) with max duration
        max_duration = max(tx.duration_s for tx in transactions)
        return tuple(tx.joint_name for tx in transactions if math.isclose(tx.duration_s, max_duration, rel_tol=1e-9, abs_tol=1e-9))

    def calculate_sampling_skew(self, timestamps: Iterable[float | None]) -> float | None:
        """Calculate sampling skew (max - min) across joints with valid feedback timestamps."""
        valid = [validate_finite_time(ts, "feedback timestamp") for ts in timestamps if ts is not None]
        if len(valid) < 2:
            return None
        return max(valid) - min(valid)

    def calculate_slot_skip(self, current_time_s: float, next_slot_s: float) -> tuple[int, float]:
        """Compute integer missed slots to skip and advance next_slot boundary forward.

        Never bursts or executes missed iterations.
        """
        now = validate_finite_time(current_time_s, "current_time_s")
        slot = validate_finite_time(next_slot_s, "next_slot_s")
        if now <= slot:
            return (0, slot)
        elapsed = now - slot
        missed = int(elapsed // self._config.nominal_period_s) + 1
        new_slot = slot + (missed * self._config.nominal_period_s)
        if new_slot <= now:
            missed += 1
            new_slot += self._config.nominal_period_s
        return (missed, new_slot)

    def end_cycle(
        self,
        now_s: float,
        feedback_ages: tuple[float | None, ...] = (),
        feedback_timestamps: tuple[float | None, ...] = (),
        transactions: tuple[JointTransactionRecord, ...] = (),
        missed_slots: int = 0,
    ) -> LoopStepRecord:
        """Complete the current cycle, update counters, and record actual timing."""
        if not self._cycle_in_progress or self._current_cycle_start_s is None:
            raise RuntimeError("end_cycle called without active start_cycle")

        now = validate_monotonic_time(now_s, self._current_cycle_start_s, "loop cycle end")
        self._high_water_s = now

        started_s = self._current_cycle_start_s
        exec_duration = now - started_s

        actual_period_s = (started_s - self._last_cycle_start_s) if self._last_cycle_start_s is not None else None
        if actual_period_s is not None:
            self._actual_periods.append(actual_period_s)
            if len(self._actual_periods) > self._config.capacity:
                self._actual_periods.pop(0)

        self._execution_durations.append(exec_duration)
        if len(self._execution_durations) > self._config.capacity:
            self._execution_durations.pop(0)

        overrun = exec_duration > self._config.nominal_period_s
        overrun_s = max(0.0, exec_duration - self._config.nominal_period_s)
        overrun_joints = self.calculate_overrun_attribution(transactions, exec_duration)

        if overrun:
            self._counter_overruns = saturate_add(self._counter_overruns, 1)
            self._total_overrun_s += overrun_s

        if missed_slots > 0:
            self._counter_missed_slots = saturate_add(self._counter_missed_slots, missed_slots)

        skew = self.calculate_sampling_skew(feedback_timestamps)
        self._latest_feedback_ages = feedback_ages
        self._latest_sampling_skew = skew

        record = LoopStepRecord(
            cycle_index=self._current_cycle_index,
            started_s=started_s,
            ended_s=now,
            execution_duration_s=exec_duration,
            actual_period_s=actual_period_s,
            overrun=overrun,
            overrun_s=overrun_s,
            overrun_joints=overrun_joints,
            missed_slots_skipped=missed_slots,
            feedback_ages_s=feedback_ages,
            sampling_skew_s=skew,
            transactions=transactions,
        )

        self._recent_cycles.append(record)
        self._counter_cycles = saturate_add(self._counter_cycles, 1)

        self._last_cycle_start_s = started_s
        self._current_cycle_start_s = None
        self._cycle_in_progress = False
        self._current_cycle_index = saturate_add(self._current_cycle_index, 1)

        return record

    def snapshot(self) -> TimingSnapshot:
        """Produce an immutable snapshot of all counters and bounded metrics."""
        counters = TimingCounters(
            cycles=self._counter_cycles,
            transactions=self._counter_transactions,
            successful_transactions=self._counter_successful,
            failed_transactions=self._counter_failed,
            timeout_transactions=self._counter_timeout,
            invalid_feedback_transactions=self._counter_invalid_feedback,
            write_cancelled_transactions=self._counter_write_cancelled,
            prewrite_deadline_transactions=self._counter_prewrite_deadline,
            driver_failure_transactions=self._counter_driver_failure,
            skipped_transactions=self._counter_skipped,
            clock_failure_transactions=self._counter_clock_failure,
            overruns=self._counter_overruns,
            missed_slots_skipped=self._counter_missed_slots,
        )

        min_p = min(self._actual_periods) if self._actual_periods else None
        max_p = max(self._actual_periods) if self._actual_periods else None
        mean_p = (sum(self._actual_periods) / len(self._actual_periods)) if self._actual_periods else None

        min_e = min(self._execution_durations) if self._execution_durations else None
        max_e = max(self._execution_durations) if self._execution_durations else None
        mean_e = (sum(self._execution_durations) / len(self._execution_durations)) if self._execution_durations else None

        min_t = min(self._transaction_durations) if self._transaction_durations else None
        max_t = max(self._transaction_durations) if self._transaction_durations else None
        mean_t = (sum(self._transaction_durations) / len(self._transaction_durations)) if self._transaction_durations else None

        last_cycle = self._recent_cycles[-1] if self._recent_cycles else None

        return TimingSnapshot(
            config=self._config,
            counters=counters,
            total_overrun_s=self._total_overrun_s,
            last_cycle=last_cycle,
            recent_cycles=tuple(self._recent_cycles),
            recent_transactions=tuple(self._recent_transactions),
            min_actual_period_s=min_p,
            max_actual_period_s=max_p,
            mean_actual_period_s=mean_p,
            min_execution_duration_s=min_e,
            max_execution_duration_s=max_e,
            mean_execution_duration_s=mean_e,
            min_transaction_duration_s=min_t,
            max_transaction_duration_s=max_t,
            mean_transaction_duration_s=mean_t,
            latest_feedback_ages_s=self._latest_feedback_ages,
            latest_sampling_skew_s=self._latest_sampling_skew,
        )

    def reset(self) -> None:
        """Reset all counters and stored history."""
        self._recent_cycles.clear()
        self._recent_transactions.clear()
        self._high_water_s = None
        self._last_cycle_start_s = None
        self._current_cycle_start_s = None
        self._current_cycle_index = 0
        self._cycle_in_progress = False

        self._counter_cycles = 0
        self._counter_transactions = 0
        self._counter_successful = 0
        self._counter_failed = 0
        self._counter_timeout = 0
        self._counter_invalid_feedback = 0
        self._counter_write_cancelled = 0
        self._counter_prewrite_deadline = 0
        self._counter_driver_failure = 0
        self._counter_skipped = 0
        self._counter_clock_failure = 0
        self._counter_overruns = 0
        self._counter_missed_slots = 0
        self._total_overrun_s = 0.0

        self._actual_periods.clear()
        self._execution_durations.clear()
        self._transaction_durations.clear()
        self._latest_feedback_ages = ()
        self._latest_sampling_skew = None
