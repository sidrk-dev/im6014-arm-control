"""Synchronous station step adapter and loop coordinator.

Executes canonical driver exchange with before_write/on_write hooks,
correlates actual outcomes, and feeds measurements to StationTimingTracker.
Transport acquisition stays outside; no port ownership or hardware I/O here.
"""

from collections.abc import Callable
import math
import time

from im6014_codec import command
from im6014_serial import (
    InvalidFeedbackError,
    TransactionTimeoutError,
    WriteCancelledError,
    exchange,
)
from joint_contract import MotorHealth, State, WriteResult
from joint_controller import JointCore
from joint_mapping import JointMap, JointSample, OutputSample
from station_timing import (
    JointTransactionRecord,
    LoopStepRecord,
    StationTimingTracker,
    validate_finite_time,
)

__all__ = (
    "StationStepCoordinator",
    "mapped_health",
    "mapped_sample",
)


def mapped_sample(mapping: JointMap, reply: dict) -> JointSample:
    """Convert decoded wire feedback dictionary to a JointSample using joint mapping."""
    output = OutputSample(
        q_rad=reply["position_rad"],
        rpm=reply["rpm"],
        tau_nm=reply["torque_nm"],
    )
    return mapping.from_output(output)


def mapped_health(reply: dict) -> MotorHealth:
    """Convert decoded wire feedback dictionary to an immutable MotorHealth record."""
    return MotorHealth(
        mode=reply["mode"],
        errors=reply["errors"],
        warnings=reply["warnings"],
        timeout=bool(reply["timeout"]),
        driver_c=float(reply["driver_c"]),
        winding_c=float(reply["winding_c"]),
    )


class StationStepCoordinator:
    """Synchronous single-writer coordinator connecting JointCore, transport, and timing.

    Exercises the canonical driver exchange before_write and on_write lifecycle,
    distinguishes all failure modes without storing raw exception text, and
    maintains bounded timing records.
    """

    def __init__(
        self,
        core: JointCore,
        bus: object,
        timing: StationTimingTracker,
        *,
        clock: Callable[[], float] | None = None,
        deadline_s: float = 0.05,
    ) -> None:
        if not isinstance(core, JointCore):
            raise ValueError("core must be a JointCore instance")
        if bus is None:
            raise ValueError("bus must be a valid transport object")
        if not isinstance(timing, StationTimingTracker):
            raise ValueError("timing must be a StationTimingTracker instance")

        self.core = core
        self.bus = bus
        self.timing = timing
        self.deadline_s = validate_finite_time(deadline_s, "deadline_s")

        if clock is not None:
            if not callable(clock):
                raise ValueError("clock must be callable")
            self._clock = clock
        else:
            self._clock = getattr(core, "_clock", time.monotonic)

    def _now(self) -> float:
        val = self._clock()
        return validate_finite_time(val, "clock")

    def step(self, *, missed_slots: int = 0) -> LoopStepRecord:
        """Execute one complete station refresh cycle.

        Prepares a core refresh plan, coordinates per-joint write claims and exchanges,
        updates core observations, and logs actual timing records.
        """
        start_s = self._now()
        cycle_idx = self.timing.start_cycle(start_s)

        plan = self.core.prepare_refresh()
        if plan is None:
            # No refresh plan prepared (idle, stopped, faulted, or checking).
            # Tick core to maintain feedback freshness watchdog and fault checks.
            try:
                self.core.tick()
            except Exception:
                pass
            end_s = self._now()
            snap = self.core.snapshot()
            feedback_ages = snap.feedback_ages_s
            feedback_timestamps = tuple(j.received_at for j in snap.joints)
            return self.timing.end_cycle(
                end_s,
                feedback_ages=feedback_ages,
                feedback_timestamps=feedback_timestamps,
                transactions=(),
                missed_slots=missed_slots,
            )

        plan_id = plan.plan_id
        sequence = plan.target.sequence
        owner = plan.target.owner
        packets = plan.packets

        tx_records: list[JointTransactionRecord] = []
        aborted = False

        for index, packet in enumerate(packets):
            spec = self.core.joint_specs[index]
            motor_id = spec.motor_id
            joint_name = spec.name

            if aborted:
                # Mode-1 writes aborted after prior failure; record skipped joint
                tx_time = self._now()
                tx = JointTransactionRecord(
                    cycle_index=cycle_idx,
                    joint_index=index,
                    joint_name=joint_name,
                    motor_id=motor_id,
                    plan_id=plan_id,
                    sequence=sequence,
                    owner=owner,
                    write_result=None,
                    outcome="SKIPPED",
                    started_s=tx_time,
                    ended_s=tx_time,
                    duration_s=0.0,
                    feedback_received=False,
                    feedback_timestamp_s=None,
                    feedback_age_s=None,
                )
                self.timing.record_transaction(tx)
                tx_records.append(tx)
                continue

            tx_start_s = self._now()
            write_result_holder: list[WriteResult | None] = [None]

            def before_write() -> bool:
                # Core write gate: claim dispatch immediately before transmission
                try:
                    return bool(self.core.claim_dispatch(plan_id, index))
                except Exception:
                    return False

            def on_write(result: WriteResult) -> None:
                write_result_holder[0] = result
                try:
                    self.core.dispatch_result(plan_id, index, result)
                except Exception:
                    pass

            outcome = "OK"
            feedback_received = False
            feedback_ts: float | None = None

            try:
                reply = exchange(
                    self.bus,
                    packet,
                    motor_id,
                    deadline_s=self.deadline_s,
                    before_write=before_write,
                    on_write=on_write,
                )
                sample = mapped_sample(spec.mapping, reply)
                health = mapped_health(reply)
                self.core.receive(motor_id, sample, health)
                feedback_received = True
                snap_joint = self.core.snapshot().joints[index]
                feedback_ts = snap_joint.received_at
            except WriteCancelledError:
                outcome = "WRITE_CANCELLED"
                aborted = True
                try:
                    self.core.stop()
                except Exception:
                    pass
            except InvalidFeedbackError:
                outcome = "INVALID_FEEDBACK"
                aborted = True
                try:
                    self.core.stop()
                except Exception:
                    pass
            except TransactionTimeoutError:
                if write_result_holder[0] is WriteResult.PREWRITE_DEADLINE:
                    outcome = "PREWRITE_DEADLINE"
                else:
                    outcome = "TIMEOUT"
                aborted = True
                try:
                    self.core.stop()
                except Exception:
                    pass
            except Exception:
                outcome = "DRIVER_FAILURE"
                aborted = True
                try:
                    self.core.stop()
                except Exception:
                    pass

            tx_end_s = self._now()
            tx_duration = tx_end_s - tx_start_s
            tx_feedback_age = (tx_end_s - feedback_ts) if feedback_ts is not None else None

            tx = JointTransactionRecord(
                cycle_index=cycle_idx,
                joint_index=index,
                joint_name=joint_name,
                motor_id=motor_id,
                plan_id=plan_id,
                sequence=sequence,
                owner=owner,
                write_result=write_result_holder[0],
                outcome=outcome,
                started_s=tx_start_s,
                ended_s=tx_end_s,
                duration_s=tx_duration,
                feedback_received=feedback_received,
                feedback_timestamp_s=feedback_ts,
                feedback_age_s=tx_feedback_age,
            )
            self.timing.record_transaction(tx)
            tx_records.append(tx)

        cycle_end_s = self._now()
        snap = self.core.snapshot()
        feedback_ages = snap.feedback_ages_s
        feedback_timestamps = tuple(j.received_at for j in snap.joints)

        return self.timing.end_cycle(
            cycle_end_s,
            feedback_ages=feedback_ages,
            feedback_timestamps=feedback_timestamps,
            transactions=tuple(tx_records),
            missed_slots=missed_slots,
        )

    def check_step(self, *, deadline_s: float | None = None) -> tuple[JointTransactionRecord, ...]:
        """Execute one round of Stop feedback queries to check all configured motors."""
        deadline = self.deadline_s if deadline_s is None else validate_finite_time(deadline_s, "deadline_s")
        tx_records: list[JointTransactionRecord] = []
        cycle_idx = self.timing.snapshot().counters.cycles

        for index, spec in enumerate(self.core.joint_specs):
            motor_id = spec.motor_id
            stop_packet = command(motor_id, mode=0)
            t_start = self._now()
            outcome = "OK"
            feedback_received = False
            feedback_ts = None
            write_result: WriteResult | None = None

            def on_write(result: WriteResult) -> None:
                nonlocal write_result
                write_result = result

            try:
                reply = exchange(
                    self.bus,
                    stop_packet,
                    motor_id,
                    deadline_s=deadline,
                    on_write=on_write,
                )
                sample = mapped_sample(spec.mapping, reply)
                health = mapped_health(reply)
                self.core.receive(motor_id, sample, health)
                feedback_received = True
                snap_joint = self.core.snapshot().joints[index]
                feedback_ts = snap_joint.received_at
            except InvalidFeedbackError:
                outcome = "INVALID_FEEDBACK"
            except TransactionTimeoutError:
                outcome = "TIMEOUT"
            except Exception:
                outcome = "DRIVER_FAILURE"

            t_end = self._now()
            tx = JointTransactionRecord(
                cycle_index=cycle_idx,
                joint_index=index,
                joint_name=spec.name,
                motor_id=motor_id,
                plan_id=None,
                sequence=None,
                owner=None,
                write_result=write_result,
                outcome=outcome,
                started_s=t_start,
                ended_s=t_end,
                duration_s=t_end - t_start,
                feedback_received=feedback_received,
                feedback_timestamp_s=feedback_ts,
                feedback_age_s=(t_end - feedback_ts) if feedback_ts is not None else None,
            )
            self.timing.record_transaction(tx)
            tx_records.append(tx)

        return tuple(tx_records)

    def run_steps(
        self,
        count: int,
        *,
        sleep_fn: Callable[[float], None] | None = None,
    ) -> list[LoopStepRecord]:
        """Run count station steps pacing by nominal period and skipping missed slots.

        Never bursts or executes back-to-back iterations to catch up on missed slots.
        """
        if type(count) is not int or count < 1:
            raise ValueError("count must be a positive integer")

        records: list[LoopStepRecord] = []
        nominal = self.timing.config.nominal_period_s
        next_slot = self._now()

        for _ in range(count):
            t_current = self._now()
            missed = 0
            if t_current < next_slot:
                if sleep_fn is not None:
                    sleep_dur = next_slot - t_current
                    if sleep_dur > 0:
                        sleep_fn(sleep_dur)
            else:
                # Overrun past scheduled slot: calculate missed slots and advance
                if next_slot < t_current:
                    missed, next_slot = self.timing.calculate_slot_skip(t_current, next_slot)

            record = self.step(missed_slots=missed)
            records.append(record)
            next_slot += nominal

        return records
