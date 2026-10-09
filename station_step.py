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
from joint_contract import CommunicationFailure, Decision, MotorHealth, State, WriteResult
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
            self.core.tick()
            end_s = self._now()
            snap = self.core.snapshot()
            feedback_ages = snap.feedback_ages_s
            return self.timing.end_cycle(
                end_s,
                feedback_ages=feedback_ages,
                # Skew describes samples received during this cycle; an idle
                # cycle has no current-cycle receive timestamps.
                feedback_timestamps=(),
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
            dispatch_decision_holder: list[Decision | None] = [None]
            claim_clock_failure = [False]

            def before_write(bound_plan_id: int = plan_id, bound_index: int = index) -> bool:
                # A failed core clock is a station fault, not a revoked-owner cancellation.
                try:
                    return self.core.claim_dispatch(bound_plan_id, bound_index)
                except Exception:
                    claim_clock_failure[0] = True
                    raise

            def on_write(
                result: WriteResult,
                bound_plan_id: int = plan_id,
                bound_index: int = index,
            ) -> None:
                write_result_holder[0] = result
                dispatch_decision_holder[0] = self.core.dispatch_result(
                    bound_plan_id, bound_index, result
                )

            outcome = "OK"
            feedback_received = False
            feedback_ts: float | None = None
            tx_feedback_age: float | None = None

            try:
                reply = exchange(
                    self.bus,
                    packet,
                    motor_id,
                    deadline_s=self.deadline_s,
                    before_write=before_write,
                    on_write=on_write,
                )
                dispatch_decision = dispatch_decision_holder[0]
                if dispatch_decision is not None and not dispatch_decision.accepted:
                    outcome = "CLOCK_FAILURE"
                    aborted = True
                else:
                    sample = mapped_sample(spec.mapping, reply)
                    health = mapped_health(reply)
                    try:
                        self.core.receive(motor_id, sample, health)
                        snapshot = self.core.snapshot()
                    except Exception:
                        outcome = "CLOCK_FAILURE"
                        self.core.communication_failed(
                            motor_id, CommunicationFailure.CLOCK_FAILURE
                        )
                        aborted = True
                    else:
                        joint = snapshot.joints[index]
                        feedback_received = joint.valid
                        feedback_ts = joint.received_at if joint.valid else None
                        tx_feedback_age = (
                            snapshot.feedback_ages_s[index] if joint.valid else None
                        )
            except WriteCancelledError:
                outcome = "WRITE_CANCELLED"
                aborted = True
            except InvalidFeedbackError:
                outcome = "INVALID_FEEDBACK"
                aborted = True
                self.core.communication_failed(
                    motor_id, CommunicationFailure.INVALID_FEEDBACK
                )
            except TransactionTimeoutError:
                if write_result_holder[0] is WriteResult.PREWRITE_DEADLINE:
                    outcome = "PREWRITE_DEADLINE"
                elif write_result_holder[0] in (WriteResult.PARTIAL, WriteResult.FAILED):
                    outcome = "DRIVER_FAILURE"
                else:
                    outcome = "TIMEOUT"
                    self.core.communication_failed(motor_id, CommunicationFailure.TIMEOUT)
                aborted = True
            except Exception:
                if claim_clock_failure[0]:
                    outcome = "CLOCK_FAILURE"
                    self.core.communication_failed(
                        motor_id, CommunicationFailure.CLOCK_FAILURE
                    )
                elif write_result_holder[0] is WriteResult.PREWRITE_DEADLINE:
                    outcome = "PREWRITE_DEADLINE"
                elif write_result_holder[0] in (WriteResult.PARTIAL, WriteResult.FAILED):
                    outcome = "DRIVER_FAILURE"
                    if dispatch_decision_holder[0] is None:
                        self.core.communication_failed(
                            motor_id, CommunicationFailure.DRIVER_FAILURE
                        )
                elif (
                    write_result_holder[0] is WriteResult.COMPLETE
                    and dispatch_decision_holder[0] is not None
                    and not dispatch_decision_holder[0].accepted
                ):
                    outcome = "CLOCK_FAILURE"
                else:
                    outcome = "DRIVER_FAILURE"
                    self.core.communication_failed(
                        motor_id, CommunicationFailure.DRIVER_FAILURE
                    )
                aborted = True

            if outcome != "OK":
                self.core.stop()

            tx_end_s = self._now()
            tx_duration = tx_end_s - tx_start_s

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
        feedback_timestamps = tuple(
            tx.feedback_timestamp_s
            for tx in tx_records
            if tx.feedback_received and tx.feedback_timestamp_s is not None
        )

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
            tx_feedback_age = None
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
                try:
                    self.core.receive(motor_id, sample, health)
                    snapshot = self.core.snapshot()
                except Exception:
                    outcome = "CLOCK_FAILURE"
                    self.core.communication_failed(
                        motor_id, CommunicationFailure.CLOCK_FAILURE
                    )
                else:
                    joint = snapshot.joints[index]
                    feedback_received = joint.valid
                    feedback_ts = joint.received_at if joint.valid else None
                    tx_feedback_age = (
                        snapshot.feedback_ages_s[index] if joint.valid else None
                    )
            except InvalidFeedbackError:
                outcome = "INVALID_FEEDBACK"
                self.core.communication_failed(
                    motor_id, CommunicationFailure.INVALID_FEEDBACK
                )
            except TransactionTimeoutError:
                if write_result is WriteResult.PREWRITE_DEADLINE:
                    outcome = "PREWRITE_DEADLINE"
                    self.core.communication_failed(
                        motor_id, CommunicationFailure.PREWRITE_DEADLINE
                    )
                elif write_result in (WriteResult.PARTIAL, WriteResult.FAILED):
                    outcome = "DRIVER_FAILURE"
                    self.core.communication_failed(
                        motor_id, CommunicationFailure.DRIVER_FAILURE
                    )
                else:
                    outcome = "TIMEOUT"
                    self.core.communication_failed(motor_id, CommunicationFailure.TIMEOUT)
            except Exception:
                outcome = "DRIVER_FAILURE"
                self.core.communication_failed(
                    motor_id, CommunicationFailure.DRIVER_FAILURE
                )

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
                feedback_age_s=tx_feedback_age,
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
        sleep = time.sleep if sleep_fn is None else sleep_fn

        for _ in range(count):
            missed = 0
            while True:
                t_current = self._now()
                if t_current < next_slot:
                    sleep(next_slot - t_current)
                    continue
                if t_current > next_slot:
                    missed, _ = self.timing.calculate_slot_skip(t_current, next_slot)
                break

            record = self.step(missed_slots=missed)
            records.append(record)
            next_slot = record.started_s + nominal
            if not math.isfinite(next_slot) or next_slot <= record.started_s:
                raise ValueError("next cycle start is not representable as a future timestamp")

        return records
