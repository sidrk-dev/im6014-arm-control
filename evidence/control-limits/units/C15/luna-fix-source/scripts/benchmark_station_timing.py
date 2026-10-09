#!/usr/bin/env python3
"""Standalone parent-run benchmark fixture script for station timing instrumentation.

Performs real-clock fake exchange and station step coordinator work across the configured
number of repeats (five by default) using an explicit six-joint fixture specification.
Records actual work/error counts, actual start-to-start periods, transaction durations,
completion overruns, host load, monotonic timer information, and raw timestamp traces.

This benchmark uses in-memory fake transport; actual rates reflect host process and
scheduler context, not physical RS485 bus throughput or physical motor cycle time.
"""

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import platform
import struct
import statistics
import sys
import time

# A direct file invocation puts only scripts/ on sys.path. Bootstrap the
# workbench root before importing its sibling modules.
REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from im6014_codec import crc32
from joint_contract import (
    JointSpec,
    MotorHealth,
    OwnerToken,
    ProtectionPolicy,
    SourceIdentity,
    Target,
    WriteResult,
)
from joint_controller import JointCore
from joint_mapping import JointMap, JointTarget
from station_step import StationStepCoordinator
from station_timing import StationTimingConfig, StationTimingTracker


def make_feedback_frame(
    motor_id: int,
    mode: int = 0,
    timeout: bool = False,
    driver_c: int = 25,
    winding_c: int = 30,
    voltage_v: float = 36.0,
    torque_nm: float = 0.0,
    rpm: float = 0.0,
    position_rad: float = 0.0,
    errors: int = 0,
    warnings: int = 0,
) -> bytes:
    """Generate a canonical 26-byte feedback frame with valid CRC32."""
    ratio = 38.0 / 3.0
    m = (motor_id & 0x0F) | ((mode & 0x07) << 4) | ((1 if timeout else 0) << 7)
    raw_torque = int(round(torque_nm * 2560.0 / ratio))
    raw_rpm = int(round(rpm * 64.0 * ratio / 60.0))
    raw_pos = int(round(position_rad * 32768.0 * ratio / math.tau))
    flags = (warnings & 0x07) << 13

    body = struct.pack(
        "<BbBBhhiIHBB",
        m,
        int(driver_c),
        int(winding_c),
        int(round(voltage_v * 2)),
        raw_torque,
        raw_rpm,
        raw_pos,
        errors,
        flags,
        0,
        0,
    )
    return b"\xfc\xee" + body + struct.pack("<I", crc32(body))


class InFlightFakeTransport:
    """Synchronous in-memory serial transport returning realistic feedback for each write."""

    def __init__(self, mode: int = 0) -> None:
        self.mode = mode
        self.timeout: float | None = 0.05
        self.write_timeout: float | None = 0.05
        self.in_waiting: int = 0
        self._buffer: bytearray = bytearray()
        self.write_count: int = 0
        self.write_bytes: int = 0
        self.read_count: int = 0
        self.read_bytes: int = 0
        self.reset_count: int = 0
        self.write_error_count: int = 0
        self.read_error_count: int = 0

    def reset_input_buffer(self) -> None:
        self.reset_count += 1
        self._buffer.clear()
        self.in_waiting = 0

    def write(self, data: bytes) -> int:
        self.write_count += 1
        try:
            # Extract motor_id from command header: byte 2 has mode and motor_id.
            if len(data) >= 3 and data[:2] == b"\xfe\xee":
                motor_id = data[2] & 0x0F
            else:
                motor_id = 0
            frame = make_feedback_frame(motor_id, mode=self.mode)
            self._buffer.extend(frame)
            self.in_waiting = len(self._buffer)
            self.write_bytes += len(data)
            return len(data)
        except Exception:
            self.write_error_count += 1
            raise

    def read(self, size: int) -> bytes:
        self.read_count += 1
        try:
            chunk = bytes(self._buffer[:size])
            del self._buffer[:size]
            self.in_waiting = len(self._buffer)
            self.read_bytes += len(chunk)
            return chunk
        except Exception:
            self.read_error_count += 1
            raise


def create_six_joint_specs() -> tuple[JointSpec, ...]:
    """Create explicit six-joint fixture specification with verified limits."""
    policy = ProtectionPolicy(
        driver_trip_c=85.0,
        driver_reset_c=70.0,
        winding_trip_c=105.0,
        winding_reset_c=90.0,
        continuous_effort_nm=10.0,
        continuous_dwell_s=0.5,
        recovery_dwell_s=0.25,
    )
    specs = []
    for i in range(6):
        specs.append(
            JointSpec(
                name=f"J{i + 1}",
                motor_id=i,
                mapping=JointMap(1.0, 1, 0.0),
                q_min_rad=-3.14,
                q_max_rad=3.14,
                v_max_rad_s=10.0,
                a_max_rad_s2=50.0,
                max_lead_rad=1.0,
                max_effort_nm=20.0,
                limits_provenance=f"fixture: six-joint benchmark spec J{i + 1}",
                protection=policy,
            )
        )
    return tuple(specs)


def run_one_repeat(
    repeat_index: int,
    cycles: int,
    nominal_period_s: float,
) -> dict:
    """Execute one measured repeat and retain all phase-specific evidence."""
    wall_start = datetime.now(timezone.utc).isoformat()
    clock = time.monotonic
    specs = create_six_joint_specs()
    transport = InFlightFakeTransport(mode=0)
    tracker = StationTimingTracker(
        StationTimingConfig(
            nominal_period_s=nominal_period_s,
            capacity=cycles * len(specs) + 12,
        )
    )
    core = JointCore(specs, clock=clock, max_reference_gap_s=1.0)
    coordinator = StationStepCoordinator(core, transport, tracker, clock=clock)

    phase_counts = {
        "setup_stop_queries": 0,
        "setup_transaction_records": 0,
        "mode1_cycles": 0,
        "mode1_transactions": 0,
        "mode1_transaction_records": 0,
        "shutdown_stop_queries": 0,
        "shutdown_transaction_records": 0,
    }
    phase_driver_counts: dict[str, dict[str, int]] = {}
    raw_traces: list[dict] = []
    failures: list[str] = []
    setup_complete = False
    mode1_start_counts: dict[str, int] | None = None
    shutdown_start_counts: dict[str, int] | None = None
    owner: OwnerToken | None = None

    def driver_counts() -> dict[str, int]:
        return {
            "write_calls": transport.write_count,
            "write_bytes": transport.write_bytes,
            "read_calls": transport.read_count,
            "read_bytes": transport.read_bytes,
            "input_resets": transport.reset_count,
            "write_errors": transport.write_error_count,
            "read_errors": transport.read_error_count,
        }

    def phase_delta(before: dict[str, int]) -> dict[str, int]:
        after = driver_counts()
        return {name: after[name] - before[name] for name in after}

    def require_decision(decision: object, label: str) -> None:
        if not getattr(decision, "accepted", False):
            code = getattr(getattr(decision, "code", None), "value", "unknown")
            raise RuntimeError(f"{label} rejected: {code}")

    def require_success_transactions(records: tuple, label: str) -> None:
        if len(records) != len(specs):
            raise RuntimeError(f"{label} returned {len(records)} records for {len(specs)} joints")
        for record in records:
            if (
                record.outcome != "OK"
                or record.write_result is not WriteResult.COMPLETE
                or not record.feedback_received
            ):
                raise RuntimeError(
                    f"{label} failed for {record.joint_name}: "
                    f"{record.outcome}/{getattr(record.write_result, 'value', None)}"
                )

    try:
        require_decision(core.begin_connect(), "begin_connect")
        require_decision(
            core.connected(SourceIdentity("fixture", "real-clock-benchmark", repeat_index)),
            "connected",
        )
        owner = core.claim_owner(f"bench-conn-{repeat_index}")
        require_decision(core.begin_check(), "begin_check")

        setup_start = driver_counts()
        setup_records = coordinator.check_step()
        phase_counts["setup_stop_queries"] = driver_counts()["write_calls"] - setup_start["write_calls"]
        phase_counts["setup_transaction_records"] = len(setup_records)
        phase_driver_counts["setup"] = phase_delta(setup_start)
        require_success_transactions(setup_records, "setup Stop check")
        require_decision(core.checked(), "checked")
        require_decision(core.arm(owner), "arm")
        setup_complete = True

        transport.mode = 1
        mode1_start_counts = driver_counts()
        next_slot: float | None = None

        for seq in range(1, cycles + 1):
            missed = 0
            if next_slot is not None:
                while True:
                    now = clock()
                    if now < next_slot:
                        time.sleep(next_slot - now)
                        continue
                    if now > next_slot:
                        missed, _ = tracker.calculate_slot_skip(now, next_slot)
                    break

            window = core.issue_window(valid_for_s=0.5)
            targets = tuple(
                JointTarget(
                    q_rad=0.05 * math.sin(seq * 0.1),
                    dq_rad_s=0.1,
                    tau_nm=0.0,
                    kp=5.0,
                    kd=0.5,
                )
                for _ in specs
            )
            target = Target(sequence=seq, joints=targets, window=window, owner=owner)
            require_decision(core.submit_target(target), f"submit_target sequence {seq}")

            loop_rec = coordinator.step(missed_slots=missed)
            phase_counts["mode1_cycles"] += 1
            phase_counts["mode1_transaction_records"] += len(loop_rec.transactions)
            phase_counts["mode1_transactions"] = (
                driver_counts()["write_calls"] - mode1_start_counts["write_calls"]
            )

            raw_traces.append({
                "cycle_index": loop_rec.cycle_index,
                "started_s": loop_rec.started_s,
                "ended_s": loop_rec.ended_s,
                "execution_duration_s": loop_rec.execution_duration_s,
                "actual_period_s": loop_rec.actual_period_s,
                "overrun": loop_rec.overrun,
                "overrun_s": loop_rec.overrun_s,
                "missed_slots_skipped": loop_rec.missed_slots_skipped,
                "feedback_ages_s": loop_rec.feedback_ages_s,
                "sampling_skew_s": loop_rec.sampling_skew_s,
                "transactions": [
                    {
                        "joint_index": tx.joint_index,
                        "joint_name": tx.joint_name,
                        "motor_id": tx.motor_id,
                        "plan_id": tx.plan_id,
                        "sequence": tx.sequence,
                        "owner": (
                            {
                                "generation": tx.owner.generation,
                                "connection_id": tx.owner.connection_id,
                            }
                            if tx.owner is not None
                            else None
                        ),
                        "started_s": tx.started_s,
                        "ended_s": tx.ended_s,
                        "duration_s": tx.duration_s,
                        "write_result": tx.write_result.value if tx.write_result else None,
                        "outcome": tx.outcome,
                        "feedback_received": tx.feedback_received,
                        "feedback_timestamp_s": tx.feedback_timestamp_s,
                        "feedback_age_s": tx.feedback_age_s,
                    }
                    for tx in loop_rec.transactions
                ],
            })
            require_success_transactions(loop_rec.transactions, f"mode1 cycle {seq}")
            if len(loop_rec.transactions) != len(specs):
                raise RuntimeError(f"mode1 cycle {seq} omitted joint transactions")
            # The next deadline is anchored to this actual start. A late cycle
            # does not create catch-up work or preserve stale phase drift.
            next_slot = loop_rec.started_s + nominal_period_s
            if not math.isfinite(next_slot) or next_slot <= loop_rec.started_s:
                raise ValueError("next benchmark cycle start is not representable")
    except Exception as exc:
        failures.append(f"{type(exc).__name__}: {exc}")
    finally:
        if mode1_start_counts is not None:
            phase_counts["mode1_transactions"] = (
                driver_counts()["write_calls"] - mode1_start_counts["write_calls"]
            )
            phase_driver_counts["mode1"] = phase_delta(mode1_start_counts)
        try:
            stop_decision = core.stop()
            if setup_complete:
                require_decision(stop_decision, "core.stop")
        except Exception as exc:
            failures.append(f"shutdown stop failed: {type(exc).__name__}: {exc}")

        if setup_complete:
            try:
                transport.mode = 0
                shutdown_start_counts = driver_counts()
                shutdown_records = coordinator.check_step()
                phase_counts["shutdown_stop_queries"] = (
                    driver_counts()["write_calls"] - shutdown_start_counts["write_calls"]
                )
                phase_counts["shutdown_transaction_records"] = len(shutdown_records)
                phase_driver_counts["shutdown"] = phase_delta(shutdown_start_counts)
                require_success_transactions(shutdown_records, "shutdown Stop check")
            except Exception as exc:
                failures.append(f"shutdown Stop check failed: {type(exc).__name__}: {exc}")

    wall_end = datetime.now(timezone.utc).isoformat()
    timing = tracker.snapshot()
    final_snapshot = core.snapshot()
    mode1_periods = [
        trace["actual_period_s"]
        for trace in raw_traces
        if trace["actual_period_s"] is not None
    ]
    mode1_transactions = [
        tx
        for trace in raw_traces
        for tx in trace["transactions"]
    ]
    mode1_durations = [tx["duration_s"] for tx in mode1_transactions]

    def describe(values: list[float]) -> dict[str, float | int | None]:
        if not values:
            return {"sample_count": 0, "min_s": None, "median_s": None, "max_s": None, "mean_s": None}
        return {
            "sample_count": len(values),
            "min_s": min(values),
            "median_s": statistics.median(values),
            "max_s": max(values),
            "mean_s": math.fsum(values) / len(values),
        }

    counters = timing.counters
    return {
        "repeat_index": repeat_index,
        "wall_start_utc": wall_start,
        "wall_end_utc": wall_end,
        "phase_counts": phase_counts,
        "phase_driver_counts": phase_driver_counts,
        "driver_counts_all_phases": driver_counts(),
        "final_core_state": final_snapshot.state.value,
        "final_core_faulted": final_snapshot.fault_latched,
        "counters_all_phases": {
            "cycles": counters.cycles,
            "transactions": counters.transactions,
            "successful_transactions": counters.successful_transactions,
            "failed_transactions": counters.failed_transactions,
            "timeout_transactions": counters.timeout_transactions,
            "invalid_feedback_transactions": counters.invalid_feedback_transactions,
            "write_cancelled_transactions": counters.write_cancelled_transactions,
            "prewrite_deadline_transactions": counters.prewrite_deadline_transactions,
            "driver_failure_transactions": counters.driver_failure_transactions,
            "skipped_transactions": counters.skipped_transactions,
            "clock_failure_transactions": counters.clock_failure_transactions,
            "overruns": counters.overruns,
            "missed_slots_skipped": counters.missed_slots_skipped,
        },
        "timing_statistics_all_phases": {
            "min_actual_period_s": timing.min_actual_period_s,
            "max_actual_period_s": timing.max_actual_period_s,
            "mean_actual_period_s": timing.mean_actual_period_s,
            "min_execution_duration_s": timing.min_execution_duration_s,
            "max_execution_duration_s": timing.max_execution_duration_s,
            "mean_execution_duration_s": timing.mean_execution_duration_s,
            "min_transaction_duration_s": timing.min_transaction_duration_s,
            "max_transaction_duration_s": timing.max_transaction_duration_s,
            "mean_transaction_duration_s": timing.mean_transaction_duration_s,
            "total_overrun_s": timing.total_overrun_s,
            "latest_sampling_skew_s": timing.latest_sampling_skew_s,
        },
        "mode1_timing_statistics": {
            "actual_start_period_s": describe(mode1_periods),
            "transaction_duration_s": describe(mode1_durations),
            "cycle_count": len(raw_traces),
            "overrun_count": sum(1 for trace in raw_traces if trace["overrun"]),
            "missed_slots_skipped": sum(trace["missed_slots_skipped"] for trace in raw_traces),
        },
        "failures": failures,
        "raw_traces": raw_traces,
    }


def validate_repeat_result(result: dict, cycles: int, joint_count: int = 6) -> list[str]:
    """Return observable work, outcome, lifecycle, or accounting failures."""
    failures = list(result.get("failures", ()))
    phase = result.get("phase_counts", {})
    expected = {
        "setup_stop_queries": joint_count,
        "mode1_cycles": cycles,
        "mode1_transactions": cycles * joint_count,
        "shutdown_stop_queries": joint_count,
    }
    for key, value in expected.items():
        actual = phase.get(key)
        if type(actual) is not int or actual != value:
            failures.append(f"phase {key} was {actual!r}, expected {value}")

    traces = result.get("raw_traces", ())
    if len(traces) != cycles:
        failures.append(f"raw trace count was {len(traces)}, expected {cycles}")
    for cycle_number, trace in enumerate(traces, start=1):
        txs = trace.get("transactions", ())
        if len(txs) != joint_count:
            failures.append(f"cycle {cycle_number} recorded {len(txs)} joint transactions")
        for tx in txs:
            if tx.get("outcome") != "OK":
                failures.append(f"cycle {cycle_number} {tx.get('joint_name')} outcome was not OK")
            if tx.get("write_result") != WriteResult.COMPLETE.value:
                failures.append(f"cycle {cycle_number} {tx.get('joint_name')} lacks a complete write")
            if tx.get("feedback_received") is not True:
                failures.append(f"cycle {cycle_number} {tx.get('joint_name')} lacks feedback")
            if tx.get("sequence") != cycle_number:
                failures.append(f"cycle {cycle_number} transaction sequence did not correlate")
            if type(tx.get("plan_id")) is not int or tx["plan_id"] < 0:
                failures.append(f"cycle {cycle_number} transaction lacks a plan ID")
            owner = tx.get("owner")
            if (
                not isinstance(owner, dict)
                or type(owner.get("generation")) is not int
                or not owner.get("connection_id")
            ):
                failures.append(f"cycle {cycle_number} transaction lacks owner correlation")
            timestamps = (tx.get("started_s"), tx.get("ended_s"), tx.get("duration_s"))
            if any(type(value) not in (int, float) or not math.isfinite(value) for value in timestamps):
                failures.append(f"cycle {cycle_number} transaction timestamps are invalid")
            elif timestamps[1] < timestamps[0] or timestamps[2] < 0:
                failures.append(f"cycle {cycle_number} transaction timestamps regress")

    counts = result.get("driver_counts_all_phases", {})
    if counts.get("write_calls") != (cycles + 2) * joint_count:
        failures.append("driver write-call count does not match setup, mode1, and shutdown work")
    if type(counts.get("read_calls")) is not int or counts["read_calls"] <= 0:
        failures.append("driver read-call count is missing")
    if counts.get("write_errors") != 0 or counts.get("read_errors") != 0:
        failures.append("fake driver recorded an I/O error")
    phase_driver = result.get("phase_driver_counts", {})
    for phase_name, expected_writes in (
        ("setup", joint_count),
        ("mode1", cycles * joint_count),
        ("shutdown", joint_count),
    ):
        actual_writes = phase_driver.get(phase_name, {}).get("write_calls")
        if actual_writes != expected_writes:
            failures.append(f"{phase_name} driver writes were {actual_writes!r}, expected {expected_writes}")
    if result.get("final_core_state") != "stopped" or result.get("final_core_faulted") is not False:
        failures.append("core did not finish in the healthy stopped state")
    counters = result.get("counters_all_phases", {})
    if counters.get("cycles") != cycles:
        failures.append("timing tracker cycle count does not match requested mode1 cycles")
    expected_transaction_records = (cycles + 2) * joint_count
    if counters.get("transactions") != expected_transaction_records:
        failures.append("timing tracker transaction record count does not match all phases")
    if counters.get("successful_transactions") != expected_transaction_records:
        failures.append("timing tracker success count does not match all phases")
    if counters.get("failed_transactions") != 0:
        failures.append("timing tracker recorded failed transactions")
    return failures


def _validate_cli_values(repeats: int, cycles: int, nominal_period_s: float, profile: bool) -> str | None:
    if type(repeats) is not int or not 1 <= repeats <= 100:
        return "repeats must be an integer from 1 through 100"
    if repeats < 5 and not profile:
        return "repeats must be at least 5 unless --profile is supplied"
    if type(cycles) is not int or not 2 <= cycles <= 10000:
        return "cycles must be an integer from 2 through 10000"
    if isinstance(nominal_period_s, bool) or not isinstance(nominal_period_s, (int, float)):
        return "nominal period must be finite and positive"
    try:
        period = float(nominal_period_s)
    except (OverflowError, TypeError, ValueError):
        return "nominal period must be finite and positive"
    if not math.isfinite(period) or not 0.0 < period <= 1.0:
        return "nominal period must be finite, positive, and no more than 1 second"
    if repeats * cycles > 10000:
        return "total mode1 cycles across repeats must not exceed 10000"
    if repeats * cycles * period > 60.0:
        return "total nominal scheduled duration must not exceed 60 seconds"
    return None


def _aggregate_mode1(repeats_data: list[dict]) -> dict:
    periods = [
        trace["actual_period_s"]
        for repeat in repeats_data
        for trace in repeat.get("raw_traces", ())
        if trace.get("actual_period_s") is not None
    ]
    durations = [
        tx["duration_s"]
        for repeat in repeats_data
        for trace in repeat.get("raw_traces", ())
        for tx in trace.get("transactions", ())
    ]

    def describe(values: list[float]) -> dict[str, float | int | None]:
        if not values:
            return {"sample_count": 0, "min_s": None, "median_s": None, "max_s": None, "mean_s": None}
        return {
            "sample_count": len(values),
            "min_s": min(values),
            "median_s": statistics.median(values),
            "max_s": max(values),
            "mean_s": math.fsum(values) / len(values),
        }

    return {
        "actual_start_period_s": describe(periods),
        "transaction_duration_s": describe(durations),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repeats", type=int, default=5, help="Number of benchmark repeats (default: 5; at least 5 unless --profile)")
    parser.add_argument("--cycles", type=int, default=50, help="Cycles per repeat (2..10000; default: 50)")
    parser.add_argument("--nominal-period-s", type=float, default=0.005, help="Finite nominal loop period, at most 1 second (default: 0.005)")
    parser.add_argument("--profile", action="store_true", help="Allow a single-repeat diagnostic profile")
    parser.add_argument("--evidence-root", type=Path, default=None, help="Evidence directory to store JSON receipt")
    args = parser.parse_args(argv)
    validation_error = _validate_cli_values(
        args.repeats, args.cycles, args.nominal_period_s, args.profile
    )
    if validation_error is not None:
        parser.error(validation_error)

    clock_info = time.get_clock_info("monotonic")
    try:
        host_load = os.getloadavg() if hasattr(os, "getloadavg") else None
    except OSError:
        host_load = None
    env_info = {
        "python_executable": sys.executable,
        "python_version": sys.version,
        "platform": platform.platform(),
        "clock_info": {
            "implementation": clock_info.implementation,
            "monotonic": clock_info.monotonic,
            "resolution_s": clock_info.resolution,
        },
        "host_load_avg": host_load,
    }

    print(
        "Starting real-clock fake-transport station timing benchmark: "
        f"{args.repeats} repeats of {args.cycles} cycles @ "
        f"{args.nominal_period_s * 1000:.1f}ms nominal"
    )
    print(f"Platform: {env_info['platform']}, clock resolution: {clock_info.resolution:.2e}s")

    repeats_data = []
    for repeat_index in range(1, args.repeats + 1):
        print(f"  Running repeat {repeat_index}/{args.repeats}...", end="", flush=True)
        try:
            result = run_one_repeat(repeat_index, args.cycles, args.nominal_period_s)
        except Exception as exc:
            result = {
                "repeat_index": repeat_index,
                "phase_counts": {},
                "phase_driver_counts": {},
                "driver_counts_all_phases": {},
                "counters_all_phases": {},
                "final_core_state": "unknown",
                "final_core_faulted": True,
                "raw_traces": [],
                "failures": [f"repeat raised {type(exc).__name__}: {exc}"],
            }
        repeat_failures = validate_repeat_result(result, args.cycles)
        result["validation_failures"] = repeat_failures
        repeats_data.append(result)
        mode1_stats = result.get("mode1_timing_statistics", {})
        period_stats = mode1_stats.get("actual_start_period_s", {})
        mean_period = period_stats.get("mean_s")
        if repeat_failures:
            print(f" failed: {len(repeat_failures)} accounting/outcome issue(s)")
        else:
            period_text = "n/a" if mean_period is None else f"{mean_period * 1000:.2f}ms"
            print(f" complete: {args.cycles * 6} mode1 transactions, mean actual period {period_text}")

    failed_repeats = [r["repeat_index"] for r in repeats_data if r["validation_failures"]]
    summary = {
        "benchmark": "C15-station-timing",
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "status": "failed" if failed_repeats else "passed",
        "failed_repeat_indices": failed_repeats,
        "configuration": {
            "joint_count": 6,
            "repeats": args.repeats,
            "cycles_per_repeat": args.cycles,
            "nominal_period_s": args.nominal_period_s,
            "profile": args.profile,
        },
        "limiting_latency_note": (
            "Measured over in-memory fake transport with synchronous exchange. "
            "Values reflect this host's software dispatch and scheduler context; "
            "they do not establish physical RS485 or motor-bus rates."
        ),
        "environment": env_info,
        "aggregate_mode1_actuals": _aggregate_mode1(repeats_data),
        "repeats": repeats_data,
    }

    if args.evidence_root:
        try:
            out_root = args.evidence_root.resolve()
            out_dir = out_root / f"C15-timing-benchmark-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')}"
            out_dir.mkdir(parents=True, exist_ok=False)
            receipt_path = out_dir / "timing-benchmark-receipt.json"
            receipt_path.write_text(json.dumps(summary, indent=2) + "\n")
            print(f"Benchmark receipt saved: {receipt_path}")
        except OSError as exc:
            print(f"Could not write benchmark receipt: {exc}", file=sys.stderr)
            return 1
    else:
        print(json.dumps(summary, indent=2))

    return 1 if failed_repeats else 0


if __name__ == "__main__":
    raise SystemExit(main())
