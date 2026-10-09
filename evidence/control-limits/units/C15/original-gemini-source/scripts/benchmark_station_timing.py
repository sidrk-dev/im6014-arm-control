#!/usr/bin/env python3
"""Standalone parent-run benchmark fixture script for station timing instrumentation.

Performs real-clock fake exchange and station step coordinator work across FIVE repeats
using an explicit six-joint fixture specification.
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
import sys
import time

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

    def reset_input_buffer(self) -> None:
        self._buffer.clear()
        self.in_waiting = 0

    def write(self, data: bytes) -> int:
        self.write_count += 1
        # Extract motor_id from command header: byte 2 has mode and motor_id
        if len(data) >= 3 and data[:2] == b"\xfe\xee":
            motor_id = data[2] & 0x0F
        else:
            motor_id = 0
        frame = make_feedback_frame(motor_id, mode=self.mode)
        self._buffer.extend(frame)
        self.in_waiting = len(self._buffer)
        return len(data)

    def read(self, size: int) -> bytes:
        chunk = bytes(self._buffer[:size])
        del self._buffer[:size]
        self.in_waiting = len(self._buffer)
        return chunk


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
    """Execute one benchmark run using real-clock fake exchange and step coordinator."""
    wall_start = datetime.now(timezone.utc).isoformat()
    clock = time.monotonic
    specs = create_six_joint_specs()
    transport = InFlightFakeTransport(mode=0)

    config = StationTimingConfig(nominal_period_s=nominal_period_s, capacity=cycles + 10)
    tracker = StationTimingTracker(config)

    core = JointCore(specs, clock=clock, max_reference_gap_s=1.0)
    coordinator = StationStepCoordinator(core, transport, tracker, clock=clock)

    # 1. Lifecycle initialization
    core.begin_connect()
    core.connected(SourceIdentity("fixture", "real-clock-benchmark", repeat_index))
    owner = core.claim_owner(f"bench-conn-{repeat_index}")
    core.begin_check()
    coordinator.check_step()
    core.checked()
    core.arm(owner)

    # Transition fake transport to FOC (mode 1) responses
    transport.mode = 1

    # 2. Execute paced cycles
    raw_traces = []
    next_slot = clock()

    for seq in range(1, cycles + 1):
        t_now = clock()
        missed = 0
        if t_now < next_slot:
            sleep_dur = next_slot - t_now
            if sleep_dur > 0:
                time.sleep(sleep_dur)
        else:
            if next_slot < t_now:
                missed, next_slot = tracker.calculate_slot_skip(t_now, next_slot)

        # Submit target with fresh intent window
        window = core.issue_window(valid_for_s=0.5)
        targets = tuple(
            JointTarget(q_rad=0.05 * math.sin(seq * 0.1), dq_rad_s=0.1, tau_nm=0.0, kp=5.0, kd=0.5)
            for _ in range(len(specs))
        )
        target = Target(sequence=seq, joints=targets, window=window, owner=owner)
        core.submit_target(target)

        # Coordinate synchronous step
        loop_rec = coordinator.step(missed_slots=missed)

        raw_traces.append({
            "cycle_index": loop_rec.cycle_index,
            "started_s": loop_rec.started_s,
            "ended_s": loop_rec.ended_s,
            "execution_duration_s": loop_rec.execution_duration_s,
            "actual_period_s": loop_rec.actual_period_s,
            "overrun": loop_rec.overrun,
            "overrun_s": loop_rec.overrun_s,
            "missed_slots_skipped": loop_rec.missed_slots_skipped,
            "transactions": [
                {
                    "joint_name": tx.joint_name,
                    "motor_id": tx.motor_id,
                    "started_s": tx.started_s,
                    "ended_s": tx.ended_s,
                    "duration_s": tx.duration_s,
                    "write_result": str(tx.write_result) if tx.write_result else None,
                    "outcome": tx.outcome,
                }
                for tx in loop_rec.transactions
            ],
        })

        next_slot += nominal_period_s

    # Stop core after execution
    core.stop()
    wall_end = datetime.now(timezone.utc).isoformat()
    snap = tracker.snapshot()

    return {
        "repeat_index": repeat_index,
        "wall_start_utc": wall_start,
        "wall_end_utc": wall_end,
        "counters": {
            "cycles": snap.counters.cycles,
            "transactions": snap.counters.transactions,
            "successful_transactions": snap.counters.successful_transactions,
            "failed_transactions": snap.counters.failed_transactions,
            "timeout_transactions": snap.counters.timeout_transactions,
            "invalid_feedback_transactions": snap.counters.invalid_feedback_transactions,
            "write_cancelled_transactions": snap.counters.write_cancelled_transactions,
            "prewrite_deadline_transactions": snap.counters.prewrite_deadline_transactions,
            "driver_failure_transactions": snap.counters.driver_failure_transactions,
            "skipped_transactions": snap.counters.skipped_transactions,
            "clock_failure_transactions": snap.counters.clock_failure_transactions,
            "overruns": snap.counters.overruns,
            "missed_slots_skipped": snap.counters.missed_slots_skipped,
        },
        "timing_statistics": {
            "min_actual_period_s": snap.min_actual_period_s,
            "max_actual_period_s": snap.max_actual_period_s,
            "mean_actual_period_s": snap.mean_actual_period_s,
            "min_execution_duration_s": snap.min_execution_duration_s,
            "max_execution_duration_s": snap.max_execution_duration_s,
            "mean_execution_duration_s": snap.mean_execution_duration_s,
            "min_transaction_duration_s": snap.min_transaction_duration_s,
            "max_transaction_duration_s": snap.max_transaction_duration_s,
            "mean_transaction_duration_s": snap.mean_transaction_duration_s,
            "total_overrun_s": snap.total_overrun_s,
            "latest_sampling_skew_s": snap.latest_sampling_skew_s,
        },
        "raw_traces": raw_traces,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repeats", type=int, default=5, help="Number of benchmark repeats (default: 5)")
    parser.add_argument("--cycles", type=int, default=50, help="Cycles per repeat (default: 50)")
    parser.add_argument("--nominal-period-s", type=float, default=0.005, help="Nominal loop period in seconds (default: 0.005)")
    parser.add_argument("--evidence-root", type=Path, default=None, help="Evidence directory to store JSON receipt")
    args = parser.parse_args()

    repo = Path(__file__).resolve().parent.parent

    # Environmental snapshot
    clock_info = time.get_clock_info("monotonic")
    host_load = os.getloadavg() if hasattr(os, "getloadavg") else None
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

    print(f"Starting real-clock station timing benchmark: {args.repeats} repeats of {args.cycles} cycles @ {args.nominal_period_s * 1000:.1f}ms nominal")
    print(f"Platform: {env_info['platform']}, clock resolution: {clock_info.resolution:.2e}s")

    repeats_data = []
    for r in range(1, args.repeats + 1):
        print(f"  Running repeat {r}/{args.repeats}...", end="", flush=True)
        res = run_one_repeat(r, args.cycles, args.nominal_period_s)
        repeats_data.append(res)
        stats = res["timing_statistics"]
        cnts = res["counters"]
        print(f" completed: {cnts['successful_transactions']} tx OK, mean period {stats['mean_actual_period_s'] * 1000:.2f}ms, overruns: {cnts['overruns']}")

    summary = {
        "benchmark": "C15-station-timing",
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "configuration": {
            "joint_count": 6,
            "repeats": args.repeats,
            "cycles_per_repeat": args.cycles,
            "nominal_period_s": args.nominal_period_s,
        },
        "limiting_latency_note": (
            "Actual timing is measured over in-memory FakeTransport with synchronous exchange. "
            "It reflects host software dispatch, serialization, and scheduler jitter. "
            "It does NOT establish physical RS485 wire throughput or physical 1-2ms motor bus rates."
        ),
        "environment": env_info,
        "repeats": repeats_data,
    }

    if args.evidence_root:
        out_root = args.evidence_root.resolve()
        out_dir = out_root / f"C15-timing-benchmark-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')}"
        out_dir.mkdir(parents=True, exist_ok=True)
        receipt_path = out_dir / "timing-benchmark-receipt.json"
        receipt_path.write_text(json.dumps(summary, indent=2) + "\n")
        print(f"Benchmark receipt saved: {receipt_path}")
    else:
        print(json.dumps({k: v for k, v in summary.items() if k != "repeats"}, indent=2))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
