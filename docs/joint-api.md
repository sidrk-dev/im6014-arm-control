# Joint mapping

The bench GUI and CLI use actuator output units. The mapping API converts external joint units before packet encoding. It does not open a serial port.

```python
from joint_mapping import JointMap, JointTarget, OutputSample

mapping = JointMap(reduction=3, direction=-1, zero_output_rad=0.25)
output = mapping.to_output(JointTarget(-0.5, 2, 12, 36, 9))
joint = mapping.from_output(OutputSample(output.q_rad, output.rpm, output.tau_nm))
```

This example produces an output position of 1.75 rad, speed of about -57.2958 rpm, torque of -4 Nm, Kp of 4 and Kd of 1. Reverse conversion recovers the original position, speed and torque. The resulting torque exceeds the bench encoder's ±3.5 Nm policy, so encoding it rejects the command. Mapping never clips a value to make it pass.

`JointTarget` uses rad, rad/s and Nm. `OutputTarget` uses rad, rpm and Nm. The offset is in actuator output radians. The internal motor reduction stays inside the codec. Ideal gain conversion assumes compatible stiffness and damping definitions. Friction, backlash, efficiency and physical calibration remain unmeasured.

Run a single offline acceptance check from the repository root:

```sh
.venv/bin/python -B scripts/check_milestone.py B11
```

The command prints the path to its receipt and raw logs under `evidence/runs`. The test checks ratios 1, 3 and 6 in both directions, nonzero offsets, independent numeric anchors and invalid values.

## Passive port inspection

```python
from im6014_port import PortConfig, inspect_port
from spin_5rpm import PORT

report = inspect_port(PortConfig(PORT, 6_000_000), ports=())
```

`ports=()` makes this example use only the configured by-id basename. The report names its evidence source and does not establish communication with the motor. Normal inspection can also read passive OS enumeration metadata. Known wrong serial numbers or interfaces reject with expected and observed values. Unknown metadata stays explicitly unverified.

For an alternate adapter route, set both `expected_serial=None` and `expected_channel=None`. This removes the FTDI identity assertion. It does not bypass later device ownership checks or establish motor communication. The bench GUI and CLI default to the recorded FTDI serial and interface 3.

## Diagnostic capture

`PacketCapture` holds at most 256 records by default. Each record contains capped packet prefixes, allowlisted decoded fields, a fixed outcome code, and explicit source and connection metadata. Exported snapshots own their lists and dictionaries. Exception messages, environment values and arbitrary serial-object fields are excluded. An unavailable or regressing capture end clock exports `ended_s=None` and preserves the exchange result or original error. An invalid start clock rejects before exchange.

The GUI includes its ring in the existing JSON snapshot. The CLI enables it in `main`; direct `run(bus, args, log)` callers retain the original uncaptured behavior. Tests opt in with `fixture` provenance. A `physical` label records the caller's declaration and does not independently prove device identity.

## Cooperating bus ownership

Both real bench open sites use `open_managed_port(PortConfig(...), serial_factory=...)`. It checks passive identity before device acquisition and construction. A successful result owns `bus`; call `close()` and inspect its boolean result. `False` means ownership remains held because closure was not confirmed. Retry cleanup before reconnecting. A context manager raises `PortCloseError` on an otherwise normal exit with unconfirmed close; its `managed_port` remains available for cleanup.

This API opens hardware when given a motor path. Its acceptance tests use Linux PTYs exclusively. The device lease coordinates these workbench clients; it cannot exclude a raw client that ignores the lock.

When passive identity metadata is incomplete, the default route reports it as unverified and still permits device acquisition; known mismatches reject. The serial factory must expose the pyserial `is_open` status for confirmed cleanup. A missing status retains the lease. Neither passive inspection nor opening establishes CRC-valid motor communication.

## Controller lifecycle and freshness

`JointCore` owns a fixed ordered set of unique `JointSpec` names and motor IDs. Its states and immutable `Snapshot` are transport-free; `connected(SourceIdentity(...))` records a declaration and never opens hardware. `Decision` evaluates to its `accepted` value.

Connect, check, and arm are distinct steps. `receive(id, JointSample(...), MotorHealth(...))` stamps each valid observation with the core's monotonic clock. `checked()` needs fresh healthy Stop-mode feedback from every active joint. `arm()` requires fresh healthy Stop-mode feedback. The configurable maximum age defaults to 0.5 seconds; this software policy is not a measured loop-rate guarantee. Snapshot `feedback_ages_s` preserves joint order and uses `None` before receipt. Timestamp zero is valid. Unknown IDs and invalid records do not refresh timestamps.

Call `tick()` to fault READY, ARMED, or MOVING when feedback ages out. Stop, disconnect/reconnect, and healthy packets do not clear a fault latch. Recovery requires `begin_check(acknowledge_fault=True)` followed by fresh healthy Stop feedback and successful `checked()`. The check clears observations first. Target dispatch, attributed fault records and command ordering are the next milestones.

## Attributed faults

`Snapshot.faults` holds immutable `JointFault` records in configured joint order. `affected_joints` derives their names. Error and warning masks accumulate by bitwise OR, and watchdog timeout and stale flags latch. Communication codes are fixed `CommunicationFailure` enum values; they do not imply an actuator watchdog timeout. Current healthy feedback clears the current communication observation while the cause remains latched. Stop and reconnect retain causes. Only an explicitly acknowledged, complete, fresh, healthy Stop check clears them. Each core stores at most one record per configured joint and a unique first-occurrence union of fixed communication codes; its size is bounded by the finite enum. `communication_error` is the most recent cause; `communication_errors` is not a chronological event history. A supplied most-recent code must belong to that union.

When opening fails and cleanup cannot confirm closure, `open_managed_port` raises `PortCloseError` with `managed_port` for retry and the original error as `__cause__`. Context exit uses the same exception when closure fails, retaining any body exception as its cause. If cleanup succeeds, the original exception propagates normally.

## Latest target and command expiry

`Target(sequence, joints, window)` contains an immutable complete tuple of `JointTarget` values in configured order and a required station-issued `IntentWindow(id, created_at_s, expires_at_s)`. `JointCore.issue_window(valid_for_s)` issues the latest offered window with a monotonic ID and bounded duration (`max_window_s`, default 1.0s). `submit_target` first expires or faults an existing goal as needed, then validates the incoming window against the latest offered window, checks expiry (exact equality with expiry is expired), checks mapped output codec representability, and enforces monotonic command sequence ordering before admitting the goal. A rejected replacement keeps an existing goal only while that goal remains valid.

`consume_target` is deleted in favor of explicit refresh lifecycle: `prepare_refresh()`, `claim_dispatch(plan_id, index)`, and `dispatch_result(plan_id, index, result)`. An admitted goal is refreshed repeatedly until its window expires. Preparing a plan proposes a refresh vector without dispatching. `claim_dispatch` verifies freshness, health, goal expiry, and ordered joint index immediately before each write. Only one claim may await an outcome; the core blocks another joint claim and plan preparation until it arrives. Stop, expiry, target replacement, or fault can invalidate the plan without discarding that in-flight receipt. `exchange` reports the write result once after the driver outcome; successful returns are deadline-checked before feedback. If the deadline expires after the pre-write hook grants a claim but before `bus.write`, it reports `WriteResult.PREWRITE_DEADLINE`: the software made no write call for that claim. This does not establish motor state or physical protection. `WriteResult.PARTIAL` means the driver returned a positive short count. `WriteResult.FAILED` means the driver raised or returned an invalid count, so byte delivery may be unknown. `Snapshot.dispatch_attempts` holds the latest outcome per joint. `dispatch_result` commits a dispatched record only for a complete write. A dispatched record's `dispatched_at_s` is the valid claim/write start time, not a fabricated completion timestamp. A complete write followed by invalid or regressing core time preserves the complete record and attempt, then faults with `CommunicationFailure.CLOCK_FAILURE` without restoring motion. `WriteResult.PREWRITE_DEADLINE` faults with `CommunicationFailure.PREWRITE_DEADLINE`; `PARTIAL` and `FAILED` fault with `DRIVER_FAILURE`, preserving any prior complete record. A late receipt for an invalidated plan is recorded but cannot restore motion. When an admitted goal expires, housekeeping withdraws it, invalidates pending plans, and disarms the core to `STOPPED` (latched fault preserved), requiring a new check and arm cycle. Sequence watermarks survive consumption, stop, fault, and disconnect.

### Offline controller example

```python
from joint_contract import IntentWindow, JointSpec, MotorHealth, SourceIdentity, State, Target, WriteResult
from joint_controller import JointCore
from joint_mapping import JointMap, JointSample, JointTarget

core = JointCore((JointSpec("J1", 0, JointMap(3, -1, .25)),), clock=lambda: 0.0)
assert core.begin_connect()
assert core.connected(SourceIdentity("fixture", "example", 1))
assert core.begin_check()
assert core.receive(0, JointSample(0, 0, 0), MotorHealth(0, 0, 0, False))
assert core.checked()
assert core.arm()
window = core.issue_window(0.5)
target = Target(6, (JointTarget(.1, 1, -6, 18, 4.5),), window=window)
assert core.submit_target(target)
plan = core.prepare_refresh()
assert plan is not None
assert core.claim_dispatch(plan.plan_id, 0)
assert core.dispatch_result(plan.plan_id, 0, WriteResult.COMPLETE)
assert core.snapshot().state is State.MOVING
assert core.snapshot().dispatched_records[0].sequence == 6
assert core.stop()
assert core.snapshot().last_sequence == 6
```

This example performs no serial operations. A station adapter must validate its own physical setup and own packet transmission when later milestones add one.

## Core and ownership limits

The injected core clock must be finite and monotonic. Invalid or regressing time rejects operations that need a fresh gate; snapshots also require that clock contract. If a complete write receipt arrives after its end clock becomes invalid, the core preserves that known write and claim timestamp, then latches a driver failure and withdraws motion until acknowledged recovery. The default clock is `time.monotonic`. Sequence `2**64 - 1` exhausts that core's stream. Start a new core for a new stream; reconnecting or acknowledging a fault never resets ordering. Owner generations remain C08.

The fixed per-user lock namespace under `/tmp` coordinates cooperating processes. A hostile local user can pre-create the name and deny access; validation fails closed. An administrator must remove a foreign-owned obstruction. Lockfiles persist to preserve flock inode identity, including after device-number reuse. Do not delete them while clients may be running. No automatic pruning or hostile-local-user availability guarantee is supplied.
