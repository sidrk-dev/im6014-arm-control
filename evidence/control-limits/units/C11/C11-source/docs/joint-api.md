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

`Target(sequence, joints, window, owner)` contains an immutable complete tuple of `JointTarget` values in configured order, a required station-issued `IntentWindow(id, created_at_s, expires_at_s)`, and the current `OwnerToken`. `JointCore.issue_window(valid_for_s)` issues the latest offered window with a monotonic ID and bounded duration (`max_window_s`, default 1.0s). `submit_target` first expires or faults an existing goal as needed, then checks the full owner token, validates the incoming window against the latest offered window, checks expiry (exact equality with expiry is expired), checks mapped output codec representability, and enforces monotonic command sequence ordering before admitting the goal. A rejected replacement keeps an existing goal only while that goal remains valid.

`consume_target` is deleted in favor of explicit refresh lifecycle: `prepare_refresh()`, `claim_dispatch(plan_id, index)`, and `dispatch_result(plan_id, index, result)`. An admitted goal is refreshed repeatedly until its window expires. Preparing a plan proposes a refresh vector without dispatching. Each `RefreshPlan` contains an exact-count tuple of immutable, validated canonical packet bytes; the adapter writes those bytes unchanged. `joint_targets` records the actual full joint reference decoded from those packets, while `target` preserves the admitted request. `claim_dispatch` verifies freshness, health, goal expiry, and ordered joint index immediately before each write. Only one claim may await an outcome; the core blocks another joint claim, plan preparation, and reconnect until the station reports that claim's actual transport result. Stop, expiry, target replacement, disconnect, or fault can invalidate the plan without discarding that in-flight receipt. `exchange` reports the write result once after the driver outcome; successful returns are deadline-checked before feedback. If the deadline expires after the pre-write hook grants a claim but before `bus.write`, it reports `WriteResult.PREWRITE_DEADLINE`: the software made no write call for that claim. This does not establish motor state or physical protection. `WriteResult.PARTIAL` means the driver returned a positive short count. `WriteResult.FAILED` means the driver raised or returned an invalid count, so byte delivery may be unknown. `Snapshot.dispatch_attempts` holds the latest outcome per joint. `dispatch_result` commits a dispatched record only for a complete write. A dispatched record's `dispatched_at_s` is the valid claim/write start time, not a fabricated completion timestamp. A complete write followed by invalid or regressing core time preserves the complete record and attempt, then faults with `CommunicationFailure.CLOCK_FAILURE` without restoring motion. `WriteResult.PREWRITE_DEADLINE` faults with `CommunicationFailure.PREWRITE_DEADLINE`; `PARTIAL` and `FAILED` fault with `DRIVER_FAILURE`, preserving any prior complete record. A late receipt for an invalidated plan is recorded but cannot restore motion or change `DISCONNECTED`; once it settles, an explicit reconnect is allowed. Reconnect still requires a new owner, acknowledged fresh healthy Stop check, and explicit arm. When an admitted goal expires, housekeeping withdraws it, invalidates pending plans, and disarms the core to `STOPPED` (latched fault preserved), requiring a new check and arm cycle. Sequence watermarks survive consumption, stop, fault, and disconnect.

### Offline controller example

```python
from joint_contract import IntentWindow, JointSpec, MotorHealth, OwnerToken, SourceIdentity, State, Target, WriteResult
from joint_controller import JointCore
from joint_mapping import JointMap, JointSample, JointTarget

clock_time = [0.0]
core = JointCore((JointSpec("J1", 0, JointMap(3, -1, .25), -2.0, 2.0, 10.0, 40.0, "fixture: software position limits"),), clock=lambda: clock_time[0], max_reference_gap_s=0.5)
assert core.begin_connect()
assert core.connected(SourceIdentity("fixture", "example", 1))
owner = core.claim_owner("example-client")
assert core.begin_check()
assert core.receive(0, JointSample(0, 0, 0), MotorHealth(0, 0, 0, False))
assert core.checked()
assert core.arm(owner)
window = core.issue_window(0.5)
target = Target(6, (JointTarget(.1, 1, -6, 18, 4.5),), window=window, owner=owner)
assert core.submit_target(target)
clock_time[0] += 0.05
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

The injected core clock must be finite and monotonic. Invalid or regressing time rejects operations that need a fresh gate; snapshots also require that clock contract. If a complete write receipt arrives after its end clock becomes invalid, the core preserves that known write and claim timestamp, then latches `CommunicationFailure.CLOCK_FAILURE` and withdraws motion until acknowledged recovery. The default clock is `time.monotonic`. Sequence `2**64 - 1` exhausts that core's stream. Start a new core for a new stream; reconnecting or acknowledging a fault never resets ordering.

### Owner generations

`JointCore` generates a fresh unpredictable random 16-byte hex epoch per instance/process via `secrets.token_hex(16)` with no persisted global state. A monotonic generation counter starts at 1 and ranges through uint64. `claim_owner(connection_id)` returns a frozen `OwnerToken(epoch, generation, connection_id)`. `connection_id` is an immutable non-empty string bounded to 64 characters and identifies the client connection; `SourceIdentity` separately identifies the motor connection. The core assumes trusted station callers; the station/server binds its actual client connection identity rather than trusting unverified JSON payload attributes, with no authentication claims beyond that boundary.

`arm(owner)` and `Target` require an exact `OwnerToken` class and matching immutable epoch, generation, and connection ID fields. Token-bearing boundary records reject subclasses. An owner mismatch returns `DecisionCode.INVALID_OWNER` after lifecycle state checks. Stop, any latched joint fault, disconnect, expired goals, and explicit takeover revoke active authority (`current_owner` becomes `None`), withdraw pending goals/plans and offered windows, and never auto-rearm. A claim during checking, ready, armed, or moving state, or a takeover of an existing owner on a connected core, places the core in `STOPPED` (while preserving `FAULT` and `DISCONNECTED`), requiring a fresh stop check and explicit arm call with the new token. Reconnecting requires claiming a fresh generation. Fault recovery also requires a new claim before checking and arming.

Durable attempt and dispatched records retain the original `OwnerToken` stamped at plan creation; an in-flight receipt completing after authority revocation preserves the actual write record with its original owner attribution without transitioning the core to `MOVING`. `Snapshot.latest_owner` retains the most recently issued token after revocation, while `Snapshot.current_owner` is `None` unless that token currently has authority. Bounded O(1) storage retains only these two tokens. When the uint64 generation space is exhausted, a new claim revokes authority, stops the core (or preserves `FAULT`/`DISCONNECTED`), and raises `OverflowError` without wrapping; start a fresh core to obtain a new epoch.

The fixed per-user lock namespace under `/tmp` coordinates cooperating processes. A hostile local user can pre-create the name and deny access; validation fails closed. An administrator must remove a foreign-owned obstruction. Lockfiles persist to preserve flock inode identity, including after device-number reuse. Do not delete them while clients may be running. No automatic pruning or hostile-local-user availability guarantee is supplied.

## Joint Position Limits

Every `JointSpec` requires explicit, immutable, finite position bounds `q_min_rad <= q_max_rad` and non-empty `limits_provenance` documenting the software threshold origin. These are fixture software limits, not physical calibration. Position requests are evaluated against inclusive bounds: exact lower and upper endpoints are accepted. Requests outside them return `DecisionCode.POSITION_OUT_OF_BOUNDS` and preserve any still-valid admitted goal, sequence watermark, owner, and window. Mapped requests must first pass the ordinary canonical codec; an unencodable mapped request returns `INVALID_ARGUMENT` and is never clamped into range. A fresh measured position outside the bounds, including during `CHECKING`, latches the attributed `JointFault.position_limit` cause and faults the core before motion. Simultaneous motor-health causes remain in the same joint fault record. `claim_dispatch` rechecks every fresh measured position and the actual prepared reference before each write. Recovery requires a new owner and an explicitly acknowledged fresh healthy Stop check with every measured position within the inclusive bounds; incoming targets cannot clear a fault. The codec owns the bounded raw position grid search through a monotonic decoded-domain callback and direction. It selects an inward representable endpoint, serializes that exact raw integer without float re-encoding, and decodes the complete canonical packet to populate actual position, velocity, feedforward, and gain reference fields. Stop packets are valid decoder inputs; mode, ID, watchdog, reserved byte, header, and CRC are checked. Repeated refreshes write the cached packet bytes unchanged.

## Speed- and acceleration-bounded trajectory reference and gap policy

Every `JointSpec` requires an explicit, finite positive maximum reference speed `v_max_rad_s` and maximum reference acceleration `a_max_rad_s2`. `JointCore` requires an explicit, finite positive `max_reference_gap_s` policy. All values describe software fixture policy unless calibrated separately.

Upon a successful healthy checked Stop, the controller initializes an unemitted trajectory reference for each joint with position equal to the checked feedback sample, zero exact velocity derivative, and timestamp equal to the sample time (`is_emitted=False`). The anchor has no cached command and cannot dispatch at $h=0$.

Preparing a refresh plan does not advance confirmed history. A `RefreshPlan` always carries one proposed reference, decoded command target, and immutable canonical packet per joint. Only a complete actual write commits that joint's proposed reference. Failed or partial writes leave its prior reference unchanged. Reference `sample_time_s` is the preparation time; `JointDispatchedRecord.dispatched_at_s` remains the actual claim/write start time.

For $h=0$, a previously emitted reference can refresh only when the admitted joint goal is unchanged. The core reuses that reference, decoded target, and exact packet bytes, including its prior position-step derivative and feedforward values; it does not encode the values again or advance their sample time. A changed goal at $h=0$ is rejected with an attributed constraint fault. If $h > \text{max\_reference\_gap\_s}$, motion is withdrawn and the controller drops to `STOPPED` (`REFERENCE_GAP_EXCEEDED`).

For $h>0$, the pure reference step intersects the speed interval $[-V,V]$, acceleration interval $[v_{\text{prev}}-A\cdot h, v_{\text{prev}}+A\cdot h]$, and position interval $[(q_{\min}-q_{\text{prev}})/h,(q_{\max}-q_{\text{prev}})/h]$. It projects the one-step objective $v_{\text{desired}} = (q_{\text{goal}}-q_{\text{prev}})/h + \dot{q}_{\text{goal}}$ into that closed intersection. The objective combines the requested position correction over this sample with velocity feedforward; it is not a continuous trajectory law.

The selected endpoint is projected inward to a representable point on the canonical codec grid. Position-step velocity is the exact rational $(q_{\text{emitted,next}}-q_{\text{emitted,prev}})/h$, where the stored finite float values and computed float difference $h$ are interpreted exactly. The independently decoded wire velocity feedforward has its own history and must satisfy both $|v|\leq V$ and $|v-v_{\text{prev,enc}}|\leq A\cdot h$. Neither stream is substituted for the other, and wire velocity is not used in position algebra. Reversal preserves previous velocity and crosses zero under the same acceleration bound.

An exactly zero objective is a stationary reference request even when position and feedforward terms cancel. For a nonzero objective, if no representable position step satisfies the closed bounds, the controller produces no mode-1 write and Stops with `DecisionCode.CONSTRAINT_VIOLATION`, attributed to each joint whose shaping failed. It does not carry hidden fractional motion or relax a bound. These are samplewise software reference limits; they do not promise a continuous motor trajectory, physical acceleration, or certified braking.
