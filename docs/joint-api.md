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

selected_path = '/dev/serial/by-id/<present path selected by the operator>'
report = inspect_port(PortConfig(selected_path, 6_000_000, 'u2d2'), ports=())
```

`PortConfig` is frozen and uses the shared `adapter_profile` registry. The default profile is `u2d2`. It rejects any known VID, PID, or interface that conflicts with `0403:6014`, interface `0`. The adapter serial is unknown and is never hardcoded. Missing metadata remains unverified. A matching FT232H-class candidate does not prove ROBOTIS provenance or motor communication.

Select `legacy-ft4232h` only for the historical board with serial `FTC2RQ7H`, VID:PID `0403:6011`, and interface `3`. Select `custom` for an intentionally unverified route. Custom does not bypass the known unwired interfaces `0` through `2` on the historical board.

`enumerate_ports()` reads pySerial's passive device list and returns at most 32 bounded records. It prefers a present `/dev/serial/by-id` alias only when that alias resolves to the enumerated tty. It never constructs a serial object, opens a tty, probes a motor, or writes packets. `default_port(records, adapter_profile)` returns a path only when exactly one record matches the selected profile. It returns an empty string for no match, multiple matches, or `custom`.

The CLI accepts `--adapter-profile` from that same registry. `spin_5rpm.py --list-ports` prints the passive list without requiring `--arm`. Without `--port`, the CLI uses only one unambiguous U2D2 candidate. It rejects missing or ambiguous candidates before creating a log or constructing a serial object. The U2D2 profile remains the default; the legacy route is never selected automatically by the CLI.

## Diagnostic capture

`PacketCapture` holds at most 256 records by default. Each record contains capped packet prefixes, allowlisted decoded fields, a fixed outcome code, and explicit source and connection metadata. Exported snapshots own their lists and dictionaries. Exception messages, environment values and arbitrary serial-object fields are excluded. An unavailable or regressing capture end clock exports `ended_s=None` and preserves the exchange result or original error. An invalid start clock rejects before exchange.

The GUI includes its ring in the existing JSON snapshot. The CLI enables it in `main`; direct `run(bus, args, log)` callers retain the original uncaptured behavior. Tests opt in with `fixture` provenance. A `physical` label records the caller's declaration and does not independently prove device identity.

## Cooperating bus ownership

Both real bench open sites use the same `PortConfig` for `inspect_port` and `open_managed_port`. The managed open checks the selected adapter profile before device acquisition and serial construction. A successful result owns `bus`; call `close()` and inspect its boolean result. `False` means ownership remains held because closure was not confirmed. Retry cleanup before reconnecting. A context manager raises `PortCloseError` on an otherwise normal exit with unconfirmed close; its `managed_port` remains available for cleanup.

This API opens hardware when given a motor path. Its acceptance tests use Linux PTYs exclusively. The device lease coordinates these workbench clients; it cannot exclude a raw client that ignores the lock.

When passive identity metadata is incomplete, inspection reports it as unverified and still permits device acquisition; known profile mismatches reject before construction. The serial factory must expose the pyserial `is_open` status for confirmed cleanup. A missing status retains the lease. Neither passive inspection nor opening establishes CRC-valid motor communication.

## Controller lifecycle and freshness

`JointCore` owns a fixed ordered set of unique `JointSpec` names and motor IDs. Each spec requires a frozen `PositionLimits(minimum_rad, maximum_rad)` interval in joint radians and a finite positive `max_velocity_rad_s`. The bounds must be finite real numbers excluding booleans, and the position minimum may equal the maximum. The core also requires a finite positive `max_reference_step_s`; neither limit has a default. Its states and immutable `Snapshot` are transport-free; `connected(SourceIdentity(...))` records a declaration and never opens hardware. `Decision` evaluates to its `accepted` value.

Connect, check, and arm are distinct steps. `receive(id, JointSample(...), MotorHealth(...))` stamps each valid observation with the core's monotonic clock. `checked()` needs fresh healthy Stop-mode feedback from every active joint. `arm(owner="teleop")` requires fresh healthy Stop-mode feedback, a measured position inside every configured interval, and a representable codec command for every seed. A position-limit rejection leaves the core READY without granting ownership or advancing its generation. A successful arm returns an `ArmDecision` containing the immutable `Ownership` grant and seeds the reference history from the measured joint positions at arm time. The owner must be a non-empty string. Snapshot exposes that same grant object while it is active and its `ownership_generation` watermark. The configurable maximum feedback age defaults to 0.5 seconds; this software policy is not a measured loop-rate guarantee. Snapshot `feedback_ages_s` preserves joint order and uses `None` before receipt. Timestamp zero is valid. Unknown IDs and invalid records do not refresh timestamps.

Call `tick()` to fault READY, ARMED, or MOVING when feedback ages out. Stop, the first attributed fault, and disconnect revoke any active ownership grant and clear pending work. Repeated revocation while no grant is active does not advance the generation. Disconnect/reconnect and healthy packets do not clear a fault latch or create a grant. Recovery requires `begin_check(acknowledge_fault=True)` followed by fresh healthy Stop feedback and successful `checked()`; an explicit `arm(owner=...)` is still required afterward.

## Attributed faults

`Snapshot.faults` holds immutable `JointFault` records in configured joint order. `affected_joints` derives their names. Error and warning masks accumulate by bitwise OR, and watchdog timeout and stale flags latch. Communication codes are fixed `CommunicationFailure` enum values; they do not imply an actuator watchdog timeout. Current healthy feedback clears the current communication observation while the cause remains latched. Stop and reconnect retain causes. Only an explicitly acknowledged, complete, fresh, healthy Stop check clears them. Each core stores at most one record per configured joint and a unique union of at most three fixed communication codes per record. `communication_error` is the most recent cause; `communication_errors` is a first-occurrence union, not a chronological event history. A supplied most-recent code must belong to that union.

When opening fails and cleanup cannot confirm closure, `open_managed_port` raises `PortCloseError` with `managed_port` for retry and the original error as `__cause__`. Context exit uses the same exception when closure fails, retaining any body exception as its cause. If cleanup succeeds, the original exception propagates normally.

## Latest target

`Target(sequence, joints, created_at_s, expires_at_s, ownership)` contains an immutable complete tuple of requested `JointTarget` values in configured order and the grant from an explicit arm. Both finite timestamps use the station's monotonic clock domain, and expiry must be later than creation. Submission checks lifecycle and feedback first, then complete vector shape and timestamps; a future creation returns `INVALID_ARGUMENT`, while an expired target, including deadline equality, returns `EXPIRED`. It then checks ownership by object identity, followed by every requested joint position against its closed configured interval, before mapping, codec validation, and sequence ordering. An out-of-range element returns `POSITION_LIMIT` for the whole vector; no clipping occurs. Exact endpoints pass the interval check when the existing codec also accepts their mapped values. An equal reconstructed record or a grant from another core returns `INVALID_OWNERSHIP`. Rejected replacement preserves the pending request and sequence watermark.

`consume_target()` returns a frozen `Reference(requested, joints, accepted_at_s, issued_at_s)`. `requested` preserves the accepted `Target`; `joints` contains the complete issued vector in joint units. A planning clock sample bounds each joint's position change to `max_velocity_rad_s * min(plan_time - last_issue, max_reference_step_s)`, clips desired velocity to the same signed velocity limit, and intersects the position corridor with the configured interval. The existing codec projectors select inward representable position and speed codes. The core verifies that normal command encoding preserves those exact codes and that decoded joint values remain inside the closed corridor. It builds the whole vector before changing reference history or clearing the pending request. If no code fits or any joint fails validation, it returns `None`, emits no partial issue, and retains the complete pending request for a later feasible tick. After projection, one final monotonic sample runs the freshness tick and checks active state, current ownership generation, and the exact target deadline; reaching the deadline discards the pending request while retaining its sequence, and stale feedback follows the existing fault and revocation rule. These release checks share that final sample. `Reference.issued_at_s` and history time use it as well. The candidate remains bounded by the earlier planning interval, so computation time only makes the step more conservative. An already expired request is discarded at the initial guard sample; a successful projection uses planning and final-release samples. A clock regression during calculation raises before release and leaves candidate history uncommitted. Acceptance time never resets the last issue time, so same-tick submit/consume streams can progress when the monotonic clock advances between issues, while queued and idle time are capped by `max_reference_step_s`. Stop, the first attributed fault, disconnect, and beginning a check clear pending work and reference history; a new explicit arm reseeds from fresh measurements.

These are bounded mathematical references and packet-grid checks. They do not certify physical speed, acceleration, tracking error, torque, stopping distance, or mechanical travel. The core produces no serial I/O; test-only fake adapters demonstrate packet construction. Target sequence is an exact integer from 0 through 2**64 - 1, excluding bool. `Snapshot.last_sequence` starts at -1 and advances only when a fully valid strictly newer target is accepted. Fully valid duplicate or older sequences reject with `DUPLICATE_OR_OLDER`; malformed or unrepresentable requested payloads return `INVALID_ARGUMENT` before sequence ordering. Invalid higher-numbered targets do not consume a sequence number. Stop, fault, consumption, disconnect and acknowledged recovery retain the watermark; use the new current grant after recovery to observe duplicate-sequence rejection.

### Offline controller example

```python
from joint_controller import JointCore, JointSpec, MotorHealth, PositionLimits, SourceIdentity, Target, State
from joint_mapping import JointMap, JointSample, JointTarget

now = [0.0]
core = JointCore(
    (JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-0.5, 0.5), max_velocity_rad_s=0.2),),
    clock=lambda: now[0], max_reference_step_s=0.02,
)
assert core.begin_connect()
assert core.connected(SourceIdentity("fixture", "example", 1))
assert core.begin_check()
assert core.receive(0, JointSample(0, 0, 0), MotorHealth(0, 0, 0, False))
assert core.checked()
grant = core.arm(owner="example")
assert grant and grant.ownership is not None
target = Target(
    6, (JointTarget(.1, 1, 0, 18, 4.5),),
    created_at_s=0.01, expires_at_s=1.0, ownership=grant.ownership,
)
now[0] = 0.01
assert core.submit_target(target)
reference = core.consume_target()
assert reference is not None
assert reference.requested is target
assert 0.0 < reference.joints[0].q_rad <= 0.002
assert abs(reference.joints[0].dq_rad_s) <= 0.2
assert reference.accepted_at_s == reference.issued_at_s == 0.01
assert core.snapshot().state is State.MOVING
assert core.consume_target() is None
assert core.stop()
assert core.snapshot().last_sequence == 6
assert core.snapshot().ownership is None
assert core.snapshot().ownership_generation == 2
```

This example performs no serial operations. Its limits are illustrative controller parameters. A station adapter must validate its own physical setup and own packet transmission when later milestones add one.

## Core and ownership limits

The injected core clock must be finite and monotonic. Invalid or regressing time raises before a target can be released; snapshots also require that clock contract. The default is `time.monotonic`. Sequence `2**64 - 1` exhausts that core's stream. Start a new core for a new stream; reconnecting or acknowledging a fault never resets ordering. Ownership is an in-process capability: only the exact object minted by the current core is accepted. A successful arm increments the generation watermark; revoking a live grant increments it once more. A later arm receives a fresh object and generation even when the owner string is unchanged. This local identity check is not a cryptographic credential or a remote authentication protocol.

The fixed per-user lock namespace under `/tmp` coordinates cooperating processes. A hostile local user can pre-create the name and deny access; validation fails closed. An administrator must remove a foreign-owned obstruction. Lockfiles persist to preserve flock inode identity, including after device-number reuse. Do not delete them while clients may be running. No automatic pruning or hostile-local-user availability guarantee is supplied.
