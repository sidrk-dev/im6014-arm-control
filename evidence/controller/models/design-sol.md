# Candidate: one managed port boundary and a typed joint request core

## Problem

B11-B14 add joint units and inspectable communications. C01-C06 add ownership and request semantics without permitting new live motion. Preserve the GUI/CLI bench, codec bytes, internal reduction, and startup interception. The grounding traces the two open sites and the current single-worker bench; neither is the future independent station. Relevant contracts are [codec](/home/sid/Documents/code/im6014Workbench/im6014_codec.py:23), [session](/home/sid/Documents/code/im6014Workbench/motor_session.py:186), [CLI](/home/sid/Documents/code/im6014Workbench/spin_5rpm.py:110), and [plan](/home/sid/Documents/code/im6014Workbench/docs/ROBOT_WORKBENCH_AGENT_PROMPT.md:208).

## Usage

The examples describe proposed interfaces. They are not executable additions.

```python
mapping = JointMap(reduction=3.0, direction=-1, zero_output_rad=0.12)
output = mapping.to_output(JointTarget(q_rad=.2, dq_rad_s=.1,
                                     tau_nm=.3, kp=2., kd=.5))
joint = mapping.from_output(OutputSample(q_rad=.4, rpm=2., tau_nm=.1))
```

Both bench open sites use the same managed boundary. They pass their actual module lookup at call time, so mocks still intercept construction.

```python
with open_managed_port(config, serial_factory=serial.Serial) as owned:
    run(owned.bus, args, log)       # CLI's existing bounded lifecycle
```

The joint core is exercised through an offline fixture runner. That runner owns fake polling/dispatch; the core cannot open a port or call `exchange`.

```python
core = JointCore(joints=(JointSpec("J1", 0, mapping),),
                 stream_id="fake-run-17", feedback_max_age_s=.1,
                 clock=clock.monotonic)
station = FakeStation(core, scripted_backend, clock)  # test support only
station.connect(); station.check(); core.arm()
core.submit(Target(sequence=1, joints=(JointTarget(.2, .0, .0, 2., .5),)))
station.step()                     # consumes once, writes only to fake backend
core.stop()
```

## Shape

Use frozen dataclasses with constructor validation and owned tuples. Keep the small immutable configuration in these modules, without a schema framework or build profile registry.

```python
JointMap(reduction: float, direction: Literal[-1, 1], zero_output_rad: float)
JointMap.to_output(target: JointTarget) -> OutputTarget
JointMap.from_output(sample: OutputSample) -> JointSample
JointSpec(name: str, motor_id: int, mapping: JointMap)
JointTarget(q_rad: float, dq_rad_s: float, tau_nm: float, kp: float, kd: float)
OutputTarget(q_rad: float, rpm: float, tau_nm: float, kp: float, kd: float)
OutputSample(q_rad: float, rpm: float, tau_nm: float)
JointSample(q_rad: float, dq_rad_s: float, tau_nm: float)
MotorHealth(mode: int, errors: int, warnings: int, timeout: bool, healthy: bool)
SourceIdentity(backend: Literal["physical", "demo", "fixture"],
               identity: str, connection: int)
Target(sequence: int, joints: tuple[JointTarget, ...])
JointStatus(sample: JointSample | None, received_at: float | None,
            valid: bool, errors: int, warnings: int, timeout: bool,
            communication_error: FailureCode | None)
Snapshot(state: State, joint_names: tuple[str, ...],
         joints: tuple[JointStatus, ...], ages_s: tuple[float | None, ...],
         pending: Target | None, sequence_watermark: int,
         affected_joints: tuple[str, ...], source: SourceIdentity)

JointCore.begin_connect() -> Decision
JointCore.connected(source: SourceIdentity) -> Decision
JointCore.begin_check(*, acknowledge_fault: bool = False) -> Decision
JointCore.receive(motor_id: int, sample: JointSample, status: MotorHealth) -> None
JointCore.communication_failed(motor_id: int, code: FailureCode) -> None
JointCore.checked() -> Decision
JointCore.arm() -> Decision
JointCore.submit(target: Target) -> Decision
JointCore.consume() -> Target | None
JointCore.tick() -> None
JointCore.stop() -> None
JointCore.disconnect() -> None
JointCore.snapshot() -> Snapshot
```

`Decision` contains acceptance and a fixed rejection code. `JointCore` hides state legality, whole-vector validation, ordering, freshness, and fault propagation. Driver lifecycle methods are an adapter seam; clients use arm/submit/stop/snapshot. FakeStation first obtains acceptance from `begin_connect` or `begin_check`, performs fake I/O only after acceptance, then reports completion. Failures report fault/disconnect before returning. Wire packets stay outside this interface, per boundary-discipline. The future station owner is the only writer. The fixture runner supplies that writer now; snapshots copy owned data. A core instance is thread confined, with no hidden worker or second mutable mirror.

| Module | Knowledge and ownership |
|---|---|
| `joint_mapping.py` | External algebra and explicitly named joint/output units |
| `im6014_codec.py` | Internal ratio, register capacity, packet and scalar validation |
| `im6014_port.py` | Passive port inspection, alias identity, lease and handle lifetime |
| `im6014_serial.py` | Existing transaction, receive scan and bounded capture |
| `joint_controller.py` | Fixed joint vector, state, observations, one target and watermark |
| Test fixture support | FakeStation, fake transport and clock; no production imports |
| Existing bench/UI files | Existing lifecycle and displays; use managed opening/captures |

The fixed ordered `JointSpec` tuple must have unique names and IDs. Vector length must match exactly. No absent joints or zero padding. Each `receive` stamps the injected monotonic clock after validation, never a caller timestamp. Wrong ID, corrupt feedback, and decoding failure cannot refresh a joint. Age uses `None` for never received; clock time zero is valid. Freshness and health are independent. Preserve the offending joint's full error/warning masks, including unknown bits, and timeout. The configured active vector defines the affected build. Any active joint failure disarms that core, clears pending, and retains its attributed cause. Healthy feedback alone cannot clear `FAULT`.

`submit` first validates the complete vector and sequence, then changes both pending and watermark together. Only `ARMED` or `MOVING` accepts targets. Each accepted update replaces the one pending record. Invalid updates preserve both existing fields. `consume` clears pending but retains the watermark. It rechecks state and current feedback before returning a target and changes `ARMED` to logical `MOVING`. A fake runner may encode and dispatch that target; no production runner exists through C06. The stream is one core instance identified by immutable `stream_id`. Sequence is an actual nonnegative integer, excluding booleans, and must strictly exceed the highest accepted value. Stop, consumption, faults, reconnect and disconnect retain this watermark. New ownership/stream reset rules belong to C08.

### Mapping and codec boundary

`joint_mapping.py` owns only external reduction. Require finite real numbers, positive reduction and an actual integer direction ±1, excluding booleans. Compute `q_out=q0+s*N*q`, `rpm_out=s*N*dq*60/(2*pi)`, `tau_out=s*tau/N`, and output gains `joint_gain/N²`. Reverse these formulas for feedback. Reject nonfinite calculated results and normalize arithmetic overflow/underflow failures to `ValueError`. `im6014_codec.py` keeps `38/3` and unchanged output bounds: Kd 0..6, Kp 0..30, torque ±3.5 Nm. Validate ID/mode as exact integers, watchdog as exact boolean, and controls as finite numeric values before arithmetic. Reject bad types with deliberate `ValueError`, including feedback buffer/expected-ID input. Verify rounded register ranges before packing; never clip. CLI and GUI translate those errors into their existing rejection/exit behavior before opening or writing.

### Port ownership and inspection

`im6014_port.py` owns `PortSpec = FtdiPortSpec | AlternatePortSpec | FixturePortSpec`, `inspect_port(spec, observed_metadata) -> PortInspection`, and `open_managed_port(spec, *, serial_factory) -> ManagedPort`. Specs have explicit path, supported integer baud 4/6 Mbps and expected identity. FTDI includes expected serial/interface channel. Alternate hardware declares its own adapter identity; fixture ports have explicit fixture provenance. Historical Channel D only ranks choices. Inspection reads metadata and filesystem identity without serial construction or traffic, and reports expected versus observed fields.

Before the serial constructor, acquire a process-wide guarded registry entry and an advisory `flock` on a stable lockfile keyed by the resolved character device's `st_rdev`, in one trusted namespace shared by cooperating clients. Direct and symlink aliases therefore collide. Never unlink a lockfile or remove a lock by PID. Keep the lock FD for the entire bus lifetime. Recheck device identity around open, including `fstat` on the opened descriptor, to reject replacement races. Explicit fixture identity supports constructor mocks; fixture selection cannot silently label physical traffic as simulated.

`ManagedPort` alone owns bus and lease. GUI retains it until worker disconnect cleanup finishes; CLI's context encloses final Stop attempts. Constructor, configuration and open failure close any partial handle and release acquisition. Normal close releases after serial close. If close leaves an open handle, retain the lease and report close unconfirmed until cleanup or process exit; releasing it early creates a second owner. OS process death closes both serial and lock descriptors. Pyserial `exclusive=True` remains an additional advisory check. This excludes cooperating application owners, not arbitrary programs ignoring locks. No import, constructor, enumeration, or dashboard startup acquires a lease.

### Diagnostics

`PacketCapture` in `im6014_serial.py` is a bounded ring, default 256 records. Each allowlisted record contains explicit physical/demo/fixture source identity, connection number, port identity/baud, requested ID, monotonic start/end, raw request up to 20 bytes, reply up to 26 bytes, decoded output fields, and fixed outcome code. Cap identity/path strings and copy only parsed source fields, so record sizes are bounded too. B14's export never serializes settings, exception text, environment or arbitrary metadata. Unknown driver exceptions become `DRIVER_FAILURE` in captures. Optional receive diagnostics retain counts of rejected/CRC frames and at most a 25-byte tail, never unbounded chunks. Existing three-argument exchange calls and actual `motor_session.exchange` lookup remain valid; diagnostic additions preserve RuntimeError compatibility and existing message behavior outside captures.

## Transition table

| Current state | Event and condition | Result |
|---|---|---|
| DISCONNECTED | Begin connection | CONNECTING; fake adapter may open |
| CONNECTING | Driver reports successful connection | CONNECTED, disarmed, no measured joints |
| CONNECTED/STOPPED | Begin explicit check | CHECKING, disarm, clear pending; fake Stop/status allowed |
| FAULT | Begin recovery check with explicit acknowledgment | CHECKING, retain attributed cause |
| CHECKING | All active joints fresh, healthy, Stop confirmed | READY, disarmed; recovery cause acknowledged |
| READY | Arm with all active joints still fresh/healthy | ARMED |
| ARMED/MOVING | Valid newer target | Replace pending; state retained |
| ARMED | Fake consumer takes pending after freshness check | MOVING |
| Any connected state | Stop | STOPPED, clear pending, disarm |
| Any connected state | Attributed fault, timeout, or stale active feedback | FAULT, clear pending, disarm |
| Any | Disconnect | DISCONNECTED, clear pending, invalidate observations |
| Any | Any unlisted transition or invalid data | Reject, no motor command or target replacement |

The fake owner calls `tick` on every step to fault stale active feedback. Stop's software transition never asserts physical Stop confirmation. Existing bench display strings and recovery behavior remain separate compatibility behavior.

## Synthesis decision and alternatives

Select this typed core plus managed driver boundary as my candidate. Parent synthesis remains pending. A structurally distinct alternative is a station daemon with a command socket, actor inbox, bus loop, client facades and subscriptions. That daemon hides connection and execution behind fewer client calls and meets the eventual ownership direction. It loses for this block because IPC identity, expiry, generations, references and dashboard independence are unfinished C07-C16 constraints. Implementing it now creates a tempting incomplete live motion service. The selected core concentrates accepted-target policy without duplicating the bench or adding dispatch capability.

## Tradeoffs accepted and remaining scope

Accept a fixture integration seam now in exchange for a smaller reviewable C02-C06 contract. Accept advisory cooperating-client ownership in exchange for portable Linux locking. Station IPC, live multi-joint refresh, owner generations, command expiry, reference limits, loaded-arm shutdown and independence remain explicit later work. Reassess the core interface when the independent station integrates; do not call the reducer a complete physical station.

## Next implementation step and acceptance plan

Start with B11 mapping, then land each milestone with parent-run evidence before the next.

| Milestone | Parent acceptance |
|---|---|
| B11 | Round trips at N=1/3/6, both signs, offsets; gains/units; unchanged codec goldens |
| B12 | NaN/infinity, bool/fractional/string IDs/modes, watchdog types, overflow and mapped overflow; no I/O; bench endpoints unchanged |
| B13 | Correct/wrong identity/channel, unsupported baud, typed alternate/fixture; zero opens during inspection |
| B14 | Request/reply/decoded comparison; cap records/tails; malicious secret/error strings absent; provenance retained |
| C01 | Same process, subprocess and aliases; acquire before constructor; failed-open/close cleanup; crash release; parent PTY checks |
| C02 | Complete transition matrix on fake backend; rejected transitions produce zero writes; Stop/reconnect never rearm |
| C03-C04 | Fake clock at zero; fresh J1/stale J2; wrong ID; one fault/unknown bit retains identity and disarms affected build |
| C05-C06 | 10,000 updates, one record, newest valid target; caller mutation; invalid/duplicate/older update preservation; watermark after consume/Stop |

Integrate B13/C01 at both existing open sites with active mock lookups, and B14 at the existing transaction/export seams. Add every new runtime/test module to startup fingerprints and baseline copied-source lists. Parent reruns the offline baseline and inspects startup images. This proposal performed source inspection only; no tests or physical operations.
