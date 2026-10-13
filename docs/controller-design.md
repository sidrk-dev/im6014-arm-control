# Joint mapping and controller design

B11-B14 and C01-C07 extend the bench with pure joint algebra, passive configuration, bounded packet diagnostics, cooperating bus ownership and a transport-free request core. Existing bench entry points retain their lifecycle and packet bytes. No new physical motion runner is introduced.

## Selection

Sol's typed core is the base. Parent scores against the six saved rubric criteria are 5, 4, 4, 5, 5, 5. Gemini's wire-aware core scores 2, 3, 2, 2, 2, 3. Both OpenCode reviewers selected Sol. Their full scores and requested-route receipts are in evidence/controller/models. Parent accepted named rad/s intermediates, passive OS metadata, the sequence example 1,5,5,2,6, and a transition table with guards. Parent rejected premature arm expiry, arbitrary stream dictionaries, mutable snapshot dictionaries, raw wire ownership in the core and a station daemon. These need later C08-C16 work.

The reviewers missed a fault recovery gap in Sol's Stop row. Stop must preserve FAULT and its attributed cause. Disconnect/reconnect must not permit a latched fault to bypass an explicitly acknowledged recovery check. A healthy packet alone never clears it. This is a parent correction to the base.

## Modules and contracts

- joint_mapping.py defines frozen JointMap, JointTarget, OutputTarget, OutputSample and JointSample. Joint units are rad, rad/s, Nm and compatible gains. Only the codec owns the internal 38/3 reduction.
- im6014_port.py defines validated PortConfig, passive inspection and ManagedPort. Both bench open sites pass their current serial.Serial factory. Ownership is acquired before construction, keyed by character-device identity in a trusted per-user directory, and held through confirmed close. Symlink aliases collide. No lockfile unlinking or PID ownership. The lock excludes cooperating workbench clients only. Constructor mocks remain injectable; PTYs provide real device identity in ownership tests.
- im6014_serial.py retains the existing three-argument exchange and adds optional bounded PacketCapture. Records include capped allowlisted metadata, request/reply bytes and decoded fields. No environment, settings or exception strings enter an export.
- joint_controller.py defines State, Decision, JointSpec, JointStatus, Target, Snapshot and JointCore. Core instances are thread confined. They own a fixed joint order, observations, one pending immutable target and one sequence watermark. No serial import, port opening or exchange occurs inside the core.
- Test-only FakeStation drives legal lifecycle transitions and fake packet writes. Positive legal actions must write expected fake commands; illegal actions must add zero writes. Production never imports it.

The Linux ownership namespace is fixed for a user. A per-caller lock-directory option, environment-dependent namespace or fallback directory would let two cooperating clients avoid each other's lock. Tests use the same production namespace and PTY device identities. They may inject serial factories, but they do not bypass device inspection or acquisition.

## Algebra

For external reduction N>0, integer direction s in {-1,1} and output offset q0:

    q_out = q0 + s*N*q_joint
    dq_out = s*N*dq_joint
    rpm_out = dq_out*60/(2*pi)
    tau_out = s*tau_joint/N
    kp_out = kp_joint/N**2
    kd_out = kd_joint/N**2

Reverse feedback uses q_joint=s*(q_out-q0)/N, dq_joint=s*(rpm_out*2*pi/60)/N and tau_joint=s*N*tau_out. Finite numeric validation excludes booleans. Overflow rejects with ValueError. Mapping never clips a target. Codec bounds remain unchanged. Literal anchors and round trips at N=1,3,6 and both directions prove software algebra only.

## Core transitions

| State | Event and guard | Result |
| --- | --- | --- |
| DISCONNECTED | begin_connect | CONNECTING |
| CONNECTING | connected | CONNECTED, observations invalidated |
| CONNECTED or STOPPED | begin_check, explicit acknowledgment if fault remains | CHECKING |
| FAULT | begin_check with explicit acknowledgment | CHECKING, cause retained until successful check |
| CHECKING | checked, all active joints fresh, healthy and Stop mode | READY, cause cleared |
| READY | arm, all active joints fresh and healthy | ARMED |
| ARMED or MOVING | submit, complete newer target with `created_at_s <= now < expires_at_s` | Replace pending only |
| ARMED or MOVING | consume, fresh healthy observations and unexpired pending target | Return pending once, MOVING |
| ARMED or MOVING | consume, expired pending target | Discard pending, retain watermark and state |
| Connected state except FAULT | stop | STOPPED, disarmed, pending cleared |
| FAULT | stop | Remain FAULT, pending cleared |
| Active state | attributed error, timeout or stale joint | FAULT, pending cleared, cause latched |
| Any | disconnect | DISCONNECTED, observations invalidated, cause and watermark retained |
| Any | unlisted transition | Rejection, no fake or physical I/O |

C02 first implements state legality and a test fixture. C03 adds per-joint clock-stamped age, including timestamp zero and wrong-ID rejection. C04 adds full error/warning/timeout attribution. C05 adds exact-length immutable target replacement. C06 enforces actual nonnegative integer sequences, strictly increasing. C07 adds required finite command creation and expiry timestamps in the station monotonic domain: future creation rejects, equality at the deadline expires, failed replacement preserves the pending target and watermark, and consumption discards an expired pending target without a fault. Dispatch freshness and expiry share one clock sample; regression raises before release. Consumption, Stop, fault and reconnect retain the watermark. A new stream/owner generation is deferred to C08.

## Acceptance and remaining scope

Each unit is authored in an isolated worktree, reviewed and run by the parent before the next. scripts/check_milestone.py records hashes, actual checks and raw logs. New modules enter the offline copyset and startup fingerprint. The final driver runs sequentially and its images are inspected. The full source and decision trail go to Muse and MiMo before independent verification and a guarded PR merge.

Software Stop does not prove physical torque off. PTY locking and algebra do not prove adapter performance or calibration. Live multi-joint execution, ownership generations, limits, effort validity, timing, loaded-arm shutdown and station independence remain later work. The pending timeout acknowledgment remains a separate hardware gate.
