Base choice: `design-sol.md` . Graft narrow fixes from `design-gemini.md` only for test phrasing, nothing architectural.

### 1. Canonical joint units / conversion ownership

**Sol 5/5:** Correct boundary. `joint_mapping.py` owns only external `q_out=q0+s*N*q, rpm_out=s*N*dq*60/2pi, tau_out=s*tau/N, gains/N^2` with reverse. Requires finite, `N>0`, `direction:int!=bool +/-1`. Normalizes overflow to `ValueError`. `im6014_codec.py` keeps `38/3`, output bounds `Kd 0-6/Kp 0-30/tau +-3.5`, exact `int!=bool` ID/mode, exact `bool` watchdog, finite checks before arithmetic, rounded-register range verify, never clip. No schema/profile system.

**Gemini 2/5:** Leaks actuator units into joint API: `JointTarget(rpm=...)` with defaults, fragmented `to_actuator_position/velocity/torque/gains` without stated `q0+s*N*q` unity. No `bool` exclusion for `direction`, no overflow normalization, no never-clip rule. Says `zero imports of RATIO` but does not prove external/internal ownership. `position/velocity/torque/kp/kd` naming mixes joint/output.

### 2. Acquire-before-open, alias/cross-process, lifetime, injection

**Sol 5/5:** `open_managed_port(spec,*,serial_factory)` used at both existing open sites with active lookup so `serial.Serial` mocks still intercept. `inspect_port` no construction/traffic. `PortSpec=Ftdi|Alternate|Fixture` with explicit path/baud 4/6M/expected identity/channel. `Channel D` ranking only. Acquire process registry + advisory `flock` on stable lockfile keyed by resolved `st_rdev`, aliases collide, never unlink/remove-by-PID, hold FD whole bus life, `fstat` recheck around open, `exclusive=True` advisory only, fixture provenance cannot masquerade physical. Failure closes partial + releases; normal close releases after close; open-handle-left retains lease as `close-unconfirmed`. No acquisition on import/enumerate/dashboard startup. Clarifies cooperating-app only.

**Gemini 3/5:** Correct `realpath/stat st_rdev` + `flock LOCK_EX|LOCK_NB` + in-process lock, acquire-before-construct, release on failed-open, kernel release on crash, advisory-only disclaimer. Fails: `lease.open_serial()` is new second open path, no `serial_factory` / `motor_session.exchange` 3-arg / startup patch preservation, no `fstat` race recheck, no close-unconfirmed retention, no typed `Alternate/Fixture` identity, `/tmp/im6014_bus_{major}_{minor}.lock` predictable namespace with no no-unlink rule, `Channel D` left as hint without historical-not-proof rule.

### 3. Transitions, per-joint age/error, no auto-rearm

**Sol 5/5:** `DISCONNECTED/CONNECTING/CONNECTED/STOPPED/CHECKING/READY/ARMED/MOVING/FAULT/STOPPED` table. `begin_connect/connected/begin_check/receive/checked/arm/submit/consume/tick/stop/disconnect` with `Decision+fixed code`. Reject unlisted/invalid with no motor command/target replace. Arm needs all active fresh+healthy; `FAULT` needs explicit ack check, healthy alone cannot clear. Any active fault/timeout/stale disarms+clears pending, retains offending masks incl unknown bits+timeout, build=affected active vector. `receive` stamps injected monotonic after validation, never caller time; `None`=never, `0`=valid; wrong-ID/corrupt/decode cannot refresh. `tick` faults stale. Stop software transition asserts no physical Stop proof.

**Gemini 2/5:** `DISCONNECTED/CONNECTING/STOPPED/ARMED/FAULTED/TIMEOUT_LATCHED` - missing `CHECKING/READY/MOVING`, adds premature `TIMEOUT_LATCHED/ack_timeout Mode0/watchdog-toggle` and 10s `arm_until` expiry (C07 expiry early). `ingest_feedback(...,receive_mono)` trusts caller time; `arm(...,max_stale_s=0.25)` caller-relaxable vs `feedback_stale>0.1s` inconsistent. `request_transition(action:str)` stringly-typed. Freshness/health separation stated but forgeable.

### 4. Bounded immutable storage + watermark

**Sol 5/5:** One `pending:Target|None` + `watermark:int`, frozen dataclasses/tuples, fixed `JointSpec` order, exact length, no padding, snapshots copy. Validate-then atomically replace both; invalid/duplicate/older preserve both. `consume` clears pending retains watermark, rechecks state/freshness, `ARMED->MOVING`. `sequence:int!=bool >=0` must `>watermark`. Retain after consume/Stop/fault/reconnect/disconnect; new stream/owner reset is C08. `stream_id` immutable per-core. 10k updates = one record newest-valid.

**Gemini 2/5:** O(1) single slot + retain-after-`stop` and invalid-preserve correct, but `Dict[int,JointTarget]` mutable with defaults allows absent/zero-pad, `Tuple[str,int,Dict]` shares mutable dict, no fixed order/copy guarantee. `_watermarks[stream_id]` per-call multi-stream violates one-documented-stream + C08 cross-owner deferral, enables replay across streams.

### 5. Small coherent API, bench/bytes, no dual owner

**Sol 5/5:** `JointMap/Spec/Target/OutputTarget/Sample/Health/SourceIdentity/Target/Status/Snapshot/Decision` + `JointCore` thread-confined, no worker/mirror, wire packets outside, fake writer only via `FakeStation`, no prod dispatch to C06. Preserves 3-arg `exchange`, `motor_session.exchange` lookup, display strings, startup fingerprints/baseline lists, both open sites.

**Gemini 2/5:** `request_transition/snapshot()->dict/get_pending_target()->Dict` broad/untyped, `recorder` injected into core mixes telemetry/arbiter, defaults invite partial vectors, `CooperativeBusOwner+lease.open_serial` duals existing bench path. No exchange/startup/bytes preservation statement.

### 6. Staging/acceptance, no premature dispatch

**Sol 5/5:** B11->C06 order, parent-run evidence, fake clock/transport + parent PTY, no physical ports, round-trips N=1/3/6 both signs, NaN/inf/bool/fractional/string/overflow no-I/O, zero-opens inspection, capped captures with provenance, alias/subprocess/crash PTY, full matrix zero-writes, J1-fresh/J2-stale/wrong-ID, fault-bit identity, 10k/invalid/duplicate/watermark-after-consume/Stop. Explicitly no prod runner, no profile/IPC, reducer!=station, C07-C16 remaining.

**Gemini 3/5:** Good B11-C06 checklist but normatives include premature arm-expiry, per-stream generations, `TIMEOUT_LATCHED` ack, 1000-record `~500kB` without 20/26B +25B-tail caps or secret/error-string exclusion proof, no zero-writes/startup-image/close-cleanup checks, no explicit no-prod-consume rule.

### Grafts/corrections to Sol base

1. Keep Sol modules/lifetimes verbatim. Do not adopt `ReductionMapping/JointCommand(rpm)/ControllerCore.Dict/stream_id-per-submit/lease.open_serial/TIMEOUT_LATCHED/10s-arm`.
2. Clarify: central `feedback_max_age_s` constant (e.g. 0.1s) for both arm and `tick`; no `max_stale_s` param; no arm timer.
3. Clarify watermark: retain across `consume/Stop/FAULT/disconnect` within instance; new owner = new `JointCore(stream_id)` per C08; document that.
4. Add `Decision.code:Enum`, `Snapshot` copy semantics, baud `Literal[4000000,6000000]`, capture `256 records, 20B req/26B reply/25B tail, DRIVER_FAILURE` allowlist with `SourceIdentity(physical/demo/fixture,identity,connection)` to acceptance checks.
