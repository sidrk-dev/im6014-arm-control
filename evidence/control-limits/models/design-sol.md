# C07-C16 candidate: core-owned goals with guarded reference refresh

## Usage

The station creates intent windows before client computation. A delayed policy result retains that window's expiry; arrival cannot make it young again. Clients receive domain snapshots and submit complete ordered joint goals.

```python
# A new policy client. No serial object, clock input, or health input.
with StationClient(socket_path) as client:
    owner = client.claim("automation")       # starts stopped
    window, observation = client.observe(valid_for_ms=200)
    client.check(acknowledge_fault=False)
    client.arm(owner)                         # explicit; never part of reconnect
    receipt = client.submit(owner, window, sequence=41,
                            joints=policy(observation),
                            observation_id=observation.id)
    # receipt reports admission, not dispatch or movement
```

```python
# A human tool. Takeover revokes the previous generation and stops first.
with StationClient(socket_path) as client:
    owner = client.takeover("human")
    client.check(acknowledge_fault=True)
    client.arm(owner)
    window, observation = client.observe(valid_for_ms=100)
    client.submit(owner, window, sequence=42, joints=complete_joint_targets,
                  observation_id=observation.id)
    client.stop()                            # priority control lane
```

```python
# Station adapter, executed by one station thread.
plan = core.prepare_refresh()                # complete domain reference, no bytes
if plan is not None:
    packets = encode_complete(plan, specs)   # validates every mapped joint
    for index, packet in enumerate(packets):
        feedback = exchange(bus, packet, motor_id=specs[index].motor_id,
            before_write=lambda: priority_then_claim(core, plan.id, index),
            on_write=lambda outcome: core.dispatch_result(plan.id, index, outcome))
        core.receive(specs[index].motor_id, mapped_sample(feedback), health(feedback))
```

`priority_then_claim` services available Stop/takeover/disconnection before calling the core's final gate. Withdrawal produces no mode-1 write. Transaction failures abort remaining motion packets and schedule Stops. The adapter cannot take a raw pending goal and write it.

## Problem and grounding

The existing `JointCore` fixes joint order, atomically replaces one immutable pending target, validates complete mapped codec commands, and preserves attributed faults through acknowledged recovery. Its `consume_target` returns unconstrained requests once. The bench GUI and CLI independently use the managed lease and never call this core. Preserve those bench paths. Extend the core and its fake callers together; replace consumption entirely.

The managed port already acquires its fixed per-user character-device lease before serial construction and retains ownership until close returns the exact boolean `False` from `is_open`. Station work must reuse this contract. Its socket namespace must never become a serial lease override.

## Shape

Use a stateful, thread-confined `JointCore` as the only writer of authority, admitted goal, expiry, reference state, observations, and fault union. A separate station process owns transport, scheduling and IPC. A pure reference module hides the feasibility mathematics. This keeps the client interface small while concentrating policy behind it, per boundary-discipline and minimize-reader-load.

| Module | Knowledge owned |
| --- | --- |
| `joint_contract.py` | Frozen domain records, explicit units, validated fixture limit records and decision/fault codes. Migrate affected imports together. |
| `joint_controller.py` | Lifecycle, clock high-water mark, ownership, one admitted complete goal, one prepared batch, per-joint reference/dispatch records and recovery. |
| `joint_reference.py` | Pure scalar feasible-interval calculation, reference construction and combined effort prediction. No state, clock or transport. |
| `station_protocol.py` | Private fixed binary protocol and `StationClient`; framing, domain conversion, connection binding and backpressure. |
| `joint_station.py` | One process/thread, PTY fixture startup, managed port, final write gate, refresh and Stop scheduling, timing measurements. |
| Existing codec/mapping/port/serial | Preserve their encoding, mapping, lease and transaction contracts. Add optional final `before_write` and write-outcome hooks to `exchange`; bench calls retain their behavior. |

No profile file or D01 schema. Explicit immutable constructor arguments supply fixture limits. No generic event bus, strategy registry, or pass-through service stack.

```python
@dataclass(frozen=True)
class Owner:
    station_epoch: bytes             # 128-bit process nonce
    generation: int                  # uint64, never wraps
    connection_id: int               # station generated

@dataclass(frozen=True)
class IntentWindow:
    id: int
    created_ns: int                  # station monotonic domain
    expires_ns: int

@dataclass(frozen=True)
class Target:
    owner: Owner
    window: IntentWindow
    sequence: int
    joints: tuple[JointTarget, ...]   # fixed order; rad, rad/s, joint Nm
    observation_id: int
    build_revision: bytes
    configuration_revision: bytes

@dataclass(frozen=True)
class Reference:
    q: float
    dq: float
    dispatched_ns: int

@dataclass(frozen=True)
class JointLimits:
    q_min: float; q_max: float
    speed_max: float; acceleration_max: float; lead_max: float
    predicted_effort_max: float; measured_effort_peak: float
    measured_effort_continuous: float; effort_dwell_ns: int
    temperature_trip: float; temperature_reset: float; recovery_dwell_ns: int

class JointCore:
    # Existing connect/check/arm/receive/Stop/disconnect lifecycle remains.
    # All stateful entry points observe one checked station time first.
    def claim(self, connection_id, kind, *, takeover=False) -> Owner: ... # not implemented
    def issue_window(self, owner, valid_for_ns) -> IntentWindow: ...     # not implemented
    def submit_target(self, target: Target) -> Decision: ...            # not implemented
    def prepare_refresh(self) -> ReferencePlan | None: ...             # not implemented
    def claim_dispatch(self, plan_id, index) -> bool: ...               # not implemented
    def dispatch_result(self, plan_id, index, result) -> Decision: ...   # not implemented
    def snapshot(self) -> Snapshot: ...                                # not implemented

def next_reference(previous, goal, measured, limits, elapsed_ns) -> ReferenceResult:
    raise NotImplementedError
```

Opaque plan IDs belong to one core instance. Final claims follow configured joint order and occur once. No wire records enter the core. `dispatch_result` records confirmed complete write, partial write or failure independently of feedback reception. References advance only on a confirmed complete write, using its claimed start time. Partial/unknown writes fault and Stop; measured state remains independent telemetry.

### Clock, ownership and admission

Use injected `monotonic_ns`, accepting only nonnegative bounded integers excluding booleans. Every stateful entry, including receive and snapshot, checks against the last observed high-water mark before changing observations or references. Equality is legal. Regression or invalid time faults and withdraws motion; it never replaces the high-water mark. Snapshot then identifies the last valid time and invalid current clock rather than fabricating fresh ages. Recovery also requires a valid clock at or beyond that mark.

The core issues one current window per active owner, bounds its lifetime, and verifies submitted window identity and stored values. IPC supplies a window ID, never client timestamps. Creation is the station's observation/window issuance, before computation. Expiry is exclusive: `now >= expires_ns` is expired. Check admission, refresh, and immediately before every actual write attempt. An admitted goal remains available for repeated bounded refresh only until expiry. Expiry clears it and requires Stop without automatic rearm.

Claim/takeover, Stop, owner disconnection and reconnect revoke the generation and window, clear goal/prepared references, and leave motion unarmed. Takeover stops before granting authority. Process restart changes epoch. The existing global sequence watermark remains increasing across owners, Stop, faults and reconnect; clients obtain it from snapshots. Exhaustion rejects further submissions.

Admission validates source binding, window, revision IDs, complete count/order, finite values, inclusive requested position bounds, requested speed, all mapped codec commands and sequence before replacing the goal/watermark. Invalid replacements preserve a still-valid goal, references and watermark. Time observation and safety housekeeping are separate mandatory transitions, so a bad replacement cannot preserve an already expired or newly faulted goal. Accepted references and batch proposals commit only after every joint passes.

### Feasible references

For each joint let previous reference be `(q0,v0)`, fresh measurement `(qm,vm)`, and elapsed seconds `h`. Initialize at healthy checked Stop with measured position and zero reference velocity. Reject initialization outside position bounds. Zero elapsed time permits only unchanged refresh. A gap beyond the configured fixture maximum causes Stop; no unlimited catch-up step.

Choose endpoint velocity `v1`, with virtual linear velocity and its integrated reference:

```
v(t) = v0 + (v1-v0)t/h
q(t) = q0 + v0*t + (v1-v0)t*t/(2*h),  0 <= t <= h
a = max(q_min, qm-lead_max)
b = min(q_max, qm+lead_max)
I = [-V,V] intersect [v0-A*h, v0+A*h]
    intersect every [v0+2*h*(a-q0-v0*t)/t²,
                     v0+2*h*(b-q0-v0*t)/t²], 0 < t <= h
```

This is a closed interval, calculated analytically from endpoints, the `t -> 0` limit and any extrema at `t = 2*(bound-q0)/v0` inside the step. There is no time sampling loop. Also intersect the endpoint braking viability inequalities `q1-max(-v1,0)²/(2*A) >= a` and `q1+max(v1,0)²/(2*A) <= b`; their monotone expressions yield analytic interval endpoints. Require positive finite `V,A,lead` and finite intermediate arithmetic.

Project `v_des = (2*(q_goal-q0)/h - v0 + dq_goal)/2` onto `I`, then set `q1=q0+h*(v0+v1)/2`. This minimizes equally scaled position and velocity error. Requested position and velocity stay recorded; emitted values are the shaped pair. Reversal crosses zero through the acceleration interval. Equal opposite velocities produce zero net displacement, but the interior extremum still counts. Never divide by average velocity or reset velocity on sign change.

Empty position/lead intersection, outward velocity at a boundary, empty interval or unsafe arithmetic yields no reference. During active refresh this latches an attributed infeasibility fault and schedules Stop. Stop-mode commands are outside the motion reference contract. A moving measurement can make lead infeasible despite prior braking viability; do not relax another limit to hide it.

Full mapped codec validation applies to incoming requests and complete shaped plans. Decode the shaped packet's numerical fields back into joint units and verify its quantized endpoint against the same constraints; an unsafe rounded result yields no dispatch. Record those emitted values. Final gates recheck measurement lead, freshness, fault, authority, expiry and elapsed-time bounds. Superseded plans cannot refresh. These virtual reference constraints establish no continuous physical trajectory.

### Effort and recovery

Predict `tau_ff + kp*(q1-qm) + kd*(v1-vm)` in joint units. With fixed gains and the reference equation this is affine in `v1`; intersect its `[-predicted_effort_max,+predicted_effort_max]` interval with `I`. A zero coefficient needs a constant-value check. Record the prediction alongside each shaped reference. This is a fixture guard, not certified protection.

Independently monitor fresh measured effort. Peak threshold trips immediately; continuous threshold above its dwell trips persistently. Attribute new causes by union with existing communication/error/warning/stale causes. Add validated temperature to received health. Temperature trip is latched; missing telemetry cannot count as healthy recovery. Healthy Stop feedback after explicit acknowledgement must satisfy temperature reset and continuous-effort recovery bounds for the configured dwell on every joint. New targets, healthy samples alone and reconnect never clear faults. Successful recovery returns READY; a separate explicit arm is required.

### Station boundary and timing

Use private same-user Unix `SOCK_SEQPACKET` IPC. Every version-1 frame is exactly 2048 bytes with a fixed header, at most 15 fixed joint slots and zero padding. Require exact size, known opcode/representation, configured names/order, zero unused slots and matching revisions. Reject truncation, nonfinite doubles and malformed records before core admission. No pickle, arbitrary objects, client health or client clock. One active owner connection, one bounded takeover candidate, one application target slot and one outstanding window keep storage bounded.

Serve one-frame Stop/takeover requests through the listener's priority control path before active target reads and each motion write gate. Targets cannot create an application FIFO. Bound accepts/reads per iteration and socket buffers; close slow malformed handshakes. Takeover revokes the active connection atomically before replacement. A lost owner connection, liveness timeout or backpressure failure triggers Stop and revocation. Priority cannot interrupt an already running synchronous driver call; record this limit.

The station owns refresh regardless of dashboard lifetime and never spawns itself from a client. C16 executable startup accepts only an explicitly validated PTY fixture endpoint with an injected deterministic actuator. It retains the existing managed lease namespace checks. Confirmed close releases ownership; unconfirmed close keeps the process in a retryable shutdown state with its lease held. Process death releases OS descriptors; a replacement station starts stopped with a new epoch and requires fresh checks and arm.

Measure loop start-to-start period, actual transaction start/end latency per joint, completion beyond scheduled deadline, missed periods, per-joint receive age and sampling skew. Skip missed schedule slots rather than bursting. Keep bounded last/max/counter records. Label timestamps and fixture provenance; do not infer actuator rate from wire time.

## Alternative and tradeoffs

A viable alternative makes `JointCore` an immutable reducer, `apply(state,event) -> (state,effects)`. A single station loop interprets Stop, reference and telemetry effects, and a `DispatchRequested` event performs the final guard. It retains the same math, lease, fault and IPC rules, but replaces every lifecycle caller with tagged events and every effect with acknowledgements. Its one public method hides transitions deeply; its large event/effect taxonomy exposes coordination to adapter callers. The selected stateful core retains recognizable lifecycle calls and needs fewer caller changes.

Accept conservative Stops when constraints become infeasible in exchange for explicit bounds. Accept a shaped position/velocity objective in exchange for coherent reference evolution. Accept partial bus dispatch records in exchange for truthful serial behavior. Prepared plans add one bounded internal state in exchange for complete validation before the first write. Parent synthesis remains pending; this seat selects the stateful core as its candidate base.

## Implementation sequence and acceptance

Exactly one Gemini writer implements each numbered milestone from the parent's accepted tip. Serialize shared API and caller edits. Parent runs the audited offline baseline before commit and dispatching the next writer.

| Milestone | Implementation and required acceptance |
| --- | --- |
| C07 | Add checked time/window/expiry and guarded dispatch; migrate FakeStation and delete `consume_target`. Test equality expiry, delayed computation, active-goal expiry, invalid replacement preservation, clock regression before receive/snapshot/dispatch and no mode-1 writes. |
| C08 | Add epoch/generation/connection authority. Test delayed prior owner, takeover during batch, Stop/reconnect revocation, sequence preservation and explicit rearm. |
| C09 | Add explicit position limits. Test both exact boundaries, adjacent outside values, wrong order/count, mapping/codec rejection and atomic replacement. |
| C10 | Introduce reference module/state and configured speed bound. Test large jumps, signed motion, zero time, gaps and complete all-joint validation. |
| C11 | Add acceleration, integrated step and braking feasibility. Test reversals, zero crossing, interior overshoot, boundary velocity and empty intervals. |
| C12 | Intersect fresh measured lead. Test lag in both directions, narrowing/empty intersection, stale/wrong-ID feedback and final-gate invalidation. |
| C13 | Add combined prediction and measured effort monitoring. Test stiffness/damping/feedforward interactions, cancellations, zero coefficient, quantization/mapping validation and distinct measured records. |
| C14 | Add temperature/persistent-effort union and recovery dwell. Test transient versus persistent effort, missing feedback, simultaneous causes, healthy-target nonrecovery and acknowledged new healthy Stop recovery. |
| C15 | Add station timing records to fake adapter. Test deterministic delays, skew, overrun/missed slots, age attribution and bounded storage. |
| C16 | Add PTY-only process/IPC/client. Test malformed/oversized frames, priority Stop/takeover, client kill while station retains lease, second cooperating opener denial, station kill/restart epoch rejection, no rearm, and retryable unconfirmed close. |

Each unit adds its source/test groups to `scripts/check_offline_baseline.sh`, startup fingerprints, explicit test invocation and CI. Add focused groups for time/authority, reference constraints, effort/recovery and station processes; preserve existing bench and lease regression groups. Update API/architecture evidence with what was actually checked. No tests were run by this design seat.

## Open questions and first step

Which measured effort convention and physical thresholds will later be verified, and what resolves the archived timeout-acknowledgement disagreement? Those questions block physical readiness, not this fixture design. Start C07 by replacing consumption with window-bound guarded dispatch and migrating all core adapter callers together.
