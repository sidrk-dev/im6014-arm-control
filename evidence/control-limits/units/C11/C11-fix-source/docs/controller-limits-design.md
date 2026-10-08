# Controller limits and station design

## Problem and usage

C07-C16 extend the verified transport-free core with expiring ownership-bound requests, joint/reference limits, measured faults and timing. A separate fixture station owns the managed PTY lease independently of its client. Existing bench GUI/CLI remain functional. Physical motor execution remains blocked by the unanswered timeout acknowledgment; fixture thresholds cannot establish production readiness.

The station issues a monotonic intent window before client computation. A client submits a complete vector with that window, sequence and (from C08) current owner token. Arrival never renews the deadline. Connect/check/claim/arm stay explicit. A submit receipt means admitted, not written or moved. The adapter prepares one full vector, validates all packets, then checks the core gate beside each actual write and records complete/partial/failed write independently of feedback.

```python
# Final C16 client; clients never open a motor port or supply telemetry/time.
with StationClient(socket_path) as client:
    owner = client.claim("human")       # revokes prior generation, stops
    client.check(acknowledge_fault=False)
    client.arm(owner)
    window, snapshot = client.observe(valid_for_s=0.2)
    decision = client.submit(owner, window.id, sequence=42, joints=targets)
    client.stop()
```

```python
# One station thread, illustrated in domain terms.
plan = core.prepare_refresh()
if plan is not None:
    packets = encode_complete(plan, core.joint_specs)
    for index, packet in enumerate(packets):
        reply = exchange(bus, packet, core.joint_specs[index].motor_id,
            before_write=lambda: service_priority_and_claim(core, plan, index),
            on_write=lambda result: core.dispatch_result(plan.id, index, result))
        core.receive(reply.id, mapped_sample(reply), mapped_health(reply))
```

The final implementation must resolve closure binding explicitly. The adapter aborts remaining mode-1 writes after any rejection/failure and schedules Stops. Serial writes across joints cannot be physically atomic; each actual partial/full write is separately recorded. Stop cannot interrupt an already-running synchronous driver call.

## Selection and synthesis

Both main candidates converged on core-owned goals with a separate station shell. Sol also supplied a structurally distinct immutable reducer alternative; Gemini considered a separate safety middleware. Parent selected Sol's stateful core: existing lifecycle calls are recognizable and one authority owns motion. The reducer would require a larger event/effect coordination surface. Middleware was rejected because a direct core call could bypass it or duplicate reference state.

| Criterion | Parent Sol | Parent Gemini | Muse Sol | Muse Gemini |
| --- | --- | --- | --- | --- |
| Final dispatch gate and ownership | 5 | 2 | 5 | 2 |
| Combined reference constraints | 4 | 2 | 5 | 2 |
| Interface depth and maintenance | 3 | 3 | 4 | 3 |
| Station independence | 4 | 3 | 5 | 2 |
| Testability and actual evidence | 5 | 3 | 5 | 2 |
| Verifiable unit sequence | 4 | 3 | 5 | 3 |

Muse chose the same base. Parent scored lower where sketches still need executable proof, and did not accept the judge's claim that the full analytic mathematics was already proven. Parent grafted Gemini's readable decision codes, selectors-based nonblocking process shell and snapshot/timing client view. Neither score is physical validation.

Parent simplified Sol's virtual trapezoidal trajectory/braking mathematics into the explicitly discrete contract below. The milestones require samplewise reference velocity and acceleration bounds, not a continuous physical trajectory. Parent replaced fixed 2048-byte binary packing with bounded JSON packets over Linux Unix SOCK_SEQPACKET: OS message boundaries plus fixed byte/count limits avoid incremental parser state and a broad packing taxonomy. No unbounded streams/queues, JSON NaN, arbitrary object deserialization or client-controlled health/time are allowed. Retain Sol's intent windows, opaque plan IDs, final write gate, epoch/generation/connection binding, partial dispatch records and PTY-only executable.

## Modules and types

- joint_controller.py owns lifecycle, checked clock high-water mark, one admitted complete goal, one prepared plan, ownership/window authority, per-joint reference and dispatch records, measured observations and fault union. Split frozen records to joint_contract.py only if needed to keep the core cohesive; migrate imports rather than maintain aliases.
- joint_reference.py owns pure feasible-interval/reference and effort calculation. It owns no clock, observations, mutable reference or transport.
- im6014_serial.py keeps bench exchange behavior and adds optional before_write/on_write hooks. The final gate runs after reset/timeout setup directly before write. False cancels with no write. Actual write result is reported even if later feedback times out.
- station_timing.py owns bounded actual loop/transaction measurements. C15 first proves it through a fake adapter; it does not depend on an unfinished C16 daemon.
- joint_station.py and station_protocol.py own the C16 single-thread fixture process/lease and bounded client IPC respectively. No physical executable or D01 profile schema is introduced.

```python
@dataclass(frozen=True)
class IntentWindow:
    id: int
    created_at_s: float
    expires_at_s: float

@dataclass(frozen=True)
class Target:
    sequence: int
    joints: tuple[JointTarget, ...]
    window: IntentWindow
    # C08 adds required OwnerToken, with epoch, generation, connection ID.

class JointCore:
    def issue_window(self, valid_for_s: float) -> IntentWindow: ...
    def submit_target(self, target: Target) -> Decision: ...
    def prepare_refresh(self) -> RefreshPlan | None: ...
    def claim_dispatch(self, plan_id: int, index: int) -> bool: ...
    def dispatch_result(self, plan_id: int, index: int, result: WriteResult) -> Decision: ...
    # Existing lifecycle and snapshot; consume_target is deleted.
```

Sketch bodies remain unimplemented in this document. Per-family types become required only in their milestone, with all callers migrated in that unit. No default unbounded limits or guessed physical thresholds. Explicit fixture configurations supply values. Configuration/build/observation metadata can be retained as fixture identifiers; D01 profile parsing remains later work.

## Time, admission and authority

Retain the existing floating-point monotonic clock API for compatibility with deterministic fixtures. Track its maximum observed valid time globally; reject invalid/regressing time before overwriting observations or releasing motion. Preserve finite-clock errors in snapshots rather than fabricate fresh ages. Station catches a clock failure, records it and Stops. A later valid clock must be at least the previous high-water mark.

Windows have finite positive bounded lifetime, station-issued ID and exact creation/expiry. Equality with expiry is expired. The core stores only the latest offered window and the admitted goal's immutable window. Issuing a newer window invalidates older incoming responses but does not renew/erase the already admitted goal. Expiry of an admitted goal withdraws motion and Stops; an invalid incoming replacement preserves an otherwise valid goal and watermark. Required safety housekeeping still expires/faults old goals. Recheck expiry at preparation and before every mode-1 write, not only admission.

C08 ownership adds a fresh core/process epoch, monotonic uint64 generation and server-generated connection identity. Stop, fault, disconnection, takeover and reconnect withdraw goal/plans and revoke authority; no automatic rearm. Old generation/epoch/connection requests reject. Sequence watermark remains globally increasing across those transitions and never wraps/resets; clients learn it from snapshots. Exhaustion rejects motion until a fresh core/epoch. A client cannot manufacture another connection's authority through JSON fields.

All admission checks happen before replacing the complete goal/watermark: domain types/count/order, window/owner, full mapped codec representability, inclusive position limits and later requested lead/effort checks. Rejected high sequences cannot consume ordering. Prepared plans are complete and bounded; replacement/revocation invalidates them. A complete write commits only that joint's actual reference record. Partial/unknown write faults and requires explicit fresh Stop recovery. Logical MOVING requires a confirmed mode-1 write; neither a prepared plan nor an admission is dispatched evidence.

## Discrete reference and effort contract

Initialize reference position from fresh healthy checked Stop measurement and reference velocity zero. For positive elapsed h since the previous reference step, choose endpoint velocity v in the closed intersection:

    [-V, V] intersection [v_prev-A*h, v_prev+A*h]
    intersection [(q_min-q_prev)/h, (q_max-q_prev)/h]
    intersection [(q_measured-lead-q_prev)/h, (q_measured+lead-q_prev)/h]

Then q_next=q_prev+position_step_velocity*h, where position_step_velocity is the exact rational decoded emitted-position difference divided by the actual float cadence h. The independently quantized encoded command velocity is a separate quantity. Apply speed and acceleration bounds to both streams using their respective confirmed prior values. Never use encoded velocity as the position-step velocity. Project the desired position/velocity objective into that interval; define the selected objective in the API docs. Position request limits reject outside inclusive bounds. Requested lead outside its documented bound rejects, while the actual reference also stays bounded at each preparation/final gate. C10 adds V; C11 adds acceleration interval; C12 adds measured lead. At zero elapsed time, only the same admitted joint goal may replay the exact cached packet, decoded target and full reference history; a fresh Stop anchor has no packet to replay, and a changed goal is rejected. Excessive elapsed gap withdraws motion instead of allowing unlimited catch-up. Reversal preserves previous velocity and crosses zero under the same acceleration bound. Empty intervals/unsafe finite arithmetic generate no reference and latch an attributed constraint fault with Stop. Never clamp q after choosing v or silently relax a bound.

These are samplewise software reference constraints. They establish no continuous motor trajectory, physical acceleration or certified braking. Timing and actual write records remain separate.

Before any write, validate every shaped mapped packet through the canonical codec. Codec quantization can cross a closed position or effort bound. Keep codec scales in the codec; expose decoded encoded-command values there if necessary, map back once and check actual encoded endpoints/effort against inclusive limits. Choose an inward representable value when possible, otherwise withdraw motion. Do not disguise a quantization overshoot with an arbitrary safety tolerance or copy codec scales into the limiter. Recheck fresh measurement, expiry, owner and limits at final gate. No source caller can bypass constraints via the deleted consume API.

Predict tau_ff+kp*(q_ref-q_measured)+kd*(dq_ref-dq_measured), including gains after encoded-command quantization. Preserve requested/admitted, shaped accepted, confirmed dispatched and measured records distinctly. Monitor measured joint effort independently. C13 trips configured peak effort; C14 adds continuous-effort dwell, explicit driver/winding temperature trip/reset bounds and attributed fault union. No defaults claim physical protection. Recovery requires explicit acknowledgment followed by complete fresh healthy Stop telemetry below recovery thresholds for the configured dwell; new targets, reconnect and healthy packets alone cannot clear latches. Missing temperature telemetry never proves healthy recovery.

## Station and timing

The C16 executable accepts explicit validated /dev/pts endpoints only, with deterministic fixture actuator semantics. It opens through the existing fixed managed lease namespace and retains the lease until confirmed close. Unconfirmed closure leaves a retryable shutdown with owner held. Clients never launch/restart it automatically. One active connection owns motion; a bounded priority control request can Stop/take over. Serve Stop/takeover/EOF/liveness before targets and each write gate, with bounded reads/accepts per iteration. Packet size/joint count/depth are fixed and oversized/truncated/malformed messages reject before mutation. Directory/socket permissions restrict to the same user. Slow clients/backpressure cannot block refresh; disconnect revokes and Stops.

Client crash must leave the station alive with its lease, Stop state and no rearm. A second cooperating opener fails. Restart uses a fresh epoch, fresh Stop checks and explicit arm. The existing bench GUI remains a separate legacy bench path; C16 tests a dashboard/client process against the new fixture station, and the managed lease excludes that bench opening the same PTY. UI migration is later E-series work, not a fabricated production feature.

Report actual start-to-start period, per-joint transaction start/end duration/outcome, completion overrun/missed slots, feedback age and sampling skew. Skip missed slots instead of bursting. Records/counters stay bounded. Nominal period is configuration, not measured rate. C15 deterministic tests validate arithmetic; parent real-clock fixture runs (five repeats) provide separate observed timing with work/error counts and host-load context. PTY/fake timing is not physical RS485 throughput.

## Verifiable units

One fresh Gemini writer per C07-C16 milestone, serialized from the accepted parent tip. Parent reads full diff/output/receipt, runs targeted and affected regression tests, records hashes/logs and commits before the next unit. Scope and test entry points are registered in check_milestone, the offline copyset/startup fingerprint and CI as each module appears. C15 fake timing works before C16 process integration. Final panel reviews, full offline/UI checks and nonwriter independent verification precede the private PR and guarded merge. All model outputs/receipts and parent selection notes are retained with requested-route evidence, not independent provider identity claims.

## Quantization decision after executable sketches

Choose emitted targets with two named velocity streams. Retain actual decoded emitted JointTarget, exact rational position_step_velocity and preparation sample time per joint only after its full write. Retain actual claim time separately. Initialize a fresh healthy Stop anchor with zero velocities; it is not a dispatched record. Codec owns raw grids, canonical decoding and bounded inward projection. Cache validated canonical bytes and send them unchanged; no decode/re-encode drift. Certificates use exact rational values of stored finite floats, without arbitrary epsilon.

Sol's pure grid proof passed parent execution; Gemini's draft proof failed its own overshoot assertion. Parent separately checked its false noninteger-ratio inference and reproduced 100 unchanged samples with a too-small acceleration interval. Evidence is in evidence/control-limits/quantization. Neither model's self-report established execution.

For h>0, an exactly zero objective asks for no net position correction after combining position error and velocity feedforward. The controller still projects zero into the feasible interval, so an existing nonzero derivative can require continued bounded motion while decelerating. A stationary reference is possible only when zero is feasible. For a nonzero objective, if no representable position step satisfies all closed constraints, Stop with an attributed constraint reason. Do not repeatedly commit unchanged writes and imply progress, hide fractional virtual motion, accumulate elapsed time contrary to actual write history, or relax bounds. A zero-objective request and an h=0 cached refresh remain distinct. C10-C12 must prove useful progression at feasible fixture cadence and conservative Stop for infeasible cadence. There is no physical trajectory guarantee.
