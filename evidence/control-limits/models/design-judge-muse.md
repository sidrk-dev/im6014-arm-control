Score table (1-5):

| Criterion | Sol: core-owned goals + guarded refresh | Gemini: integrated core + async daemon |
|---|---|---|
|1 One owner / irreversible dispatch gate|5|2|
|2 Coherent reference constraints|5|2|
|3 Interface depth / maintenance|4|3|
|4 Real station independence|5|2|
|5 Testability / measurable evidence|5|2|
|6 Incremental completion|5|3|

1. One owner and irreversible dispatch gate
Sol: Station-issued IntentWindow(id,created_ns,expires_ns); IPC supplies ID, never client timestamps. Expiry exclusive now>=expires_ns checked at admission, refresh, and immediately before every write via priority_then_claim + claim_dispatch(plan.id,index). Takeover stops first, revokes generation/window/goal/prepared refs. Owner(station_epoch 128b, generation never wraps, connection_id); restart new epoch. Global sequence watermark across owners/Stop/faults/reconnect; exhaustion rejects. Invalid replacement preserves still-valid goal, but separate housekeeping transition prevents preserving already-expired/faulted goal. dispatch_result records complete/partial/failure; refs advance only on confirmed complete write. Superseded plans cannot refresh. Adapter cannot write raw pending goal.
Gemini: RequestedAction(sequence,owner_generation,ttl_s) uses client-supplied relative TTL. No epoch/nonce, no window identity, no connection binding. Generation int only: restart replay not prevented despite C16 test claiming epoch rejection. No per-write final gate; dispatch_reference->mapping->write has TOCTOU for expiry/revoke/takeover/Stop. No partial-write record, no plan IDs, no invalid-vs-expired preservation rule.

2. Coherent constraints
Sol: Pure joint_reference.py analytic closed interval: I=[-V,V] cap [v0-Ah,v0+Ah] cap lead-derived [v0+2h*(a-q0-v0t)/t^2,...] with t->0 limit, extrema at t=2*(bound-q0)/v0, braking inequalities q1-max(-v1,0)^2/2A>=a etc. Project v_des=(2*(q_goal-q0)/h -v0+dq_goal)/2, q1=q0+h(v0+v1)/2. Zero time only unchanged refresh; gap>fixture max => Stop. Empty/outward/unsafe => no reference, infeasibility fault + Stop. Quantized shaped packet decoded back and rechecked. Effort affine in v1 intersected with I, zero-coeff check; independent peak/immediate + continuous/dwell + temp trip/reset/dwell, unioned, acked healthy-Stop recovery only.
Gemini: project_kinematic_step(q_target,dq_target,q_ref,dq_ref,q_meas,dt) omits measured velocity, integrated position, extrema, braking viability, zero-time/gap, quantized revalidation. Position/lead as atomic reject only, no shaping interaction. Reversal as lower-bound clamp cannot guarantee zero-crossing or interior overshoot handling. Effort submit-reject only in C13; dwell/union deferred without counters; no stale/wrong-ID lead invalidation or final-gate recheck.

3. Interface depth
Sol: joint_contract / joint_controller / joint_reference stateless / station_protocol fixed binary / joint_station + preserved codec/mapping/port/serial with optional before_write/outcome hooks; bench paths preserved; consume_target deleted. 6 cohesive modules, no profile/D01, no event bus. Cost: plan/claim/dispatch_result surface larger but justified.
Gemini: 4 modules (kinematics/controller/daemon/client) cohesive, deletes consume_target. Weaker: JSON length-prefixed frames risk nonfinite/unbounded parsing; JointTarget carries kp/kd per-request without per-joint limit binding clarity; daemon now/dt duplicates core _last_now without high-water rule.

4. Station independence
Sol: Private same-user SOCK_SEQPACKET, every v1 frame exactly 2048B, 15 fixed slots, zero padding; reject truncation/nonfinite/malformed before admission. Bounded: 1 owner, 1 takeover candidate, 1 target slot, 1 window; bound accepts/reads/buffers, close slow handshakes. Priority Stop/takeover before target reads and each write gate; cannot interrupt running sync driver call recorded. Station owns refresh, never spawned by client. C16 PTY-only validated fixture + deterministic actuator, retains managed-lease namespace. Confirmed close requires is_open is False else retryable shutdown lease-held; death releases fds, replacement stopped new epoch.
Gemini: Private Unix stream 0700, length-prefixed JSON, no max size, no per-iteration bounds, no priority lane, no lease-close distinction, no PTY-only validation, PortConfig(/dev/ttyUSB0) invites hardware bypass. Disconnect Stop/no-rearm claimed but without liveness/backpressure/revocation rules. Default nominal_period 0.010 / 1.6ms-per-motor estimate is unsupported; violates no-guessed-timing rule.

5. Testability / evidence
Sol: Deterministic negatives: equality expiry, delayed computation, active-goal expiry, invalid preservation, regression before receive/snapshot/dispatch, no mode-1 writes; takeover during batch; exact boundaries + epsilon; all-joint atomicity; reversals/zero-cross/interior/boundary/empty; lead both directions/narrow/empty/stale+final-gate; stiffness/damping/ff cancellation/zero-coeff/quantization; transient vs persistent/missing/simultaneous/healthy-no-recovery/acked recovery; delays/skew/overrun/missed/age/bounded storage; malformed/oversized/priority/kill-client lease-retained/kill-station epoch/no-rearm/unconfirmed-close. Timing: start-to-start, per-joint start/end latency, deadline miss, missed-slot skip, bounded last/max/counters, labeled provenance, no wire-time rate inference. Truthfully no tests run.
Gemini: Only happy-path + single reject per milestone; missing equality, invalid-preservation, clock regression, order/count, zero/gap, interior/empty, stale/final-gate, quantization, missing/simultaneous, malformed/oversized, priority-during-batch, epoch, unconfirmed-close, bounded storage. Metrics nominal/actual/latency/overruns only, no per-joint latency, skew, age attribution, skip policy, or provenance.

6. Incremental completion
Sol: C07-C16 ten serialized single-writer units from parent tip, parent-run offline baseline before commit/dispatch, per-unit source/test groups + fingerprint + explicit invocation + CI, preserve bench/lease regressions, update API/architecture with checked evidence, preserve JointCore thread-confinement/transport-free and lease/encoding contracts.
Gemini: Ten units listed with file additions to check_offline_baseline.sh/fingerprint and consume->dispatch migration, but no writer serialization, no parent-owned test/Git/hardware rule, no copyset enumeration, no bench preservation statement, no architecture-evidence update.

Recommendation
Base: Sol. Only design with window-bound expiry at actual write, analytic feasible interval + braking + quantization recheck, bounded fixed-frame IPC + priority gate, correct lease-close and epoch handling, and measurable timing.

At most 3 grafts from Gemini:
1. DecisionCode string enum (EXPIRED/STALE_GENERATION/POSITION_OUT_OF_BOUNDS/LEAD_EXCEEDED/EFFORT_LIMIT_EXCEEDED/CLOCK_REGRESSED) for readable receipts.
2. poll_snapshot convenience returning state + LoopMetrics for clients, mapped onto Sol snapshot.
3. selectors-based non-blocking daemon loop pattern as implementation detail inside Sol bounded-iteration listener.

Critical gaps parent must resolve (both are sketches, no physical validation):
- Effort/thermal truth: measured effort convention, gain compatibility, Kp unit error, conservative predicted/peak/continuous + temp trip/reset/dwell values unverified; prediction is guard not certified protection.
- Timeout-ack firmware disagreement unresolved; fake receipt cannot decide.
- Fixture JointLimits source and revisions (no profile file per Sol) plus sequence-exhaustion/generation-wrap/epoch-persistence rules.
- Achievable loop/transaction timing must be measured on PTY/fixture; 100Hz/1.6ms assumptions unsupported; define start-to-start, per-joint latency, overrun/missed-slot, age/skew provenance.
- Confirm exclusive expiry, zero-time/gap Stop, infeasibility-Stop vs reject, Stop-mode outside reference contract, and confirmed-close retryable-shutdown behavior in implementation.
