Review of supplied text only. No implementation, no tests run.

Intent: C07-C10 in existing thread-confined transport-free `JointCore`, preserve C06 sequence/latest-only/fault guarantees, add mandatory expiry/ownership, configured joint position + reference velocity limits. No physical I/O / production station.

Candidates:
A: `design-sol.md` — unified core, extend immutable `Target`, one pending slot + reference state in `JointCore`.
B: `design-parent.md` — separate `CommandGate` + `ReferenceGenerator`, keep `Target` as payload, `CommandEnvelope` + `OwnerToken` + `JointLimits`.
Gemini seat: failed preflight, no design. Not scored, no substitution.

Scores vs rubric:

1 dispatch-time expiry + owner revocation:
 A: strong. Submit checks `created_at_s <= now < expires_at_s`, consume re-samples one `now`, rechecks expiry+ownership before release, equality=expired, expired clears slot retains watermark no fault, no fake bytes. Revocation on stop/fault/disconnect/takeover, identity-matched capability, generation advances once if live, failed ops never mint, no implicit rearm on reconnect/check. Matches grounding: stop/fault/disconnect clear pending retain watermark.
 B: partial/fail. Envelope has absolute deadline, gate revocation + clear both slots required, polls until deadline. No equality rule, no single-`now`, no accepted-vs-issued time, no identity-vs-equality, no failed-arm/check semantics, no generation lifecycle.

2 lifecycle/latest-only/sequence:
 A: pass. One pending request + acceptance time + reference pos + last-issue time, no queue/per-owner dict/timer, preserve malformed-before-duplicate/older, only last step commits, any failure preserves prior pending/accept/watermark/reference, one-shot consume, no autonomous trajectory.
 B: fail, self-admitted. Station must coordinate three mutable owners: gate token+envelope, core slot, generator vector+time. Raw `JointCore.submit_target` remains callable bypass, clearing one slot leaves another active trajectory. Fails split-ownership / two-ways-to-do-one-task screen. Sequence across gate vs core watermark undefined.

3 position + reference-velocity incl. streaming cadence, clock regression, quantization:
 A: pass with gaps. C09 closed configured joint-radians intervals, no clamp, endpoints iff codec accepts, arm seed rejects out-of-range no clipping, whole-vector atomic, reduction 1/3/6 both directions offsets covered in matrix. C10: `dt=min(now-last_issued,max_reference_step_s)`, per-joint `q_issued=q_ref+clamp(q_req-q_ref,±v*dt)`, `dq_issued=clamp(dq_req,±v)`, `accepted_at_s` no banking, `now>=accepted_at_s`, zero-dt holds, second consume None no time update, idle capped, recheck mapping/codec before commit atomic. Incomplete on quantization + regression, see below.
 B: fail/underspecified. Seed rejects outside interval, limit displacement by `v*elapsed`, no banking before arrival, bounded `dq`. No `max_reference_step_s` cap so long-idle unbounded unless assumed, no accepted/issued distinction beyond arrival timestamp, no per-joint/sign, reversal, mapping-direction, atomicity, regression, or codec rounding rule.

4 smallest coherent API without bypasses/split state:
 A: pass. Frozen `PositionLimits, Ownership, ArmDecision, Target+deadlines+ownership, Reference{requested,joints,accepted_at_s,issued_at_s}`, `JointSpec` +limits+`max_velocity`, `JointCore(max_reference_step_s, arm(owner))`, +`EXPIRED,INVALID_OWNERSHIP,POSITION_LIMIT`, snapshot `ownership/generation[/last_reference]`. No wrapper, validation/limiting in core, `joint_mapping.py` pure, `im6014_codec.py` boundary, `arm_manual.py`/bench untouched.
 B: fail by own statement: “would choose unified core if it can expose one required command contract.” Larger contract, repeats lifecycle, changes callers anyway.

5 deterministic verification + full caller migration:
 A: strong directionally. Maps `joint_controller.py:552,592,605,615`, `test_controller.py:25,53,59` to read `Reference.joints` never `requested`, `test_controller_targets.py:18,452`, `docs/joint-api.md:77, controller-design, architecture`, README, `scripts/check_offline_baseline.sh:81`, `test_startup.py:112`, `scripts/check_milestone.py:10` CHECKS, per-row C07-C10 matrix with fake time + packet checks. States no tests passed.
 B: partial. Lists fake-time checks: equality, queue expiry, delayed previous-owner, reconnect/failed takeover, atomic rejection, ±bounds, repeated steps, long idle, zero elapsed, regression, packet inspection of bounded refs not limiter flag. No caller/docs/script migration.

Base selection: A unified core. B cannot be base: blocking bypass + split state violates thread-confined single-owner intent and C05 consumed-once/latest-only.

Useful grafts from B to A (already partly taken):
- accepted-goal vs dispatched-reference distinction, incl. target-arrival timestamp → A’s `Reference.requested` + `accepted_at_s` + `issued_at_s`.
- packet assertions on actual bounded refs, not limiter flag.
- explicit check list: exact equality expiry, queue delay no deadline refresh, previous-owner delayed delivery, failed takeover, atomic vector rejection, long-idle arrival, zero elapsed, regression after valid feedback.

Concrete blocking deficiencies to fix in A before C07:
1. Staging contradiction: shape shows final mandatory fields, text says implement only current row, no placeholders. Define per-row signatures. `max_reference_step_s`, `PositionLimits`, `max_velocity`, `created/expires/ownership`, `Reference` return cannot all be required at C07. Same for `consume_target()->Reference|None` comment “returns Target through C09” — that is a mid-batch breaking return change requiring two migrations of same sites.
2. Decision precedence: “expiry/ownership may precede mapping after shape” + keep malformed-before-duplicate/older is non-deterministic for combined failures. Need total order: shape → expiry → ownership → position-interval → mapping/codec → sequence, with stated codes incl. future `created_at_s` → `INVALID_ARGUMENT`.
3. Clock regression: “verify no regression vs reference time, require now>=accepted” undefined on violation — exception from `_age` vs `Decision`, which code, single-`now` use with `tick()` fault path. Source `joint_controller.py:_now/_age/tick/consume_target` raises on non-monotonic; tests need deterministic expectation.
4. Quantization inward rule: “use inward representable values, keep knowledge in codec” does not name `project_position_raw/project_speed_raw` with `map_decoded_*+direction`, nor velocity projection. Recheck via `mapping.to_output+command()` alone is round-nearest, insufficient to enforce closed joint limit after decode.
5. Arm seed representability: failed arm for pose outside interval specified, but seed that maps to unrepresentable `command()` — code/state/generation effect unspecified. Ditto snapshot `ownership=None + ownership_generation` lifecycle after revocation needs explicit invariant.
