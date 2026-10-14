# C07-C10 design candidate

This is a source-grounded sketch, not an implementation or verification result. The parent owns synthesis, tests, evidence and hardware I/O. The recommendation extends the existing thread-confined `JointCore`; it adds no station daemon and leaves the independent bench and browser controllers in place.

## Usage first

The final C10 caller supplies explicit fixture limits, receives an ownership grant at explicit arm, and submits a complete immutable request. The existing fake station can consume one issued reference and use its joint tuple to build the existing codec commands.

```python
clock = DeterministicClock(0.0)
core = JointCore((JointSpec(
    name="J1", motor_id=0, mapping=JointMap(3, -1, .25),
    position_limits=PositionLimits(-.5, .5),
    max_velocity_rad_s=.2,
),), clock=clock, max_reference_step_s=.02)
station = FakeStation(core)
assert station.connect()
assert station.check()                 # Fresh healthy Stop fixture at q=.1.
grant = station.arm(owner="teleop")
assert grant
assert grant.ownership is not None

clock.value = .01
request = Target(
    sequence=0, joints=(JointTarget(.4, .8, 0, 1, .1),),
    created_at_s=.01, expires_at_s=.11, ownership=grant.ownership,
)
assert station.submit(request)
reference = station.consume()          # One issued record, not the request.
assert reference.requested == request
assert reference.joints[0].q_rad == .102
assert reference.joints[0].dq_rad_s == .2
assert station.consume() is None
```

That .2 rad/s limit and interval are fixture choices, not proposed physical settings. Use approximate numerical assertions in executable examples.

A streaming producer continues its sequence and keeps the same grant. The station may submit and consume on the same tick.

```python
# The prior reference was issued at .01 with q=.102.
clock.value = .02
assert station.submit(Target(
    sequence=1, joints=(JointTarget(.4, .8, 0, 1, .1),),
    created_at_s=.02, expires_at_s=.12, ownership=grant.ownership,
))
reference = station.consume()
assert reference.joints[0].q_rad == .104 # dt=.02-.01; the stream advances.
```

Takeover uses the existing Stop/check/arm lifecycle. It does not need a second ownership API.

```python
old = grant.ownership
assert station.stop()                  # Revokes old, clears pending, disarms.
assert station.check()                 # Fresh healthy Stop observation required.
grant = station.arm(owner="human")     # Fresh explicit arm mints new generation.
assert grant and grant.ownership is not old
assert not station.submit(Target(
    sequence=2, joints=(JointTarget(.2, 0, 0, 1, .1),),
    created_at_s=clock.value, expires_at_s=clock.value + .1,
    ownership=old,
))
```

The producer stamps creation before any queue or asynchronous computation. Submitting late must not refresh its deadline. These timestamps belong to the station's monotonic clock domain; later remote adapters must retain station-issued deadlines, not compare clocks from separate hosts.

## Shape and signatures

All new records are frozen dataclasses. Constructors reject booleans as numbers, non-finite values and wrong record/tuple types. `Target` retains the existing uint64 sequence and ordered nonempty tuple checks.

```python
@dataclass(frozen=True)
class PositionLimits:
    minimum_rad: float
    maximum_rad: float                 # finite, minimum <= maximum

@dataclass(frozen=True)
class Ownership:
    owner: str                        # nonempty identity, no transport object
    generation: int                   # actual positive int, never reset

@dataclass(frozen=True)
class ArmDecision(Decision):
    ownership: Ownership | None = None # present iff accepted

@dataclass(frozen=True)
class JointSpec:
    name: str
    motor_id: int
    mapping: JointMap
    position_limits: PositionLimits   # required beginning at C09
    max_velocity_rad_s: float          # required positive finite at C10

@dataclass(frozen=True)
class Target:
    sequence: int
    joints: tuple[JointTarget, ...]
    created_at_s: float                # required beginning at C07
    expires_at_s: float                # finite and > created_at_s
    ownership: Ownership              # required beginning at C08

@dataclass(frozen=True)
class Reference:
    requested: Target                 # preserve requested/issued distinction
    joints: tuple[JointTarget, ...]    # complete issued values
    accepted_at_s: float               # core-stamped, not caller supplied
    issued_at_s: float                 # same clock domain

class JointCore:
    def __init__(self, joints: Iterable[JointSpec], *,
                 feedback_max_age_s: float = .5, clock=None,
                 max_reference_step_s: float):
        raise NotImplementedError     # required positive finite at C10

    def arm(self, *, owner: str) -> ArmDecision:
        raise NotImplementedError     # owner required beginning at C08

    def submit_target(self, target: Target) -> Decision:
        raise NotImplementedError

    def consume_target(self) -> Reference | None:
        raise NotImplementedError     # returns Target through C09

    # Existing connect/check/receive/tick/stop/disconnect remain.
```

Add `EXPIRED`, `INVALID_OWNERSHIP` and `POSITION_LIMIT` decisions. Future creation time returns `INVALID_ARGUMENT`. Snapshot adds `ownership: Ownership | None` and `ownership_generation: int`. At C10 it may add `last_reference: Reference | None` to expose the bounded result. It retains `pending_target` as the original request and retains the core-lifetime `last_sequence` semantics.

The core mints an ownership object only after successful explicit arm. Match the exact minted capability by object identity, not just equal owner/generation fields. An independently constructed equal record and a grant from another core are inert. This is a local capability within the existing thread-confined API, not an authentication or network token. Snapshot and arm results may expose that same frozen object to trusted local callers. Revocation drops it and advances the generation once when a live grant exists; failed operations never mint a grant. A later arm advances again. Python integers need no generation rollover policy.

The core keeps one pending request plus its acceptance timestamp and a reference position tuple plus last issue time. No queue, per-owner dictionary, background timer or second mutable state owner is needed. Validation and limiting stay inside the core, per boundary discipline and single ownership. The caller supplies policy configuration, lifetime, sequence and an arm grant; one submit/consume pair hides the enforcement.

## Exact behavior

Submission keeps current lifecycle and freshness guards. After complete target shape validation, validate `created_at_s <= now < expires_at_s`, current ownership, every configured position interval and every existing mapping/codec constraint, then strictly increasing sequence. Only the last step commits the pending request, acceptance timestamp and watermark together. Any failed submission preserves the prior pending request, acceptance timestamp, watermark and reference state. Keep the existing rule that a malformed or unrepresentable vector rejects before duplicate/older ordering. Expiry and ownership checks may precede mapping work after shape validation.

Consumption samples one finite monotonic `now`, performs the existing per-joint health/freshness fault checks with that same timestamp, and rechecks pending expiry and ownership immediately before release. Deadline equality is expired. An expired request clears the pending slot, returns `None`, retains sequence and does not invent a motor fault. A stale joint still latches its existing attributed fault. No rejected consume may build fake FOC bytes.

At C09, reject the whole vector when any requested q lies outside its closed configured joint interval. Do not silently clamp requests. Exact endpoints are accepted if the codec also accepts them. These are mechanical configuration values in joint radians, independent of reduction, direction, encoder range or firmware defaults. Before C10 arm seeds a reference, it must reject a measured pose outside that interval; it must not clip an out-of-range measurement into a different seed.

At C10, successful arm seeds q_ref from every fresh measured joint sample and stamps last issue time at arm time. Arm must also ensure the seeded positions can form representable commands. Each issuance uses:

```text
elapsed = now - last_issued_at_s
dt = min(elapsed, max_reference_step_s)
step_i = max_velocity_i * dt
q_issued_i = q_ref_i + clamp(q_requested_i - q_ref_i, -step_i, +step_i)
dq_issued_i = clamp(dq_requested_i, -max_velocity_i, +max_velocity_i)
```

`tau_nm`, `kp` and `kd` retain the accepted values. Recheck every generated joint value through the existing mapping/codec before committing any reference. Commit q_ref, last issue time, clearing pending and logical MOVING only after the complete candidate succeeds. An unexpected generated mapping/codec rejection emits nothing and cannot partially update the reference. It can discard the unusable pending request while retaining the watermark and previous reference.

Acceptance stamps `accepted_at_s` and verifies no regression relative to the current reference time; it does not reset or advance q_ref or last issue time. Consumption requires `now >= accepted_at_s`. Do not use `now - accepted_at_s` as the sole displacement budget: newest-target streaming often accepts and dispatches at the same instant and would stall forever. Elapsed time before acceptance and queue time can jointly grant at most the configured step cap. They cannot accumulate a larger idle jump. At .01 and .02 the example advances .002 rad each time; after a ten-second idle it advances at most .004 rad with the same .02-second cap, provided all feedback and the new deadline are valid.

For elapsed zero, q stays at the seed/previous reference and desired velocity still obeys its independent cap. A second consume with no request returns `None` and does not update reference time. Each consume remains one-shot; callers must submit fresh requests to continue progress. This batch does not create an autonomous trajectory or refresh loop. Reference velocity and feedforward limits do not certify physical speed, acceleration, tracking error or effort.

This candidate guarantees bounds in the joint reference domain. Packet fixtures must separately decode position and speed near interval/rate boundaries and state the exact quantization uncertainty. They must not describe those bytes as an exact mechanical or velocity guarantee. The existing `im6014_codec.project_position_raw` and `project_speed_raw` already choose inward representable codes; reuse them if synthesis requires exact decoded packet bounds and can preserve the transport-free result. Do not add hand-coded bit layouts to `JointCore` or enlarge the public API for raw codes. Physical speed and mechanical protection remain unverified.

## Lifecycle changes

| Event | Ownership and reference outcome | State outcome |
|---|---|---|
| Successful explicit arm from READY | Mint fresh capability; at C10 seed measured q and current time | ARMED |
| Failed arm, including measured pose outside configured interval | No grant, no changed generation/reference | Existing state |
| Submit/consume | Require current capability; one pending slot; sequence retained | Existing accepted behavior |
| Stop, including takeover's first operation | Revoke capability; clear pending and reference history | STOPPED, or existing FAULT retained |
| First attributed fault while owned | Revoke once; clear pending/reference; retain cause and sequence | FAULT |
| Repeated fault or Stop after revocation | No new authority; no repeated reference reset to measurements | Existing fault/disarmed behavior |
| Disconnect, including repeated disconnect | Revoke live capability; clear pending/reference and observations | DISCONNECTED, faults/sequence retained |
| Reconnect/healthy receive/acknowledged successful check | No grant and no reference seed; check may clear existing cause | CONNECTED/CHECKING/READY |

Old capabilities remain invalid after new check/arm even when the owner name is reused or the old request has a much larger sequence. Stop/check/arm is the explicit takeover operation; a caller cannot change the owner while ARMED or MOVING simply by calling arm again.

## Actual callers and module map

`joint_controller.py:552` validates requests, `:592` consumes once, `:604` stops, and `:614` disconnects. All authority and reference state belongs here. `joint_mapping.py` stays a pure conversion module; `im6014_codec.py` remains the packet/quantization boundary. No production serial imports are added.

`test_controller.py:25` defines the only station adapter, `:53` delegates arm, and `:59` constructs actual FOC bytes after consume. Migrate its arm signature, every `JointSpec`, `core_at`, and target fixture construction. At C10 its byte builder reads `Reference.joints`. It may return the reference for assertions but must never encode `Reference.requested.joints`.

`test_controller_targets.py:18` holds C05 tests, `:453` holds C06 tests. Migrate every request to mandatory deadline/ownership fields and every spec to mandatory limits. Replace equality of consumed request with equality of `reference.requested`, and independently assert the issued joints. Tests with a frozen zero clock must either expect zero position displacement or advance time deliberately. The 10,000-update test still proves one pending request, one consume and the newest valid sequence; it cannot assume a jump to the newest q after C10.

`docs/joint-api.md:77` is the executable-style API example and must migrate with the affected milestone. Update `docs/controller-design.md`, `docs/architecture.md` and README controller description to match the completed behavior. `arm_manual.py` has its own ManualJointSpec, ownership, gates and worker loop; it imports no JointCore and is not a caller. The Qt bench uses MotorSession. Their entry points and lifecycle remain untouched.

New test modules must enter `scripts/check_offline_baseline.sh:81` copyset and its unittest invocation, `test_startup.py:112` fingerprints, and `scripts/check_milestone.py:10` CHECKS. Existing scripts currently list only C01-C06. Updating these explicit audit lists is required; the parent owns their execution and source receipts.

## Milestone sequence and acceptance matrix

Implement only the current row, migrate its callers, then stop for parent source review and the named check before implementing the next row. Do not introduce inactive placeholder limits or optional ownership fields to make later stages compile.

| Unit | Minimal contract change | Required behavior checks |
|---|---|---|
| C07 | Required Target creation/expiry; EXPIRED; preserve Target return | Timestamp zero; invalid/bool/nonfinite/reversed deadlines; future creation; equality expired; producer queue delay; valid submit then expiry at consume; expired higher sequence preserves old pending/watermark; discard at consume retains watermark; wrong state and stale feedback retain existing semantics; fake accepted bytes and zero expired bytes |
| C08 | Required ownership; arm owner and ArmDecision; immutable grant snapshot | Current owner accepted; old high-sequence request rejected after Stop/check/arm, takeover, packet fault, communication fault, stale tick and disconnect/reconnect; same name receives fresh generation; equal fabricated token and other-core grant rejected; new owner cannot arm while moving; failed check/arm never grants; fault latch still needs explicit acknowledgment; watermark remains core-lifetime |
| C09 | Required PositionLimits per JointSpec; POSITION_LIMIT | Both endpoints and adjacent outside values; asymmetric intervals; zero-width interval; bad configuration; reduction 1/3/6, both directions and nonzero offsets; second-joint rejection is atomic; codec-invalid in-range target still rejects; all prior target/ref/watermark values preserved; fake bytes correspond to accepted interval values |
| C10 | Required max velocity and max step; Reference return; seed/limit state | Seed from nonzero measured pose; immediate zero-dt issuance; submit+consume at successive same ticks makes progress; per-joint/sign velocity differences; positive/negative jumps; direction reversal; requested feedforward above both bounds; delayed acceptance and delayed consume cannot bank beyond max step; idle with refreshed observations produces bounded step; no-pending consumes do not change time; only accepted issuance advances reference; limit/mapping failures are atomic; Stop/fault/reconnect requires fresh arm and reseed; decoded codec-boundary packets respect documented rounding rule |

Across each row, retain C01-C06 negative checks and positive packet checks. The parent records actual command, source hashes, exit status, raw log and remaining physical dependency in the ledger. This candidate reports no tests as passed.

## Whole-shape alternatives and rationale

1. Extend immutable Target and keep all enforcement in JointCore, as recommended. A producer supplies one complete record and obtains one bounded result. Lifecycle, latest target, ownership, deadlines and reference history share one thread-confined owner. The reference introduces one necessary record to distinguish requested from issued values. No public unfiltered dispatch route survives.
2. Keep Target as a pure requested vector and put deadlines/ownership into a CommandEnvelope owned by a separate command gate, with a separate ReferenceFilter after JointCore. The station would coordinate gate acceptance, core submission, gate dispatch recheck, filter seed/step/reset and codec writes. That shape can isolate a reusable filter, but callers own the ordering and must clear several objects on every fault/Stop/reconnect. Keeping the raw core callable creates a bypass; hiding it behind a new wrapper changes every caller anyway. It has a larger contract and repeats lifecycle knowledge without an independent service or release need. If the parent chooses an envelope as a private record inside JointCore, that is an implementation detail of the first shape, not this separate-layer alternative.
3. Replace the one-shot pending slot with a retained goal and expose advance_reference() on every refresh tick. That handles continuous ramping between producer updates and can hide the future station cadence, but it changes C05's consumed-once behavior and introduces continued motion from one retained request. It requires an explicit policy for goal expiry, refresh, quiescence and build-specific stop. Defer it until the station/refresh work demands it.

Accept one-shot bounded references in exchange for preserving the proven latest-target contract and avoiding an unrequested trajectory engine. Accept a mandatory finite time cap in exchange for giving up unbounded catch-up after scheduler stalls. Keep global sequence ordering across grants to preserve C06; later independent producer streams require an explicit contract migration rather than silently resetting sequence on takeover.

The sole failed Gemini seat is not a second viable candidate. Parent synthesis must compare this sketch with its independently authored whole-shape alternative and record that route failure without model substitution. Start implementation with only C07 deadline fields, checks and caller migrations.
