# Joint mapping and controller design

B11-B14 and C01-C10 extend the bench with pure joint algebra, passive configuration, bounded packet diagnostics, cooperating bus ownership and a transport-free request core. Existing bench entry points retain their lifecycle and packet bytes. No new physical motion runner is introduced.

## B11-C06 design selection

Sol's typed core is the base. The [Muse review](../evidence/controller/models/design-muse-output.md) and [MiMo review](../evidence/controller/models/design-mimo-output.md) both selected it over the wire-aware Gemini candidate. Those reviews concern the initial B11-C06 design; their scores and requested-route receipts are in evidence/controller/models. Parent accepted named rad/s intermediates, passive OS metadata, the sequence example 1,5,5,2,6, and a transition table with guards. Parent rejected premature arm expiry, arbitrary stream dictionaries, mutable snapshot dictionaries, raw wire ownership in the core and a station daemon. Ownership generations and identity-checked local grants are part of C08; requested-position intervals are C09; bounded issued references and codec-grid checks are C10.

The reviewers missed a fault recovery gap in Sol's Stop row. Stop must preserve FAULT and its attributed cause. Disconnect/reconnect must not permit a latched fault to bypass an explicitly acknowledged recovery check. A healthy packet alone never clears it. This is a parent correction to the base.

## C07-C10 design selection

The [native design](../evidence/controller-commands/review/design-sol.md) keeps lifecycle, ownership and bounded references in one core. The [parent alternative](../evidence/controller-commands/review/design-parent.md) splits them across a gate, core and filter. [Muse selected the unified core](../evidence/controller-commands/review/crossjudge-muse-os.md). The configured Gemini design attempt failed before submission; its [receipt](../evidence/controller-commands/review/design-gemini-receipt.json) is retained.

Commands require explicit expiry and the exact current ownership grant. Joint intervals and speed limits are required configuration. Existing codec projectors choose inward representable references, and ordinary encoding must preserve the selected codes. The [synthesis](../evidence/controller-commands/review/synthesis.md) records the selected shape and reviewer corrections.

## Modules and contracts

- joint_mapping.py defines frozen JointMap, JointTarget, OutputTarget, OutputSample and JointSample. Joint units are rad, rad/s, Nm and compatible gains. Only the codec owns the internal 38/3 reduction.
- im6014_port.py defines validated PortConfig, passive inspection and ManagedPort. Both bench open sites pass their current serial.Serial factory. Ownership is acquired before construction, keyed by character-device identity in a trusted per-user directory, and held through confirmed close. Symlink aliases collide. No lockfile unlinking or PID ownership. The lock excludes cooperating workbench clients only. Constructor mocks remain injectable; PTYs provide real device identity in ownership tests.
- im6014_serial.py retains the existing three-argument exchange and adds optional bounded PacketCapture. Records include capped allowlisted metadata, request/reply bytes and decoded fields. No environment, settings or exception strings enter an export.
- joint_controller.py defines State, Decision, ArmDecision, Ownership, PositionLimits, JointSpec, JointStatus, Target, Reference, Snapshot and JointCore. Core instances are thread confined. They own a fixed joint order, observations, one current ownership capability, one pending immutable target, bounded issued-reference history and one sequence watermark. No serial import, port opening or exchange occurs inside the core.
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
| READY | arm with a non-empty owner, all active joints fresh, healthy, Stop mode, measured positions inside configured intervals and ordinary codec commands encodable for all measured seeds | ARMED, mint immutable ownership capability and seed reference history |
| ARMED or MOVING | submit, complete newer target with current grant, `created_at_s <= now < expires_at_s` and every requested position in range | Replace pending only |
| READY | arm with a measured position outside its configured interval | Reject with POSITION_LIMIT; stay READY without a grant or generation change |
| READY | arm with any measured seed that cannot pass mapping and ordinary command encoding | Reject with INVALID_ARGUMENT; stay READY without a grant or generation change |
| ARMED or MOVING | submit, any requested position outside its configured interval | Reject the whole vector with POSITION_LIMIT; retain pending target and sequence |
| ARMED or MOVING | consume, fresh healthy observations, current grant, unexpired target and a complete feasible codec-grid vector | Return bounded Reference once, MOVING and advance reference history |
| ARMED or MOVING | consume, no inward codec-grid value fits for any joint | Emit no issue, retain pending target and reference history for a later feasible tick |
| ARMED or MOVING | consume, expired pending target | Discard pending, retain watermark and state |
| Connected state except FAULT | stop | STOPPED, ownership revoked, pending cleared |
| FAULT | stop | Remain FAULT, ownership stays revoked, pending cleared |
| Active state | first attributed error, timeout or stale joint | FAULT, ownership revoked, pending cleared, cause latched |
| Any | disconnect | DISCONNECTED, ownership revoked, observations invalidated, cause and watermark retained |
| Any | unlisted transition | Rejection, no fake or physical I/O |

C02 first implements state legality and a test fixture. C03 adds per-joint clock-stamped age, including timestamp zero and wrong-ID rejection. C04 adds full error/warning/timeout attribution. C05 adds exact-length immutable target replacement. C06 enforces actual nonnegative integer sequences, strictly increasing. C07 adds required finite command creation and expiry timestamps in the station monotonic domain: future creation rejects, equality at the deadline expires, failed replacement preserves the pending target and watermark, and consumption discards an expired pending target without a fault. Dispatch freshness and expiry share one clock sample; regression raises before release. C08 adds a required owner to explicit arm, mints identity-checked local ownership grants, and revokes them on Stop, first attributed fault or disconnect. Revocation clears pending work and advances the generation watermark once; reconnect, recovery checks and healthy packets do not grant authority. C09 adds required finite closed joint-position intervals. Arm rejects an out-of-range measured pose without changing READY state or ownership generation; submit rejects the entire out-of-range vector after timestamp and ownership checks but before mapping, codec and sequence checks. C10 requires finite positive per-joint velocity and per-issue time limits. Arm verifies each original measured seed through mapping and ordinary packet encoding before requiring an inward grid code inside the interval. A planning sample bounds position change from the last issued representable joint position by velocity times elapsed time capped at the configured issue step; desired speed is clipped to its signed limit. Existing codec projectors choose inward codes, and ordinary command encoding must preserve them. After the complete vector is projected, a final monotonic sample runs freshness checks and validates active state, current ownership generation, and the exact deadline. Expiry during projection discards the pending request and retains its sequence; stale feedback follows the existing attributed fault and revocation path. Successful References stamp `issued_at_s` and reference history with that final sample. Since the candidate was bounded using the earlier planning interval, projection delay only makes the step more conservative. A clock regression during calculation raises before release. An already expired request needs one guard sample; successful projection uses planning and final-release samples. A vector is committed atomically; failed projection or a rejected final gate leaves candidate history uncommitted. Acceptance time does not reset issue history, so streaming progresses across advancing ticks while idle or queued time cannot accumulate beyond the cap. C09/C10 limits are software constraints in joint units and do not claim mechanical or physical guarantees. The core-lifetime sequence watermark survives these boundaries, and post-recovery duplicate checks use the current grant. Consumption checks authority again before release.

## Acceptance and remaining scope

Each unit is authored in an isolated worktree, reviewed and run by the parent before the next. scripts/check_milestone.py records hashes, actual checks and raw logs. New modules enter the offline copyset and startup fingerprint. The final driver runs sequentially and its images are inspected. The full controller source and decision trail go to Muse and MiMo for review before handoff. Their requested routes, terminal receipts, findings and parent triage are retained with the evidence.

Software Stop does not prove physical torque off. PTY locking and algebra do not prove adapter performance or calibration. Live multi-joint execution, physical velocity tracking, effort validity, timing, loaded-arm shutdown and station independence remain later work. Position and speed bounds constrain codec-representable software references only; they are not verified mechanical limits or measured physical motion. The pending timeout acknowledgment remains a separate hardware gate.
