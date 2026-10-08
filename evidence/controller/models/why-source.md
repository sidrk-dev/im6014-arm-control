# Bus ownership lineage for B11-B14 and C01-C06

Read-only source investigation. The parent owns implementation, Git mutations, tests, package operations and hardware I/O. This investigator ran none of those operations and dispatched no agents. This report applies to inspected workbench HEAD `d8d810888f76c676acaa1d54bc9e790249592f45`, with the parent's in-progress workflow and decision files visible. References below use `/home/sid/Documents/code/im6014Workbench` as the repository root.

## The question

Why does the existing bench use one dashboard worker to own serial communication, with an independently runnable CLI, and what must the next controller work preserve or change?

## The code in question

- `motor_session.py:131-175` constructs one worker, request queue, Stop event and cancellation generation. `:186-213` performs a transaction and updates the selected motor's state; `:215-268` handles Stop, disconnect, health checking and explicit timeout acknowledgment.
- `motor_session.py:270-292` opens a demo or real bus, sets `exclusive=True` for real serial and checks Stop/status. `:400-428` serializes requests, prioritizes Stop and disconnects during worker cleanup.
- `spin_5rpm.py:81-134` validates CLI inputs, opens its own exclusive serial connection, runs a bounded test and attempts final Stop before closing.
- `im6014_serial.py:62-108` implements one request against an already owned bus. It does not acquire station ownership.

## What we found

1. [Direct] The original application explicitly describes a single dashboard owner and serialized UI requests. The inherited `69c8219d764a24d03dadf1df011efb027f0f8996`, by Siddharth Rajasekaran on 2026-10-02, contains the module statement, "One serial owner for the desktop dashboard" and "No motor operation occurs on import or construction. UI requests are serialized." Its original exchange comment says, "One owner and one outstanding transaction; discard data older than this request." These are author-written constraints, rather than an inference from threading style. Sources: `git show 69c8219:motor_session.py:1-4`; `git show 69c8219:spin_5rpm.py:81-84`.

2. [Direct] Bench observation operations actively command the motor. The manual says Connect, Check, scanning and idle monitoring all send Stop/status, and tells the operator to close other programs using the same adapter first. A second client acting as a status viewer could therefore interfere with a moving motor. The same manual describes the dashboard as controlling one selected motor on one channel and requires disconnecting before selecting another ID. Sources: [USER_MANUAL.md:45](/home/sid/Documents/code/im6014Workbench/USER_MANUAL.md:45), [USER_MANUAL.md:130](/home/sid/Documents/code/im6014Workbench/USER_MANUAL.md:130), [USER_MANUAL.md:134](/home/sid/Documents/code/im6014Workbench/USER_MANUAL.md:134).

3. [Direct] PR #1 established a reproducible baseline before arm development and deliberately kept the working bench implementation. It states that physical commissioning and six-joint control are later milestones. It gives no claim that the original dashboard worker is a completed shared station. Source: [PR #1 body](https://github.com/sidrk-dev/im6014-workbench/pull/1), created 2026-10-07 by sidrk-dev and merged 2026-10-07. The source lineage is also explicit in [docs/architecture.md:7](/home/sid/Documents/code/im6014Workbench/docs/architecture.md:7).

4. [Direct] PR #2 preserves command bytes and callers while making feedback recovery and transaction deadlines testable. Its tradeoff states, "A transceiver object would add ownership layers without improving the one-owner bus contract." It says the dashboard and CLI share the new serial function through preserved bench aliases, and health gates and motion lifecycle retain their prior behavior. Source: [PR #2 body](https://github.com/sidrk-dev/im6014-workbench/pull/2), sidrk-dev, 2026-10-07. The design record explains that the object candidate added a per-exchange object and duplicate checks, and explicitly preserves `motor_session.exchange` lookup for startup interception. Sources: [docs/boundary-design.md:15](/home/sid/Documents/code/im6014Workbench/docs/boundary-design.md:15), [docs/boundary-design.md:29](/home/sid/Documents/code/im6014Workbench/docs/boundary-design.md:29), [docs/boundary-design.md:31](/home/sid/Documents/code/im6014Workbench/docs/boundary-design.md:31).

5. [Direct] Current architecture distinguishes the existing worker and CLI from future station ownership. It states that the CLI is an alternative bus owner, neither owner is yet a complete six-joint station, and Linux serial exclusivity is not proof of C01's cross-process controller contract. MakerMods and LeRobot adapters must later submit through the station and must not open another serial connection. Sources: [docs/architecture.md:26](/home/sid/Documents/code/im6014Workbench/docs/architecture.md:26), [docs/architecture.md:34](/home/sid/Documents/code/im6014Workbench/docs/architecture.md:34).

6. [Direct] The plan explicitly requires one station process to own the physical bus and actuator refresh loop. Dashboard, recorder, teleoperator, cloud policy client and agent tools submit requests to it. It also requires bounded latest-target storage, command sequence numbers and per-joint receive age and fault status. Sources: [plan:148](/home/sid/Documents/code/im6014Workbench/docs/ROBOT_WORKBENCH_AGENT_PROMPT.md:148), [plan:160](/home/sid/Documents/code/im6014Workbench/docs/ROBOT_WORKBENCH_AGENT_PROMPT.md:160), [plan:210](/home/sid/Documents/code/im6014Workbench/docs/ROBOT_WORKBENCH_AGENT_PROMPT.md:210), [plan:227](/home/sid/Documents/code/im6014Workbench/docs/ROBOT_WORKBENCH_AGENT_PROMPT.md:227).

7. [Direct] The external mapping is a new boundary outside the verified codec. The plan says to apply internal motor reduction inside the protocol boundary and external transmission conversion once outside it. It specifies `q_out = q0 + s*N*q_joint`, output velocity `s*N*dq_joint` and ideal output torque `s*tau_joint/N`, with compatible stiffness and damping scaling by `1/N^2`. Sources: [plan:186](/home/sid/Documents/code/im6014Workbench/docs/ROBOT_WORKBENCH_AGENT_PROMPT.md:186), [plan:202](/home/sid/Documents/code/im6014Workbench/docs/ROBOT_WORKBENCH_AGENT_PROMPT.md:202), [plan:204](/home/sid/Documents/code/im6014Workbench/docs/ROBOT_WORKBENCH_AGENT_PROMPT.md:204). The earlier design expressly deferred external mapping, sequences and a hard real-time process boundary. Source: [docs/boundary-design.md:41](/home/sid/Documents/code/im6014Workbench/docs/boundary-design.md:41).

8. [Direct] Stop semantics are part of the existing operator contract. Stop takes priority, invalidates old actions and requires a fresh check before rearming. Fault recovery must not resume the previous test. Disconnect and close request Stop and release the adapter; failed confirmation remains visibly unconfirmed. Stop releases torque and may let a loaded arm fall. Sources: [USER_MANUAL.md:102](/home/sid/Documents/code/im6014Workbench/USER_MANUAL.md:102), [USER_MANUAL.md:104](/home/sid/Documents/code/im6014Workbench/USER_MANUAL.md:104), [USER_MANUAL.md:106](/home/sid/Documents/code/im6014Workbench/USER_MANUAL.md:106), [USER_MANUAL.md:108](/home/sid/Documents/code/im6014Workbench/USER_MANUAL.md:108).

9. [Direct] The serial boundary has a conditional software deadline contract. Its docstring states that the bus has one owner, synchronous driver calls cannot be forcibly interrupted, and timeout settings restore before success returns. The design explains the zero write-timeout workaround and keeps opens, closes, motion retries and target storage outside this boundary. Sources: [im6014_serial.py:63](/home/sid/Documents/code/im6014Workbench/im6014_serial.py:63), [docs/boundary-design.md:21](/home/sid/Documents/code/im6014Workbench/docs/boundary-design.md:21), [docs/boundary-design.md:25](/home/sid/Documents/code/im6014Workbench/docs/boundary-design.md:25).

## What we can reasonably infer

- [Supported] Existing ownership is inherited bench scope, intentionally preserved through the transport extraction. `git blame -L 270,292 motor_session.py` attributes every connect line to `69c8219`. `git log --follow -- motor_session.py` has only the inherited commit and `d76914c`. The latter changes only demo timeout attributes and demo read waiting in that file. Together with PR #1's baseline scope and PR #2's preservation rationale, the record supports keeping the bench available while adding an explicit controller boundary.
- [Inferred] Serialization likely protects request/reply association as well as motion ordering. The original exchange resets old input for each transaction, while the current exchange still performs one reset and one write and expects the requested ID. Concurrent transactions on the same byte stream could discard one another's replies. The author comment states the one-outstanding-transaction premise, but no dedicated incident or author statement explains this exact failure chain. Sources: inherited `spin_5rpm.py:81-84`; current `im6014_serial.py:62-88`.
- [Inferred] C03 should extend the existing identity discipline into a per-joint record, rather than replacing it with one global freshness clock. The previous B08 acceptance says foreign feedback must not refresh the requested motor. The inspected `test_protocol.py:281-302` checks that latest state, reply time, counters and samples stay unchanged after foreign-only input. The new C03 explicitly says fresh J1 feedback cannot conceal stale J2. This is source/test inspection, not a new test execution.
- [Inferred] C05's streaming targets likely need a separate bounded record from the current UI job queue. The existing queue holds lifecycle operations and tests; the new plan specifically forbids an unlimited FIFO of old teleoperation targets. Discrete operations can still require ordered handling, while replacing the latest target prevents delayed playback. The record does not mandate a particular queue implementation. Sources: `motor_session.py:134,164-170`; plan `:227,326`.

## Competing hypotheses

- [Speculative] A prior physical accident may have motivated worker ownership. The inspected history and PRs contain no accident, incident ticket or postmortem link. The warning about active Stop/status communication is enough to explain the documented compatibility constraint, but it does not establish that an accident happened.
- [Speculative] The worker structure may also have been chosen to keep Qt responsive. Qt requests and the heartbeat are present, but the inspected ownership comments and PR rationale do not state this as the reason. This possible secondary rationale should not substitute for the explicit single-owner and bench-scope record.

## What we don't know

- The inherited commit is a root import of the working bench. Its message has no deeper ownership rationale, and no prior commits are reachable in this inspected lineage. We cannot reconstruct pre-import deliberation from these commits.
- No connected issue, document, chat, observability, exception or analytics MCP applies to this repository. No inaccessible category was treated as searched. PR #1 and #2 have no linked closing issues, no review objects and one parent-posted verification comment each. No issue or incident ID appears in the reviewed commit messages.
- The record does not specify C01's actual interprocess locking or service protocol, lifetime policy, crash recovery, canonical adapter identity or alias handling. `exclusive=True` alone is explicitly insufficient evidence for the later station contract.
- The record does not specify the precise scope of per-build fault propagation for C04. Versioned physical build profiles are later D milestones, so this block should describe whatever active joint/build contract it actually implements rather than imply full D01-D06 readiness.
- The UI cancellation `generation` is not proof of C06 stream sequence validation or C08 ownership generations. Those contracts have separate acceptance predicates. C07 command expiry and C08 ownership generations remain later work.
- Physical external ratios, axis directions, offsets, loaded-arm Stop handling and achievable multi-joint rates remain unverified. The plan calls ratios and dimensions provisional. The new block's workflow explicitly scopes fake transport and Linux pseudo-terminals. Sources: plan `:69-85,318`; `docs/b11-c06-workflow.md` final paragraph.
- `USER_MANUAL.md:189` refers to `DASHBOARD_VALIDATION.md`, which is absent in the workbench checkout. The cited current manual limitation and PR verification records were read, but that missing local document was not read. This does not establish that its upstream contents are absent.

## Sources consulted

- Source control history: read-only Git log, blame and diffs for `motor_session.py`, `spin_5rpm.py`, commits `69c8219` and `d76914c`; current six-commit workbench log; PR #1 and #2 bodies, comments, review lists and linked-issue lists through `gh`. Inspected local architecture, boundary design/review, manual, plan and selected test source. Searched all history subjects for `incident|postmortem|exclusive|owner|timeout`; it surfaced boundary ownership documentation, not an incident record.
- Issue/ticket tracker: no applicable connected MCP available. PR linked-issue fields were empty. No tracker search occurred.
- Long-form documents: no applicable connected MCP available. In-repository docs were inspected under source-control evidence, including architecture, manual, plan and boundary design.
- Real-time team chat: no applicable connected MCP available. No chat search occurred.
- Infrastructure observability: no applicable connected MCP available. No metrics, incident or runtime service query occurred.
- Error/exception tracking: no applicable connected MCP available. No exception-service query occurred.
- Product analytics warehouse: no applicable connected MCP available. No warehouse query occurred.

## Preserve / Change / Avoid / Risk constraints

### Preserve

- Keep the existing working bench available during B11-B14, as the plan explicitly requires at `:297`. Preserve byte-identical command fixtures, motor-output units, the internal `38/3` conversion, strict reply identity, aliases used by startup interception and the single transaction contract. Sources: PR #2; boundary design `:15-21`; plan `:301-310`.
- Preserve disconnected/disarmed startup, demo isolation, Stop priority, cancellation of old actions, explicit timeout acknowledgment, latched disarm and visible failed Stop confirmation. Preserve the operator meaning of Stop as torque release. Sources: manual `:23,33-45,102-108`.
- Keep serial opening/closing and target ownership outside `im6014_serial.exchange`. Its timeout restoration and no-motion-retry contract stay intact. Sources: boundary design `:21,25`.
- Preserve raw validated packets and decoded fields as inspectable diagnostic evidence. Manual exports already retain source/session, motor ID, port and baud tags. B14 should build on an explicit capture contract rather than silently changing those meanings. Sources: manual `:160-164`; `motor_session.py:202-205`; plan `:314`.

### Change

- B11 adds an external mapping boundary once outside the codec, with finite positive ratios, direction and offset semantics made explicit. Acceptance covers ratios 1, 3 and 6, both directions and nonzero offsets. B12 rejects nonfinite values, invalid IDs and encoded overflow before dispatch. Sources: plan `:186-204,311-312`.
- B13 makes adapter/channel and supported baud part of validated port configuration and returns useful errors. A historical `/dev/ttyUSB3` name must not become proof of present hardware identity. Sources: plan `:313`; manual `:31,43,156`.
- C01 creates a controller ownership contract that a second controller cannot bypass to independently open the configured hardware. The contract must state whether it covers bench entry points and how they share or exclude station access. C02 rejects illegal transitions before motor dispatch. Sources: architecture `:26,34`; plan `:210-212,322-323`.
- C03/C04 retain receive time, validity and fault identity per configured joint. C05 keeps bounded latest-target storage. C06 rejects duplicate or older sequences without replacing an accepted newer target. Sources: plan `:148-151,227,324-327`.

### Avoid

- Do not reuse a per-exchange object as proof of bus ownership. The previous design rejected that extra layer for this exact reason. Do not present Linux port exclusivity as completed C01 station independence. Sources: PR #2 tradeoff; architecture `:26`; boundary design `:31`.
- Do not let a GUI, recorder, cloud policy or future MakerMods adapter open its own hardware connection. Do not make Connect, Check or scan sound passive. Sources: plan `:212`; architecture `:34`; manual `:45`.
- Do not aggregate joint freshness so one reply validates other joints. Do not use the UI Stop generation as a substitute for command sequence ordering or claim later ownership-generation/expiry milestones are complete. Sources: plan `:324,327-329`.
- Do not merge internal and external reduction conversions, treat bench gains/limits as actuator ratings, fabricate missing joint measurements, or claim loaded-arm safety from simulated responses. Sources: boundary design `:19`; plan `:111,180,204`; manual `:108,189-191`.

### Risk

- [Inferred] Multiple path aliases or different clients could defeat a lock keyed only by an unnormalized string. The station design needs one explicit configured-hardware identity and a demonstrated second-owner rejection contract. The inspected record does not select the locking mechanism.
- [Direct] A driver or kernel operation can outlive the deadline if it ignores its own timeout. Preserve that limitation in reports; software checks and pseudo-terminals do not establish hard real-time interruption. Source: boundary design `:25`; PR #2 verification comment.
- [Direct] Stop releases torque. Shared controller states must not imply that Stop is a verified loaded-arm hold or park procedure. Sources: manual `:108`; plan `:231-239`.
- [Direct] There are two inherited CLI issues in the prior independent review: zero rpm still applies friction torque, and a large negative friction input produces a traceback before the later usage-error check. The prior batch intentionally left these unchanged. They should be considered when B12 touches CLI validation, with any accepted correction scoped and evidenced explicitly. Sources: [docs/boundary-review.md:13](/home/sid/Documents/code/im6014Workbench/docs/boundary-review.md:13), [docs/boundary-review.md:15](/home/sid/Documents/code/im6014Workbench/docs/boundary-review.md:15), PR #2 verification comment.

## Confidence summary

The single-owner bench contract and its deliberate preservation in B01-B10 have direct source and PR support. The new station, per-joint state and bounded target requirements are explicit in the plan. The exact locking mechanism, pre-import motivations and physical multi-joint behavior remain unknown; the inference about concurrent reply loss and bounded streaming target separation follows from the inspected transaction behavior and acceptance predicates.

## Memory lookup provenance

A quick memory search identified the original bench and reminded this investigator that adapter identity must be rediscovered. Current ownership and compatibility conclusions above come from live Git, PR and file inspection. Memory `MEMORY.md:100-102` points to rollout `01a0f5a4-5afb-74f2-bbc1-28194b233c29`; if the parent relies on this reminder in its user response, retain the required memory citation. No memory files were edited.
