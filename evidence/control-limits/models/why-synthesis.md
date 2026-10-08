# C07-C16 rationale and constraints

### The Question

How should C07-C16 extend the existing controller while preserving the transport-free `JointCore` and managed device lease?

### The Code in Question

Baseline `244811bb6b924355852c37cddde8d00fbb0fab50`, PR #3. `joint_controller.py:218,291,552,592,604` contains `Target`, `JointCore`, submission, consumption and Stop. `im6014_port.py:53,61,81,235` contains `PortCloseError`, `ManagedPort`, cleanup and managed opening. Citations below refer to this workbench baseline, except the current workflow scope note.

### What We Found

- **[Direct]** B11-C06 selected the typed core and explicitly rejected raw wire ownership inside it. The core owns fixed joint order, observations, one immutable pending target and a sequence watermark; the separate fake adapter demonstrates packet writes. This preserves existing bench entry points. `docs/controller-design.md:3,7,16-17`; squash commit `244811b`.
- **[Direct]** The fixed per-user lease namespace prevents cooperating callers from avoiding each other's lock. Device aliases collide, acquisition precedes serial construction, and ownership lasts through confirmed close. Review required exact boolean `False` to confirm closure and retained a retryable owner on cleanup failure. Persistent lockfiles preserve flock inode identity. `docs/controller-design.md:14,19`; `docs/bench-review.md:8,24,30,57`.
- **[Direct]** The plan requires expiry before dispatch, rejection of earlier owner generations, documented position and reference limits, combined effort handling, persistent fault latches, actual timing measurements and dashboard failure independence. It requires separate requested, accepted, dispatched and measured records. These are requirements, not evidence of implementation or physical success. `docs/ROBOT_WORKBENCH_AGENT_PROMPT.md:154-170,328-339`.

### What We Can Reasonably Infer

- **[Inferred]** The station-daemon rejection appears limited to B11-C06. The selection defers these items to C07-C16, while the broader plan calls for one station owning the bus and refresh loop. The future station's process and API shape remain unsettled. `docs/controller-design.md:7,58`; `docs/ROBOT_WORKBENCH_AGENT_PROMPT.md:210-212`.
- **[Inferred]** C08 likely needs a distinct motion-ownership contract. The physical lease excludes cooperating openers, and the GUI generation discards queued jobs after Stop. Neither establishes rejection of delayed policy commands after takeover. `docs/architecture.md:29`; `motor_session.py:179-185,475-490`; `docs/ROBOT_WORKBENCH_AGENT_PROMPT.md:229,329`.

### Competing Hypotheses

A permanent ban on a station daemon and a staged deferral both fit the word "rejected" alone. The explicit later scope and one-station plan favor deferral; they do not select an IPC mechanism or establish an existing production station. `docs/controller-design.md:7,58`; `docs/architecture.md:29-31`.

### What We Don't Know

The earlier PR3 body is absent despite the coverage note calling it retained. The inspected squash message and committed documents do not establish a historical double-owner incident, physical timing, loaded-arm behavior or protection from predicted effort. Concrete limit values, effort validity and final station design need explicit decisions or measurements. Hardware remains outside this task's scope because the timeout acknowledgment is unanswered. `why-history.md`; `why-coverage.md`; `docs/c07-c16-workflow.md:51`.

### Sources Consulted

- Source control history: supplied per-file history and three commit bodies, `69c8219`, `d76914c`, `244811b`; synthesis spot-checked `244811b`, relevant source and review citations. No live PR lookup.
- Issue / ticket tracker: unavailable; no matching repository MCP.
- Long-form documents: local design, architecture, API, plan, review and decision records; no relevant connected document session.
- Real-time team chat: unavailable; no matching MCP.
- Infrastructure observability: unavailable; no matching MCP.
- Error / exception tracking: unavailable; no matching MCP.
- Product analytics warehouse: unavailable; no matching MCP. These gaps do not show that records never existed.

### Preserve / Change / Avoid / Risk

- **Preserve.** Keep fixed order, complete immutable targets, one pending slot, full mapped codec validation, per-joint freshness and acknowledged recovery. Invalid replacements preserve pending and watermark. Retain ordering across Stop, fault and reconnect, plus acquisition-before-construction and confirmed-close ownership. `docs/joint-api.md:54-68,96-98`.
- **Change.** Add C07-C16 at the request and station boundaries while retaining the core's transport separation. Document expiry, generation and limiting semantics, four action records and actual timing provenance. `docs/controller-design.md:16,52,58`; `docs/ROBOT_WORKBENCH_AGENT_PROMPT.md:165-170,328-337`.
- **Avoid.** Do not hide rejection through mapping clipping, duplicate codec limits, silently reset ordering, introduce caller-specific lock namespaces, unlink active lockfiles or force-close worker-owned I/O. `docs/controller-design.md:19,32`; `docs/bench-review.md:53,57-61`.
- **Risk.** Advisory locking cannot stop raw clients; passive identity cannot prove communication. Fake I/O and PTYs cannot establish physical Stop, actuator rate, effort protection or station independence. `docs/joint-api.md:48-50`; `docs/controller-design.md:58`; `docs/ROBOT_WORKBENCH_AGENT_PROMPT.md:223,318,339`.

### Confidence Summary

The core and lease rationale is directly documented and the spot-checked citations agree. Station deferral and the need to distinguish three ownership mechanisms are interpretations of that record. The missing PR body and unavailable external evidence categories limit historical conclusions; this synthesis ran no tests or hardware operations.
