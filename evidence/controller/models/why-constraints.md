### The Question

Why retain the current bench while adding B11-B14 and C01-C06, and which ownership, unit and safety contracts constrain the new controller?

### The Code in Question

The inspected workbench baseline is `d8d810888f76c676acaa1d54bc9e790249592f45`. `MotorSession` owns the dashboard worker and exposes `request`, `snapshot` and `close` at [motor_session.py:131](/home/sid/Documents/code/im6014Workbench/motor_session.py:131). The standalone CLI opens its own bus at [spin_5rpm.py:110](/home/sid/Documents/code/im6014Workbench/spin_5rpm.py:110). The codec converts internal reduction; `im6014_serial.exchange` performs one transaction against an already owned bus.

### What We Found

- [Direct] PR #1 says to keep the working bench as a reproducible baseline before arm development. PR #2 says to preserve command bytes, callers, health gates and motion lifecycle during communication extraction. These are explicit scope decisions. Sources: [PR #1](https://github.com/sidrk-dev/im6014-workbench/pull/1), [PR #2](https://github.com/sidrk-dev/im6014-workbench/pull/2).
- [Direct] The bench has an explicit single-owner contract, and its observation operations send Stop/status. Running another client can therefore command the same motor. The architecture identifies the CLI as an alternative owner and explicitly says Linux serial exclusivity does not establish C01. Sources: [motor_session.py:1](/home/sid/Documents/code/im6014Workbench/motor_session.py:1), [manual:45](/home/sid/Documents/code/im6014Workbench/USER_MANUAL.md:45), [architecture:26](/home/sid/Documents/code/im6014Workbench/docs/architecture.md:26).
- [Direct] The plan requires one station process to own the bus and refresh loop; other clients submit requests to it. Per-joint timestamps, validity and faults, bounded latest targets and command sequence numbers are explicit requirements. Sources: [plan:148](/home/sid/Documents/code/im6014Workbench/docs/ROBOT_WORKBENCH_AGENT_PROMPT.md:148), [plan:210](/home/sid/Documents/code/im6014Workbench/docs/ROBOT_WORKBENCH_AGENT_PROMPT.md:210), [plan:227](/home/sid/Documents/code/im6014Workbench/docs/ROBOT_WORKBENCH_AGENT_PROMPT.md:227).
- [Direct] Operator Stop invalidates old queued actions, requires a new check, and leaves faults disarmed. Returning healthy feedback does not resume motion. Stop releases torque and can let a loaded arm fall. Source: [manual:102](/home/sid/Documents/code/im6014Workbench/USER_MANUAL.md:102).
- [Direct] The boundary design preserves startup interception through `motor_session.exchange`, bench aliases, field units and bench encoding limits. It assigns opens, closes, target storage and motion lifecycle outside `exchange`. Sources: [boundary design:15](/home/sid/Documents/code/im6014Workbench/docs/boundary-design.md:15), [boundary design:19](/home/sid/Documents/code/im6014Workbench/docs/boundary-design.md:19).

### What We Can Reasonably Infer

- [Inferred] Separate latest-target storage appears appropriate for streaming commands while retaining ordered lifecycle requests. The existing job queue handles discrete actions; C05 requires bounded storage after 10,000 target updates. The record does not prescribe its implementation. Sources: [motor_session.py:164](/home/sid/Documents/code/im6014Workbench/motor_session.py:164), [plan:326](/home/sid/Documents/code/im6014Workbench/docs/ROBOT_WORKBENCH_AGENT_PROMPT.md:326).
- [Inferred] Serialization likely protects reply association as well as motion order. Each exchange resets old input and expects one ID; concurrent exchanges could discard another request's reply. This failure chain has no cited incident. Source: [im6014_serial.py:63](/home/sid/Documents/code/im6014Workbench/im6014_serial.py:63).

### Preserve / Change / Avoid / Risk constraints

#### Preserve

- Retain the working UI `request`/`snapshot`/`close` interface and startup interception patch points, including `motor_session.exchange` and bench aliases. Preserve disconnected/disarmed startup, demo isolation and inspectable snapshots. Sources: [session:159](/home/sid/Documents/code/im6014Workbench/motor_session.py:159), [boundary design:15](/home/sid/Documents/code/im6014Workbench/docs/boundary-design.md:15), [PR #1](https://github.com/sidrk-dev/im6014-workbench/pull/1).
- Preserve byte-identical fixtures, output-shaft units, internal `38/3` conversion, strict feedback identity and existing bench bounds. These bounds are compatibility policy, not actuator ratings. Preserve the single reset/write exchange, timeout restoration and absence of motion retries. Source: [boundary design:19](/home/sid/Documents/code/im6014Workbench/docs/boundary-design.md:19).
- Preserve Stop priority, cancellation of old actions, explicit timeout acknowledgment, fault latching, deliberate rearming and visible failed Stop confirmation. Neither fresh feedback nor reconnection may restart a previous target automatically. Source: [manual:102](/home/sid/Documents/code/im6014Workbench/USER_MANUAL.md:102).

#### Change

These are direct acceptance requirements at [plan:311](/home/sid/Documents/code/im6014Workbench/docs/ROBOT_WORKBENCH_AGENT_PROMPT.md:311) and [plan:322](/home/sid/Documents/code/im6014Workbench/docs/ROBOT_WORKBENCH_AGENT_PROMPT.md:322).

| Unit | Required constraint |
| --- | --- |
| B11 | Apply external reduction once outside the codec. Round-trip ratios 1, 3 and 6 with both directions and nonzero offsets. Use `q_out=q0+s*N*q_joint`, `dq_out=s*N*dq_joint`, `tau_out=s*tau_joint/N`; compatible gains scale by `1/N²`. Validate mapping parameters before dispatch. |
| B12 | Reject NaN, infinity, invalid IDs and encoded overflow before motor dispatch, including values produced by mapping. Keep existing bench limits. |
| B13 | Validate explicit adapter/channel and supported baud configuration; wrong configuration must yield a useful diagnostic. Enumeration alone does not prove communication. |
| B14 | Provide comparable raw packets and decoded fields without credentials. Define capture provenance and bounds; retain existing demo/real, session, ID and units meanings. |
| C01 | Enforce cross-process ownership so a second controller cannot independently open the same configured hardware. State how dashboard and CLI access share or exclude station ownership. |
| C02 | Reject illegal controller transitions before any motor command. Keep Stop and fault recovery disarmed until the explicit recovery and arm sequence. |
| C03 | Track receive age and validity per configured joint; fresh J1 must not conceal stale J2. |
| C04 | Retain each fault's motor identity and define its effect on the active joint/build contract. |
| C05 | Keep bounded latest-target storage after 10,000 updates and retain the newest valid target. |
| C06 | Reject duplicate and older sequences without replacing the newer accepted target. |

Mapping formulas are explicit at [plan:186](/home/sid/Documents/code/im6014Workbench/docs/ROBOT_WORKBENCH_AGENT_PROMPT.md:186). Existing capture meanings are documented at [manual:162](/home/sid/Documents/code/im6014Workbench/USER_MANUAL.md:162).

#### Avoid

- Do not count a per-exchange object or `exclusive=True` as completed station ownership. Neither GUI nor future recorder, policy or MakerMods adapter may create a competing hardware connection. Sources: [architecture:26](/home/sid/Documents/code/im6014Workbench/docs/architecture.md:26), [plan:212](/home/sid/Documents/code/im6014Workbench/docs/ROBOT_WORKBENCH_AGENT_PROMPT.md:212).
- Do not substitute the UI cancellation `generation` for C06 sequence ordering. C07 expiry and C08 ownership generations remain later predicates. Sources: [session:164](/home/sid/Documents/code/im6014Workbench/motor_session.py:164), [plan:327](/home/sid/Documents/code/im6014Workbench/docs/ROBOT_WORKBENCH_AGENT_PROMPT.md:327).
- Do not merge internal/external reduction, fabricate absent measured joints, or treat simulated Stop as loaded-arm validation. Sources: [plan:180](/home/sid/Documents/code/im6014Workbench/docs/ROBOT_WORKBENCH_AGENT_PROMPT.md:180), [plan:204](/home/sid/Documents/code/im6014Workbench/docs/ROBOT_WORKBENCH_AGENT_PROMPT.md:204), [manual:108](/home/sid/Documents/code/im6014Workbench/USER_MANUAL.md:108).

#### Risk

- [Inferred] Locks keyed only by a port string may miss aliases. Choose and verify canonical hardware identity, lock lifetime, crash release and second-owner rejection. The record specifies the result, not the locking mechanism.
- [Direct] Synchronous driver calls cannot be forcibly interrupted. Pseudo-terminal success does not establish hard real-time deadlines. Source: [boundary design:25](/home/sid/Documents/code/im6014Workbench/docs/boundary-design.md:25).
- [Direct] Prior review records zero-rpm CLI friction torque and traceback-producing large negative friction input. B12 should consider these existing paths explicitly. Source: [boundary review:13](/home/sid/Documents/code/im6014Workbench/docs/boundary-review.md:13).

### What We Don't Know

Pre-import motivation, physical ratios/directions/offsets, loaded-arm procedures and achievable joint rates remain unverified. C04 build-wide propagation and C01 service/locking mechanics are unspecified. The manual references an absent local `DASHBOARD_VALIDATION.md`; its contents were not examined. The current block scopes fake transport and Linux pseudo-terminals at [workflow:36](/home/sid/Documents/code/im6014Workbench/docs/b11-c06-workflow.md:36).

### Sources Consulted

- Source control history: investigator examined six-commit history, inherited ownership comments, blame and diffs; this synthesis reread current source/docs and verified PR #1/#2 bodies through `gh`.
- Issue/ticket tracker: no applicable connected MCP; investigator found empty PR linked-issue fields. No tracker search occurred.
- Long-form documents: in-repository plan, manual, architecture, boundary design/review and workflow inspected; no applicable external document MCP.
- Real-time team chat: unavailable; not searched.
- Infrastructure observability: unavailable; not searched.
- Error/exception tracking: unavailable; not searched.
- Product analytics warehouse: unavailable; not searched.

### Confidence Summary

The preservation rationale and ten acceptance predicates have direct documentation support. Storage shape and alias risks are inferences. This read-only synthesis ran no tests, Git mutations, packages or hardware I/O and claims no new validation result.
