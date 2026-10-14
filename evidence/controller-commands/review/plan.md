# C07-C10 resumed controller work

1. `how` over the affected subsystem.
2. `architect` for parallel design exploration.
3. Write the throughput checkpoint as four todo items.
4. Delegate code-writing through provider dispatch using your configured feature descriptor.
5. Verify on the matching surface.
6. Rebase into small, ordered commits. Stack follow-ups.
7. If the design is contested, `interrogate` before shipping.
8. Run Opening a PR.

Throughput checkpoint
- Blocking first steps. Read prior ledger and current core; select the ownership and expiry contract before writing.
- Independent workstreams. Parallel read-only design candidates; one core writer; parent checks and evidence.
- Shared mutable state. A thread-confined JointCore owns lifecycle, pending command, owner generation and reference state.
- Smallest safe decomposition. C07, C08, C09, C10 run sequentially because they share the core and its callers. Parent validates each unit before advancing.

Architecture phases
1. Ground
2. Sketch
3. Agree
4. Implement
5. Scrap

Arena phases
1. Frame
2. Fan out
3. Cross-judge
4. Pick
5. Graft
6. Verify

Rubric
- Reject expired and prior-owner requests at both acceptance and consumption.
- Preserve faults, latest-only storage, explicit arming and fixed joint order.
- Bound commanded position and reference velocity in joint units without invented hardware limits.
- Keep one transport-free owner and minimize public API complexity.
- Prove boundary and lifecycle behavior through fake clocks and fake packet dispatch.

Physical commissioning and production station integration remain outside this software batch.
