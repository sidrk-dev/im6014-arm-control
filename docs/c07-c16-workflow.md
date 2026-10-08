# C07-C16 workflow

## Feature

- [ ] `how` over the affected subsystem.
- [ ] `architect` for parallel design exploration.
- [ ] Write the throughput checkpoint as four todo items.
- [ ] Delegate code-writing through provider dispatch.
- [ ] Verify on the matching surface.
- [ ] Rebase into small, ordered commits. Stack follow-ups.
- [ ] If the design is contested, `interrogate` before shipping.
- [ ] Run Opening a PR.

## Architect

- [ ] Ground.
- [ ] Sketch.
- [ ] Agree.
- [ ] Implement.
- [ ] Scrap, only if the selected shape repeatedly fails.

## Arena

- [ ] Frame.
- [ ] Fan out.
- [ ] Cross-judge.
- [ ] Pick.
- [ ] Graft.
- [ ] Verify.

## Throughput checkpoint

- [ ] Blocking first steps. Read current source/plan, verify merged baseline, trace core and serial/session, choose typed design before code.
- [ ] Independent workstreams. Two grounding slices, independent design alternatives, and later reviewers each own separate artifacts.
- [ ] Shared mutable state. Core API and its callers change together, so implementation units serialize and each writer gets a fresh isolated worktree.
- [ ] Smallest safe decomposition. One Gemini writer per milestone. Parent owns tests/Git/hardware and accepts each unit before next; no overlapping core writes.

## Units

- [ ] C07.
- [ ] C08.
- [ ] C09.
- [ ] C10.
- [ ] C11.
- [ ] C12.
- [ ] C13.
- [ ] C14.
- [ ] C15.
- [ ] C16.

Hardware remains blocked on the unanswered timeout acknowledgment. Fake I/O and PTYs only. Source baseline 244811bb6b924355852c37cddde8d00fbb0fab50. Original repository and user plan stay untouched. Gemini is the task implementation and additional review route; saved global roles unchanged. Reset credits remain unused.
