# C07-C16 workflow

## Feature

- [x] `how` over the affected subsystem.
- [x] `architect` for parallel design exploration.
- [x] Write the throughput checkpoint as four todo items.
- [ ] Delegate code-writing through provider dispatch.
- [ ] Verify on the matching surface.
- [ ] Rebase into small, ordered commits. Stack follow-ups.
- [ ] If the design is contested, `interrogate` before shipping.
- [ ] Run Opening a PR.

## Architect

- [x] Ground.
- [x] Sketch.
- [x] Agree.
- [ ] Implement.
- [ ] Scrap, only if the selected shape repeatedly fails.

## Arena

- [x] Frame.
- [x] Fan out.
- [x] Cross-judge.
- [x] Pick.
- [x] Graft.
- [x] Verify.

## Throughput checkpoint

- [x] Blocking first steps. Read current source/plan, verify merged baseline, trace core and serial/session, choose typed design before code.
- [x] Independent workstreams. Two grounding slices, independent design alternatives, and later reviewers each own separate artifacts.
- [x] Shared mutable state. Core API and its callers change together, so implementation units serialize and each writer gets a fresh isolated worktree.
- [x] Smallest safe decomposition. One Gemini writer per milestone. Parent owns tests/Git/hardware and accepts each unit before next; no overlapping core writes.

## Units

- [x] C07. Parent software checks accepted; final branch review remains pending.
- [x] C08. Parent software checks accepted; final branch review remains pending.
- [x] C09. Parent software checks accepted; final branch review remains pending.
- [x] C10. Parent software checks accepted; final branch review remains pending.
- [x] C11. Parent software checks accepted; final branch review remains pending.
- [x] C12. Parent software checks accepted; final branch review remains pending.
- [x] C13. Parent software checks accepted; final branch review remains pending.
- [x] C14. Parent software checks accepted; final branch review remains pending.
- [x] C15. Exact accepted source recovered and freshly software verified; final branch review remains pending.
- [ ] C16.

Hardware remains blocked on the unanswered timeout acknowledgment. Fake I/O and PTYs only. Source baseline 244811bb6b924355852c37cddde8d00fbb0fab50. Original repository and user plan stay untouched. Gemini is the task implementation and additional review route; saved global roles unchanged. At most one reset credit is authorized if this run exhausts; one has been redeemed; no second credit is authorized.

## Updated reset-credit authorization

The user explicitly authorized at most one full reset credit during this run if usage exhausts. One credit was redeemed at the C13 settled boundary at 2026-10-08 20:16 UTC; supported backend reset and fresh primary1%/secondary0% confirmed, with two credits unused. No second credit is authorized. This supersedes earlier instructions to leave all credits unused. Redeem only through a supported available capability and verify fresh limits and credit status; record the redemption durably. After one credit, use normal resets. Preserve this orchestrator and chat.
