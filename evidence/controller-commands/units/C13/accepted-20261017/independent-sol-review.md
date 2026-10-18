Independent read-only C13 release review, 2026-10-17.

Verdict is APPROVE for the frozen C13 software-only artifact against accepted C12 base `227ec4ecc712c0cee48a50c9e234e70dd065b8e3`. No remaining release-blocking correctness finding emerged from the connected source review. No source was edited and no test, network, provider, service, or hardware operation was run by this reviewer.

I read the Comment Sicko agent in full and applied its deletion criteria as report-only recommendations. I read poteto-mode and the Type System Discipline, Prove It Works, and unslop guides. Prove It Works changed the review from relying on the handoff to reading the connected source, comparing actual SHA256 values, and inspecting the parent probe scripts and results. Type System Discipline focused the review on the required validated effort limit, immutable target records, and conversion back into canonical joint units.

All 21 entries in `/tmp/hapi-mini-swarm-20261017/c13-direction-parent-freeze.json` match both `/tmp/c13-direction-parent-check-20261017` and the externally owned original worktree. Key SHA256 values are:

- `joint_controller.py` is `e705a68992692a539fc8d722981c837ab92547c72cb9fa99f0d5dabebf2f5309`.
- `joint_records.py` is `0c6be443e3041ed6004b9d62aef571753bbad4cd467119a5237b88ecb139aec6`.
- `joint_reference.py` is `c88ba8ec27e87640a59b5d33ebe9165e8eadc6293125ac66ca52b8437da41aba`.
- `test_controller_effort.py` is `264f2b76b1ac3c9a5a3a58cff08a6f9cb4e3f2745c08b44020ac6454d9921524`.

The substantive conclusions below are static source findings. Line numbers refer to the frozen tree.

- `joint_records.py:115` and `159-168` make `max_effort_nm` mandatory, finite, positive, and immutable. `joint_controller.py:417-421` rejects excessive requested feedforward before pending target or sequence changes at `442-443`. The connected existing test fixtures and documented example supply the new required field.
- `joint_reference.py:77-149` computes the signed combined effort using the actual encoded and decoded position, speed, gains, and feedforward. The gain reconstruction uses the square of external reduction, and `joint_mapping.py:64-106` supplies the matching direction and torque conversions. Nonfinite or unrepresentable calculations suppress issuance.
- `joint_reference.py:188-196` intersects the effort interval with both the existing motion corridor and the segment joining previous released position to the already bounded candidate. Therefore pacing cannot introduce the former direction reversal. It also cannot relax position, velocity, acceleration, or measured lead bounds. The segment follows the bounded candidate, so an existing legitimate braking step can remain opposite the macro goal.
- `joint_reference.py:195-202` reprojects through the existing codec and recomputes effort after projection. It preserves requested gains and feedforward. `367-375` applies pacing only when the original candidate exceeds the budget. The progress checks at `437-441` still suppress an unsettled channel with no grid-code change.
- `joint_controller.py:487-529` retains the final freshness, ownership, deadline, release rate, measured lead, and effort vetoes. History, pending work, and MOVING state change only at `531-544`, after every joint passes. A failed effort check emits no partial reference and does not commit candidate history.

The inspected parent log `/tmp/hapi-mini-swarm-20261017/c13-direction-parent-tests.log` records 124 tests and OK. The 32-case progress probe independently decodes outgoing packets and checks direction, effort cap, and preserved gains/feedforward across reductions, direction signs, goal signs, offsets, and feedforward signs. Its results are in `c13-direction-progress-parent-results.json`. The inspected final-release probe establishes a non-None planned candidate before shifting feedback, then records the veto and recovery matching an uninterrupted control in `c13-direction-final-release-parent-results.json`. The reversal reproduction records suppression with pending work retained in `c13-direction-parent-repro.json`. These are parent-run results, not a fresh execution by this reviewer. The current baseline `checks.tsv` records zero exits for all nine groups. No material missing parent check was found for this bounded gate.

I read the full supplied Gemini review after establishing the source conclusions. Its receipt records complete status and exit 0 with pinned argv, while `modelVerified` is false. It must not be described as independently verified model identity. The Muse receipt records an HTTP 400 invocation dropout and no completed review. No retry was made. Persistent or duration-based effort protection belongs to C14 and is outside this C13 verdict. Physical behavior and production station integration remain unverified.

Comment-only candidate deletions are separate from the release verdict. They require no behavior change.

- `joint_reference.py:81` is a private helper docstring repeating `_decoded_effort_terms` and its return data. Delete one block and one line.
- `joint_reference.py:164` is a private helper docstring narrating the explicit interval intersection and reprojection. Delete one block and one line.
- `test_controller_effort.py:1-13` is a module summary duplicating test names and assertions. Delete one block and 13 lines.
- `test_controller_effort.py` has redundant assertion or setup narration at lines `108`, `128`, `141`, `173`, `185`, `201`, `214`, `218`, `228`, `241`, `245`, `261`, `266`, `281`, `536-537`, and `604`. Delete 16 blocks and 17 lines. The setup values and assertions already state the behavior; the last two comments also contain emphatic prose.

The deletion recommendation totals 19 blocks and 32 lines across two files. The number actually deleted is zero. MUST KILL symbol flags total zero, because the scoped explanatory comments do not hide a constraint requiring a code reshape. Keep two newly introduced or modified public API contract docstring blocks, `compute_predicted_effort` at `joint_reference.py:115-119` and `plan_reference` at `217-225`. Other keep-list exception counts are zero in the introduced scope. Unchanged comments, external review artifacts, and explanatory Markdown documents were not included in the deletion sweep. No broader abstraction, formatting, or style expansion is recommended.
