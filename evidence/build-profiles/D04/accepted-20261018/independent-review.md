What this change does: D04 stores installed locked joints separately from active coordinates. Each locked record retains an explicit aggregate body mass and fixed angle, including units, provenance and unknown status, without a motor, bus or mapping identity.

Looks good. No concrete source blockers or scoped requirements gaps found.

Source review covered the actual files in `/tmp/d04-codex-20261018`, their read-only diff, the supplied D04 design, D01-D03 callers/tests, the positional Core boundary and fake station, milestone/baseline registration, startup fingerprints, and `docs/locked-joints.md`.

- **Optional-field compatibility:** `build_profile.py:13-14,195-196,237-251,310-329` retains all prior required top-level fields, defaults omitted `locked_joints` to an empty tuple, rejects explicit null or non-array input, and omits the new field when empty. Existing positional constructor arguments remain in their original positions. The existing active measurement parser was extracted into `_parse_measurement` with equivalent required keys, unknown-key rejection, constructor validation and ordering. Existing version 1 profiles retain their prior serialization shape and values.
- **Mass, angle and unknowns:** `build_profile.py:63-88,169-185,261-280` requires both measurements and their exact semantic names/units (`body_mass`/kg and `locked_angle`/rad). Existing `Measurement` validation rejects booleans, nonnumeric/nonfinite known values, missing known values and numeric values attached to unknown status. Source text is required. Known mass cannot be negative. Serialization preserves known values, status, units and source; unknown values remain absent from JSON. No mass is inferred for absent coordinates.
- **Duplicate names and invalid structure:** `build_profile.py:217-225,274-280,339-345` rejects repeated locked names, active/locked overlap, invalid record types, missing locked fields, extra motor/bus/mapping keys, and repeated JSON keys at every object depth. Frozen records and tuple collections retain the existing detached configuration model.
- **Active dimensions and callers:** `BuildProfile.joint_names()` and `require_active_joint()` use only `joints`. `BuildPlan` still binds exact ordered active names, backend and identity (`build_profiles.py:83-95`); locked records do not alter D01-D03 factory defaults. The Core accepts only `JointSpec` objects (`joint_controller.py:95-106`) and checks exact target length before queuing (`joint_controller.py:474-527`). D04's fake test explicitly builds specs from active entries and checks an extra target coordinate produces no pending target, consumed reference or additional write. This supports the stated configuration boundary; named lookup grants no readiness or motion permission.
- **Verification integration and documentation:** `test_locked_joints.py` covers known/unknown round-trip, absent and locked lookup rejection, unchanged active dimensions, duplicate/disjoint names, units and negative mass, strict JSON structure, empty-field compatibility and the fake Core rejection. `scripts/check_milestone.py` registers D04. The baseline copies and runs the new test with its existing fake dependencies; startup fingerprints include it. The documentation matches the implementation and explicitly leaves geometry, dynamics, physical lock detection and production owner integration outside this unit.

Reviewed source identities, copied exactly from the parent-provided `review/source-hashes.json`:

| File | SHA-256 |
| --- | --- |
| `build_profile.py` | `2848bdbbdc928a3d66d05495062e9aeafbc6c9d00a1edb50de99920bcce6c10f` |
| `test_locked_joints.py` | `ead311dcb3f776b44eb2a6f20662cfc181876a44063a3eaca44e7ea963c402a6` |
| `scripts/check_milestone.py` | `c576e73798a2e30917cf45435132ecec66380cb008813de10e95400e6d44fdee` |
| `scripts/check_offline_baseline.sh` | `f04e33e75b3a0534c34808c32f8cf61889b64d2924ba10e2f092785d445c1cab` |
| `test_startup.py` | `7f3f1a4e03c3d1b69f97646939ad82f45f9ac73eb0c85a8d568a2a68c7518477` |
| `docs/locked-joints.md` | `fe0f79426d66feea60c9c7e07c47c006b8840d0839cbfa060e1be820825f63cb` |

Verdict: Ship for the scoped D04 software configuration source gate. Final source identity checks and milestone acceptance remain with the parent.

Not checked: No imports, tests, checks, hash recomputation, providers, network or physical I/O were run. Parent-reported test results and captures were not independently revalidated. Production integration, physical behavior and dynamic load calculations are not established by this review. Only this assigned report was written; the inspected checkout was not edited and no Git mutations were performed.
