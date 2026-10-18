# D04 Independent Source Acceptance Review

**Milestone:** D04 — Absent and installed locked joint handling  
**Assigned Workspace:** `/tmp/d04-codex-20261018`  
**Verdict:** **ACCEPT / NO BLOCKERS** — Ship the software source gate.

No concrete bugs, regressions, or missing acceptance guards were found in the supplied milestone implementation. This review is strictly read-only and software-bounded: no tests or imports were executed, no files or Git metadata were modified, no commands were run, and no physical hardware results are claimed.

---

## 1. Scope and Verification Summary

The D04 milestone introduces explicit handling for absent and installed locked joints into [`build_profile.py`](file:///tmp/d04-codex-20261018/build_profile.py), supported by unit tests in [`test_locked_joints.py`](file:///tmp/d04-codex-20261018/test_locked_joints.py), documentation in [`docs/locked-joints.md`](file:///tmp/d04-codex-20261018/docs/locked-joints.md), and integration into baseline audit scripts [`scripts/check_milestone.py`](file:///tmp/d04-codex-20261018/scripts/check_milestone.py), [`scripts/check_offline_baseline.sh`](file:///tmp/d04-codex-20261018/scripts/check_offline_baseline.sh), and [`test_startup.py`](file:///tmp/d04-codex-20261018/test_startup.py).

Supplied parent-run verification evidence confirms:
- **8 focused tests** in [`test_locked_joints.py`](file:///tmp/d04-codex-20261018/test_locked_joints.py) pass (`OK`, 0.002s; receipt `/tmp/d04-codex-evidence-20261018/focused/D04-20261018T230129508079Z/receipt.json`).
- **50 combined build tests** pass (`OK`, 0.010s; 15 [`test_build_profile.py`](file:///tmp/d04-codex-20261018/test_build_profile.py), 11 [`test_build_profile_boundary.py`](file:///tmp/d04-codex-20261018/test_build_profile_boundary.py), 7 [`test_build_profiles.py`](file:///tmp/d04-codex-20261018/test_build_profiles.py), 9 [`test_provisional_builds.py`](file:///tmp/d04-codex-20261018/test_provisional_builds.py), and 8 [`test_locked_joints.py`](file:///tmp/d04-codex-20261018/test_locked_joints.py)).
- **All 9 offline baseline groups** pass with exit code 0 (`interpreter`, `packet-tests`, `protocol-tests`, `serial-deadline-tests`, `milestone-tests`, `dashboard-tests`, `startup-verifier-tests`, `startup-real`, `startup-demo`).

---

## 2. Detailed Technical Inspection

### A. Separation of Active Coordinates and Locked Joints
- **Omission of absent joints:** Absent joints are entirely omitted from both `BuildProfile.joints` and `BuildProfile.locked_joints`. No placeholder masses, zero-padded records, or synthetic coordinate entries are generated ([`test_locked_joints.py#L51-L56`](file:///tmp/d04-codex-20261018/test_locked_joints.py#L51-L56)).
- **Coordinate separation:** [`BuildProfile.joint_names()`](file:///tmp/d04-codex-20261018/build_profile.py#L234-L235) returns `tuple(joint.name for joint in self.joints)` exclusively. Locked joints in `locked_joints` do not enter active coordinate lists, preserving the 1-DOF active dimension for `arm_j1` while keeping attached mass records.
- **Disjoint name sets and uniqueness:** [`BuildProfile.__post_init__`](file:///tmp/d04-codex-20261018/build_profile.py#L217-L225) enforces:
  1. `self.locked_joints` must be a tuple containing only [`LockedJoint`](file:///tmp/d04-codex-20261018/build_profile.py#L169-L186) instances.
  2. Locked joint names must be unique (`len(locked_names) == len(set(locked_names))`).
  3. Active and locked joint names must be disjoint (`set(names) & set(locked_names)` raises `ValueError`).

### B. LockedJoint Structure and Provenance
- **Field constraints:** [`LockedJoint`](file:///tmp/d04-codex-20261018/build_profile.py#L169-L186) contains only `name: str`, `mass: Measurement`, and `position: Measurement`.
- **No motor/bus/mapping attributes:** Locked joints define no `motor_id`, `bus`, `actuator_identity`, `encoder`, `mapping`, or `position_limits` ([`test_locked_joints.py#L48-L49`](file:///tmp/d04-codex-20261018/test_locked_joints.py#L48-L49)).
- **Measurement integrity:**
  - `self.mass` is validated to be a [`Measurement`](file:///tmp/d04-codex-20261018/build_profile.py#L63-L92) with `name == 'body_mass'` and `units == 'kg'`. If known, `self.mass.value >= 0` is strictly required ([`build_profile.py#L181-L182`](file:///tmp/d04-codex-20261018/build_profile.py#L181-L182)).
  - `self.position` is validated to be a [`Measurement`](file:///tmp/d04-codex-20261018/build_profile.py#L63-L92) with `name == 'locked_angle'` and `units == 'rad'`.
  - When `status == "unknown"`, `value` must be `None` (no numeric value allowed), ensuring unknown remains unknown without default zero-filling.
  - Serialization via [`LockedJoint.to_dict()`](file:///tmp/d04-codex-20261018/build_profile.py#L184-L185) preserves provenance fields (`status`, `units`, `source`, and conditional `value`).

### C. Active Config Selector vs. Readiness / Authority
- **Config-only selector:** [`BuildProfile.require_active_joint(name)`](file:///tmp/d04-codex-20261018/build_profile.py#L227-L232) validates that `name` is an unpadded non-empty string and looks only in `self.joints`. If `name` is locked or absent, it raises `ValueError(f'joint {name} is absent or locked in this build')` ([`test_locked_joints.py#L42-L50`](file:///tmp/d04-codex-20261018/test_locked_joints.py#L42-L50)).
- **Authority boundary:** It returns static configuration [`JointEntry`](file:///tmp/d04-codex-20261018/build_profile.py#L91-L167) only; it conveys no execution readiness, arming capability, or physical movement authority.

### D. Core Positional Dimension Rejection with Pure Fake Station
- [`test_locked_joints.py#L102-L121`](file:///tmp/d04-codex-20261018/test_locked_joints.py#L102-L121) sets up [`JointCore`](file:///tmp/d04-codex-20261018/joint_controller.py) purely from `configured.joints` (1-element `J1` spec) with [`FakeStation`](file:///tmp/d04-codex-20261018/test_controller.py).
- Submitting a 2D target `(value, value)` is rejected with [`DecisionCode.INVALID_ARGUMENT`](file:///tmp/d04-codex-20261018/joint_records.py#L30-L42).
- The test verifies:
  1. No reference is generated and `station.consume()` returns `None`.
  2. Actuator writes are unchanged.
  3. `core.snapshot().pending_target` remains `None`.
  4. Passing a `LockedJoint` directly to `JointCore` raises `ValueError`, proving locked joints cannot be misused as `JointSpec`.
- Demonstrates safe positional coordinate rejection on the existing fake station without touching production owner integration.

### E. JSON Boundary and Schema Version 1 Backward Compatibility
- **Top-level optional field:** `locked_joints` is added to `TOP_LEVEL_FIELDS` ([`build_profile.py#L13`](file:///tmp/d04-codex-20261018/build_profile.py#L13)), but excluded from `REQUIRED_TOP_LEVEL_FIELDS` ([`build_profile.py#L14`](file:///tmp/d04-codex-20261018/build_profile.py#L14)).
- **Backward compatibility:** Existing version 1 profiles without `locked_joints` deserialize with `locked_joints=()`. [`BuildProfile.to_dict()`](file:///tmp/d04-codex-20261018/build_profile.py#L237-L249) omits the `'locked_joints'` key when empty, preserving byte-identical JSON round-tripping for legacy records ([`test_locked_joints.py#L96-L101`](file:///tmp/d04-codex-20261018/test_locked_joints.py#L96-L101)).
- **Parser strictness:**
  - `_parse_locked_joint` rejects unknown fields (e.g. `motor_id`, `bus`, `mapping`), non-dict items, and missing required keys (`name`, `mass`, `position`).
  - `parse_json` enforces duplicate key rejection at all nesting levels via `unique_object` hook ([`build_profile.py#L339-L345`](file:///tmp/d04-codex-20261018/build_profile.py#L339-L345)) and rejects non-finite tokens via `reject_constant` ([`build_profile.py#L336-L337`](file:///tmp/d04-codex-20261018/build_profile.py#L336-L337)).

---

## 3. Reviewed File Hashes

The SHA-256 hashes of the inspected workspace files match `/tmp/d04-codex-evidence-20261018/review/source-hashes.json`:

| File | SHA-256 |
|---|---|
| [`build_profile.py`](file:///tmp/d04-codex-20261018/build_profile.py) | `2848bdbbdc928a3d66d05495062e9aeafbc6c9d00a1edb50de99920bcce6c10f` |
| [`test_locked_joints.py`](file:///tmp/d04-codex-20261018/test_locked_joints.py) | `ead311dcb3f776b44eb2a6f20662cfc181876a44063a3eaca44e7ea963c402a6` |
| [`scripts/check_milestone.py`](file:///tmp/d04-codex-20261018/scripts/check_milestone.py) | `c576e73798a2e30917cf45435132ecec66380cb008813de10e95400e6d44fdee` |
| [`scripts/check_offline_baseline.sh`](file:///tmp/d04-codex-20261018/scripts/check_offline_baseline.sh) | `f04e33e75b3a0534c34808c32f8cf61889b64d2924ba10e2f092785d445c1cab` |
| [`test_startup.py`](file:///tmp/d04-codex-20261018/test_startup.py) | `7f3f1a4e03c3d1b69f97646939ad82f45f9ac73eb0c85a8d568a2a68c7518477` |
| [`docs/locked-joints.md`](file:///tmp/d04-codex-20261018/docs/locked-joints.md) | `fe0f79426d66feea60c9c7e07c47c006b8840d0839cbfa060e1be820825f63cb` |

---

## 4. Boundary and Scope Disclosures

- **Read-only execution:** No tests or runtime scripts were run by this reviewer. Verification claims rely directly on supplied parent artifacts and logs.
- **Physical hardware untouched:** No serial or motor port was opened.
- **Future milestones:** D05 calibration binding, D06 validation invalidation, dynamic mass/inertia transforms, gravity compensation, and production station integration remain pending as planned.

No blocking issues found; source is approved for acceptance.