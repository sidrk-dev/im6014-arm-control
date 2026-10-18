# D03 Milestone Review: Provisional Later Build Profile Metadata

**Review Scope:** [`build_profiles.py`](file:///tmp/d03-codex-20261018/build_profiles.py), [`test_provisional_builds.py`](file:///tmp/d03-codex-20261018/test_provisional_builds.py), [`docs/provisional-builds.md`](file:///tmp/d03-codex-20261018/docs/provisional-builds.md), and associated baseline wiring in [`scripts/check_milestone.py`](file:///tmp/d03-codex-20261018/scripts/check_milestone.py), [`scripts/check_offline_baseline.sh`](file:///tmp/d03-codex-20261018/scripts/check_offline_baseline.sh), and [`test_startup.py`](file:///tmp/d03-codex-20261018/test_startup.py).  
**Status:** **Approved (Software Review Only)** — No concrete code blockers or schema defects found. Software verification is complete; physical readiness, physical commissioning, and hardware integration remain pending.

---

### 1. Concrete Findings and Defect Analysis
- **Blockers / Bugs:** None detected. The D03 implementation cleanly extends the D01 schema and D02 workflow models without breaking existing contracts.
- **D01/D02 Regressions:** 42 focused tests (covering [`test_build_profile.py`](file:///tmp/d03-codex-20261018/test_build_profile.py), [`test_build_profile_boundary.py`](file:///tmp/d03-codex-20261018/test_build_profile_boundary.py), [`test_build_profiles.py`](file:///tmp/d03-codex-20261018/test_build_profiles.py), and [`test_provisional_builds.py`](file:///tmp/d03-codex-20261018/test_provisional_builds.py)) pass cleanly with zero failures. Host offline baseline passes all 9 test suites (297 tests).
- **Immutability & Singleton Enforcement:** [`BuildDefinition`](file:///tmp/d03-codex-20261018/build_profiles.py#L22-L29) and [`BuildPlan`](file:///tmp/d03-codex-20261018/build_profiles.py#L78-L96) are frozen dataclasses. [`BuildPlan.__post_init__`](file:///tmp/d03-codex-20261018/build_profiles.py#L83-L96) checks `any(self.definition is definition for definition in DEFINITIONS)`, enforcing reference identity to canonical definitions and rejecting mutated copies.

---

### 2. Configuration Declarations vs. Physical Readiness
- **Intended Stage Names:** Build IDs (`arm_j1_j2` through `arm_j1_j2_j3_j4_j5_j6`, `arm_6_tool`, `sim_6_nominal`, `sim_6_measured`) represent intended configuration stages, not assembled or physical facts.
- **Evidence Requirements:** Every physical definition requires un-waivable commissioning evidence in `required_evidence` (such as `joint_mapping_verified`, `link_geometry_verified`, `base_mounting_verified`, `current_session_reference`, and `tool_and_camera_measured` for `arm_6_tool`).
- **`sim_6_measured` Nomenclature:** The name `sim_6_measured` targets a future simulation workflow updated from physical measurements; it requires `assembled_geometry_measured` and `simulation_matches_build_revision` in its `required_evidence`. The profile definition itself does not assert or synthesize measured data.

---

### 3. Unverified Measurements, Units, Provenance & Round-Trip
- **No Inferred/Nominal Numeric Defaults:** In [`provisional_build()`](file:///tmp/d03-codex-20261018/build_profiles.py#L132-L153), unspecified `body_mass` and `link_length` are populated as [`Measurement`](file:///tmp/d03-codex-20261018/build_profile.py#L63-L89) objects with `status='unknown'`, `value=None`, and `source='not measured for this build revision'`. No nominal mass (e.g. 535 g) or link length (e.g. 0.35 m) is silently injected.
- **Unit & Value Enforcement:** Caller-provided measurements are strictly validated against required SI units (`'kg'` for mass, `'m'` for length) and non-negativity (`value >= 0` when known).
- **Provenance Preservation:** Declared and measured entries preserve their explicit sources and statuses.
- **JSON Round-Trip:** Serialization via [`to_json()`](file:///tmp/d03-codex-20261018/build_profile.py#L210) omits `"value"` for unknown measurements. Deserialization via [`parse_json()`](file:///tmp/d03-codex-20261018/build_profile.py#L162-L181) reconstructs identical detached instances (`restored == plan.profile`). Measurement entries are deterministically sorted by name in [`JointEntry.__post_init__`](file:///tmp/d03-codex-20261018/build_profile.py#L129-L133).

---

### 4. Robot, Backend, and Exact Ordered Joint-Set Identity
- **Robot Boundary:** [`BuildPlan.__post_init__`](file:///tmp/d03-codex-20261018/build_profiles.py#L88-L89) enforces `self.profile.robot == 'im6014-arm'`.
- **Backend Matching:** [`BuildPlan.__post_init__`](file:///tmp/d03-codex-20261018/build_profiles.py#L92-L93) enforces `self.profile.backend == self.definition.backend`. Attempting to use a `'simulation'` backend for a physical definition or vice versa is rejected with `ValueError`.
- **Exact Ordered Joint Sequence:** [`BuildPlan.__post_init__`](file:///tmp/d03-codex-20261018/build_profiles.py#L94-L95) enforces `self.profile.joint_names() == self.definition.joint_names`. Truncated, augmented, or re-ordered joint tuples fail immediately.
- **Bus Collision Guard:** Inherited from [`BuildProfile.__post_init__`](file:///tmp/d03-codex-20261018/build_profile.py#L193-L195), `(bus, motor_id)` pairs must be strictly unique.

---

### 5. Simulation vs. Physical Workflows
- **Workflows:**
  - Simulation definitions (`sim_6_nominal`, `sim_6_measured`) restrict commissioned actions to `frozenset({Workflow.INSPECT, Workflow.SIMULATE})`, using `MechanicalSetup.SIMULATED`. They omit `ARM_JOINT_CONTROL`, `BOUNDED_BENCH_TEST`, and `CALIBRATE`.
  - Physical definitions (`arm_j1_j2` through `arm_6_tool`) restrict commissioned actions to `frozenset({Workflow.INSPECT, Workflow.CALIBRATE, Workflow.ARM_JOINT_CONTROL})`, using `MechanicalSetup.MOUNTED_BASE`. They omit `SIMULATE` and `BOUNDED_BENCH_TEST`.
  - Bench definition (`bench_j1`) retains `frozenset({Workflow.INSPECT, Workflow.CALIBRATE, Workflow.BOUNDED_BENCH_TEST})`, using `MechanicalSetup.SECURED_BENCH`.

---

### 6. Authority, Saved Zero, and Future Milestone Boundaries
- **No Motion or Calibration Authority:** Objects in [`build_profiles.py`](file:///tmp/d03-codex-20261018/build_profiles.py) and [`build_profile.py`](file:///tmp/d03-codex-20261018/build_profile.py) carry no execution methods, live serial handles, permission flags, or zero-offset authority.
- **Separation of Concerns:**
  - **D04** (absent joints vs. installed passive/locked mass contributors),
  - **D05** (binding calibration offsets to specific build identities and hardware revisions), and
  - **D06** (invalidation of validation evidence upon physical or parameter changes)  
  remain intentionally deferred. D03 introduces no unvalidated premature hooks for these downstream stages.

---

### Verification Summary
- **Verified Code:** [`build_profiles.py`](file:///tmp/d03-codex-20261018/build_profiles.py), [`test_provisional_builds.py`](file:///tmp/d03-codex-20261018/test_provisional_builds.py), [`docs/provisional-builds.md`](file:///tmp/d03-codex-20261018/docs/provisional-builds.md).
- **Checks Verified by Parent:** Focused D01/D02/D03 suite (42 tests pass) and full host offline baseline (297 tests pass across 9 test groups).
- **Physical Boundary:** No hardware ports opened; fake serial / offline test boundary maintained. Physical verification remains pending.