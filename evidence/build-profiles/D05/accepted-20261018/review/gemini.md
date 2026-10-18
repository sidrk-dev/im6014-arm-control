### D05 Milestone Review: Calibration Artifact Binding

No blockers found. The milestone implementation adheres strictly to the assigned scope and constraints.

---

### Scope & Architectural Compliance

1. **Exact Canonical Profile and Artifact Binding**
   - [`bind_calibration`](file:///tmp/d05-codex-20261018/calibration_binding.py#L42-L44) computes digests using [`_profile_digest`](file:///tmp/d05-codex-20261018/calibration_binding.py#L8-L11) (SHA-256 of [`BuildProfile.to_json()`](file:///tmp/d05-codex-20261018/build_profile.py#L239-L240)) and [`_artifact_digest`](file:///tmp/d05-codex-20261018/calibration_binding.py#L14-L17) (SHA-256 of raw immutable `bytes`).
   - [`CalibrationBinding.require_match`](file:///tmp/d05-codex-20261018/calibration_binding.py#L35-L39) enforces that both the canonical profile configuration and artifact bytes match their bound digests, raising descriptive `ValueError` exceptions upon mismatch.
   - Any difference in top-level attributes (`revision`, `build`, `robot`, `backend`), joint definitions, mappings, measurement provenance, or locked joint mass/angle alters the profile JSON digest and is rejected.

2. **Strict Boundary and Input Validation**
   - [`CalibrationBinding`](file:///tmp/d05-codex-20261018/calibration_binding.py#L20-L40) is a frozen dataclass.
   - [`CalibrationBinding.__post_init__`](file:///tmp/d05-codex-20261018/calibration_binding.py#L27-L33) verifies that `calibration_id` and `source` are non-empty unpadded strings and that `profile_sha256` and `artifact_sha256` are exact 64-character lowercase hexadecimal strings (`[0-9a-f]{64}`).
   - Artifacts must be non-empty instances of type `bytes` (rejecting `bytearray`, `memoryview`, `str`, `None`, and empty byte sequences).

3. **No Premature Semantic Parsing or Hardware Authorization**
   - The artifact remains opaque; calibration parsing, internal calibration calculation, zero-angle restoration, and persistence loaders are explicitly omitted and deferred.
   - No live application methods exist (`apply`, `restore`, `arm`, `enable`, `permits`, `calibrated`, `allowed_actions` are absent), ensuring no unauthorized live-hardware control or state assumption.
   - Declared mappings and unverified actuator identities remain unchanged.
   - Documentation in [`docs/calibration-binding.md`](file:///tmp/d05-codex-20261018/docs/calibration-binding.md) clearly states these boundaries and notes that D06 will manage dependency invalidation.

---

### Verification & Test Audit

- **Focused Tests ([`test_calibration_binding.py`](file:///tmp/d05-codex-20261018/test_calibration_binding.py))**:
  All 7 test cases in [`D05CalibrationBinding`](file:///tmp/d05-codex-20261018/test_calibration_binding.py#L18-L80) pass cleanly (audited in [`focused-corrected.log`](file:///tmp/d05-codex-evidence-20261018/focused-corrected.log) and [`milestone.log`](file:///tmp/d05-codex-evidence-20261018/milestone.log)):
  - [`test_exact_profile_and_opaque_artifact_match`](file:///tmp/d05-codex-20261018/test_calibration_binding.py#L19-L25)
  - [`test_another_revision_build_robot_or_backend_is_rejected`](file:///tmp/d05-codex-20261018/test_calibration_binding.py#L27-L32)
  - [`test_actuator_bus_id_mapping_and_provenance_changes_reject`](file:///tmp/d05-codex-20261018/test_calibration_binding.py#L34-L41)
  - [`test_locked_mass_angle_and_active_measurement_changes_reject`](file:///tmp/d05-codex-20261018/test_calibration_binding.py#L43-L50)
  - [`test_changed_artifact_is_rejected_even_if_build_matches`](file:///tmp/d05-codex-20261018/test_calibration_binding.py#L52-L55)
  - [`test_strict_input_and_digest_boundaries`](file:///tmp/d05-codex-20261018/test_calibration_binding.py#L57-L70)
  - [`test_binding_is_immutable_metadata_without_live_application`](file:///tmp/d05-codex-20261018/test_calibration_binding.py#L72-L79)
- **Baseline Integration**:
  - Registered D05 in [`scripts/check_milestone.py`](file:///tmp/d05-codex-20261018/scripts/check_milestone.py#L35).
  - Included in [`scripts/check_offline_baseline.sh`](file:///tmp/d05-codex-20261018/scripts/check_offline_baseline.sh#L81-L101) file manifest and test suite.
  - Included in [`test_startup.py`](file:///tmp/d05-codex-20261018/test_startup.py#L112-L119) fingerprint manifest.
  - All 9 baseline checks exited 0 with zero hardware I/O (audited in [`baseline.log`](file:///tmp/d05-codex-evidence-20261018/baseline.log)).