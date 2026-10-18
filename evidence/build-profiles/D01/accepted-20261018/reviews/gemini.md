### Executive Assessment

**Status: No concrete blocker.** The new D01 Workbench build profile schema artifact in [build_profile.py](file:///tmp/d01-codex-20261018/build_profile.py) is structurally sound, strictly bounded, and ready for parent acceptance. It satisfies all defined milestone constraints for D01 without encroaching on later geometry, tool, or motion authority milestones.

---

### Authority Analysis: Motion and Calibration Authority

**A persisted profile does NOT grant motion or calibration authority.**

1. **No Authority Attributes or Methods**:
   - [`BuildProfile`](file:///tmp/d01-codex-20261018/build_profile.py#L170-L211) provides only `joint_names()`, `to_dict()`, and `to_json()`. It contains no `permits()`, `allowed_actions()`, `connect()`, `arm()`, or `enable()` interfaces.
   - [`JointEntry`](file:///tmp/d01-codex-20261018/build_profile.py#L92-L167) provides only `measurement(name)` and `to_dict()`. It contains no `calibrated` flag, zero-homing bypass, or motion permissions.
   - [`Measurement`](file:///tmp/d01-codex-20261018/build_profile.py#L64-L89) provides only immutable value/provenance storage and `to_dict()`.

2. **Isolated Dependency Direction**:
   - [build_profile.py](file:///tmp/d01-codex-20261018/build_profile.py#L1-L6) imports only standard library modules (`dataclasses`, `json`, `math`) and pure domain records ([`JointMap`](file:///tmp/d01-codex-20261018/joint_mapping.py#L51-L97) from `joint_mapping.py` and [`PositionLimits`](file:///tmp/d01-codex-20261018/joint_records.py#L88-L108) from `joint_records.py`).
   - Neither [`JointController`](file:///tmp/d01-codex-20261018/joint_controller.py) nor [`ArmManualSession`](file:///tmp/d01-codex-20261018/arm_manual.py) imports `build_profile.py`.
   - Parsing a profile performs no filesystem I/O, device discovery, port opening, or serial communication.

3. **Provenanced Mappings Remain Inactive**:
   - Even when `mapping_status` is `"measured"`, the profile acts solely as a configuration descriptor. Live control must still undergo runtime checks, freshness monitoring, state verification, and explicit `arm()` / `checked()` sequences through the controller.

---

### Prioritized Actionable Observations

While there are no functional blockers, two minor maintainability and diagnostic precision observations were identified in the implementation:

#### 1. Inaccurate Error Message on Missing Value for Declared Measurements
- **Location**: [build_profile.py#L80-L84](file:///tmp/d01-codex-20261018/build_profile.py#L80-L84)
- **Path**:
  ```python
  else:
      if self.value is None:
          raise ValueError("a measured measurement requires a value")
      object.__setattr__(self, "value", _number(self.value, "measurement value"))
  ```
- **Finding**: When `self.status == "declared"` and `self.value is None`, the exception states `"a measured measurement requires a value"`.
- **Impact**: Low (diagnostic clarity only). Both `"measured"` and `"declared"` correctly enforce the presence of a finite number, but the diagnostic message mislabels `"declared"` inputs as `"measured"`.
- **Recommended Action**: Change message to `f"a {self.status} measurement requires a value"` or `"a measured or declared measurement requires a value"`.

#### 2. Schema Asymmetry between Explicit `null` and Omission
- **Location**: [build_profile.py#L16](file:///tmp/d01-codex-20261018/build_profile.py#L16), [build_profile.py#L253](file:///tmp/d01-codex-20261018/build_profile.py#L253), [build_profile.py#L254](file:///tmp/d01-codex-20261018/build_profile.py#L254)
- **Finding**:
  - `actuator_identity` is in `REQUIRED_JOINT_FIELDS` and requires an explicit `null` (or string) with an accompanied provenance string in `actuator_identity_source`.
  - In contrast, optional fields `position_limits` and `measurements` are omitted when absent. If an incoming payload contains `"position_limits": null`, `_parse_limits` calls `_object(record["position_limits"], "position_limits")` and raises `ValueError: position_limits must be a JSON object`.
- **Impact**: Low (schema convention). Documented behavior in [docs/build-profile.md](file:///tmp/d01-codex-20261018/docs/build-profile.md#L30-L35) treats limits as optional records (omitted if unused), but consumers writing JSON must know not to serialize `null` for `position_limits`.

---

### Verification and Invariant Audit

The supplied parent verification reports and workspace test suites were audited:

1. **Schema Integrity & Round-Trip Determinism**:
   - [`BuildProfile.to_json()`](file:///tmp/d01-codex-20261018/build_profile.py#L210-L211) uses `sort_keys=True` and `allow_nan=False`.
   - `joints` order is strictly preserved as defined.
   - `measurements` are deterministically sorted by name in [`JointEntry.__post_init__`](file:///tmp/d01-codex-20261018/build_profile.py#L129-L133).
   - Types do not oscillate on round-trip: reductions and limits normalize to `float`, directions and motor IDs remain `int`.

2. **Malformed Input and Boundary Protection**:
   - **Key Duplication**: [`parse_json()`](file:///tmp/d01-codex-20261018/build_profile.py#L277-L296) hooks `object_pairs_hook=unique_object` across all object depths, rejecting duplicate JSON keys before parsing.
   - **Identifier Whitespace**: [`_text()`](file:///tmp/d01-codex-20261018/build_profile.py#L22-L25) strictly enforces non-empty and unpadded strings (`value == value.strip()`), preventing slot shadowing or whitespace collision.
   - **Bus Slot Uniqueness**: [`BuildProfile.__post_init__`](file:///tmp/d01-codex-20261018/build_profile.py#L193-L195) verifies uniqueness of `(bus, motor_id)` pairs, correctly permitting identical motor IDs on distinct buses while rejecting collisions on the same bus.
   - **Non-finite Values**: All non-finite floats (`NaN`, `Infinity`, `-Infinity`, overflow exponentials) are rejected at both JSON parse and constructor boundaries.

3. **Parent Test Verification**:
   - The parent-run report (`/tmp/d01-codex-evidence-20261018/focused/D01-20261018T015324339703Z/receipt.json`) confirmed 25 passing unit and boundary tests across [test_build_profile.py](file:///tmp/d01-codex-20261018/test_build_profile.py) and [test_build_profile_boundary.py](file:///tmp/d01-codex-20261018/test_build_profile_boundary.py).
   - No repository modifications or file writes were performed during this inspection.