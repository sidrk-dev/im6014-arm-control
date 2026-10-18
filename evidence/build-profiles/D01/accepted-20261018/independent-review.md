# D01 independent final source review

What this change does: D01 adds an immutable, versioned JSON boundary for build and active-joint identity, mappings, provenance, and optional scalar measurements and position limits. It preserves explicit joint order and exposes serialization and lookup only.

Verdict: Accept the D01 software schema source. No concrete blocker found. Parent owns checks, Git, milestone acceptance, and release.

## Reviewed identity

Checkout: `/tmp/d01-codex-20261018`. Read-only HEAD: `aff702696e40feb6bad8f5a8b1eb4c5585a2c4bd`. The seven modified/new files below matched `/tmp/d01-codex-evidence-20261018/final-source-hashes.json` when inspected.

| File | SHA-256 |
| --- | --- |
| `build_profile.py` | `80ed9c88d4b5696f6cdbf7386f9c548816302153ef128c21f00b323da52c9143` |
| `test_build_profile.py` | `0b338f0d656993087cd6e6721ba06f40261decd5cfdebd3f42e78682b4c8f732` |
| `test_build_profile_boundary.py` | `6aac02d523746cffcf217637eada63c1fc10ca4fb6a1b4b4155e2618f0c45c76` |
| `scripts/check_milestone.py` | `32942cc814c95ad39601baa18f600aa8290614e111a19a280f708a34d8bca911` |
| `scripts/check_offline_baseline.sh` | `d084141cf4eba5836c81178e2252edb49d2c342795daeea82a8efcc3ce19e16d` |
| `test_startup.py` | `ddb7bb30b2871ccd57c832d0616db7183a6acba234cbdf1ad6e026f01a7284c8` |
| `docs/build-profile.md` | `d79523a99fc9493b9d877e67cfaebcf11428bb4bf07a7fd50393b971a79bc798` |

## Contract and caller coverage

Read the prepared legacy read-only contract first. Grounded the review in `/tmp/d01-codex-design-20261018.md`, `AGENTS.md`, `docs/ROBOT_WORKBENCH_AGENT_PROMPT.md` build identities, required configuration table and D01-D06, plus `docs/architecture.md`.

Read all of `build_profile.py`, both new test modules, the documentation, and the three integration diffs with their surrounding driver/fingerprint code. Traced numeric construction into `JointMap.__post_init__` (`joint_mapping.py:55`) and `PositionLimits.__post_init__` (`joint_records.py:92`), and read the existing `JointSpec` contract. The reused records remain frozen and already reject booleans, wrong numeric types, overflow and nonfinite values; mapping also requires a positive reduction and exact integer direction of -1 or 1.

Repository searches found no production consumer of `build_profile`, `BuildProfile`, or `JointEntry` outside the new module. Existing callers are the two test modules; the milestone runner, baseline source-copy list and test command, and startup fingerprint list now include both suites and the module consistently. Expanding the fixed CHECKS values with `.split()` preserves the previous single-target entries and passes D01's two module names as separate unittest arguments.

Read the live boundary in `arm_manual.py`: fixed `MANUAL_PROFILES`, profile selection and authority reset, current calibration, and arming conditions (`:76`, `:110`, `:420`, `:431`, `:1460`, `:1499`). Read the dashboard owner import and the independent `JointCore.checked/arm` path (`joint_controller.py:365`, `:389`). The new parser is absent from these paths. Persisted `mapping.zero_output_rad` therefore remains metadata; parsing cannot install a trusted current zero or bypass Connect, Check, calibration, freshness or Enable.

Connected source hashes at review:

| File | SHA-256 |
| --- | --- |
| `joint_mapping.py` | `a39502e207e9dac4f9dae1827ba4dacd4b9e835633ea11d738834e3be152fd0b` |
| `joint_records.py` | `faa06f7c30878fb986086ec35e55dbcdde7977ff6b5fb407906c56b1f00beaeb` |
| `arm_manual.py` | `fde1954031c442dd671f8c1df792bf140803b67bf71f598341c3ece868e567de` |
| `joint_controller.py` | `4f9e4fb8865052787db085cbf900c52c1eac7e5706984de3726f645e4f7d1f17` |
| `arm_dashboard.py` | `5d399187dcb26c53fb10922abb5b51f96dbbf9ec93340964e854129b3bac0f8c` |

## Source findings

No must-fix or should-fix finding in the assigned scope.

- Required schema and mapping keys are explicit. Unknown fields fail at every defined object boundary. JSON object-pair handling rejects repeated keys at every depth before dictionaries discard them (`build_profile.py:277-296`).
- Names and bus identifiers reject padding. Duplicate joint names and duplicate `(bus, motor_id)` slots fail, while distinct buses can reuse an ID. Joint order remains unchanged (`:105-128`, `:178-195`).
- Physical actuator identity is explicitly sourced or null for unknown. Mapping provenance requires a source and declared/measured status. Encoder labels are restricted metadata; they add no decoding behavior.
- Measured and declared scalars require a finite value, units and source. Unknown scalars retain `None` and serialize without a value. Optional absent limits/measurements must be omitted; explicit null objects reject. The final diagnostic now correctly names a missing declared value (`:71-88`, `:214-228`). The targeted regression test and omission documentation agree with the final source.
- Direct records validate tuple contents and duplicate names/slots. JSON construction creates detached frozen values; serialization creates fresh dictionaries/lists. Measurements sort deterministically while joints retain their declared order.
- No motion-permission, calibration-restoration, discovery, file, serial, hardware or service operation is added to the profile module. Unknown measurements do not become invented numeric defaults.

D01 is the approved identity/mapping/provenance slice. It does not complete later geometry, tool, camera, policy, passive-body, build-specific readiness, calibration binding or invalidation behavior. Those remain D02-D06 or later work; their absence is not a blocker against the supplied D01 design.

## Comments and deslop

The three new Python files and added integration lines contain no comments/docstrings after the parent's narration deletion. No suppression, workaround justification, extra compatibility layer, dependency, or duplicated numeric mapping/limit validator was introduced. No actionable deletion or style finding.

## Proof limits

This was a fresh source review only. No source/Git edits, test execution, imports, product runs, providers, network, secrets, hardware, services or subagents were used. Only this report was written.

Read the parent's `final-focused.log` recording 26 tests passed, and final `diagnostic-final-baseline.log` plus its run `checks.tsv`, recording all nine groups with exit code 0. These are parent-run evidence, not independent execution by this reviewer. The parent's `parent-final-proof.json` records 282 milestone tests, unchanged owner code, matching source manifests, inspected captures and zero startup I/O sentinels; this source verdict does not independently certify those physical/runtime outcomes.

Read `review-triage.json`: Gemini completed without a blocker on the earlier pinned bytes; its declared-value diagnostic was fixed in these final bytes. Muse's HTTP 400 dropout is unavailable review evidence, not acceptance. This independent verdict is based on the final source listed above, not provider consensus.
