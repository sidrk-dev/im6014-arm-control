# D03 independent source acceptance review

What this change does: D03 adds eight provisional build definitions for the two-through-six-joint physical arm, the tool configuration, and nominal/measured simulation targets. The factory preserves explicit D01 joint configuration and represents missing mass and length as unknown measurements with units and a source.

Looks good. No concrete source defect or missing acceptance-critical test was found within D03's prepared design. **Verdict: Ship the software source gate.** This does not establish physical readiness or replace the parent's final exact-source regression and capture review.

## Review scope and evidence

Reviewed the actual changed and new source in `/tmp/d03-codex-20261018`, the prepared independent contract, design, and supplied six-file hash manifest. Read the connected D01 schema, D01 boundary tests, D02 tests, mapping validation, build documentation, and callers. Repository search found the new factory/definitions used by their tests and documentation; no runtime owner, transport, calibration, or motor-control caller was added.

The intended load is local construction and serialization of small immutable configuration records. There is no service, concurrent writer, or hardware execution path in this change.

Checked:

- `build_profiles.py:49-75,83-95` defines all eight requested later stages and binds a plan to the canonical definition object, robot `im6014-arm`, exact build ID, backend, and ordered joint names. A reconstructed or modified definition cannot substitute for the canonical one.
- `build_profiles.py:132-153` requires explicit D01 `JointEntry` values and revision. It copies each joint rather than changing its caller's value. Existing bus, ID, mapping, identity, provenance, and measurement status are preserved. D01 still rejects duplicate bus/ID slots and invalid or missing mappings/provenance.
- `build_profiles.py:140-149` accepts `body_mass` only in kg and `link_length` only in m, with nonnegative known values. D01 rejects nonfinite known measurements and unknown measurements carrying numbers. Missing geometry becomes an unknown measurement with no numeric value; it is never promoted by a measured-simulation build name.
- `build_profiles.py:59-72` keeps physical and simulated mechanical setups and conditional workflows distinct. Simulation has only inspect/simulate metadata. Required physical evidence and the extra tool/camera evidence are requirements, without readiness or permission state.
- All nine D03 test methods were read. They cover stage identities, unknown geometry, JSON round trips, preserved declared/measured provenance, distinct workflow scopes, altered robot/build/backend/joint set/canonical definition, explicit inputs, duplicate bus IDs, geometry units and signs, and the original J1 backend restriction. Existing D01/D02 test expectations remain compatible by inspection.
- `scripts/check_milestone.py:33`, `scripts/check_offline_baseline.sh:81,101`, and `test_startup.py:111-119` include D03 in the focused runner, byte-identical baseline source copy, aggregate regression group, and startup fingerprints. `docs/provisional-builds.md` accurately describes unknowns, conditional evidence, synthetic example values, and deferred D04/D05/D06 behavior.

## Exact reviewed bytes

SHA-256 values below were computed from the inspected checkout and match every entry in `/tmp/d03-codex-evidence-20261018/review/source-hashes.json`.

| File | SHA-256 |
|---|---|
| `build_profiles.py` | `1ff3682ec404e26c9b55f24c4ed506bfc528af3a3201202d385f99384f5e590f` |
| `test_provisional_builds.py` | `1a4d7ef9cac0717cd7fa5fd124a2d02d656c105337701264ad88e76774a77015` |
| `scripts/check_milestone.py` | `17f9246bb5f654d820976ac07eb2078b862d49ec47b372812190c1da0bd4fbe2` |
| `scripts/check_offline_baseline.sh` | `02948ee5f5b66b25534f448554f9a5dc8f27298b3f36d870b14559dfffc2b721` |
| `test_startup.py` | `400b10588aaab980b8f3441912cb1fcfd0eb56c3175914256ab20da8b2eac9bc` |
| `docs/provisional-builds.md` | `1fc7c1ebfa2c363b6bc6f825a7050bacff5ca95908c30b78e38ebd0adee69912` |

Not checked: This read-only review executed no project imports or tests, used no network/provider/hardware path, and made no source or Git changes. The parent's reported nine focused passes, earlier sandbox socket/lock failures, intermediate host baseline, and final exact-source host baseline are separate execution evidence; I did not independently validate their results or screenshots. This is a native independent source review, with no model-diverse approval claimed. D04 passive bodies, D05 calibration identity, D06 invalidation, live reference restoration, and physical commissioning remain outside this gate.
