# Independent D05 source review

What this change does: D05 records a calibration ID and source alongside SHA256 hashes of the existing canonical full build profile and exact opaque artifact bytes. `require_match` rejects a changed serialized profile or changed artifact. It supplies metadata only and introduces no calibration calculation, saved-zero restoration, or runtime authority.

No actionable source findings for the stated scope. This is a source review, not milestone acceptance; the parent owns checks and acceptance.

## Reviewed behavior

- `calibration_binding.py:8-17,27-44`: the binding hashes `BuildProfile.to_json()` encoded as UTF-8 and the original artifact bytes. Artifact input must be nonempty and exactly `bytes`. Both recorded digests must be exactly 64 lowercase hexadecimal characters; IDs and sources must be explicit nonempty unpadded strings. The frozen record has no mutable collection fields.
- `build_profile.py:85-88,141-166,184-185,237-251`: the existing serializer covers every current schema field. It includes schema version, robot/build/backend/revision, ordered active joints, bus and motor ID, nullable actuator identity and its source, encoder convention, all mapping values and provenance, optional limits, all named measurements with status/units/source/value, and locked-joint names/mass/angle with their provenance. Dictionary keys are sorted; active and locked joint order remain part of the profile identity. Measurement names are keys, and the two locked measurement names are fixed by constructor validation. I found no omitted current configuration field that can change while retaining the same serialized profile.
- `test_calibration_binding.py:19-79`: the seven tests inspect matching/round-trip use, changed build/revision/robot/backend, actuator/bus/ID/mapping/provenance, active measurements and locked bodies, changed artifact bytes, strict constructor boundaries, frozen metadata, and absence of readiness/application methods. I read these tests; I did not execute or import them.
- `scripts/check_milestone.py:35,39-47`: D05 maps to its focused test module, and the milestone fingerprint includes both new root Python files through the existing root-file glob.
- `scripts/check_offline_baseline.sh:81,101` and `test_startup.py:111-119`: both new files are copied into the audited baseline source and included in startup fingerprints; the D05 tests are included in the baseline milestone group.
- `docs/calibration-binding.md:3,19-23`: the documentation preserves opaque artifact scope, unknown identity/provenance, and the fresh-session reference and explicit-enable boundary. A match between declarations with unknown identity remains a match between those declarations; it establishes no measured identity, calibration correctness, or permission.
- Static references to the new API occur only in its module, its test, and its documentation example. The baseline/startup code refers to file names for checks and fingerprints. I found no production owner/Core integration with the binding. I did not use Git to assert that unrelated runtime files were unchanged.

Expected use reviewed: local, detached configuration and artifact records. There is no global mutable registry, I/O, or concurrency machinery in this unit. Persistence loading, artifact interpretation, calibration semantics, and runtime application remain outside this review's scope.

## Source identities

Checkout inspected: `/tmp/d05-codex-20261018`. I independently calculated the following SHA256 values with `sha256sum`; these are not merely copied from the supplied manifest. All six entries in `/tmp/d05-codex-evidence-20261018/review/source-hashes.json` matched. I calculated those six values twice during review and saw no change. The canonical serializer hash also matches the recorded D04 review manifest.

| File | Independently calculated SHA256 |
| --- | --- |
| `calibration_binding.py` | `febcb3d173c11eb115e3a6deb463f917162008f3df144748a2d5ebc4e1640846` |
| `test_calibration_binding.py` | `d74b303d1c3393aa9ee8a80a181c2f12600dcf7df21ea38c93f071a91731a88f` |
| `scripts/check_milestone.py` | `cf3905d2b10cdaf1b2407bdc6e321cd23a8fae6a4fa79d01253ed488e5cb1856` |
| `scripts/check_offline_baseline.sh` | `521a3c24112852a006ca069792f22edce85d9a0d58ff515ee0535b59befe8179` |
| `test_startup.py` | `7c99efd9d602391a4c2f59c9d06a9ff48f21604df5d573859a77131edf6275c6` |
| `docs/calibration-binding.md` | `7dad9d3ed4a4965c8056372ad824e131cc2f758c2860ff491bfe329870df492a` |
| `build_profile.py` | `2848bdbbdc928a3d66d05495062e9aeafbc6c9d00a1edb50de99920bcce6c10f` |
| `build_profiles.py` | `1ff3682ec404e26c9b55f24c4ed506bfc528af3a3201202d385f99384f5e590f` |
| `joint_mapping.py` | `a39502e207e9dac4f9dae1827ba4dacd4b9e835633ea11d738834e3be152fd0b` |
| `joint_records.py` | `faa06f7c30878fb986086ec35e55dbcdde7977ff6b5fb407906c56b1f00beaeb` |
| `test_build_profiles.py` | `d8285bd205dd76b99e0b35026da850a5599b7550248b2793dad2ac320f766ab5` |
| `test_locked_joints.py` | `ead311dcb3f776b44eb2a6f20662cfc181876a44063a3eaca44e7ea963c402a6` |
| `docs/build-profile.md` | `d79523a99fc9493b9d877e67cfaebcf11428bb4bf07a7fd50393b971a79bc798` |

Verdict: Ship for the stated D05 source scope, subject to the parent's checks and acceptance.

Not checked: no imports, test execution, Git operations, network/providers, or hardware operations were performed. Parent-supplied test status was not independently established by this source review; no physical result is claimed.
