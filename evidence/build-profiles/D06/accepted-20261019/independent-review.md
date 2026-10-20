What this change does: D06 stores immutable, caller-reported validation outcomes with fingerprints of their declared configuration inputs. Relevant changes return a fresh pending record with no previous result metadata; every completed record includes the full build profile.

Verdict: ACCEPT for the narrow D06 software source contract at the seven pins below. No must-fix correctness defect or blocking regression gap found. This is independent source acceptance, not the parent milestone acceptance or an external reviewer verdict.

I resumed this same unfinished review after the host interruption. I read the prepared prompt-only native contract and standing orders, inspected the real checkout at `/tmp/d06-codex-recovery-20261019`, and wrote only this report and `comment-review.md`. I ran no tests, imported no application code, accessed no hardware or network, dispatched no reviewers, and changed no Git state. The only Python execution in this review used standard-library file reading, hashing, and saved-JSON inspection.

The read-only Git inspection identifies HEAD as the exact D05 base `d0093b1656aec80dca5d24f753af634c2c1f2347`. The worktree has the seven expected D06 source/documentation/test files changed or added. `build_profile.py` and `calibration_binding.py` have no diff against that base.

Independent SHA256 calculation, reading each file as bytes without application imports, matched all seven entries in `review/source-hashes.json`:

| File | Actual SHA256 |
|---|---|
| validation_records.py | 2122e4f7d6c1b4a817cbf4730e0b014286a8e0edc3c5b29f12becbc0e64fb112 |
| test_validation_invalidation.py | 65b12482b062775cd9d4430a14d05c32d643082911f6bf0919f2954f3d17b2db |
| docs/validation-invalidation.md | b0ae40f85919a64cb5fa663cddb751fbfc05c20c563493f2e5a767318b798226 |
| docs/architecture.md | bc4b08cc832c21d9f84720cdc4226802f30686011401fb9c4b913b4d3b2bc5f5 |
| scripts/check_milestone.py | b6bc01f1fc164c48682930c0af13abed6e31152c13b92a118bfc20851b18cd1d |
| scripts/check_offline_baseline.sh | ca48d1c8b62e3f621d6dd9de25aedd638b80d658aa438cae3e10f2920417e907 |
| test_startup.py | 9dda425f6a481a072fa2d9387d0565c0f6a99e36a65a6408afaca68be7473dcf |

The source supports the required behavior:

- [Record construction](/tmp/d06-codex-recovery-20261019/validation_records.py:76) rejects mutable dependency/hash containers, empty or invalid dependency sets, missing PROFILE, malformed/duplicate hashes, incomplete completed records, and result metadata on pending records. Dependency pairs are sorted before retention.
- [Input fingerprints](/tmp/d06-codex-recovery-20261019/validation_records.py:37) include exact artifact bytes and source, and use the existing canonical full-profile serialization. A reduction, identity, revision, backend, ordered joint configuration, or serialized nested fact change therefore changes the mandatory PROFILE fingerprint. Selected tool and mount artifacts are independent dependencies.
- [Result reporting](/tmp/d06-codex-recovery-20261019/validation_records.py:109) requires an explicit passed/failed status and evidence reference, captures every selected fingerprint, and rejects missing selected artifacts. It has no test execution or evidence certification claim.
- [Invalidation](/tmp/d06-codex-recovery-20261019/validation_records.py:120) applies to both passed and failed outcomes. A mismatch constructs a new default pending record, clearing evidence and hashes. An already pending record returns unchanged, so reverting configuration does not complete that returned record.
- [Unknown physical facts](/tmp/d06-codex-recovery-20261019/build_profile.py:63) retain their existing representation. D06 serializes the supplied profile and never changes its mapping, actuator identity, measurements, or uncertainty status. A caller-reported schema pass can therefore coexist with declared mappings or unknown physical facts, as the documentation explicitly states.
- [Integration](/tmp/d06-codex-recovery-20261019/scripts/check_milestone.py:36) adds D06 dispatch. The baseline copies both new Python files before running the D06 tests, and startup fingerprints include both. There is no new production runtime caller or owner/adapter change.

I read all 16 D06 test definitions and traced the constructor, reporting, invalidation, and profile serialization paths. Existing tests cover unchanged passed/failed results, reduction changes and metadata clearing, profile identity changes, tool/mount content and provenance, selected dependency independence, missing selected inputs, sticky pending after revert, fresh reporting, strict new context/artifact types, immutable records, malformed hashes, and canonical profile roundtrip. I also read the parent sketch, synthesis, and grounded alternative designs as rationale; the acceptance above comes from the final source.

One nonblocking coverage gap remains: the exact-byte rule is not isolated by an artifact change that preserves JSON meaning. A useful future regression case is a selected tool changing from `b'{"tool":"x"}'` to `b'{ "tool":"x" }'` with the same source; the expected returned status is pending. The current implementation hashes bytes directly and handles this correctly by inspection. No implementation fix is requested.

Concrete API boundary traced from source, not executed here: after `pending = completed.invalidate_changed(changed)`, `pending.invalidate_changed(original)` remains pending. Calling `completed.invalidate_changed(original)` can still return the old completed object. The latter is the documented immutable-history limitation, not a defect in this scope; callers must retain the returned current record. Likewise `report_result(current_context, status=PASSED, evidence=old_reference)` records the caller's declaration and cannot establish that the referenced check actually ran for those inputs.

Saved parent evidence actually inspected:

- `before.log` contains the original `ModuleNotFoundError: No module named 'validation_records'`. This is an import-stage red result, not proof that a behavioral mutation was detected.
- `focused/D06-20261019T225852465658Z/stderr.log` records 16 passing tests. Its receipt records exit 0, the exact D05 base, unchanged source, and hashes matching the reviewed source and integration files. The older `focused.log` contains a 14-test iteration; it is not the final focused receipt.
- `baseline-host/run-20261019T230004Z-LpddUCer/checks.tsv` records all nine groups at exit 0. The milestone log records 329 tests and OK. Both startup JSON files record exit 0, no main exception or callback errors, successful closure, and zero serial-open/demo-open/exchange sentinels. Their D06 and startup source hashes match the reviewed files. These are inspected parent-run results, not checks run by this reviewer.
- The retained sandbox baseline has milestone and dashboard failures. I inspected its explicit `/tmp` ownership exception, forbidden socket exception, dashboard assertion, and failing summary. The host pass is separate evidence; I have not erased or reclassified the sandbox failures as passing results.

Not checked: application execution by this reviewer; visual correctness of saved screenshots; physical behavior; semantic tool/transform validity; evidence-reference contents; persistence and runtime retention; external Gemini/Muse reviews or receipts; CI, deployment, or merge readiness. Parent summary JSONs about mutation/doc execution and screenshot review were read as summaries and were not substituted for independent execution or visual inspection. No broader readiness conclusion follows from this report.

The companion comment audit reports zero deletions and zero MUST KILL flags. No source edits are requested by that audit.
