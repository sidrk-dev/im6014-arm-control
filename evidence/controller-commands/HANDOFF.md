# C07-C10 handoff

C07 requires finite command creation/expiry timestamps and rejects deadline equality. C08 accepts only the current identity-checked arm grant and revokes it on Stop, fault or disconnect. C09 enforces required closed position intervals in joint radians. C10 issues complete inward codec references bounded by per-joint velocity and a finite time cap, with a final release-time freshness and expiry gate.

Final checks: all16 C10 tests and all9 post-review offline groups pass, including153 milestone tests. The unchanged arm/browser checks passed67 Python tests and5 JavaScript files. Parent ran every acceptance check and inspected final real/demo startup and dark-demo captures. The final receipt confirms checked-source hashes match. All I/O was fake, isolated PTY, demo or disconnected startup.

The parent caught expiry/freshness changes during projection and recorded four failing-before regression cases. Muse review prompted surfaced programming TypeErrors, public-contract tests and removal of an unreachable guard; two TypeError cases failed before the fix. One migrated assertion used the wrong record field and was corrected after its actual failing run. Raw failures and final receipts are retained.

Principles used: model-the-domain kept requested and issued records distinct; sequence-verifiable-units kept four milestone commits independently checked; fix-root-causes moved validation to the actual release boundary; test-behavior-not-implementation proves history through following references; prove-it-works required actual packet, fixture, baseline and image results.

The decision trail is append-only. Local references in it have committed aliases in review/trail-evidence-index.json. C07-C09 historical receipts remain beside the final C10 correction receipts. The initial source review packet is identified by SHA256 and per-file hashes; accepted changes after that review have their own final hashes and tests.

## Attention

reviewed by opencode/muse-spark-1.3-contributor-free through the configured requested route

Parent triage accepted the supported findings and records each rejected recommendation in review/review-triage.md. MiMo's one submission failed with HTTP502 and produced no findings; that seat was not retried or replaced. Model receipts record requests and backend reports, not independent identity attestation. Gemini's design attempt failed before submission, so design-provider diversity was reduced.

The transport-free core is not used by the production browser/bench controller. Actual U2D2 compatibility, physical speed/travel/effort, acceleration, measured-position lead and station integration remain unverified. C11-C16 are pending.
