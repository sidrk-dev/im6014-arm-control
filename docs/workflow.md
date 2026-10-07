# A01-A10 implementation workflow

## Pstack phases

- [x] Read the Principles section of poteto-mode.
- [x] Phase A: Frame.
- [x] Phase B: Design the workflow.
- [x] Phase C: Run the loop.
- [x] Phase D: Keep the audit trail.
- [x] Phase E: Verify and hand back.

## Definition of done

All A01-A10 have parent-reviewed evidence. The original repository is unchanged. Offline tests pass in a fresh locked environment. Startup and close produce no serial opens or writes. The screenshot is inspected. The new private GitHub repository contains reproducible source and evidence. Physical verification remains pending.

## Units

1. Record source state and preserve the plan. Initialize the ledger.
2. Trace entry points and audit packet paths. Record environment.
3. Execute and capture the existing offline baseline tests.
4. Compare two designs for repeatable evidence and safe startup checks. Judge and select one.
5. Lock dependencies, install into a fresh environment, and verify exact versions.
6. Implement only the chosen startup verification and evidence tooling. Parent checks each addition.
7. Document component boundaries and exact live model routes. Review with Muse and MiMo.
8. Parent reruns checks and inspects screenshots. Review the audit trail and comments.
9. Commit verified units. Create and push one private GitHub repository. Verify its contents and local preservation.

## Rigor and throughput

Ten baseline milestones, one existing small Python dashboard, one package environment, and one new repository. Hardware and cloud inference are outside this run. The first checkpoint is the existing offline suite; run no speculative new arm features until it passes. One writer owns each implementation unit. Read-only design exploration can run concurrently with parent-owned baseline capture.

## Architecture phases

1. Ground
2. Sketch
3. Agree (skip: default workflow has no requested approval checkpoint)
4. Implement
5. Scrap (skip unless repeated shape failures occur)

## Arena phases

1. Frame
2. Fan out
3. Cross-judge
4. Pick
5. Graft
6. Verify

Rubric: preserve bench behavior; prove absence of physical I/O; rerunnable evidence; minimal dependencies and code; clear component boundaries.

## Final independent verification and landing

A fresh read-only Sol verifier returned **PASS+NOTES** on candidate `91634b433b3b917458fa51d2fbe14bceb8c57299` against base `7a6881e6da6884fa1e796a662e9e3f347b7d349c`. The binary stable patch ID is `ea1f7768acbaf5f486aadca6b665f79ade304837`. It ran the complete offline driver and checked the actual source, screenshots, dependencies, ledger, receipts, and preserved original checkout.

[PR #1](https://github.com/sidrk-dev/im6014-workbench/pull/1) merged at `25200eda48654ddb3a4cdd08a58ffb67b4d1919d` after current-head CI passed. The parent required the expected head in the server merge operation, confirmed both automatic merge and merge queue requests were absent, and compared the merged tree with the verified candidate tree. They are identical. This closing commit adds factual audit material only.

The complete verdict, fresh execution, source audits, candidate CI, and merge records are in [the final verification directory](../evidence/verification/final/). Its [artifact map](../evidence/verification/final/artifact-map.json) hashes every preserved file and maps historical receipt paths to the portable `run/` copy. Raw GLX whitespace and inherited adapter text clipping remain documented nonblocking notes. No physical motor operation or CUDA computation was performed.

The closing evidence copy also preserves the verifier’s exact binary diff and native host-response envelope. Their original whitespace is retained and is reported by whole-diff whitespace checks; authored documentation and metadata are checked separately.
