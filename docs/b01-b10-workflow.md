# B01-B10 communication boundary workflow

- [x] Read the Principles section of poteto-mode.
- [x] Phase A: Frame.
- [x] Phase B: Design the workflow.
- [x] Phase C: Run the loop.
- [x] Phase D: Keep the audit trail.
- [ ] Phase E: Verify and hand back.

## Definition of done

B01-B10 each have parent-run evidence against actual source. Existing packet fixtures remain byte-identical. Feedback rejects malformed length, header, CRC, and identity. The serial transaction handles fragmented and noisy streams while respecting a monotonic deadline, and records no fresh state from another motor. A reusable fake records requests and emits controlled chunks. Independently derived signed output-unit fixtures pass. The existing dashboard and disconnected startup pass. Model review, ledger, source revisions and CI are preserved in the private repository. Physical checks have their own identity and cannot turn a fixture into physical proof.

## Scope and throughput

Ten related behaviors in one small transport boundary. One writer implements each verified unit sequentially; parallel read-only design and model review use separate output files. Parent owns Git, test execution and physical I/O. A deterministic fake verifies parser and deadline cases. A bounded pseudo-terminal check verifies the real pyserial timeout contract. Existing bench APIs remain callable. No station, joint mapping, calibration changes, firmware changes or cloud integration are included.

## Ordered units

1. Capture the pre-change baseline and trace callers.
2. Compare two module designs, judge them and record a small public interface.
3. B01. Extract the pure encoder with unchanged compatibility fixtures.
4. B02. Verify decoder fields against a fixed packet and separately captured physical feedback if available.
5. B03. Check every payload and CRC corruption.
6. B04. Check exact lengths and headers.
7. B05. Provide a reusable scripted fake serial transport.
8. B06. Decode byte-at-a-time replies.
9. B07. Recover from noise and damaged candidates before a valid frame.
10. B08. Reject wrong motor identity without accepting it as fresh feedback.
11. B09. Bound complete transactions including write/read, restore transport timeouts and test missing input.
12. B10. Verify exact signed output-unit mapping with independent constants.
13. Rerun the complete offline product checks and inspect renders.
14. Perform parent-only Stop/status and optional bounded low-speed bench validation after communication gates pass and port ownership is clear.
15. Review source and audit trail with Muse and MiMo; fix accepted findings and rerun affected checks.
16. Publish a PR, obtain a fresh independent verdict, verify CI, and land with an expected-head guard.

## Hardware authorization

The operator authorizes low-speed physical tests. They report an unobstructed motor mounted on a solid base, a 36 V supply, ability to cut power immediately, and a successful recent workbench spin. This is operator-reported setup. The parent rediscovers the adapter, checks its owner, uses Stop/status first, and stops on any invalid response or fault. No automatic timeout acknowledgment, clearing, reset, ID change or calibration operation is authorized by this run. Motion is limited to the existing CLI at 1 rpm, 2 seconds, measured-speed trip 2 rpm and travel trip 15 degrees, with zero position stiffness and bounded feedforward. A physical result is recorded from actual decoded traces, not the operator's earlier result.
