# C13: combined effort prediction

This unit bounds the predicted sum of feedforward, stiffness and damping effort using the encoded and decoded command. If necessary, it paces position within the existing motion corridor. It never creates a reversed position step solely to cancel excessive effort. A final observation check can veto the complete vector without committing reference history.

## Verification

- 16 effort tests and 124 controller tests passed on the cleaned source.
- All nine offline baseline groups passed, including 199 milestone tests.
- Independent probes passed 32 decoded mapping cases, a final-release feedback-change veto and recovery, and the previously failing reverse-cancellation case.
- The parent inspected four current legacy Qt dashboard captures. Both disconnected startup modes matched all 33 source fingerprints and recorded zero open/exchange attempts. These captures do not represent a change to the arm web dashboard.
- A 40-tick perfect-tracking synthetic trace reached the target. This establishes planner progress under that synthetic feedback, not physical tracking.
- Gemini completed the final configured review and approved it. Its model evidence is pinned argv, with `modelVerified=false`. Muse returned HTTP 400 and produced no review; it was not retried or replaced.
- A fresh independent Sol reviewer approved the logic. The subsequent cleanup deleted 19 redundant comment/docstring blocks, 32 lines, while retaining both public contracts. Syntax and AST equality excluding docstrings proved executable equivalence. The final checks above ran after cleanup.

Earlier review rounds exposed a stalled macro-target and a position-direction inversion; both were reproduced and corrected before acceptance. Their reviews and failed evidence remain retained. The final review applies to the direction correction, not the earlier candidates.

Run the product checks from the repository root:

```bash
python -B -m unittest test_controller_effort
bash scripts/check_offline_baseline.sh /absolute/path/to/python /tmp/c13-baseline
```

The independent probes accept the repository and an output JSON path:

```bash
python -B evidence/controller-commands/units/C13/accepted-20261017/decoded-progress.py "$PWD" /tmp/c13-progress.json
python -B evidence/controller-commands/units/C13/accepted-20261017/final-release-veto.py "$PWD" /tmp/c13-release.json
python -B evidence/controller-commands/units/C13/accepted-20261017/reverse-cancellation.py "$PWD" /tmp/c13-reverse.json
```

The startup test replaces discovery, serial constructors and packet exchange with rejecting sentinels before starting the disconnected UI. See [the inspection](c13-startup-io-proof.md).

## Limits

These are software checks, not certified actuator protection. Persistent effort and temperature latching belong to C14. Physical tracking, production station integration and hardware operation remain unverified. All physical I/O remains blocked after the damaged adapter incident.
