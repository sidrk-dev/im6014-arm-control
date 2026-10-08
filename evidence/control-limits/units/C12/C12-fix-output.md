# C12 authoring report

## Changes made

- `C12-fix-workbench/joint_reference.py`: admission and zero-cadence cached-reference lead checks now compare normalized finite float values as exact `Fraction`s over inclusive bounds. Positive-cadence shaping and final-gate exact checks remain intact.
- `C12-fix-workbench/joint_controller.py`: a clock callback exception or invalid/regressing clock at `claim_dispatch` stops the core, revokes owner/window/goal/prepared plan, and re-raises the original error before granting a claim. The broad `dispatch_result` clock-error handling after a complete write remains unchanged. Removed redundant sample-presence checks where `JointStatus.valid` already guarantees a sample.
- `C12-fix-workbench/test_controller_lead.py`: moved the C12 test class into its own module; added regressions for both reproduced float lead failures, inclusive binary equality, and invalid, regressing, and callback-failing final-gate clocks. Corrected the conflicting fixtures to use consumed-sequence progression, stale-goal withdrawal, valid windows and goals, and actual shaped/canonical emitted references. Seeded cases certify decoded packet values and exact position, speed, acceleration, and lead limits across both directions, noninteger reductions, reversals, and infeasible cases.
- `C12-fix-workbench/test_controller_limits.py`: retained the C09–C11 test classes without the appended C12 class.
- `C12-fix-workbench/scripts/check_milestone.py`, `C12-fix-workbench/scripts/check_offline_baseline.sh`, and `C12-fix-workbench/test_startup.py`: registered, copied, invoked, and fingerprinted the dedicated lead test module.
- `C12-fix-workbench/docs/joint-api.md`: documented exact inclusive lead comparisons and final-gate clock failure withdrawal behavior.

## Checks and boundaries

No tests, checks, Python, or other code were run, as required by the file-authoring-only assignment. Only read-only source inspection and file edits were performed. The parent owns the assigned C12, whole-baseline, certificate, gate-consistency, and reproduction checks. No hardware was accessed.
