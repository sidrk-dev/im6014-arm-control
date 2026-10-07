# Joint mapping and controller workflow

## Figure it out

- [x] Read the Principles section of poteto-mode.
- [x] Frame the ten acceptance predicates and scope.
- [x] Design the workflow.
- [ ] Run each unit as a measured experiment.
- [ ] Keep the audit trail in docs/decisions.tsv.
- [ ] Verify the whole and hand back.

## Architect

- [x] Ground.
- [x] Sketch.
- [x] Agree. Parent selected the typed Sol core with explicit corrections.
- [ ] Implement.
- [ ] Scrap. Skip unless the selected shape fails repeated checks.

## Units

- [x] B11 external joint mapping.
- [x] B12 numerical input rejection.
- [x] B13 explicit port configuration.
- [x] B14 diagnostic capture.
- [ ] C01 one bus owner.
- [ ] C02 controller state transitions.
- [ ] C03 per-joint feedback age.
- [ ] C04 per-joint error status.
- [ ] C05 bounded latest target.
- [ ] C06 command sequence validation.
- [ ] Full regression and inspected images.
- [ ] Comment review and configured OpenCode review panel.
- [ ] Fresh independent verdict, exact-head CI, unarmed queue and guarded merge.

The baseline is d8d810888f76c676acaa1d54bc9e790249592f45. The parent baseline driver returned eight zero exit codes before edits. This block uses fake transport and Linux pseudo-terminals. Physical mapping, loaded-arm Stop procedures and measured loop rates remain pending.
