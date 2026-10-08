# C12 fixture repairs

Changed only the two assigned fixture files in `C12-fixtures-workbench`:

- `test_controller_lead.py`: `_quarter_cache` now uses zero stiffness and damping, matching the passing parent C12 reproduction while retaining the quarter-position case, exact outside-bound regression, and binary lead-equality/cache checks. The negative-direction fixture now uses `q_meas = 0.125`, lead `0.25`, and goal `0.375`, so the asserted upper boundary and closed interval are exact binary values. Its decoded canonical-packet proof remains.
- `test_controller_expiry.py`: the regressing final-gate clock case now expects fail-closed withdrawal to `STOPPED`, with owner, goal, plan, and dispatch claim withdrawn; sequence, references, joint observations, and write/attempt histories unchanged; zero writes; and the old plan rejected after clock repair without rearming. The bad-preparation and Stop-with-invalid-clock checks remain.

The repaired runtime files were left unchanged; the optional redundant freshness-predicate simplification was not applied.

Read-only inspection confirmed that the supplied parent logs reproduce the four fixture failures addressed above. No tests or code-execution checks were run, as required by the file-authoring-only contract. Focused tests, the offline baseline, and retained parent reproductions remain for parent verification.
