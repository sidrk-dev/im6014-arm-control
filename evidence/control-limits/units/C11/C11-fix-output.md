# C11 acceleration repair

Authored only; this lane did not run tests or checks. Parent verification is still required.

Changed in `C11-fix-workbench`:

- `test_controller_limits.py`: kept command windows at the configured 1.0 s limit; refreshed fixture observations and target windows in long sequences; corrected the tiny-cadence clock; made the equality case assert exact `Fraction` equality on a decoded codec-grid endpoint; added acceleration-limit input cases and a C11 COMPLETE/FAILED two-joint history check; replaced the fabricated near-limit reference with complete-write history; separated a feasible near-limit approach from an exact infeasible Stop; checked both velocity streams at the final dispatch gate; and changed the finite extreme case to assert the constrained Stop.
- `docs/controller-limits-design.md` and `docs/joint-api.md`: clarified that a zero objective is projected into the feasible interval, so a previous nonzero velocity may continue while decelerating.

No production source or protocol bytes changed. The controller remains 926 lines. No hardware was accessed.
