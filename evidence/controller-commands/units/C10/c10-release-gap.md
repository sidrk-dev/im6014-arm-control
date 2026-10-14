# Parent timing reproduction

The initial10 C10 tests pass. A separate fake-clock probe wrapped joint_controller.project_position_raw so clock.value advanced from0.05 to0.2 during projection, while the accepted request deadline was0.1 and feedback_max_age_s was1.0. Core returned a Reference with issued_at_s0.05 and FakeStation wrote1 FOC packet, even though current fake time was0.2. The assertion requiring no reference and no packet failed. This was pure fake/codec I/O, with no port access.

Configuration was JointSpec J1/ID0/JointMap(1,1,0)/PositionLimits(-1,1)/max_velocity1, core max_reference_step_s0.1. Arm/check at fake time0, then accepted Target sequence1, q0.5, dq0, tau0, kp1,kd0.1, created0, expires0.1. Before consume set time0.05. The patched projector ran the real projection then advanced the fake clock to0.2. This reproduces the stale planning timestamp being used as a release decision and issue timestamp.
