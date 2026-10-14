# Controller design decision

Use the native Sol unified-core sketch. The Muse cross-judge selected the same base after reading both candidates and the current source. The separate gate/filter sketch fails because callers could bypass it and lifecycle state would have three owners. Keep its distinction between the accepted goal and issued reference. Gemini failed preflight and contributed no candidate. Muse's first launcher invocation failed before submission; its permitted OS continuation completed with one submission and zero provider tool rejections.

## Stage signatures

- C07 adds only required created_at_s and expires_at_s to Target. consume_target still returns Target once.
- C08 adds required Ownership to Target and owner to arm. Successful arm returns its minted immutable capability. Revocation retains faults and sequence history. Consumption still returns Target once.
- C09 adds required PositionLimits to JointSpec and rejects out-of-range complete vectors atomically.
- C10 adds required maximum velocity to JointSpec and maximum reference step to the constructor. It replaces the consumed result with Reference and migrates every caller. There is no legacy unfiltered route in the final version.

These deliberate internal migrations allow the required single-milestone checks and commits. They are not public compatibility layers or placeholder fields.

## Reviewer corrections

After lifecycle, observation completeness and fresh-health gates, request validation has a fixed order. Validate the Target shape, future creation and expiry, ownership when present, configured position intervals when present, mapping and codec, then sequence. Constructors reject malformed immutable records. A future-created target returns INVALID_ARGUMENT; equality at expiry returns EXPIRED.

Clock violations raise ValueError before target release. The core validates each sampled finite time against its previous sampled time. Dispatch freshness and deadline checks use the same time sample. Tests assert zero packet writes and unchanged request/reference storage after a regression.

At C10, reuse existing project_position_raw and project_speed_raw with the authoritative external mapping and direction. Select inward representable values in the intersection of the configured position interval and the bounded reference-step interval. Verify decoded packet positions and speed against those intervals. No broad tolerance may conceal outward rounding. If the interval has no representable code, issue no reference and mutate no reference history. Keep wire details at the codec boundary and the issued public result in joint units.

At C10, arm first validates all fresh measured seeds, configured intervals and mapping/codec representability. Failure returns the relevant rejection with no ownership grant or generation change. A successful explicit arm seeds reference history; Stop, first fault and disconnect revoke authority and clear reference state. Snapshot authority is None whenever no grant is active. Its generation watermark never decreases.

The C07 implementation starts now. Future units remain pending until the parent runs their actual checks.
