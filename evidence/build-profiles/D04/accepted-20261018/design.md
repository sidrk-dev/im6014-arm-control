# D04 ground and bounded design

Continue from D03 only after its exact-head CI is green. Existing schema profile.joints denotes active configured coordinates, all with required mappings. Canonical BuildPlan checks exact active dimensions; no runtime station integration exists. Core uses positional targets and rejects target dimensions different from configured specs; unknown feedback motor IDs reject separately.

Chosen shape: keep absent coordinates omitted, active entries unchanged, add a separate immutable LockedJoint record/profile.locked_joints tuple. A locked joint has explicit name, aggregate attached-body mass Measurement (kg, measured/declared/unknown) and lock-angle Measurement (rad, measured/declared/unknown). No motor/bus/mapping or command identity exists on a locked record. Active/locked name sets disjoint; duplicate names reject. Unknown mass/position stay unknown, never zero-filled. JSON optional field omitted when empty preserves version1 profile compatibility. No implicit body defaults.

Rejected alternative: one optional-field JointEntry state bag for absent/active/locked; invalid motor/mapping combinations and invented placeholders. Also reject new controller/station adapter before explicit production integration unit.

Boundary: profile.require_active_joint(name) selects only active configuration, not readiness/permission; pure Core constructed explicitly from active entries must reject extra locked/absent coordinates using the existing exact-dimension guard. Parent fake transport test proves no reference issue, not physical motor behavior. Locked aggregate mass survives serialization; dynamic transforms/inertias and load validation later.

Actual parsers and controller inspected; proceed with this shape after exact-green D03. No silent positional truncation. No hardware/external owner edits.
