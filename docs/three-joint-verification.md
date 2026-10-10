# Three-joint dashboard verification

## Scope

The three-motor profile uses IDs 0, 1 and 2 on one serial bus. Each joint has its own zero, applied settings, feedback and Stop outcome. The default profile controls only J1 and keeps J2/J3 virtual.

This candidate has not replaced the running J1 dashboard. Release is held until browser jogging and visual inspection pass.

## Software evidence

Parent checks on the frozen candidate passed:

- 15 three-joint and 30 J1 tests.
- Eight UI helper groups and five finite geometry/frame checks.
- Two actual loopback HTTP checks and three allocated-PTY checks using real pySerial with simulated motor replies.
- Nine offline baseline groups on the host. Sandbox ownership and socket failures were retained; ownership rules were not weakened.
- Delayed simulated serial replies with repeated input refresh maintained motion authority and produced references for all three joints.
- Executed UI functions preserved edited drafts, cleared the scene on Stop, replaced presets with limit previews, rejected range preview while armed, and kept an asynchronous Apply bound to its requested joint.

Muse and Gemini reviews were inspected and triaged. MiMo returned HTTP 502 without review output. A fresh independent Sol reader inspected the final corrections and found no remaining source blocker, while retaining the release gates below.

The local evidence run is `/tmp/arm-three-joints-20261010`. Its final source manifest, raw failures, receipts, review findings and independent report are retained there. The integration adds this verification record to the frozen source.

## Browser and hardware boundaries

Actual in-app browser actions passed profile selection, Connect, startup timeout acknowledgment, all three individual zeros and Checks, and explicit Enable. A changed J1 limit appeared in the preview; Escape requested Stop and returned to live view.

The keyboard jogging attempt stopped on stale state during browser automation timeouts. Final browser jogging and captured pixel inspection remain incomplete. Backend motion tests do not establish those outcomes.

No physical three-motor test was performed. Before real commissioning, assign unique IDs one motor at a time, verify direction and travel, support gravity-loaded links, and confirm Stop outcomes. CAD arcs show entered limits; they do not establish physical collision clearance. Xbox hardware operation with all three real motors remains unverified.
