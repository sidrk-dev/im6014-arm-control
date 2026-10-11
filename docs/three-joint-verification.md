# Three-joint dashboard verification

## Scope

The three-motor profile uses IDs 0, 1 and 2 on one serial bus. Each joint has its own zero, applied settings, feedback and Stop outcome. The default profile controls only J1 and keeps J2/J3 virtual.

The browser jog and visual release gates have passed. The source remains subject to exact-head CI and guarded merge before deployment.

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

The earlier keyboard attempt stopped on stale state during browser automation timeouts; that failure is retained. A later stable in-app browser run used the actual on-screen J1/J2/J3 controls. All three simulated encoders changed, release returned to holding with zero measured velocity, and Stop requested mode0 for all IDs. The final Connect pipeline calibrated all three entered poses while disarmed; the single Enable checkbox armed, and Stop-to-Enable recovery performed a fresh check. White/black controls and the colored CAD model were visually inspected. The evidence is in `/tmp/j1-power-20261010/browser-exact-final-all-joints.json` and `dashboard-final.jpg`.

No physical three-motor test was performed. Before real commissioning, assign unique IDs one motor at a time, verify direction and travel, support gravity-loaded links, and confirm Stop outcomes. CAD arcs show entered limits; they do not establish physical collision clearance. Xbox hardware operation with all three real motors remains unverified.

## J1 usability correction

The original user fault was a measured-speed trip above the hardcoded 1.5 RPM ceiling, then a client request error that hid its cause. Configured speed now sets the trip with a bounded servo margin. The first actual motor fault remains visible across secondary UI Stops. Connect applies captured settings and fresh declared poses; one Enable checkbox checks and arms. Blank hidden-joint drafts and Kp zero reject before Connect.

Parent checks passed 67 focused Python cases after the final fault-state correction, and all five JavaScript files. Muse and Gemini findings were triaged; MiMo failed HTTP502 with no retry. A fresh nonwriter Sol inspection reproduced and verified the blank-draft fix and accepted the corrected source subject to final artifact/browser/CI gates. Comment cleanup removed 81 narration lines with AST proof, keeping external firmware/browser contracts.

The unloaded J1 motor at 36V completed 92.14 degrees standalone and 90.29 degrees through the dashboard controller at 5 RPM, Kp 30/Kd 3 and 4 Nm protection. Short damped feedforward stages reached a 15 Nm command with 23.46 RPM peak; this is not a 15 Nm delivered load measurement. Every test ended with confirmed Stop and port close. Physical J2/J3 and physical Xbox control remain unverified.
