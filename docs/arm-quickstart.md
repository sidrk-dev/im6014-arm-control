# Arm dashboard quick start

## Start

From the Workbench checkout, run:

```sh
./start_arm.sh
```

Open http://127.0.0.1:8768/. For a USB-free simulation, run `./start_arm.sh --demo` instead. Stop an existing dashboard before starting another one on the same port. The original Development repository is separate.

## Connect and move J1

1. Secure the motor and keep its travel clear. Keep the physical E-stop accessible.
2. Keep the 36 V motor supply off while you inspect the cable's pinout, continuity, polarity, and isolation. Follow [U2D2 connection and verification](u2d2.md). The ordered U2D2 has not arrived, so its physical compatibility is unverified.
3. Select **J1 only** and **U2D2** under **Adapter profile**. Use **Refresh** to list serial devices. Select the path that matches the inspected cable. The FT232H-class candidate label does not prove ROBOTIS provenance or motor communication.
4. Put the motor at the position you want to call zero. Leave **J1 angle at this pose** at `0`, or enter its known physical angle. Check the minimum and maximum angles. Connect uses this current physical pose as that entered angle; it never homes the motor or restores a saved absolute zero.
5. After the electrical inspection, power the motor and select **Connect**. It verifies actual Stop feedback, acknowledges a startup timeout while stopped if necessary and restores the watchdog, applies the entered gains and limits, and records a fresh encoder reference. The motor stays disarmed. A failed step shows its reason and requests Stop.
6. Release jog keys and center controller sticks. Check **Enable movement**. The dashboard runs the health check and enables jogging. There is no second Arm button or held Space/RB requirement.
7. Hold **A/D** or the on-screen minus/plus buttons to move J1. Release to hold position. **STOP**, **Esc**, **Xbox B**, or unchecking Enable requests Stop and disarms. After resolving a reported issue, checking Enable performs the re-check automatically. It never clears a retained motor protection fault.

The current J1 UI settings are **Kp 30, Kd 3, maximum 5 RPM, effort protection 4 Nm**, with a 50% jog multiplier. These were checked on the unloaded bench motor. They are not an assembled-arm tune. J2/J3 keep slow commissioning defaults.

The dashboard shows fresh measured joint feedback separately from preview animation. If another tab owns the connection, disconnect there first. A disconnected preview never proves a motor is connected.

## Three motors on one cable

Assign each motor a unique ID using the vendor tool **one motor at a time**, before daisy-chaining:

| Joint | Motor ID |
| --- | --- |
| J1 base yaw | 0 |
| J2 shoulder | 1 |
| J3 elbow | 2 |

All use 6 Mbaud. Select **3-joint daisychain** while disconnected. Support the shoulder/elbow links mechanically because Stop can release torque. Confirm the displayed support acknowledgment before physical Connect.

Select each target joint to enter its actual current angle, motor direction and travel limits before Connect. Connect applies all three captured declarations and settings while stopped. All selected motors must return fresh healthy feedback and have a valid current reference before Enable can succeed. A missing or faulted motor stops the entire configured arm; Stop attempts all IDs even when one does not reply. Reconnect invalidates every previous reference.

Do not use CAD collision appearance as a measured physical clearance. Preview min/max and the travel arc are visual aids. They never command motors. Set physical limits inside measured clearance, including cables and fasteners. Real J2/J3 motion and loaded-arm support have not yet been tested here.

## Reference angles, limits and tuning

**Set zero here** maps the selected joint's current stopped encoder position to zero. **Use entered angle** maps it to the angle in the field. These buttons remain available for recalibration after Connect. Neither moves the motor. Confirm direction with a small jog before increasing travel.

Draft fields accept any finite number and warn about recommended ranges. Applying physical settings still requires ordered limits containing the declared angle, Kp above zero through 30, Kd 0–6, speed above zero through 100 RPM, and effort above zero through 15 Nm. J1 applied travel stays within -90° to +90° for wire management. J2/J3 travel is operator-defined and must be physically commissioned.

The **effort ceiling is a protection trip, not a firmware torque clamp**. Both predicted PD effort and measured feedback are checked. The measured-speed trip follows the configured speed with a servo margin. Increasing speed or effort is deliberate configuration, not automatic fault recovery.

While stopped, edit **Gains & motion limits**, select **Apply**, then check Enable. Change one gain at a time. Kp changes position stiffness; Kd changes damping. The motor uses hybrid PD control and has no exposed integral tuning field. Never increase gains or effort to conceal a collision, jam or unstable oscillation.

## Xbox and presets

Focus the page and press A once for browser gamepad discovery. Select **Xbox controller** while stopped and center both sticks before Enable. Left stick X controls J1, left stick Y J2, and right stick Y J3. B stops. If the in-app browser cannot see the controller, open the same dashboard URL in Chrome or Firefox. Physical Xbox operation is still unverified.

Ready pose, calibration pose and figure-eight are preview features. In J1-only mode, the optional follow checkbox can move real J1 after explicit Enable and its input-neutral gate. Releasing keys does not pause automatic following; use Stop. In three-joint mode, presets remain visual only and physical path follow is disabled.

## Troubleshooting

| Message or symptom | Action |
| --- | --- |
| No serial adapter | Reconnect the adapter, select Refresh, then choose the path that matches the inspected cable. |
| Several FT232H candidates | Select the candidate whose path matches the inspected cable. The dashboard leaves the default blank when candidates are ambiguous. |
| Missing ID / no motor reply | Check motor power, connector orientation, wiring, unique IDs and 6 Mbaud. |
| Another tab controls the motor | Disconnect in that tab. Do not start a second serial controller. |
| Movement stopped with measured speed or effort | Read the actual reported value and trip threshold. Resolve instability or obstruction before re-enabling. |
| Dashboard server is not responding | Check the terminal running `./start_arm.sh`. The dashboard requests Stop when communication fails. |
| Startup timeout | Connect handles the approved Stop-only acknowledgment and immediately restores the watchdog. Manual diagnostics remain available if setup fails. |
| Retained protection fault | Resolve the cause, disconnect, then restart the dashboard process and declare the current poses again. This retained fault requires a process restart; Connect and Enable never clear it or restart motion. |
| Stale feedback | Check USB/power and the server; enable only after fresh feedback returns. |

## What was physically verified

These October 10, 2026 measurements used the historical FT4232H adapter, not the ordered U2D2. The single secured unloaded J1 motor at 36 V completed a 92.14° position trajectory and a separate **90.29° move through the dashboard controller** using 5 RPM, Kp 30, Kd 3 and 4 Nm protection. Each ended with confirmed Stop and no motor faults.

Short velocity-damped feedforward ramps reached a **15 Nm command**, with Stop between stages and a peak observed speed of 23.46 RPM. This does not establish 15 Nm delivered load torque. The isolated high-torque test encoder did not change production packet limits. Raw evidence and failed attempts are retained in the local run record.

These observations do not validate a loaded three-joint arm, physical Xbox operation, gravity compensation or impedance control.
