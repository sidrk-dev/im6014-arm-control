# Arm control: one real J1, two virtual joints

## Start

From this repository, run:

```bash
./start_arm.sh
```

Open **http://127.0.0.1:8768/**. The server starts disconnected. For a USB-free preview, use `./start_arm.sh --demo` instead. Stop the server with Ctrl+C before switching modes. If port 8768 is already in use, close the previous dashboard terminal; do not run two controllers. The older `start_dashboard.sh` opens the separate bench UI.

The launcher uses `.venv` when present. To install on another computer:

```bash
python3 -m venv .venv
.venv/bin/pip install pyserial
./start_arm.sh
```

## Connect and move J1

1. Secure the unloaded motor, clear its output, supply power, and keep the physical E-stop reachable. Close the old development controller. This dashboard controls **ID 0 only** at 6,000,000 baud. The dropdown lists detected ports; Channel D uses its stable by-id path when adapter metadata identifies it. Listing and Refresh are passive and do not open or probe a port. If the list is empty, reconnect the adapter and Refresh. A custom path can be entered, but its adapter identity remains unverified until Connect and Health Check return current feedback.
2. If a fresh status reports startup timeout in mode 0, confirm the secured-motor checkbox and select **Acknowledge Timeout**. It acknowledges only the timeout bit in Stop, then restores watchdog protection. Other motor faults are not cleared. Stale feedback disables acknowledgment and Enable; the dashboard labels cached values as stale and shows the last reply age.
3. With the motor stopped, point it along your chosen zero direction and click **Set J1 zero here (no movement)**. Alternatively, enter its measured angle and click **Use measured angle for J1**. This declares a software reference; it does not discover a mechanical home. Default travel is **−90° to +90°**, the software ceiling; choose a narrower window when your setup needs it. The dashboard performs the required Check after calibration.
4. While disarmed, select **Keyboard and on-screen buttons** or a detected standard Xbox controller. Release all jog keys and center both sticks. Confirm your setup with the jogging checkbox, then click **Enable jogging**. Arm requires fresh Ready/Stop feedback and sends a neutral sample; release keys or center sticks once after Enable before motion is accepted.
5. Press **A / D** for J1, click its on-screen **− / +** buttons, or use the selected Xbox stick directly. Releasing keys/buttons or centering the stick holds the current position. **Esc, Xbox B, or STOP removes torque**. Press **Health Check** and explicitly enable again after Stop. W/S and Q/E move virtual J2/J3, which have no attached motor or health feedback.

Do not navigate away while moving: hiding or closing the owner tab requests Stop. An independently opened tab starts as an observer; only an explicit successful Connect with a new session can take ownership after the current owner disconnects. Browser Duplicate Tab or an opener can copy sessionStorage and reuse the same authenticated owner session. Use one controlling tab and open another independently to observe; do not duplicate the controlling tab. A resumed owner is confirmed through authenticated session status. An observer cannot send jog input or claim an armed motor. If the owner tab is lost, use the visible owner session or restart the server after confirming the motor is physically safe; do not assume a new tab has taken control.

## Xbox

Focus the dashboard and press **A once** to let the browser detect the controller. A newly detected or replaced controller requires **Health Check**. Select Xbox as the input source while disarmed. If the in-app browser cannot read controllers, open the same address in Chrome or Firefox. Center both sticks, then explicitly **Enable jogging** and center once more to satisfy the post-Arm neutral gate.

Use a standard-mapped controller: left stick X jogs J1 directly, left stick Y is virtual J2, and right stick Y is virtual J3. **B stops**. The input source cannot be changed while armed; stop, Check, and select another source while disarmed. Small stick drift is filtered. The software mapping is checked; a physical Xbox controller has not been tested on this laptop.

## Calibration, limits and PD tuning

Open **Advanced: gains and motion limits** while disarmed. Draft fields accept any finite number and show warnings outside recommendations. **Save draft without applying** stores the draft in backend memory, without changing motor commands; drafts are discarded when the server stops. **Apply settings** applies only values accepted by physical command limits. The displayed **Applied** line is authoritative.

Current physical command limits are Kp > 0 up to 30, Kd 0–6, speed up to 1 rpm, effort up to 1 Nm, and J1 travel within ±90°. The UI defaults to the full ±90° software window; narrow it before arming if your setup needs less travel. Your wire-management requirement limits J1 to 180° total even though the unloaded output can rotate farther. Ordered limits must contain the declared current angle. A reconnect requires a new angle declaration; power-cycle absolute-reference behavior remains unverified.

Defaults **Kp 15, Kd 3, 0.5 rpm, 1 Nm**, with the 50% jog multiplier, come from the unloaded J1 bench checks. They are not an assembled-arm tune. Change one gain at a time while disarmed, Apply, then explicitly re-enable. Kp controls stiffness, Kd damping. There is **no exposed integral gain**; these are output PD gains, not firmware PID calibration. Never increase effort merely to hide a jam or oscillation.

## What has been demonstrated

Earlier recorded unloaded J1 runs moved **2.71°** and **0.52°** in bounded bench/dashboard checks; both ended in confirmed Stop and disconnect. The earlier Kd 0.1 tune also produced an intermittent speed-protection trip; it is not the default. With Kd 1, two bounded dashboard runs moved +0.39° and −0.94°, without errors, then confirmed Stop and disconnect. These are historical unloaded bench observations, not an assembled-arm tune or current telemetry. A CAD view alone is not physical feedback. Physical Xbox use and an assembled three-motor arm are not verified. Gravity compensation and impedance control remain deferred until basic commissioning is complete.

## If Connect fails

- **Port missing:** reconnect the FTDI adapter and Refresh. When identified as Channel D, its stable path is `/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTC2RQ7H-if03-port0` on this laptop. Adapter identities differ on another computer; an unknown adapter is not verified by appearing in the list.
- **Already owned:** close/disconnect the old dashboard or development controller first. Restart this server if its controlling tab was lost.
- **Permission denied:** use your normal serial-device access setup; do not run the whole dashboard as root.
- **No healthy reply:** verify power, motor ID 0, Channel D wiring and 6M baud. Do not change IDs, firmware or baud by trial and error.
- **Check required / stale feedback:** the dashboard labels stale telemetry and locks acknowledgment and Enable. Reconnect the adapter, Refresh and select Channel D, verify motor power and ID 0, then run Health Check. The dashboard never silently rearms.

## Ready pose, Calibration pose and Figure-8

The pose buttons animate a **virtual preview** using the linked arm simulator’s CAD colors and kinematics. Ready is `[0, -20, 302]°`; Calibration is `[0, 0, 360]°`. Equivalent nearest-turn angles are used to avoid unnecessary full revolutions in the preview. Calibration pose does **not** seek hard stops, home the motor, or change the declared software zero. Figure-8 traces the reference 150 mm horizontal / 70 mm vertical Cartesian path with a nominal 10-second period. Preview joint motion is rate limited and geometry-checked; it is not a collision certificate.

To link a preview to real J1, select a pose or path and check **Follow preview trajectory with real J1** while disarmed. After fresh Ready/Stop feedback, explicitly Enable and satisfy the same neutral-input gate. The selected preview then follows automatically without a held modifier. Only J1 follows, through the same bounded rate/effort/travel/watchdog path; J2/J3 remain virtual. Stale feedback or a target outside the applied J1 window pauses following. No motor moves merely because a pose/path button was clicked. Use Live view to return to manual joint jogging; changing paths while armed requests Stop first.

CAD colors, pose values and path conventions come from `ANGELOREYES-afk/arm_3dof_sim` revision `3719fc517a1612eba4ad63d95ad8c9985c0719cc`. The meshes remain the updated local MainAssem (3) model. Rendering uses simple Lambert colors, with no metallic/glow materials or shadow effects.

## Larger unloaded J1 travel

A later tune (Kp 15, Kd 3, 0.5 rpm command limit, 1 Nm ceiling) passed a 30.06° run and a continuation to 88.53° in the retained calibrated frame, with no errors, confirmed Stop and disconnect. These settings are now the defaults. The UI starts with the full ±90° software travel window, while the independent 1.5 rpm protection remains active. Physical measured speed can exceed command speed during transients. These tests used the single secured unloaded motor, not an assembled arm.

If a retained overload protection fault appears, inspect and correct its cause before restarting the server. Disconnect/reconnect intentionally does not erase protection history or re-enable motion.
