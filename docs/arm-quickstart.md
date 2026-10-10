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

1. Secure the unloaded motor, clear its output, supply power, and keep the physical E-stop reachable. Close the old development controller. This dashboard controls **ID 0 only**, Channel D, 6,000,000 baud. The full adapter path is filled automatically; click **Connect**. Live mode, voltage, temperature and encoder feedback establish communication.
2. If startup timeout appears, confirm the secured-motor checkbox and select **Acknowledge Timeout**. It acknowledges only the timeout bit in Stop, then restores watchdog protection. Other motor faults are not cleared.
3. Enter the current known angle and select **Use this pose as zero / known angle**. Entering 0 declares the current pose your software zero; it does not discover a mechanical home. Initial travel is **−5° to +5°**. The dashboard performs the required Check after calibration.
4. Confirm your setup with the jogging checkbox, then **Enable jogging**. Release all controls to neutral first. The motor stays in Stop until a fresh jog press.
5. Hold **Space + A/D** for J1, or hold its on-screen −/+ button. Release to hold position. **Esc / STOP removes torque**. Press **Check** and explicitly enable again after Stop. W/S and Q/E move virtual J2/J3, which have no attached motor or health feedback.

Do not switch browser tabs while moving: blur/hide requests Stop. Reload retains this tab's session but still stops motion. Another tab cannot take ownership of the motor. If a controlling tab is lost, stop/restart the server to recover ownership. Disconnect before opening another controller.

## Xbox

Use a standard-mapped controller: left stick X is J1, left stick Y is virtual J2, and right stick Y is virtual J3. **Hold RB** to select the gamepad and jog; **B stops**. Changing devices can require Check and re-enabling. Small stick drift is filtered. The software mapping is checked; a physical Xbox controller has not been tested on this laptop.

## Calibration, limits and PD tuning

Open **Advanced: gains and motion limits** while disarmed. Draft fields accept any finite number and show warnings outside recommendations. **Save draft without applying** stores the draft in backend memory, without changing motor commands; drafts are discarded when the server stops. **Apply settings** applies only values accepted by physical command limits. The displayed **Applied** line is authoritative.

Current physical command limits are Kp > 0 up to 30, Kd 0–6, speed up to 1 rpm, effort up to 1 Nm, and J1 travel within ±90°. Keep initial travel at ±5°. Your wire-management requirement limits J1 to 180° total even though the unloaded output can rotate farther. Ordered limits must contain the declared current angle. A reconnect requires a new angle declaration; power-cycle absolute-reference behavior remains unverified.

Defaults **Kp 15, Kd 3, 0.5 rpm, 1 Nm**, with the 50% jog multiplier, come from the unloaded J1 bench checks. They are not an assembled-arm tune. Change one gain at a time while disarmed, Apply, then explicitly re-enable. Kp controls stiffness, Kd damping. There is **no exposed integral gain**; these are output PD gains, not firmware PID calibration. Never increase effort merely to hide a jam or oscillation.

## What has been demonstrated

The secured unloaded J1 moved **2.71°** in a bounded bench run and **0.52°** in an earlier production dashboard HTTP run; both ended in confirmed Stop and disconnect. The earlier Kd 0.1 tune also produced an intermittent speed-protection trip; it is not the default. With Kd 1, two bounded dashboard runs moved +0.39° and −0.94°, without errors, then confirmed Stop and disconnect. These are unloaded bench observations, not an assembled-arm tune. The updated browser's default Connect has returned ID 0, 36 V and encoder feedback, and its timeout acknowledgment restored the watchdog in Stop. A CAD view alone is not physical feedback. Physical Xbox use and an assembled three-motor arm are not verified. Gravity compensation and impedance control remain deferred until basic commissioning is complete.

## If Connect fails

- **Port missing:** reconnect the FTDI adapter. Channel D is `/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTC2RQ7H-if03-port0` on this laptop. Adapter identities differ on another computer.
- **Already owned:** close/disconnect the old dashboard or development controller first. Restart this server if its controlling tab was lost.
- **Permission denied:** use your normal serial-device access setup; do not run the whole dashboard as root.
- **No healthy reply:** verify power, motor ID 0, Channel D wiring and 6M baud. Do not change IDs, firmware or baud by trial and error.
- **Check required / stale feedback:** stop, inspect the reason, then Check. The dashboard never silently rearms.

## Ready pose, Calibration pose and Figure-8

The pose buttons animate a **virtual preview** using the linked arm simulator’s CAD colors and kinematics. Ready is `[0, -20, 302]°`; Calibration is `[0, 0, 360]°`. Equivalent nearest-turn angles are used to avoid unnecessary full revolutions in the preview. Calibration pose does **not** seek hard stops, home the motor, or change the declared software zero. Figure-8 traces the reference 150 mm horizontal / 70 mm vertical Cartesian path with a nominal 10-second period. Preview joint motion is rate limited and geometry-checked; it is not a collision certificate.

To link a preview to the real J1, select **Follow preview with real J1** while disarmed, Check and explicitly enable jogging, then hold Space or Xbox RB. Only J1 follows, through the same bounded rate/effort/travel/watchdog path. The displayed J1 then uses encoder feedback; J2/J3 remain virtual. Targets beyond the applied J1 window pause following and show a warning. No motor moves merely because a pose/path button was clicked. Use Live view to return to manual joint jogging; changing paths while armed requests Stop first.

CAD colors, pose values and path conventions come from `ANGELOREYES-afk/arm_3dof_sim` revision `3719fc517a1612eba4ad63d95ad8c9985c0719cc`. The meshes remain the updated local MainAssem (3) model. Rendering uses simple Lambert colors, with no metallic/glow materials or shadow effects.

## Larger unloaded J1 travel

A later tune (Kp 15, Kd 3, 0.5 rpm command limit, 1 Nm ceiling) passed a 30.06° run and a continuation to 88.53° in the retained calibrated frame, with no errors, confirmed Stop and disconnect. These settings are now the defaults; initial user travel still starts at ±5° until you explicitly apply a wider window. Physical measured speed can exceed command speed during transients; the independent 1.5 rpm protection remains active. These tests used the single secured unloaded motor, not an assembled arm.

If a retained overload protection fault appears, inspect and correct its cause before restarting the server. Disconnect/reconnect intentionally does not erase protection history or re-enable motion.
