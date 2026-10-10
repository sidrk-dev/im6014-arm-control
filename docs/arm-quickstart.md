# Arm control: one real J1, two virtual joints

> **Physical scope:** this dashboard sends motor commands to J1 at ID 0. J2 and J3 move only in the colored CAD preview; they have no physical motor, health feedback, or Stop confirmation here.

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

1. Secure the unloaded motor, clear its output, supply power, and keep the physical E-stop reachable. Close the old development controller. This dashboard controls **ID 0 only** at 6,000,000 baud. Select the USB adapter marked **Channel D · Recommended**, then select **Connect**. Listing and Refresh are passive: they do not open or probe a port. Built-in `/dev/ttyS0`, `/dev/ttyS1`, and similar laptop ports are hidden so they cannot be mistaken for the USB motor adapter. If no adapter appears, reconnect the USB-RS485 adapter and select **Refresh**. For a different adapter, open **Using a different adapter?** and enter its **Port path**.
2. Select **Check motor** and wait for the fresh-feedback status. A listed path alone is not a connection. If fresh feedback reports a startup timeout, keep the motor stopped, confirm the setup is secure and you can cut power, then select **Acknowledge timeout**. It acknowledges only the timeout bit in Stop, then restores watchdog protection. Other motor faults are not cleared. Stale feedback disables acknowledgment and movement; the dashboard labels cached values as stale and shows the last reply age.
3. With the motor stopped, point it along your chosen zero direction and click **Set J1 zero here (0°)**. Alternatively, enter the motor's known angle at its current pose and click **Use entered angle**. This declares a software reference; it does not discover a mechanical home. Default travel is **−90° to +90°**, the software ceiling; choose a narrower window when your setup needs it.
4. While stopped, select **Keyboard and on-screen buttons** or a detected standard Xbox controller. Release all jog keys and center both sticks. Confirm the setup checkbox, then click **Enable J1 movement**. Enable requires fresh Ready/Stop feedback and sends a neutral sample; release keys or center sticks once after enabling before motion is accepted.
5. Press **A / D** for J1, click its on-screen **− / +** buttons, or use the selected Xbox stick directly. **Enable J1 movement** is the explicit start action; no key or button must be held as a deadman. Releasing a jog key/button or centering the stick holds position. **Esc, Xbox B, or STOP sends Stop and disarms**. Select **Check motor**, then enable movement again after Stop. W/S and Q/E move virtual J2/J3 only.

Do not navigate away while moving: hiding or closing the owner tab requests Stop. An independently opened tab starts as an observer; only an explicit successful Connect with a new session can take ownership after the current owner disconnects. Browser Duplicate Tab or an opener can copy sessionStorage and reuse the same authenticated owner session. Use one controlling tab and open another independently to observe; do not duplicate the controlling tab. A resumed owner is confirmed through authenticated session status. An observer cannot send jog input or claim an armed motor. If the owner tab is lost, use the visible owner session or restart the server after confirming the motor is physically safe; do not assume a new tab has taken control.

## Xbox

Focus the dashboard and press **A once** to let the browser detect the controller. A newly detected or replaced controller requires **Check motor**. Select Xbox as the input source while stopped. If the in-app browser cannot read controllers, open the same address in Chrome or Firefox. Center both sticks, then explicitly **Enable J1 movement** and center once more before moving.

Use a standard-mapped controller: left stick X jogs J1 directly, left stick Y is virtual J2, and right stick Y is virtual J3. **B stops**. The input source cannot be changed while armed; stop, Check, and select another source while disarmed. Small stick drift is filtered. The software mapping is checked; a physical Xbox controller has not been tested on this laptop.

## Calibration, limits and PD tuning

Open **Advanced: PD Gains & Motion Limits** while disarmed. Draft fields accept any finite number and show warnings outside recommendations. **Save draft** stores the draft in backend memory without changing motor commands; drafts are discarded when the server stops. **Apply settings** applies only values accepted by physical command limits. The displayed **Applied** line is authoritative.

Current physical command limits are Kp > 0 up to 30, Kd 0–6, speed up to 1 rpm, effort up to 1 Nm, and J1 travel within ±90°. The UI defaults to the full ±90° software window; narrow it before arming if your setup needs less travel. Your wire-management requirement limits J1 to 180° total even though the unloaded output can rotate farther. Ordered limits must contain the declared current angle. A reconnect requires a new angle declaration; power-cycle absolute-reference behavior remains unverified.

Defaults **Kp 15, Kd 3, 0.5 rpm, 1 Nm**, with the 50% jog multiplier, come from the unloaded J1 bench checks. They are not an assembled-arm tune. Change one gain at a time while disarmed, Apply, then explicitly re-enable. Kp controls stiffness, Kd damping. There is **no exposed integral gain**; these are output PD gains, not firmware PID calibration. Never increase effort merely to hide a jam or oscillation.

## What has been demonstrated

Earlier recorded unloaded J1 runs moved **2.71°** and **0.52°** in bounded bench/dashboard checks; both ended in confirmed Stop and disconnect. The earlier Kd 0.1 tune also produced an intermittent speed-protection trip; it is not the default. With Kd 1, two bounded dashboard runs moved +0.39° and −0.94°, without errors, then confirmed Stop and disconnect. These are historical unloaded bench observations, not an assembled-arm tune or current telemetry. A CAD view alone is not physical feedback. Physical Xbox use and an assembled three-motor arm are not verified. Gravity compensation and impedance control remain deferred until basic commissioning is complete.

## If Connect fails

- **No compatible adapter found:** reconnect the FTDI USB-RS485 adapter and select **Refresh**. The Channel D path on this laptop is `/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTC2RQ7H-if03-port0`. Adapter identities differ on other computers; seeing a path in the list does not prove motor communication.
- **Channel D is missing but another adapter appears:** do not guess. Check the FTDI adapter and cable. Open **Using a different adapter?** and enter its **Port path** only if you know your adapter exposes a different path.
- **Already owned:** close/disconnect the old dashboard or development controller first. Restart this server if its controlling tab was lost.
- **Permission denied:** use your normal serial-device access setup; do not run the whole dashboard as root.
- **Adapter appears but no fresh J1 feedback:** verify the motor has separate DC power, motor ID 0, Channel D wiring and 6M baud. Port listing alone is not a connection. Do not change IDs, firmware or baud by trial and error.
- **Check required / stale feedback:** the dashboard marks feedback stale and locks movement. Reconnect the adapter, select Refresh and Channel D, verify motor power and ID 0, then select **Check motor**. The dashboard never silently enables movement.

## Ready pose, Calibration pose and Figure-8

The pose buttons animate a **virtual preview** using the linked arm simulator’s CAD colors and kinematics. Ready is `[0, -20, 302]°`; Calibration is `[0, 0, 360]°`. Equivalent nearest-turn angles are used to avoid unnecessary full revolutions in the preview. Calibration pose does **not** seek hard stops, home the motor, or change the declared software zero. Figure-8 traces the reference 150 mm horizontal / 70 mm vertical Cartesian path with a nominal 10-second period. Preview joint motion is rate limited and geometry-checked; it is not a collision certificate.

To link a preview to real J1, select a pose or path and check **Follow preview trajectory with real J1** while disarmed. After fresh Ready/Stop feedback, explicitly Enable and satisfy the same neutral-input gate. The selected preview then follows automatically without a held modifier. Only J1 follows, through the same bounded rate/effort/travel/watchdog path; J2/J3 remain virtual. Stale feedback or a target outside the applied J1 window pauses following. No motor moves merely because a pose/path button was clicked. Use Live view to return to manual joint jogging; changing paths while armed requests Stop first.

CAD colors, pose values and path conventions come from `ANGELOREYES-afk/arm_3dof_sim` revision `3719fc517a1612eba4ad63d95ad8c9985c0719cc`. The meshes remain the updated local MainAssem (3) model. Rendering uses simple Lambert colors, with no metallic/glow materials or shadow effects.

## Larger unloaded J1 travel

A later tune (Kp 15, Kd 3, 0.5 rpm command limit, 1 Nm ceiling) passed a 30.06° run and a continuation to 88.53° in the retained calibrated frame, with no errors, confirmed Stop and disconnect. These settings are now the defaults. The UI starts with the full ±90° software travel window, while the independent 1.5 rpm protection remains active. Physical measured speed can exceed command speed during transients. These tests used the single secured unloaded motor, not an assembled arm.

If a retained overload protection fault appears, inspect and correct its cause before restarting the server. Disconnect/reconnect intentionally does not erase protection history or re-enable motion.
