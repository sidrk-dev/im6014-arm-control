# Arm control: J1 or three daisy-chained joints

> **Choose your profile:** J1-only controls ID 0 and keeps J2/J3 virtual. The three-joint profile controls IDs 0, 1 and 2 on one bus after individual commissioning. Real J2/J3 commissioning and assembled-arm collision clearance have not been physically verified.

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

1. Secure the unloaded motor, clear its output, supply power, and keep the physical E-stop reachable. Close the old development controller. This dashboard controls **ID 0 only** at 6,000,000 baud in the default `arm_j1` profile. Select the USB adapter marked **Channel D · Recommended**, then select **Connect**. Listing and Refresh are passive: they do not open or probe a port. Built-in `/dev/ttyS0`, `/dev/ttyS1`, and similar laptop ports are hidden so they cannot be mistaken for the USB motor adapter. If no adapter appears, reconnect the USB-RS485 adapter and select **Refresh**. For a different adapter, open **Using a different adapter?** and enter its **Port path**.
2. Select **Check motor** and wait for the fresh-feedback status. A listed path alone is not a connection. If fresh feedback reports a startup timeout, keep the motor stopped, confirm the setup is secure and you can cut power, then select **Acknowledge timeout**. It acknowledges only the timeout bit in Stop, then restores watchdog protection. Other motor faults are not cleared. Stale feedback disables acknowledgment and movement; the dashboard labels cached values as stale and shows the last reply age.
3. With the motor stopped, select the target joint (J1) and point it along your chosen zero direction, then click **Set J1 zero here (0°)**. Alternatively, enter the motor's known angle at its current pose and click **Use entered angle**. This declares a software reference; it does not discover a mechanical home. Default travel is **−90° to +90°**, the software ceiling; choose a narrower window when your setup needs it.
4. While stopped, select **Keyboard and on-screen buttons** or a detected standard Xbox controller. Release all jog keys and center both sticks. Confirm the setup checkbox, then click **Enable J1 movement**. Enable requires fresh Ready/Stop feedback and sends a neutral sample; release keys or center sticks once after enabling before motion is accepted.
5. Press **A / D** for J1, click its on-screen **− / +** buttons, or use the selected Xbox stick directly. **Enable J1 movement** is the explicit start action; no key or button must be held as a deadman. Releasing a jog key/button or centering the stick holds position. **Esc, Xbox B, or STOP sends Stop and disarms**. Select **Check motor**, then enable movement again after Stop. In J1 profile, W/S and Q/E move virtual J2/J3 preview only.

Do not navigate away while moving: hiding or closing the owner tab requests Stop. An independently opened tab starts as an observer; only an explicit successful Connect with a new session can take ownership after the current owner disconnects. Browser Duplicate Tab or an opener can copy sessionStorage and reuse the same authenticated owner session. Use one controlling tab and open another independently to observe; do not duplicate the controlling tab. A resumed owner is confirmed through authenticated session status. An observer cannot send jog input or claim an armed motor. If the owner tab is lost, use the visible owner session or restart the server after confirming the motor is physically safe; do not assume a new tab has taken control.

## 3-Joint Daisychain Profile (`arm_j1_j2_j3`)

The dashboard supports a 3-joint daisychain configuration on a single RS-485 bus at 6 Mbaud:
- **Joint 1 (Base Yaw):** Motor ID 0
- **Joint 2 (Shoulder Pitch):** Motor ID 1
- **Joint 3 (Elbow Pitch):** Motor ID 2

### Motor ID Assignment (Vendor Tool Required)
> **IMPORTANT:** Set motor IDs one at a time using the vendor programming tool **before** connecting the daisychain bus. The dashboard does **not** program or reassign motor IDs. Connecting multiple motors with default ID 0 on the same bus will cause RS-485 bus contention and communication faults. If a motor ID is missing or fails to respond, the telemetry card displays actionable guidance to verify individual motor IDs, wiring, and DC power.

### Profile Selection & Mechanical Support Confirmation
1. While **disconnected**, select the **3-joint daisychain · 3 motors (IDs 0, 1, 2)** arm profile from the dropdown. Profiles cannot be switched while connected.
2. For physical operation, you must check the dedicated mechanical support confirmation checkbox: **"Arm is mechanically supported (Stop releases motor torque on all joints)"**.
   - **Safety note:** Stop, disconnect, or a fault requests mode-0 torque disable on every selected motor. Only valid replies from all IDs confirm Stop. Mechanically support the links because a received Stop releases torque; a missing reply cannot establish that torque was disabled.
   - The existing fault acknowledgment and arm enable checkboxes do not satisfy this requirement; explicit confirmation is mandatory. Demo mode does not require physical support confirmation.
3. Select the Channel D adapter and click **Connect**.

### Per-Joint Calibration & Travel Configuration
1. Select each joint in sequence (**J1**, **J2**, **J3**) using the **Target joint** selector in Step 3.
2. For each joint:
   - Move the joint to a known position or your desired zero pose.
   - Click **Set [Joint] zero here (0°)** or enter the known angle and click **Use entered angle**.
   - Each joint requires its own software zero declaration. Automatic calibration or homing is not performed.
   - **J1 travel:** Wire-management constraints enforce a **−90° to +90°** ceiling on J1.
   - **J2 and J3 travel:** Operator-configured based on your physical setup. The CAD view does not verify physical collision boundaries; verify physical clearance and mechanical stops before arming.
   - If the configured travel span exceeds 360° (one full turn), a span warning is displayed. Draft fields accept any finite values, but physical command limits apply when settings are applied.
3. **Visual limit preview:** Click **Preview min limit** or **Preview max limit** to view the 3D visual preview at the configured travel extremes. Preview min/max is **visual-only and NEVER submits motor commands**. The range preview persists across telemetry updates until **Live view**, **Stop**, **Enable movement**, or a joint change.
4. **All joints commissioned before Arm:** All configured joints must have a valid software zero and pass **Check motor** before **Enable movement** is allowed. While commissioning, the individual joint's measured angle is truthfully displayed as soon as that joint is zeroed, even if other joints remain uncalibrated.

### 3-Joint Jogging and Presets
- When armed in the 3-joint profile, all 3 physical motors respond to jog inputs:
  - **Keyboard:** **A / D** moves J1, **W / S** moves J2, **Q / E** moves J3.
  - **Xbox controller:** Left stick X moves J1, Left stick Y moves J2, Right stick Y moves J3.
- **Preset path following disabled:** In 3-joint mode, preset path follow (e.g. Figure-8) is explicitly disabled to prevent uncoordinated multi-joint physical trajectories. Presets remain available as virtual 3D previews only.

## Xbox

Focus the dashboard and press **A once** to let the browser detect the controller. A newly detected or replaced controller requires **Check motor**. Select Xbox as the input source while stopped. If the in-app browser cannot read controllers, open the same address in Chrome or Firefox. Center both sticks, then explicitly **Enable movement** and center once more before moving.

Use a standard-mapped controller: left stick X jogs J1 directly, left stick Y moves J2, and right stick Y moves J3. **B stops**. The input source cannot be changed while armed; stop, Check, and select another source while disarmed. Small stick drift is filtered. The software mapping is checked; a physical Xbox controller has not been tested on this laptop.

## Calibration, limits and PD tuning

Open **Advanced: PD Gains & Motion Limits** while disarmed. Draft fields accept any finite number and show warnings outside recommendations. **Save draft** stores the draft for the currently selected joint in backend memory without changing motor commands; drafts are discarded when the server stops. **Apply settings** applies only values accepted by physical command limits for the selected joint. The displayed **Applied** line is authoritative.

Current physical command limits are Kp > 0 up to 30, Kd 0–6, speed up to 1 rpm, effort up to 1 Nm, and J1 travel within ±90°. J2 and J3 travel limits are operator-configured. The UI defaults to the full ±90° software window; narrow it before arming if your setup needs less travel. Your wire-management requirement limits J1 to 180° total even though the unloaded output can rotate farther. Ordered limits must contain the declared current angle. A reconnect requires a new angle declaration; power-cycle absolute-reference behavior remains unverified.

Defaults **Kp 15, Kd 3, 0.5 rpm, 1 Nm**, with the 50% jog multiplier, come from the unloaded J1 bench checks. They are not an assembled-arm tune. Change one gain at a time while disarmed, Apply, then explicitly re-enable. Kp controls stiffness, Kd damping. There is **no exposed integral gain**; these are output PD gains, not firmware PID calibration. Never increase effort merely to hide a jam or oscillation.

## What has been demonstrated

Earlier recorded unloaded J1 runs moved **2.71°** and **0.52°** in bounded bench/dashboard checks; both ended in confirmed Stop and disconnect. The earlier Kd 0.1 tune also produced an intermittent speed-protection trip; it is not the default. With Kd 1, two bounded dashboard runs moved +0.39° and −0.94°, without errors, then confirmed Stop and disconnect. These are historical unloaded bench observations, not an assembled-arm tune or current telemetry. A CAD view alone is not physical feedback. Physical Xbox use and an assembled three-motor arm are not verified. Gravity compensation and impedance control remain deferred until basic commissioning is complete.

## If Connect fails

- **No compatible adapter found:** reconnect the FTDI USB-RS485 adapter and select **Refresh**. The Channel D path on this laptop is `/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTC2RQ7H-if03-port0`. Adapter identities differ on other computers; seeing a path in the list does not prove motor communication.
- **Channel D is missing but another adapter appears:** do not guess. Check the FTDI adapter and cable. Open **Using a different adapter?** and enter its **Port path** only if you know your adapter exposes a different path.
- **Already owned:** close/disconnect the old dashboard or development controller first. Restart this server if its controlling tab was lost.
- **Permission denied:** use your normal serial-device access setup; do not run the whole dashboard as root.
- **Adapter appears but no fresh feedback / missing ID:** verify motors have separate DC power, IDs are set one at a time using vendor tool (IDs 0, 1, 2), Channel D wiring and 6M baud. Port listing alone is not a connection. Do not change IDs, firmware or baud by trial and error.
- **Physical 3-joint connect rejected:** confirm the mechanical support checkbox is checked before clicking Connect. Stop releases torque on all 3 motors.
- **Check required / stale feedback:** the dashboard marks feedback stale and locks movement. Reconnect the adapter, select Refresh and Channel D, verify motor power and IDs, then select **Check motor**. The dashboard never silently enables movement.

## Ready pose, Calibration pose and Figure-8

The pose buttons animate a **virtual preview** using the linked arm simulator’s CAD colors and kinematics. Ready is `[0, -20, 302]°`; Calibration is `[0, 0, 360]°`. Equivalent nearest-turn angles are used to avoid unnecessary full revolutions in the preview. Calibration pose does **not** seek hard stops, home the motor, or change the declared software zero. Figure-8 traces the reference 150 mm horizontal / 70 mm vertical Cartesian path with a nominal 10-second period. Preview joint motion is rate limited and geometry-checked; it is not a collision certificate.

To link a preview to real J1 in the single-joint profile, select a pose or path and check **Follow preview trajectory with real J1** while disarmed. After fresh Ready/Stop feedback, explicitly Enable and satisfy the same neutral-input gate. The selected preview then follows automatically without a held modifier. Only J1 follows, through the same bounded rate/effort/travel/watchdog path; J2/J3 remain virtual. In 3-joint daisychain mode, physical path following is disabled and presets remain virtual preview only. No motor moves merely because a pose/path button was clicked. Use Live view to return to manual joint jogging; changing paths while armed requests Stop first.

CAD colors, pose values and path conventions come from `ANGELOREYES-afk/arm_3dof_sim` revision `3719fc517a1612eba4ad63d95ad8c9985c0719cc`. The meshes remain the updated local MainAssem (3) model. Rendering uses simple Lambert colors, with no metallic/glow materials or shadow effects.

## Larger unloaded J1 travel

A later tune (Kp 15, Kd 3, 0.5 rpm command limit, 1 Nm ceiling) passed a 30.06° run and a continuation to 88.53° in the retained calibrated frame, with no errors, confirmed Stop and disconnect. These settings are now the defaults. The UI starts with the full ±90° software travel window, while the independent 1.5 rpm protection remains active. Physical measured speed can exceed command speed during transients. These tests used the single secured unloaded motor, not an assembled arm.

If a retained overload protection fault appears, inspect and correct its cause before restarting the server. Disconnect/reconnect intentionally does not erase protection history or re-enable motion.

If another browser tab owns the connected motor session, return to that tab and disconnect first. If it is gone, stop the dashboard server with Ctrl+C and restart disconnected; the new tab cannot silently take over an open motor bus.
