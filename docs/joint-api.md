# Joint mapping

The bench GUI and CLI use actuator output units. The mapping API converts external joint units before packet encoding. It does not open a serial port.

```python
from joint_mapping import JointMap, JointTarget, OutputSample

mapping = JointMap(reduction=3, direction=-1, zero_output_rad=0.25)
output = mapping.to_output(JointTarget(-0.5, 2, 12, 36, 9))
joint = mapping.from_output(OutputSample(output.q_rad, output.rpm, output.tau_nm))
```

This example produces an output position of 1.75 rad, speed of about -57.2958 rpm, torque of -4 Nm, Kp of 4 and Kd of 1. Reverse conversion recovers the original position, speed and torque. The resulting torque exceeds the bench encoder's ±3.5 Nm policy, so encoding it rejects the command. Mapping never clips a value to make it pass.

`JointTarget` uses rad, rad/s and Nm. `OutputTarget` uses rad, rpm and Nm. The offset is in actuator output radians. The internal motor reduction stays inside the codec. Ideal gain conversion assumes compatible stiffness and damping definitions. Friction, backlash, efficiency and physical calibration remain unmeasured.

Run a single offline acceptance check from the repository root:

```sh
.venv/bin/python -B scripts/check_milestone.py B11
```

The command prints the path to its receipt and raw logs under `evidence/runs`. The test checks ratios 1, 3 and 6 in both directions, nonzero offsets, independent numeric anchors and invalid values.

## Passive port inspection

```python
from im6014_port import PortConfig, inspect_port
from spin_5rpm import PORT

report = inspect_port(PortConfig(PORT, 6_000_000), ports=())
```

`ports=()` makes this example use only the configured by-id basename. The report names its evidence source and does not establish communication with the motor. Normal inspection can also read passive OS enumeration metadata. Known wrong serial numbers or interfaces reject with expected and observed values. Unknown metadata stays explicitly unverified.

For an alternate adapter route, set both `expected_serial=None` and `expected_channel=None`. This removes the FTDI identity assertion. It does not bypass later device ownership checks or establish motor communication. The bench GUI and CLI default to the recorded FTDI serial and interface 3.

## Diagnostic capture

`PacketCapture` holds at most 256 records by default. Each record contains capped packet prefixes, allowlisted decoded fields, a fixed outcome code, and explicit source and connection metadata. Exported snapshots own their lists and dictionaries. Exception messages, environment values and arbitrary serial-object fields are excluded.

The GUI includes its ring in the existing JSON snapshot. The CLI enables it in `main`; direct `run(bus, args, log)` callers retain the original uncaptured behavior. Tests opt in with `fixture` provenance. A `physical` label records the caller's declaration and does not independently prove device identity.

## Cooperating bus ownership

Both real bench open sites use `open_managed_port(PortConfig(...), serial_factory=...)`. It checks passive identity before device acquisition and construction. A successful result owns `bus`; call `close()` and inspect its boolean result. `False` means ownership remains held because closure was not confirmed. Retry cleanup before reconnecting. A context manager raises `PortCloseError` on an otherwise normal exit with unconfirmed close; its `managed_port` remains available for cleanup.

This API opens hardware when given a motor path. Its acceptance tests use Linux PTYs exclusively. The device lease coordinates these workbench clients; it cannot exclude a raw client that ignores the lock.
