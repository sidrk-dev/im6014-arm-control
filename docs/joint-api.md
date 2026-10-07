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
