# Versioned build profile identity and mappings

`build_profile.parse_json(text)` parses a detached, immutable build configuration. `BuildProfile.to_json()` writes deterministic JSON, preserving joint order. Schema version 1 rejects duplicate object keys, unknown fields, unsupported backends or versions, duplicate joint names, duplicate motor IDs on a bus, invalid external reductions, missing mappings and nonfinite numbers. Different buses may reuse a motor ID. Names and bus identities cannot have leading or trailing whitespace.

This D01 slice covers build identity, active joint identity, mapping provenance and optional limits/measurements. It is not the complete geometry, dynamics, tool, camera, dataset or policy schema. Installed passive bodies are a later D04 unit. An empty joint list declares no active joints; it does not invent passive mass.

```python
from build_profile import parse_json

profile = parse_json('''{
  "schema_version": 1,
  "robot": "example-arm",
  "build": "bench_j1",
  "backend": "demo",
  "revision": "synthetic-example-1",
  "joints": [{
    "name": "J1",
    "bus": "demo-bus",
    "motor_id": 0,
    "actuator_identity": null,
    "actuator_identity_source": "not read in this synthetic example",
    "encoder": "unmarked",
    "mapping": {"reduction": 1, "direction": 1, "zero_output_rad": 0},
    "mapping_source": "synthetic example only",
    "mapping_status": "declared",
    "measurements": {
      "attached_mass": {"status": "unknown", "units": "kg", "source": "not measured"}
    }
  }]
}''')
assert profile.joint_names() == ("J1",)
assert profile.joints[0].measurement("attached_mass").value is None
assert parse_json(profile.to_json()) == profile
```

An actuator identity is an explicit string or `null` for unknown, with a source. Mappings require a source and `declared` or `measured` status. `JointMap` keeps the existing joint-side to actuator-output mapping; reductions exclude the motor's internal geartrain. Encoder labels (`rotor_incremental`, `output_absolute`, `unmarked`) are configuration metadata. This parser does not add an encoder decoder or infer physical semantics from a label.

Each optional scalar measurement requires units, a source, and `measured`, `declared` or `unknown` status. Unknown measurements contain no value. Supplied measurements must be finite; dimension-specific physical validation belongs to the feature that consumes them. A declared number is not upgraded to measured during round-trip. Optional `position_limits` and `measurements` records must be omitted when absent; explicit `null` is rejected. Position limits, when present, reuse `PositionLimits` validation; this record alone does not assert verified collision clearance.

Parsing a profile performs no filesystem, discovery, serial or motor operations. It does not attach to `ArmManualSession`, restore a trusted absolute zero, grant motion permission, clear faults or replace Connect/Enable. Even a source-labeled measured mapping must undergo the current live session's reference and readiness checks. D02 will distinguish bench and mounted configurations without using a build name as proof of calibration.

Run the focused software check with:

```sh
python scripts/check_milestone.py D01 --python /usr/bin/python --evidence-root /tmp/d01-check
```
