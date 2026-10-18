Read-only NEW D05 Workbench milestone review. Human approved Workbench source to Gemini/Muse; no HAPI-fork material. Root personally implements; no test/import execution, Git, hardware, other providers or subagents. Goal is exact binding of an existing opaque calibration artifact to full canonical profile identity, NOT a calibration engine/data parser, saved-zero restoration, hardware readiness or live apply permission. CalibrationBinding immutableID/source/profileSHA256/artifactSHA256; require_match rejects changed revision/config or artifact. Unknown/declared facts remain unverified, no new UI requirements or owner integration. 7focused parent tests pass; audited disconnected full baseline running. Find concrete bugs or scope gaps, call out missing artifact semantic validation only if this binding contract falsely claims it (docs explicitly defer it). No persistence loader/API; Python constructor validates metadata, bytes immutable. D06 dependency invalidation later. Return findings or no blockers; parent owns acceptance.

## calibration_binding.py
```
from dataclasses import dataclass
import hashlib
import re

from build_profile import BuildProfile


def _profile_digest(profile: BuildProfile) -> str:
    if not isinstance(profile, BuildProfile):
        raise ValueError('profile must be a BuildProfile')
    return hashlib.sha256(profile.to_json().encode('utf-8')).hexdigest()


def _artifact_digest(artifact: bytes) -> str:
    if type(artifact) is not bytes or not artifact:
        raise ValueError('calibration artifact must be non-empty immutable bytes')
    return hashlib.sha256(artifact).hexdigest()


@dataclass(frozen=True)
class CalibrationBinding:
    calibration_id: str
    source: str
    profile_sha256: str
    artifact_sha256: str

    def __post_init__(self):
        for value in (self.calibration_id, self.source):
            if type(value) is not str or not value.strip() or value != value.strip():
                raise ValueError('calibration ID and source must be non-empty unpadded strings')
        for digest in (self.profile_sha256, self.artifact_sha256):
            if type(digest) is not str or re.fullmatch('[0-9a-f]{64}', digest) is None:
                raise ValueError('calibration binding digests must be lowercase SHA256 hex strings')

    def require_match(self, profile: BuildProfile, artifact: bytes) -> None:
        if self.profile_sha256 != _profile_digest(profile):
            raise ValueError('calibration belongs to a different build revision or configuration')
        if self.artifact_sha256 != _artifact_digest(artifact):
            raise ValueError('calibration artifact content differs from its binding')


def bind_calibration(profile: BuildProfile, artifact: bytes, *,
                     calibration_id: str, source: str) -> CalibrationBinding:
    return CalibrationBinding(calibration_id, source, _profile_digest(profile), _artifact_digest(artifact))

```

## test_calibration_binding.py
```
from dataclasses import FrozenInstanceError, replace
import unittest

from build_profile import Measurement, parse_json
from calibration_binding import CalibrationBinding, bind_calibration
from joint_mapping import JointMap
from test_build_profiles import arguments
from build_profiles import arm_j1
from test_locked_joints import locked

ARTIFACT = b'{"synthetic_calibration_fixture":1}'


def profile():
    return replace(arm_j1(**arguments()).profile, locked_joints=(locked(),))


class D05CalibrationBinding(unittest.TestCase):
    def test_exact_profile_and_opaque_artifact_match(self):
        configured = profile()
        binding = bind_calibration(configured, ARTIFACT, calibration_id='synthetic-cal-1', source='synthetic fixture')
        self.assertIsNone(binding.require_match(configured, ARTIFACT))
        self.assertIsNone(binding.require_match(parse_json(configured.to_json()), bytes(ARTIFACT)))
        self.assertEqual(binding.calibration_id, 'synthetic-cal-1')
        self.assertEqual(binding.source, 'synthetic fixture')

    def test_another_revision_build_robot_or_backend_is_rejected(self):
        configured = profile()
        binding = bind_calibration(configured, ARTIFACT, calibration_id='synthetic', source='synthetic')
        for changes in ({'revision':'other'}, {'build':'other'}, {'robot':'other'}, {'backend':'simulation'}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                binding.require_match(replace(configured, **changes), ARTIFACT)

    def test_actuator_bus_id_mapping_and_provenance_changes_reject(self):
        configured = profile();joint = configured.joints[0]
        binding = bind_calibration(configured, ARTIFACT, calibration_id='synthetic', source='synthetic')
        for changes in ({'actuator_identity':'another-tag'}, {'bus':'another-bus'}, {'motor_id':1},
                        {'mapping':JointMap(3, 1, 0)}, {'mapping_source':'another-source'},
                        {'mapping_status':'measured'}, {'encoder':'output_absolute'}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                binding.require_match(replace(configured, joints=(replace(joint, **changes),)), ARTIFACT)

    def test_locked_mass_angle_and_active_measurement_changes_reject(self):
        configured = profile();binding = bind_calibration(configured, ARTIFACT, calibration_id='synthetic', source='synthetic')
        for changed in (replace(configured, locked_joints=()),
                        replace(configured, locked_joints=(replace(locked(), mass=Measurement('body_mass','measured','kg','synthetic',1)),)),
                        replace(configured, locked_joints=(replace(locked(), position=Measurement('locked_angle','declared','rad','synthetic',1)),)),
                        replace(configured, joints=(replace(configured.joints[0], measurements=(Measurement('body_mass','unknown','kg','not weighed'),)),))):
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                binding.require_match(changed, ARTIFACT)

    def test_changed_artifact_is_rejected_even_if_build_matches(self):
        binding = bind_calibration(profile(), ARTIFACT, calibration_id='synthetic', source='synthetic')
        with self.assertRaises(ValueError):
            binding.require_match(profile(), ARTIFACT + b' ')

    def test_strict_input_and_digest_boundaries(self):
        for artifact in (b'', '', None, bytearray(ARTIFACT)):
            with self.subTest(artifact=artifact), self.assertRaises(ValueError):
                bind_calibration(profile(), artifact, calibration_id='synthetic', source='synthetic')
        for name in ('calibration_id','source'):
            for bad in ('', ' padded ', None):
                values=dict(calibration_id='synthetic',source='synthetic');values[name]=bad
                with self.subTest(name=name,bad=bad), self.assertRaises(ValueError):
                    bind_calibration(profile(), ARTIFACT, **values)
        for digest in ('a'*63, 'A'*64, 'x'*64, None):
            with self.subTest(digest=digest), self.assertRaises(ValueError):
                CalibrationBinding('synthetic','synthetic',digest,'a'*64)
        with self.assertRaises(ValueError):
            bind_calibration(None, ARTIFACT, calibration_id='synthetic', source='synthetic')

    def test_binding_is_immutable_metadata_without_live_application(self):
        configured = profile();binding = bind_calibration(configured, ARTIFACT, calibration_id='synthetic', source='synthetic')
        with self.assertRaises(FrozenInstanceError):
            binding.profile_sha256 = 'a'*64
        for name in ('apply','restore','arm','enable','permits','calibrated','allowed_actions'):
            self.assertFalse(hasattr(binding,name))
        self.assertIsNone(configured.joints[0].actuator_identity)
        self.assertEqual(configured.joints[0].mapping_status,'declared')


if __name__ == '__main__': unittest.main()

```

## build_profile.py
```
from dataclasses import dataclass
import json
import math

from joint_mapping import JointMap
from joint_records import PositionLimits

SUPPORTED_SCHEMA_VERSIONS = (1,)
ENCODER_CONVENTIONS = ("rotor_incremental", "output_absolute", "unmarked")
MEASUREMENT_STATUSES = ("measured", "declared", "unknown")
BACKENDS = ("physical", "simulation", "demo")

TOP_LEVEL_FIELDS = ("schema_version", "robot", "build", "backend", "revision", "joints", "locked_joints")
REQUIRED_TOP_LEVEL_FIELDS = tuple(name for name in TOP_LEVEL_FIELDS if name != "locked_joints")
JOINT_FIELDS = ("name", "bus", "motor_id", "actuator_identity", "actuator_identity_source", "encoder", "mapping", "mapping_source", "mapping_status", "position_limits", "measurements")
REQUIRED_JOINT_FIELDS = tuple(name for name in JOINT_FIELDS if name not in ("position_limits", "measurements"))
MAPPING_FIELDS = ("reduction", "direction", "zero_output_rad")
LIMIT_FIELDS = ("minimum_rad", "maximum_rad")
MEASUREMENT_FIELDS = ("status", "units", "source", "value")


def _text(value, name):
    if type(value) is not str or not value.strip() or value != value.strip():
        raise ValueError(f"{name} must be a non-empty unpadded string")
    return value


def _object(record, name):
    if not isinstance(record, dict):
        raise ValueError(f"{name} must be a JSON object")
    return record


def _reject_unknown_fields(record, allowed, name):
    for key in record:
        if not isinstance(key, str):
            raise ValueError(f"{name} field names must be strings")
    unknown = sorted(set(record) - set(allowed))
    if unknown:
        raise ValueError(f"{name} has unknown fields: {', '.join(unknown)}")
    return record


def _require_keys(record, required, name):
    missing = sorted(set(required) - set(record))
    if missing:
        raise ValueError(f"{name} is missing required fields: {', '.join(missing)}")
    return record


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a real number")
    try:
        result = float(value)
    except (OverflowError, TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be finite") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


@dataclass(frozen=True)
class Measurement:
    name: str
    status: str
    units: str
    source: str
    value: float | None = None

    def __post_init__(self):
        _text(self.name, "measurement name")
        _text(self.units, "measurement units")
        _text(self.source, "measurement source")
        if self.status not in MEASUREMENT_STATUSES:
            raise ValueError("measurement status must be measured, declared or unknown")
        if self.status == "unknown":
            if self.value is not None:
                raise ValueError("an unknown measurement must not carry a value")
        else:
            if self.value is None:
                raise ValueError(f"a {self.status} measurement requires a value")
            object.__setattr__(self, "value", _number(self.value, "measurement value"))

    def to_dict(self):
        if self.status == "unknown":
            return {"status": "unknown", "units": self.units, "source": self.source}
        return {"status": self.status, "units": self.units, "source": self.source, "value": self.value}


@dataclass(frozen=True)
class JointEntry:
    name: str
    bus: str
    motor_id: int
    actuator_identity: str | None
    actuator_identity_source: str
    encoder: str
    mapping: JointMap
    mapping_source: str
    mapping_status: str
    position_limits: PositionLimits | None = None
    measurements: tuple[Measurement, ...] = ()

    def __post_init__(self):
        _text(self.name, "joint name")
        _text(self.bus, "bus")
        if self.actuator_identity is not None:
            _text(self.actuator_identity, "actuator identity")
        _text(self.actuator_identity_source, "actuator identity source")
        _text(self.mapping_source, "mapping source")
        if self.mapping_status not in ("declared", "measured"):
            raise ValueError("mapping status must be declared or measured")
        if type(self.motor_id) is not int or not 0 <= self.motor_id <= 14:
            raise ValueError("motor_id must be an integer from 0 to 14")
        if self.encoder not in ENCODER_CONVENTIONS:
            raise ValueError("encoder must be a declared encoder convention label")
        if not isinstance(self.mapping, JointMap):
            raise ValueError("mapping must be a JointMap")
        if self.position_limits is not None and not isinstance(self.position_limits, PositionLimits):
            raise ValueError("position_limits must be PositionLimits or None")
        if type(self.measurements) is not tuple or any(
            not isinstance(measurement, Measurement) for measurement in self.measurements
        ):
            raise ValueError("measurements must be a tuple of Measurement")
        names = [measurement.name for measurement in self.measurements]
        if len(names) != len(set(names)):
            raise ValueError("measurement names must be unique")
        object.__setattr__(
            self,
            "measurements",
            tuple(sorted(self.measurements, key=lambda measurement: measurement.name)),
        )

    def measurement(self, name):
        for measurement in self.measurements:
            if measurement.name == name:
                return measurement
        return None

    def to_dict(self):
        record = {
            "name": self.name,
            "bus": self.bus,
            "motor_id": self.motor_id,
            "actuator_identity": self.actuator_identity,
            "actuator_identity_source": self.actuator_identity_source,
            "mapping_source": self.mapping_source,
            "mapping_status": self.mapping_status,
            "encoder": self.encoder,
            "mapping": {
                "reduction": self.mapping.reduction,
                "direction": self.mapping.direction,
                "zero_output_rad": self.mapping.zero_output_rad,
            },
        }
        if self.position_limits is not None:
            record["position_limits"] = {
                "minimum_rad": self.position_limits.minimum_rad,
                "maximum_rad": self.position_limits.maximum_rad,
            }
        if self.measurements:
            record["measurements"] = {
                measurement.name: measurement.to_dict() for measurement in self.measurements
            }
        return record


@dataclass(frozen=True)
class LockedJoint:
    name: str
    mass: Measurement
    position: Measurement

    def __post_init__(self):
        _text(self.name, "locked joint name")
        for field, label, units in ((self.mass, 'body_mass', 'kg'),
                                    (self.position, 'locked_angle', 'rad')):
            if not isinstance(field, Measurement) or field.name != label or field.units != units:
                raise ValueError(f'locked joint {label} must be a {units} Measurement')
        if self.mass.value is not None and self.mass.value < 0:
            raise ValueError('locked body mass must be nonnegative when known')

    def to_dict(self):
        return {'name': self.name, 'mass': self.mass.to_dict(), 'position': self.position.to_dict()}


@dataclass(frozen=True)
class BuildProfile:
    schema_version: int
    robot: str
    build: str
    backend: str
    revision: str
    joints: tuple[JointEntry, ...] = ()
    locked_joints: tuple[LockedJoint, ...] = ()

    def __post_init__(self):
        if type(self.schema_version) is not int or self.schema_version not in SUPPORTED_SCHEMA_VERSIONS:
            raise ValueError("schema_version must be a supported version")
        _text(self.robot, "robot")
        _text(self.build, "build")
        if self.backend not in BACKENDS:
            raise ValueError("backend must be physical, simulation or demo")
        _text(self.revision, "revision")
        if type(self.joints) is not tuple or any(
            not isinstance(joint, JointEntry) for joint in self.joints
        ):
            raise ValueError("joints must be a tuple of JointEntry")
        names = [joint.name for joint in self.joints]
        if len(names) != len(set(names)):
            raise ValueError("joint names must be unique")
        slots = [(joint.bus, joint.motor_id) for joint in self.joints]
        if len(slots) != len(set(slots)):
            raise ValueError("motor ids must be unique per bus")

        if type(self.locked_joints) is not tuple or any(
            not isinstance(joint, LockedJoint) for joint in self.locked_joints
        ):
            raise ValueError('locked_joints must be a tuple of LockedJoint')
        locked_names = [joint.name for joint in self.locked_joints]
        if len(locked_names) != len(set(locked_names)):
            raise ValueError('locked joint names must be unique')
        if set(names) & set(locked_names):
            raise ValueError('a joint cannot be both active and locked')

    def require_active_joint(self, name: str) -> JointEntry:
        _text(name, 'requested joint name')
        for joint in self.joints:
            if joint.name == name:
                return joint
        raise ValueError(f'joint {name} is absent or locked in this build')

    def joint_names(self) -> tuple[str, ...]:
        return tuple(joint.name for joint in self.joints)

    def to_dict(self):
        record = {
            "schema_version": self.schema_version,
            "robot": self.robot,
            "build": self.build,
            "backend": self.backend,
            "revision": self.revision,
            "joints": [joint.to_dict() for joint in self.joints],
        }
        if self.locked_joints:
            record['locked_joints'] = [joint.to_dict() for joint in self.locked_joints]
        return record

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True, allow_nan=False)


def _parse_limits(record):
    raw = _object(record["position_limits"], "position_limits")
    _reject_unknown_fields(raw, LIMIT_FIELDS, "position_limits")
    _require_keys(raw, LIMIT_FIELDS, "position_limits")
    return PositionLimits(minimum_rad=raw["minimum_rad"], maximum_rad=raw["maximum_rad"])


def _parse_measurement(name, value):
    entry = _object(value, f'measurement {name}')
    _reject_unknown_fields(entry, MEASUREMENT_FIELDS, f'measurement {name}')
    _require_keys(entry, ('status', 'units', 'source'), f'measurement {name}')
    return Measurement(name=name, status=entry['status'], units=entry['units'],
                       source=entry['source'], value=entry.get('value'))


def _parse_measurements(record):
    raw = _object(record['measurements'], 'measurements')
    return tuple(_parse_measurement(name, value) for name, value in raw.items())


def _parse_locked_joint(record):
    _object(record, 'locked joint')
    fields = ('name', 'mass', 'position')
    _reject_unknown_fields(record, fields, 'locked joint')
    _require_keys(record, fields, 'locked joint')
    return LockedJoint(record['name'], _parse_measurement('body_mass', record['mass']),
                       _parse_measurement('locked_angle', record['position']))


def _parse_joint(record):
    _reject_unknown_fields(record, JOINT_FIELDS, "joint")
    _require_keys(record, REQUIRED_JOINT_FIELDS, "joint")
    raw_mapping = _object(record["mapping"], "mapping")
    _reject_unknown_fields(raw_mapping, MAPPING_FIELDS, "mapping")
    _require_keys(raw_mapping, MAPPING_FIELDS, "mapping")
    mapping = JointMap(
        reduction=raw_mapping["reduction"],
        direction=raw_mapping["direction"],
        zero_output_rad=raw_mapping["zero_output_rad"],
    )
    parsed = JointEntry(
        name=record["name"],
        bus=record["bus"],
        motor_id=record["motor_id"],
        actuator_identity=record["actuator_identity"],
        actuator_identity_source=record["actuator_identity_source"],
        mapping_source=record["mapping_source"],
        mapping_status=record["mapping_status"],
        encoder=record["encoder"],
        mapping=mapping,
        position_limits=_parse_limits(record) if "position_limits" in record else None,
        measurements=_parse_measurements(record) if "measurements" in record else (),
    )
    return parsed


def from_dict(raw):
    _object(raw, "build profile")
    _reject_unknown_fields(raw, TOP_LEVEL_FIELDS, "build profile")
    _require_keys(raw, REQUIRED_TOP_LEVEL_FIELDS, "build profile")
    raw_joints = raw["joints"]
    if not isinstance(raw_joints, list):
        raise ValueError("joints must be a JSON array")
    joints = tuple(_parse_joint(_object(joint, f"joints[{index}]")) for index, joint in enumerate(raw_joints))
    raw_locked = raw.get('locked_joints', [])
    if not isinstance(raw_locked, list):
        raise ValueError('locked_joints must be a JSON array')
    return BuildProfile(
        schema_version=raw["schema_version"],
        robot=raw["robot"],
        build=raw["build"],
        backend=raw["backend"],
        revision=raw["revision"],
        joints=joints,
        locked_joints=tuple(_parse_locked_joint(record) for record in raw_locked),
    )


def parse_json(text):
    if not isinstance(text, str):
        raise ValueError("profile JSON must be a string")

    def reject_constant(token):
        raise ValueError(f"profile JSON contains the non-finite token {token}")

    def unique_object(pairs):
        record = {}
        for name, value in pairs:
            if name in record:
                raise ValueError(f"profile JSON repeats field {name}")
            record[name] = value
        return record

    try:
        raw = json.loads(text, parse_constant=reject_constant, object_pairs_hook=unique_object)
    except json.JSONDecodeError as exc:
        raise ValueError(f"profile JSON is malformed: {exc}") from exc
    return from_dict(raw)

```

## docs/calibration-binding.md
```
# Bind a calibration artifact to its build

D05 binds an exact existing artifact to the full canonical build profile. It records a calibration ID, source, profile SHA256 and artifact SHA256 as immutable metadata. It does not calculate calibration, parse or certify artifact contents, access files, or apply a saved reference.

```python
from build_profiles import arm_j1
from joint_mapping import JointMap
from calibration_binding import bind_calibration

plan = arm_j1(revision='synthetic-revision', bus='synthetic-bus', motor_id=0,
    mapping=JointMap(1, 1, 0), mapping_source='synthetic example',
    actuator_identity=None, actuator_identity_source='unverified')
artifact = b'synthetic calibration artifact only'
binding = bind_calibration(plan.profile, artifact,
    calibration_id='synthetic-calibration-1', source='synthetic fixture')
binding.require_match(plan.profile, artifact)
```

A different revision, build, backend, robot, actuator identity, bus/ID, mapping, measurement provenance, or locked-body mass/angle changes the full-profile hash. Changed artifact bytes also reject, including a whitespace-only edit. Profiles and nonempty immutable artifact bytes must both match. IDs and sources must be explicit, nonempty and unpadded. The record has no persistence parser; any future input loader must validate its own JSON/duplicate-key boundary.

Unknown actuator identities and declared mappings remain unverified. Matching this binding does not mean that calibration values are correct, the hardware matches the declaration, or motion is allowed. The artifact is opaque because its calibration engine/schema is a separate concern. A real owner must retain fresh current-session references, identity/health checks, fault state and explicit enable. There is no apply/restore/arm method and no new owner integration. Reconnecting must not trust a saved absolute zero.

D06 will track affected validation after configuration changes. This unit provides exact artifact/build binding only; it does not selectively invalidate validation records or commission hardware.

```

## exact diff
diff --git a/scripts/check_milestone.py b/scripts/check_milestone.py
index 7fdc69e..f17acf1 100644
--- a/scripts/check_milestone.py
+++ b/scripts/check_milestone.py
@@ -32,6 +32,7 @@ CHECKS = {
     'D02': 'test_build_profiles',
     'D03': 'test_provisional_builds',
     'D04': 'test_locked_joints',
+    'D05': 'test_calibration_binding',
 }
 
 
diff --git a/scripts/check_offline_baseline.sh b/scripts/check_offline_baseline.sh
index a362ffc..b0cdcad 100755
--- a/scripts/check_offline_baseline.sh
+++ b/scripts/check_offline_baseline.sh
@@ -78,7 +78,7 @@ run_logged() {
     return 0
 }
 
-files=(motor_dashboard.py motor_session.py spin_5rpm.py im6014_codec.py im6014_serial.py im6014_transport.py fake_serial.py test_spin_5rpm.py test_dashboard.py test_startup.py test_startup_verifier.py test_protocol.py test_serial_deadline.py evidence/boundary/vendor-crc-vectors.json USER_MANUAL.md joint_mapping.py test_joint_mapping.py test_numerical_inputs.py im6014_port.py test_port_config.py test_diagnostic_capture.py test_bus_owner.py joint_controller.py joint_records.py test_controller.py test_controller_targets.py test_controller_ownership.py test_controller_limits.py test_controller_velocity.py joint_reference.py test_controller_acceleration.py test_controller_lead.py test_controller_effort.py test_controller_faults.py station_timing.py test_station_timing.py arm_manual.py arm_dashboard.py test_station_independence.py build_profile.py test_build_profile.py test_build_profile_boundary.py build_profiles.py test_build_profiles.py test_provisional_builds.py test_locked_joints.py)
+files=(motor_dashboard.py motor_session.py spin_5rpm.py im6014_codec.py im6014_serial.py im6014_transport.py fake_serial.py test_spin_5rpm.py test_dashboard.py test_startup.py test_startup_verifier.py test_protocol.py test_serial_deadline.py evidence/boundary/vendor-crc-vectors.json USER_MANUAL.md joint_mapping.py test_joint_mapping.py test_numerical_inputs.py im6014_port.py test_port_config.py test_diagnostic_capture.py test_bus_owner.py joint_controller.py joint_records.py test_controller.py test_controller_targets.py test_controller_ownership.py test_controller_limits.py test_controller_velocity.py joint_reference.py test_controller_acceleration.py test_controller_lead.py test_controller_effort.py test_controller_faults.py station_timing.py test_station_timing.py arm_manual.py arm_dashboard.py test_station_independence.py build_profile.py test_build_profile.py test_build_profile_boundary.py build_profiles.py test_build_profiles.py test_provisional_builds.py test_locked_joints.py calibration_binding.py test_calibration_binding.py)
 for name in "${files[@]}"; do
     mkdir -p -- "$scratch/$(dirname -- "$name")" || exit 2
     if ! cp -- "$repo_dir/$name" "$scratch/$name" || ! cmp -s -- "$repo_dir/$name" "$scratch/$name"; then
@@ -98,7 +98,7 @@ cd -- "$scratch" || exit 2
 run_logged packet-tests env PYTHONDONTWRITEBYTECODE=1 "$python_path" test_spin_5rpm.py
 run_logged protocol-tests env PYTHONDONTWRITEBYTECODE=1 "$python_path" -m unittest -v test_protocol
 run_logged serial-deadline-tests env PYTHONDONTWRITEBYTECODE=1 "$python_path" -m unittest -v test_serial_deadline
-run_logged milestone-tests env PYTHONDONTWRITEBYTECODE=1 "$python_path" -m unittest -v test_joint_mapping test_numerical_inputs test_port_config test_diagnostic_capture test_bus_owner test_controller test_controller_targets test_controller_ownership test_controller_limits test_controller_velocity test_controller_acceleration test_controller_lead test_controller_effort test_controller_faults test_station_timing test_station_independence test_build_profile test_build_profile_boundary test_build_profiles test_provisional_builds test_locked_joints
+run_logged milestone-tests env PYTHONDONTWRITEBYTECODE=1 "$python_path" -m unittest -v test_joint_mapping test_numerical_inputs test_port_config test_diagnostic_capture test_bus_owner test_controller test_controller_targets test_controller_ownership test_controller_limits test_controller_velocity test_controller_acceleration test_controller_lead test_controller_effort test_controller_faults test_station_timing test_station_independence test_build_profile test_build_profile_boundary test_build_profiles test_provisional_builds test_locked_joints test_calibration_binding
 run_logged dashboard-tests env QT_QPA_PLATFORM=offscreen PYTHONDONTWRITEBYTECODE=1 "$python_path" test_dashboard.py
 for name in dashboard_preview.png dashboard_dark_preview.png dashboard_test_results.json; do
     if [[ -f "$scratch/$name" ]]; then
diff --git a/test_startup.py b/test_startup.py
index 68d1ae5..dcf5e79 100644
--- a/test_startup.py
+++ b/test_startup.py
@@ -110,7 +110,7 @@ def _cleanup_after_main(window, state):
 
 def _fingerprint(root):
     names = ("motor_dashboard.py", "motor_session.py", "spin_5rpm.py", "im6014_codec.py", "im6014_serial.py", "im6014_transport.py", "fake_serial.py", "USER_MANUAL.md",
-             "test_spin_5rpm.py", "test_dashboard.py", "test_startup.py", "test_startup_verifier.py", "test_protocol.py", "test_serial_deadline.py", "evidence/boundary/vendor-crc-vectors.json", "joint_mapping.py", "test_joint_mapping.py", "test_numerical_inputs.py", "im6014_port.py", "test_port_config.py", "test_diagnostic_capture.py", "test_bus_owner.py", "joint_controller.py", "joint_records.py", "test_controller.py", "test_controller_targets.py", "test_controller_ownership.py", "test_controller_limits.py", "test_controller_velocity.py", "joint_reference.py", "test_controller_acceleration.py", "test_controller_lead.py", "test_controller_effort.py", "test_controller_faults.py", "station_timing.py", "test_station_timing.py", "arm_manual.py", "arm_dashboard.py", "test_station_independence.py", "build_profile.py", "test_build_profile.py", "test_build_profile_boundary.py", "build_profiles.py", "test_build_profiles.py", "test_provisional_builds.py", "test_locked_joints.py")
+             "test_spin_5rpm.py", "test_dashboard.py", "test_startup.py", "test_startup_verifier.py", "test_protocol.py", "test_serial_deadline.py", "evidence/boundary/vendor-crc-vectors.json", "joint_mapping.py", "test_joint_mapping.py", "test_numerical_inputs.py", "im6014_port.py", "test_port_config.py", "test_diagnostic_capture.py", "test_bus_owner.py", "joint_controller.py", "joint_records.py", "test_controller.py", "test_controller_targets.py", "test_controller_ownership.py", "test_controller_limits.py", "test_controller_velocity.py", "joint_reference.py", "test_controller_acceleration.py", "test_controller_lead.py", "test_controller_effort.py", "test_controller_faults.py", "station_timing.py", "test_station_timing.py", "arm_manual.py", "arm_dashboard.py", "test_station_independence.py", "build_profile.py", "test_build_profile.py", "test_build_profile_boundary.py", "build_profiles.py", "test_build_profiles.py", "test_provisional_builds.py", "test_locked_joints.py", "calibration_binding.py", "test_calibration_binding.py")
     files = {}
     for name in names:
         path = root / name
