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
