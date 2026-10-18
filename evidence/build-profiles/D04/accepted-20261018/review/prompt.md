Read-only D04 Workbench source review, newly scoped milestone. Human approves Gemini/Muse reviews for this Workbench repo. No HAPI-fork source. Root personally implements; parent owns tests/Git/hardware. Inspect actual supplied material, no tools needed, no test/import execution/provider/hardware/subagents/Git. Seek concrete bugs. Goal: absent omitted, installed locked joints separate from active coordinates while retaining aggregate mass and fixed-angle Measurement records/provenance; no invented values. Unknown remains unknown. Locked has no motor/bus/mapping. Named active-config selector not readiness/authority. Core positional dimension rejection shown with existing pure fake station, not production owner integration. D05/D06 later. Version1 optional field backward compatibility; strict duplicate JSON key/field/measurement guards. Actual8focused and50combinedbuild tests pass; full audited disconnected baseline running. No physical results claimed.

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

## build_profiles.py
```
from dataclasses import dataclass, replace
from enum import Enum

from build_profile import BuildProfile, JointEntry, Measurement
from joint_mapping import JointMap


class MechanicalSetup(Enum):
    SECURED_BENCH = 'secured_bench'
    MOUNTED_BASE = 'mounted_base'
    SIMULATED = 'simulated'


class Workflow(Enum):
    INSPECT = 'inspect'
    CALIBRATE = 'calibrate'
    BOUNDED_BENCH_TEST = 'bounded_bench_test'
    ARM_JOINT_CONTROL = 'arm_joint_control'
    SIMULATE = 'simulate'


@dataclass(frozen=True)
class BuildDefinition:
    build_id: str
    mechanical_setup: MechanicalSetup
    required_evidence: frozenset[str]
    actions_when_commissioned: frozenset[Workflow]
    backend: str
    joint_names: tuple[str, ...]


BENCH_J1 = BuildDefinition(
    'bench_j1',
    MechanicalSetup.SECURED_BENCH,
    frozenset({'adapter_and_wiring_inspected', 'actuator_identity_verified',
               'bench_secured', 'bench_clearance_verified', 'current_session_reference'}),
    frozenset({Workflow.INSPECT, Workflow.CALIBRATE, Workflow.BOUNDED_BENCH_TEST}),
    'physical', ('J1',),
)
ARM_J1 = BuildDefinition(
    'arm_j1',
    MechanicalSetup.MOUNTED_BASE,
    frozenset({'adapter_and_wiring_inspected', 'actuator_identity_verified',
               'base_mounting_verified', 'attached_load_verified',
               'arm_clearance_verified', 'current_session_reference'}),
    frozenset({Workflow.INSPECT, Workflow.CALIBRATE, Workflow.ARM_JOINT_CONTROL}),
    'physical', ('J1',),
)
LATER_PHYSICAL_BUILDS = tuple(
    BuildDefinition(
        'arm_' + '_'.join(f'j{index}' for index in range(1, count + 1)),
        MechanicalSetup.MOUNTED_BASE,
        ARM_J1.required_evidence | frozenset({'joint_mapping_verified', 'link_geometry_verified'}),
        ARM_J1.actions_when_commissioned,
        'physical', tuple(f'J{index}' for index in range(1, count + 1)),
    )
    for count in range(2, 7)
)
ARM_6_TOOL = BuildDefinition(
    'arm_6_tool', MechanicalSetup.MOUNTED_BASE,
    LATER_PHYSICAL_BUILDS[-1].required_evidence | frozenset({'tool_and_camera_measured'}),
    ARM_J1.actions_when_commissioned, 'physical', LATER_PHYSICAL_BUILDS[-1].joint_names,
)
SIM_6_NOMINAL = BuildDefinition(
    'sim_6_nominal', MechanicalSetup.SIMULATED,
    frozenset({'provisional_geometry_disclosed'}), frozenset({Workflow.INSPECT, Workflow.SIMULATE}),
    'simulation', LATER_PHYSICAL_BUILDS[-1].joint_names,
)
SIM_6_MEASURED = BuildDefinition(
    'sim_6_measured', MechanicalSetup.SIMULATED,
    frozenset({'assembled_geometry_measured', 'simulation_matches_build_revision'}),
    SIM_6_NOMINAL.actions_when_commissioned, 'simulation', SIM_6_NOMINAL.joint_names,
)
PROVISIONAL_DEFINITIONS = (*LATER_PHYSICAL_BUILDS, ARM_6_TOOL, SIM_6_NOMINAL, SIM_6_MEASURED)
DEFINITIONS = (BENCH_J1, ARM_J1, *PROVISIONAL_DEFINITIONS)


@dataclass(frozen=True)
class BuildPlan:
    definition: BuildDefinition
    profile: BuildProfile

    def __post_init__(self):
        if not isinstance(self.profile, BuildProfile):
            raise ValueError('profile must be a BuildProfile')
        if not any(self.definition is definition for definition in DEFINITIONS):
            raise ValueError('definition must be a canonical supported build definition')
        if self.profile.robot != 'im6014-arm':
            raise ValueError('profile robot must match im6014-arm build definitions')
        if self.profile.build != self.definition.build_id:
            raise ValueError('profile build identity does not match its workflow definition')
        if self.profile.backend != self.definition.backend:
            raise ValueError('profile backend does not match its workflow definition')
        if self.profile.joint_names() != self.definition.joint_names:
            raise ValueError('profile joints do not match the exact ordered build joint names')


def _j1_plan(definition, *, revision: str, bus: str, motor_id: int,
             mapping: JointMap, mapping_source: str, actuator_identity: str | None,
             actuator_identity_source: str) -> BuildPlan:
    joint = JointEntry(
        name='J1', bus=bus, motor_id=motor_id,
        actuator_identity=actuator_identity,
        actuator_identity_source=actuator_identity_source,
        encoder='unmarked', mapping=mapping,
        mapping_source=mapping_source, mapping_status='declared',
    )
    return BuildPlan(definition, BuildProfile(
        schema_version=1, robot='im6014-arm', build=definition.build_id,
        backend='physical', revision=revision, joints=(joint,),
    ))


def bench_j1(*, revision: str, bus: str, motor_id: int, mapping: JointMap,
             mapping_source: str, actuator_identity: str | None,
             actuator_identity_source: str) -> BuildPlan:
    return _j1_plan(BENCH_J1, revision=revision, bus=bus, motor_id=motor_id,
                    mapping=mapping, mapping_source=mapping_source,
                    actuator_identity=actuator_identity,
                    actuator_identity_source=actuator_identity_source)


def arm_j1(*, revision: str, bus: str, motor_id: int, mapping: JointMap,
           mapping_source: str, actuator_identity: str | None,
           actuator_identity_source: str) -> BuildPlan:
    return _j1_plan(ARM_J1, revision=revision, bus=bus, motor_id=motor_id,
                    mapping=mapping, mapping_source=mapping_source,
                    actuator_identity=actuator_identity,
                    actuator_identity_source=actuator_identity_source)


def provisional_build(build_id: str, *, revision: str, joints: tuple[JointEntry, ...]) -> BuildPlan:
    definition = next((item for item in PROVISIONAL_DEFINITIONS if item.build_id == build_id), None)
    if definition is None:
        raise ValueError('build_id must name a supported provisional build')
    if type(joints) is not tuple or any(not isinstance(joint, JointEntry) for joint in joints):
        raise ValueError('joints must be a tuple of explicit JointEntry values')
    configured = []
    for joint in joints:
        for name, units in (('body_mass', 'kg'), ('link_length', 'm')):
            value = joint.measurement(name)
            if value is not None and (value.units != units or (value.value is not None and value.value < 0)):
                raise ValueError(f'{name} requires {units} units and a nonnegative value when known')
        measurements = joint.measurements + tuple(
            Measurement(name, 'unknown', units, 'not measured for this build revision')
            for name, units in (('body_mass', 'kg'), ('link_length', 'm'))
            if joint.measurement(name) is None
        )
        configured.append(replace(joint, measurements=measurements))
    return BuildPlan(definition, BuildProfile(
        schema_version=1, robot='im6014-arm', build=build_id,
        backend=definition.backend, revision=revision, joints=tuple(configured),
    ))

```

## test_locked_joints.py
```
from dataclasses import FrozenInstanceError, replace
import json
import unittest

from build_profile import LockedJoint, Measurement, from_dict, parse_json
from build_profiles import BuildPlan, arm_j1
from joint_controller import JointCore
from joint_mapping import JointTarget
from joint_records import DecisionCode, JointSpec, PositionLimits, Target
from test_build_profiles import arguments
from test_controller import DeterministicClock, FakeStation, SYNTHETIC_PROTECTION


def locked(name='J2', *, known=True):
    return LockedJoint(name,
        Measurement('body_mass', 'measured' if known else 'unknown', 'kg',
                    'synthetic aggregate mass fixture', 0.535 if known else None),
        Measurement('locked_angle', 'declared' if known else 'unknown', 'rad',
                    'synthetic lock-angle fixture', 0.5 if known else None))


def profile():
    return arm_j1(**arguments()).profile


class D04LockedJoints(unittest.TestCase):
    def test_locked_mass_and_position_survive_roundtrip_outside_active_coordinates(self):
        active = profile()
        configured = replace(active, locked_joints=(locked(), locked('J3', known=False)))
        restored = parse_json(configured.to_json())
        self.assertEqual(restored, configured)
        self.assertEqual(restored.joint_names(), ('J1',))
        self.assertEqual(restored.locked_joints[0].mass.value, 0.535)
        self.assertEqual(restored.locked_joints[0].position.value, 0.5)
        self.assertEqual(restored.locked_joints[0].mass.source, 'synthetic aggregate mass fixture')
        self.assertEqual(restored.locked_joints[1].mass.status, 'unknown')
        self.assertIsNone(restored.locked_joints[1].mass.value)
        self.assertNotIn('value', restored.to_dict()['locked_joints'][1]['mass'])
        self.assertEqual(configured.joints, active.joints)
        self.assertEqual(BuildPlan(arm_j1(**arguments()).definition, configured).profile, configured)

    def test_named_config_lookup_rejects_locked_and_absent_joints(self):
        configured = replace(profile(), locked_joints=(locked(),))
        self.assertIs(configured.require_active_joint('J1'), configured.joints[0])
        for name in ('J2', 'J3', ' J1 ', '', None):
            with self.subTest(name=name), self.assertRaises(ValueError):
                configured.require_active_joint(name)
        self.assertFalse(hasattr(configured.locked_joints[0], 'motor_id'))
        self.assertFalse(hasattr(configured.locked_joints[0], 'mapping'))

    def test_no_placeholder_mass_or_extra_coordinate_is_created_for_absence(self):
        configured = profile()
        self.assertEqual(configured.locked_joints, ())
        self.assertNotIn('locked_joints', configured.to_dict())
        self.assertEqual(parse_json(configured.to_json()), configured)
        self.assertEqual(configured.joint_names(), ('J1',))

    def test_duplicate_and_active_locked_overlapping_names_are_rejected(self):
        for values in ((locked('J1'),), (locked(), locked()), [locked()], (None,)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                replace(profile(), locked_joints=values)
        with self.assertRaises(ValueError):
            locked(' padded ')
        with self.assertRaises(FrozenInstanceError):
            locked().name = 'J4'

    def test_locked_measurement_names_units_and_values_are_checked(self):
        for mass in (None, Measurement('mass', 'measured', 'kg', 'synthetic', 1),
                     Measurement('body_mass', 'measured', 'g', 'synthetic', 535),
                     Measurement('body_mass', 'measured', 'kg', 'synthetic', -1)):
            with self.subTest(mass=mass), self.assertRaises(ValueError):
                LockedJoint('J2', mass, locked().position)
        for position in (None, Measurement('locked_angle', 'declared', 'degrees', 'synthetic', 45),
                         Measurement('another_angle', 'declared', 'rad', 'synthetic', 1)):
            with self.subTest(position=position), self.assertRaises(ValueError):
                LockedJoint('J2', locked().mass, position)

    def test_json_boundary_rejects_unknown_fields_nulls_missing_and_motor_mappings(self):
        original = replace(profile(), locked_joints=(locked(),)).to_dict()
        for value in (None, {}, [None], [{}]):
            changed = {**original, 'locked_joints':value}
            with self.subTest(value=value), self.assertRaises(ValueError):
                from_dict(changed)
        for field in ('name', 'mass', 'position'):
            record = dict(original['locked_joints'][0]);del record[field]
            with self.subTest(field=field), self.assertRaises(ValueError):
                from_dict({**original, 'locked_joints':[record]})
        for extra in ('motor_id', 'bus', 'mapping'):
            record = {**original['locked_joints'][0], extra:0}
            with self.subTest(extra=extra), self.assertRaises(ValueError):
                from_dict({**original, 'locked_joints':[record]})
        text = json.dumps(original).replace('"name": "J2"', '"name": "J2", "name": "J3"')
        with self.assertRaises(ValueError):
            parse_json(text)

    def test_old_version_one_json_remains_byte_compatible_when_no_locked_joints(self):
        configured = profile()
        self.assertEqual(configured.to_dict().keys(), {'schema_version', 'robot', 'build', 'backend', 'revision', 'joints'})
        self.assertEqual(from_dict(configured.to_dict()).to_json(), configured.to_json())
        self.assertEqual(from_dict({**configured.to_dict(), 'locked_joints':[]}), configured)

    def test_fake_core_has_only_active_dimensions_and_rejects_extra_targets_without_issue(self):
        configured = replace(profile(), locked_joints=(locked(),))
        clock = DeterministicClock(0)
        specs = tuple(JointSpec(joint.name, joint.motor_id, joint.mapping, PositionLimits(-10, 10),
                    100, 100, 100, 100, protection=SYNTHETIC_PROTECTION) for joint in configured.joints)
        core = JointCore(specs, clock=clock, max_reference_step_s=1)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner='synthetic').accepted)
        writes = tuple(station.writes)
        value = JointTarget(q_rad=0.1, dq_rad_s=0, tau_nm=0, kp=0, kd=0)
        target = Target(1, (value, value), created_at_s=0, expires_at_s=1, ownership=core.snapshot().ownership)
        self.assertEqual(station.submit(target).code, DecisionCode.INVALID_ARGUMENT)
        self.assertIsNone(station.consume())
        self.assertEqual(tuple(station.writes), writes)
        self.assertIsNone(core.snapshot().pending_target)
        self.assertEqual(tuple(spec.name for spec in core.joint_specs), ('J1',))
        with self.assertRaises(ValueError):
            JointCore((locked(),), clock=clock, max_reference_step_s=1)


if __name__ == '__main__': unittest.main()

```

## docs/locked-joints.md
```
# Absent and installed locked joints

D04 keeps active coordinates in `BuildProfile.joints` and installed locked joints in a separate immutable `locked_joints` tuple. An absent joint is omitted from both collections. Active and locked names cannot overlap or repeat. A locked joint has no motor ID, bus or mapping, so it cannot be used as an active `JointSpec`.

Every locked joint explicitly records its aggregate attached-body mass (including whichever motor/link bodies are assigned to that joint) and fixed angle as `Measurement` objects. Mass uses kg and cannot be negative when known; angle uses rad. Each record carries declared/measured/unknown status and source. Unknowns have no numeric value. The profile retains locked mass through JSON round-trip; it never zero-fills absent joints, invents passive masses or adds locked coordinates to the active vector. Avoid double-counting bodies when providing aggregate masses.

```python
from dataclasses import replace
from build_profile import LockedJoint, Measurement
from build_profiles import BuildPlan, arm_j1
from joint_mapping import JointMap

# Synthetic example only; no mechanical installation or calibration is asserted.
plan = arm_j1(revision='example-locked-1', bus='synthetic-bus', motor_id=0,
    mapping=JointMap(1, 1, 0), mapping_source='synthetic example',
    actuator_identity=None, actuator_identity_source='not verified')
locked = LockedJoint('J2',
    Measurement('body_mass', 'unknown', 'kg', 'not weighed for this revision'),
    Measurement('locked_angle', 'unknown', 'rad', 'not measured for this revision'))
configured = replace(plan.profile, locked_joints=(locked,))
plan = BuildPlan(plan.definition, configured)
assert plan.profile.joint_names() == ('J1',)
assert plan.profile.locked_joints[0].mass.value is None
```

`require_active_joint(name)` selects configuration only. It rejects locked and absent names; it grants no readiness or motion permission. The pure Core uses positional targets, not named requests: callers must use the exact active order and must not silently truncate extra coordinates. The fake test constructs its Core only from active entries and proves that an extra coordinate is rejected with no issued reference or write. A locked record itself is rejected as a Core spec.

This is a software configuration boundary. It neither detects a physical lock nor commands one, and does not integrate a new adapter into the current dashboard owner. Full geometry, mass transforms, inertias and gravity compensation remain later dynamics work. D05 calibration binding and D06 validation invalidation are separate. Hardware I/O stays blocked after the adapter incident.

Old schema-version-1 JSON without `locked_joints` still parses; empty locked tuples are omitted when serialized. New locked fields reject unknown keys, missing measurements and any attempted motor/bus/mapping fields.

```

## exact diff
diff --git a/build_profile.py b/build_profile.py
index a81f3e9..5f70c52 100644
--- a/build_profile.py
+++ b/build_profile.py
@@ -10,8 +10,8 @@ ENCODER_CONVENTIONS = ("rotor_incremental", "output_absolute", "unmarked")
 MEASUREMENT_STATUSES = ("measured", "declared", "unknown")
 BACKENDS = ("physical", "simulation", "demo")
 
-TOP_LEVEL_FIELDS = ("schema_version", "robot", "build", "backend", "revision", "joints")
-REQUIRED_TOP_LEVEL_FIELDS = TOP_LEVEL_FIELDS
+TOP_LEVEL_FIELDS = ("schema_version", "robot", "build", "backend", "revision", "joints", "locked_joints")
+REQUIRED_TOP_LEVEL_FIELDS = tuple(name for name in TOP_LEVEL_FIELDS if name != "locked_joints")
 JOINT_FIELDS = ("name", "bus", "motor_id", "actuator_identity", "actuator_identity_source", "encoder", "mapping", "mapping_source", "mapping_status", "position_limits", "measurements")
 REQUIRED_JOINT_FIELDS = tuple(name for name in JOINT_FIELDS if name not in ("position_limits", "measurements"))
 MAPPING_FIELDS = ("reduction", "direction", "zero_output_rad")
@@ -166,6 +166,25 @@ class JointEntry:
         return record
 
 
+@dataclass(frozen=True)
+class LockedJoint:
+    name: str
+    mass: Measurement
+    position: Measurement
+
+    def __post_init__(self):
+        _text(self.name, "locked joint name")
+        for field, label, units in ((self.mass, 'body_mass', 'kg'),
+                                    (self.position, 'locked_angle', 'rad')):
+            if not isinstance(field, Measurement) or field.name != label or field.units != units:
+                raise ValueError(f'locked joint {label} must be a {units} Measurement')
+        if self.mass.value is not None and self.mass.value < 0:
+            raise ValueError('locked body mass must be nonnegative when known')
+
+    def to_dict(self):
+        return {'name': self.name, 'mass': self.mass.to_dict(), 'position': self.position.to_dict()}
+
+
 @dataclass(frozen=True)
 class BuildProfile:
     schema_version: int
@@ -174,6 +193,7 @@ class BuildProfile:
     backend: str
     revision: str
     joints: tuple[JointEntry, ...] = ()
+    locked_joints: tuple[LockedJoint, ...] = ()
 
     def __post_init__(self):
         if type(self.schema_version) is not int or self.schema_version not in SUPPORTED_SCHEMA_VERSIONS:
@@ -194,11 +214,28 @@ class BuildProfile:
         if len(slots) != len(set(slots)):
             raise ValueError("motor ids must be unique per bus")
 
+        if type(self.locked_joints) is not tuple or any(
+            not isinstance(joint, LockedJoint) for joint in self.locked_joints
+        ):
+            raise ValueError('locked_joints must be a tuple of LockedJoint')
+        locked_names = [joint.name for joint in self.locked_joints]
+        if len(locked_names) != len(set(locked_names)):
+            raise ValueError('locked joint names must be unique')
+        if set(names) & set(locked_names):
+            raise ValueError('a joint cannot be both active and locked')
+
+    def require_active_joint(self, name: str) -> JointEntry:
+        _text(name, 'requested joint name')
+        for joint in self.joints:
+            if joint.name == name:
+                return joint
+        raise ValueError(f'joint {name} is absent or locked in this build')
+
     def joint_names(self) -> tuple[str, ...]:
         return tuple(joint.name for joint in self.joints)
 
     def to_dict(self):
-        return {
+        record = {
             "schema_version": self.schema_version,
             "robot": self.robot,
             "build": self.build,
@@ -206,6 +243,9 @@ class BuildProfile:
             "revision": self.revision,
             "joints": [joint.to_dict() for joint in self.joints],
         }
+        if self.locked_joints:
+            record['locked_joints'] = [joint.to_dict() for joint in self.locked_joints]
+        return record
 
     def to_json(self) -> str:
         return json.dumps(self.to_dict(), sort_keys=True, allow_nan=False)
@@ -218,15 +258,26 @@ def _parse_limits(record):
     return PositionLimits(minimum_rad=raw["minimum_rad"], maximum_rad=raw["maximum_rad"])
 
 
+def _parse_measurement(name, value):
+    entry = _object(value, f'measurement {name}')
+    _reject_unknown_fields(entry, MEASUREMENT_FIELDS, f'measurement {name}')
+    _require_keys(entry, ('status', 'units', 'source'), f'measurement {name}')
+    return Measurement(name=name, status=entry['status'], units=entry['units'],
+                       source=entry['source'], value=entry.get('value'))
+
+
 def _parse_measurements(record):
-    raw = _object(record["measurements"], "measurements")
-    parsed = []
-    for name, value in raw.items():
-        entry = _object(value, f"measurement {name}")
-        _reject_unknown_fields(entry, MEASUREMENT_FIELDS, f"measurement {name}")
-        _require_keys(entry, ("status", "units", "source"), f"measurement {name}")
-        parsed.append(Measurement(name=name, status=entry["status"], units=entry["units"], source=entry["source"], value=entry.get("value")))
-    return tuple(parsed)
+    raw = _object(record['measurements'], 'measurements')
+    return tuple(_parse_measurement(name, value) for name, value in raw.items())
+
+
+def _parse_locked_joint(record):
+    _object(record, 'locked joint')
+    fields = ('name', 'mass', 'position')
+    _reject_unknown_fields(record, fields, 'locked joint')
+    _require_keys(record, fields, 'locked joint')
+    return LockedJoint(record['name'], _parse_measurement('body_mass', record['mass']),
+                       _parse_measurement('locked_angle', record['position']))
 
 
 def _parse_joint(record):
@@ -264,6 +315,9 @@ def from_dict(raw):
     if not isinstance(raw_joints, list):
         raise ValueError("joints must be a JSON array")
     joints = tuple(_parse_joint(_object(joint, f"joints[{index}]")) for index, joint in enumerate(raw_joints))
+    raw_locked = raw.get('locked_joints', [])
+    if not isinstance(raw_locked, list):
+        raise ValueError('locked_joints must be a JSON array')
     return BuildProfile(
         schema_version=raw["schema_version"],
         robot=raw["robot"],
@@ -271,6 +325,7 @@ def from_dict(raw):
         backend=raw["backend"],
         revision=raw["revision"],
         joints=joints,
+        locked_joints=tuple(_parse_locked_joint(record) for record in raw_locked),
     )
 
 
diff --git a/scripts/check_milestone.py b/scripts/check_milestone.py
index 09d2fce..7fdc69e 100644
--- a/scripts/check_milestone.py
+++ b/scripts/check_milestone.py
@@ -31,6 +31,7 @@ CHECKS = {
     'D01': 'test_build_profile test_build_profile_boundary',
     'D02': 'test_build_profiles',
     'D03': 'test_provisional_builds',
+    'D04': 'test_locked_joints',
 }
 
 
diff --git a/scripts/check_offline_baseline.sh b/scripts/check_offline_baseline.sh
index 11f9d38..a362ffc 100755
--- a/scripts/check_offline_baseline.sh
+++ b/scripts/check_offline_baseline.sh
@@ -78,7 +78,7 @@ run_logged() {
     return 0
 }
 
-files=(motor_dashboard.py motor_session.py spin_5rpm.py im6014_codec.py im6014_serial.py im6014_transport.py fake_serial.py test_spin_5rpm.py test_dashboard.py test_startup.py test_startup_verifier.py test_protocol.py test_serial_deadline.py evidence/boundary/vendor-crc-vectors.json USER_MANUAL.md joint_mapping.py test_joint_mapping.py test_numerical_inputs.py im6014_port.py test_port_config.py test_diagnostic_capture.py test_bus_owner.py joint_controller.py joint_records.py test_controller.py test_controller_targets.py test_controller_ownership.py test_controller_limits.py test_controller_velocity.py joint_reference.py test_controller_acceleration.py test_controller_lead.py test_controller_effort.py test_controller_faults.py station_timing.py test_station_timing.py arm_manual.py arm_dashboard.py test_station_independence.py build_profile.py test_build_profile.py test_build_profile_boundary.py build_profiles.py test_build_profiles.py test_provisional_builds.py)
+files=(motor_dashboard.py motor_session.py spin_5rpm.py im6014_codec.py im6014_serial.py im6014_transport.py fake_serial.py test_spin_5rpm.py test_dashboard.py test_startup.py test_startup_verifier.py test_protocol.py test_serial_deadline.py evidence/boundary/vendor-crc-vectors.json USER_MANUAL.md joint_mapping.py test_joint_mapping.py test_numerical_inputs.py im6014_port.py test_port_config.py test_diagnostic_capture.py test_bus_owner.py joint_controller.py joint_records.py test_controller.py test_controller_targets.py test_controller_ownership.py test_controller_limits.py test_controller_velocity.py joint_reference.py test_controller_acceleration.py test_controller_lead.py test_controller_effort.py test_controller_faults.py station_timing.py test_station_timing.py arm_manual.py arm_dashboard.py test_station_independence.py build_profile.py test_build_profile.py test_build_profile_boundary.py build_profiles.py test_build_profiles.py test_provisional_builds.py test_locked_joints.py)
 for name in "${files[@]}"; do
     mkdir -p -- "$scratch/$(dirname -- "$name")" || exit 2
     if ! cp -- "$repo_dir/$name" "$scratch/$name" || ! cmp -s -- "$repo_dir/$name" "$scratch/$name"; then
@@ -98,7 +98,7 @@ cd -- "$scratch" || exit 2
 run_logged packet-tests env PYTHONDONTWRITEBYTECODE=1 "$python_path" test_spin_5rpm.py
 run_logged protocol-tests env PYTHONDONTWRITEBYTECODE=1 "$python_path" -m unittest -v test_protocol
 run_logged serial-deadline-tests env PYTHONDONTWRITEBYTECODE=1 "$python_path" -m unittest -v test_serial_deadline
-run_logged milestone-tests env PYTHONDONTWRITEBYTECODE=1 "$python_path" -m unittest -v test_joint_mapping test_numerical_inputs test_port_config test_diagnostic_capture test_bus_owner test_controller test_controller_targets test_controller_ownership test_controller_limits test_controller_velocity test_controller_acceleration test_controller_lead test_controller_effort test_controller_faults test_station_timing test_station_independence test_build_profile test_build_profile_boundary test_build_profiles test_provisional_builds
+run_logged milestone-tests env PYTHONDONTWRITEBYTECODE=1 "$python_path" -m unittest -v test_joint_mapping test_numerical_inputs test_port_config test_diagnostic_capture test_bus_owner test_controller test_controller_targets test_controller_ownership test_controller_limits test_controller_velocity test_controller_acceleration test_controller_lead test_controller_effort test_controller_faults test_station_timing test_station_independence test_build_profile test_build_profile_boundary test_build_profiles test_provisional_builds test_locked_joints
 run_logged dashboard-tests env QT_QPA_PLATFORM=offscreen PYTHONDONTWRITEBYTECODE=1 "$python_path" test_dashboard.py
 for name in dashboard_preview.png dashboard_dark_preview.png dashboard_test_results.json; do
     if [[ -f "$scratch/$name" ]]; then
diff --git a/test_startup.py b/test_startup.py
index 212c7f1..68d1ae5 100644
--- a/test_startup.py
+++ b/test_startup.py
@@ -110,7 +110,7 @@ def _cleanup_after_main(window, state):
 
 def _fingerprint(root):
     names = ("motor_dashboard.py", "motor_session.py", "spin_5rpm.py", "im6014_codec.py", "im6014_serial.py", "im6014_transport.py", "fake_serial.py", "USER_MANUAL.md",
-             "test_spin_5rpm.py", "test_dashboard.py", "test_startup.py", "test_startup_verifier.py", "test_protocol.py", "test_serial_deadline.py", "evidence/boundary/vendor-crc-vectors.json", "joint_mapping.py", "test_joint_mapping.py", "test_numerical_inputs.py", "im6014_port.py", "test_port_config.py", "test_diagnostic_capture.py", "test_bus_owner.py", "joint_controller.py", "joint_records.py", "test_controller.py", "test_controller_targets.py", "test_controller_ownership.py", "test_controller_limits.py", "test_controller_velocity.py", "joint_reference.py", "test_controller_acceleration.py", "test_controller_lead.py", "test_controller_effort.py", "test_controller_faults.py", "station_timing.py", "test_station_timing.py", "arm_manual.py", "arm_dashboard.py", "test_station_independence.py", "build_profile.py", "test_build_profile.py", "test_build_profile_boundary.py", "build_profiles.py", "test_build_profiles.py", "test_provisional_builds.py")
+             "test_spin_5rpm.py", "test_dashboard.py", "test_startup.py", "test_startup_verifier.py", "test_protocol.py", "test_serial_deadline.py", "evidence/boundary/vendor-crc-vectors.json", "joint_mapping.py", "test_joint_mapping.py", "test_numerical_inputs.py", "im6014_port.py", "test_port_config.py", "test_diagnostic_capture.py", "test_bus_owner.py", "joint_controller.py", "joint_records.py", "test_controller.py", "test_controller_targets.py", "test_controller_ownership.py", "test_controller_limits.py", "test_controller_velocity.py", "joint_reference.py", "test_controller_acceleration.py", "test_controller_lead.py", "test_controller_effort.py", "test_controller_faults.py", "station_timing.py", "test_station_timing.py", "arm_manual.py", "arm_dashboard.py", "test_station_independence.py", "build_profile.py", "test_build_profile.py", "test_build_profile_boundary.py", "build_profiles.py", "test_build_profiles.py", "test_provisional_builds.py", "test_locked_joints.py")
     files = {}
     for name in names:
         path = root / name
