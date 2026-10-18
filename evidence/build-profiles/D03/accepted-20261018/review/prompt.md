Review D03 Workbench provisional later build profile metadata. Human approved Gemini/Muse source reviews of this Workbench repository. This is a materially new milestone artifact, not a replay of D02 failed seats. Read-only. No tools needed for supplied material; no Git/tests/provider/hardware/subagents. Root personally implemented. Find concrete blockers/bugs and minimal fixes; distinguish config declarations from physical readiness. Check unverified measurements, units/provenance/roundtrip, exact robot/backend/joint-set identity, simulation vs physical workflows, no movement or saved-zero authority. D04 passive bodies, D05 calibration binding and D06 invalidation deliberately later. Parent42focused D01/D02/D03 tests pass, no hardware. Return review only, not physical/merge approval.

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

TOP_LEVEL_FIELDS = ("schema_version", "robot", "build", "backend", "revision", "joints")
REQUIRED_TOP_LEVEL_FIELDS = TOP_LEVEL_FIELDS
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
class BuildProfile:
    schema_version: int
    robot: str
    build: str
    backend: str
    revision: str
    joints: tuple[JointEntry, ...] = ()

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

    def joint_names(self) -> tuple[str, ...]:
        return tuple(joint.name for joint in self.joints)

    def to_dict(self):
        return {
            "schema_version": self.schema_version,
            "robot": self.robot,
            "build": self.build,
            "backend": self.backend,
            "revision": self.revision,
            "joints": [joint.to_dict() for joint in self.joints],
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True, allow_nan=False)


def _parse_limits(record):
    raw = _object(record["position_limits"], "position_limits")
    _reject_unknown_fields(raw, LIMIT_FIELDS, "position_limits")
    _require_keys(raw, LIMIT_FIELDS, "position_limits")
    return PositionLimits(minimum_rad=raw["minimum_rad"], maximum_rad=raw["maximum_rad"])


def _parse_measurements(record):
    raw = _object(record["measurements"], "measurements")
    parsed = []
    for name, value in raw.items():
        entry = _object(value, f"measurement {name}")
        _reject_unknown_fields(entry, MEASUREMENT_FIELDS, f"measurement {name}")
        _require_keys(entry, ("status", "units", "source"), f"measurement {name}")
        parsed.append(Measurement(name=name, status=entry["status"], units=entry["units"], source=entry["source"], value=entry.get("value")))
    return tuple(parsed)


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
    return BuildProfile(
        schema_version=raw["schema_version"],
        robot=raw["robot"],
        build=raw["build"],
        backend=raw["backend"],
        revision=raw["revision"],
        joints=joints,
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

## test_build_profiles.py
```
from dataclasses import FrozenInstanceError, replace
import unittest

from build_profile import parse_json
from build_profiles import (ARM_J1, BENCH_J1, BuildPlan, MechanicalSetup,
                            Workflow, arm_j1, bench_j1)
from joint_mapping import JointMap


def arguments():
    return dict(revision='synthetic-1', bus='synthetic-bus', motor_id=0,
                mapping=JointMap(6, -1, 0.25), mapping_source='synthetic design',
                actuator_identity=None, actuator_identity_source='not measured')


class D02BuildWorkflows(unittest.TestCase):
    def test_profiles_have_distinct_identity_and_mechanical_requirements(self):
        bench, mounted = bench_j1(**arguments()), arm_j1(**arguments())
        self.assertEqual(bench.profile.build, 'bench_j1')
        self.assertEqual(mounted.profile.build, 'arm_j1')
        self.assertEqual(bench.definition.mechanical_setup, MechanicalSetup.SECURED_BENCH)
        self.assertEqual(mounted.definition.mechanical_setup, MechanicalSetup.MOUNTED_BASE)
        self.assertIn('bench_secured', bench.definition.required_evidence)
        self.assertNotIn('bench_secured', mounted.definition.required_evidence)
        self.assertIn('attached_load_verified', mounted.definition.required_evidence)
        self.assertNotIn('attached_load_verified', bench.definition.required_evidence)

    def test_conditional_workflow_scopes_stay_distinct(self):
        self.assertIn(Workflow.BOUNDED_BENCH_TEST, BENCH_J1.actions_when_commissioned)
        self.assertNotIn(Workflow.ARM_JOINT_CONTROL, BENCH_J1.actions_when_commissioned)
        self.assertIn(Workflow.ARM_JOINT_CONTROL, ARM_J1.actions_when_commissioned)
        self.assertNotIn(Workflow.BOUNDED_BENCH_TEST, ARM_J1.actions_when_commissioned)
        for definition in (BENCH_J1, ARM_J1):
            self.assertIn('current_session_reference', definition.required_evidence)
            self.assertNotIn('arm', {action.value for action in definition.actions_when_commissioned})
            self.assertNotIn('release', {action.value for action in definition.actions_when_commissioned})

    def test_factories_never_invent_mapping_or_calibration(self):
        for factory in (bench_j1, arm_j1):
            with self.subTest(factory=factory.__name__):
                for field in arguments():
                    values = arguments()
                    del values[field]
                    with self.assertRaises(TypeError):
                        factory(**values)
                plan = factory(**arguments())
                joint = plan.profile.joints[0]
                self.assertEqual(joint.mapping, arguments()['mapping'])
                self.assertEqual(joint.mapping_status, 'declared')
                self.assertIsNone(joint.actuator_identity)
                self.assertEqual(joint.motor_id, 0)
                for obj in (plan, plan.definition, plan.profile, joint):
                    self.assertFalse(hasattr(obj, 'permits'))
                    self.assertFalse(hasattr(obj, 'calibrated'))
                    self.assertFalse(hasattr(obj, 'allowed_actions'))

    def test_explicit_configurations_round_trip_without_readiness_upgrade(self):
        for factory in (bench_j1, arm_j1):
            plan = factory(**arguments())
            again = parse_json(plan.profile.to_json())
            self.assertEqual(again, plan.profile)
            self.assertEqual(BuildPlan(plan.definition, again), plan)
            self.assertEqual(again.joints[0].mapping_status, 'declared')
            self.assertNotEqual(plan.definition.required_evidence, frozenset())

    def test_mismatched_or_modified_definitions_are_rejected(self):
        bench, mounted = bench_j1(**arguments()), arm_j1(**arguments())
        with self.assertRaises(ValueError):
            BuildPlan(BENCH_J1, mounted.profile)
        with self.assertRaises(ValueError):
            BuildPlan(ARM_J1, bench.profile)
        with self.assertRaises(ValueError):
            BuildPlan(replace(BENCH_J1, required_evidence=frozenset()), bench.profile)
        with self.assertRaises(ValueError):
            BuildPlan(replace(BENCH_J1, required_evidence=set(BENCH_J1.required_evidence)), bench.profile)
        with self.assertRaises(ValueError):
            BuildPlan(BENCH_J1, replace(bench.profile, joints=()))
        with self.assertRaises(ValueError):
            BuildPlan(BENCH_J1, 'bench_j1')

    def test_values_and_scopes_are_immutable(self):
        plan = arm_j1(**arguments())
        with self.assertRaises(FrozenInstanceError):
            plan.profile = bench_j1(**arguments()).profile
        with self.assertRaises(FrozenInstanceError):
            plan.definition.mechanical_setup = MechanicalSetup.SECURED_BENCH
        with self.assertRaises(AttributeError):
            plan.definition.actions_when_commissioned.add(Workflow.BOUNDED_BENCH_TEST)

    def test_identity_and_mapping_validation_stays_at_d01_boundary(self):
        for factory in (bench_j1, arm_j1):
            for changes in ({'bus':' padded '}, {'motor_id':True}, {'mapping':None},
                            {'revision':''}, {'mapping_source':''}, {'actuator_identity_source':''}):
                with self.subTest(factory=factory.__name__, changes=changes):
                    values = arguments()
                    values.update(changes)
                    with self.assertRaises(ValueError):
                        factory(**values)


if __name__=='__main__':
    unittest.main()

```

## test_provisional_builds.py
```
from dataclasses import replace
import unittest

from build_profile import JointEntry, Measurement, parse_json
from build_profiles import (ARM_6_TOOL, BENCH_J1, BuildPlan, MechanicalSetup,
                            PROVISIONAL_DEFINITIONS, SIM_6_MEASURED, SIM_6_NOMINAL,
                            Workflow, bench_j1, provisional_build)
from joint_mapping import JointMap


def joints(count):
    return tuple(JointEntry(
        name=f'J{index + 1}', bus='synthetic-bus', motor_id=index,
        actuator_identity=None, actuator_identity_source='not physically verified',
        encoder='unmarked', mapping=JointMap(index + 1, -1, 0.25),
        mapping_source='synthetic design fixture', mapping_status='declared',
    ) for index in range(count))


class D03ProvisionalBuilds(unittest.TestCase):
    def test_all_later_stages_have_exact_ordered_joint_dimensions(self):
        self.assertEqual([item.build_id for item in PROVISIONAL_DEFINITIONS], [
            'arm_j1_j2', 'arm_j1_j2_j3', 'arm_j1_j2_j3_j4', 'arm_j1_j2_j3_j4_j5',
            'arm_j1_j2_j3_j4_j5_j6', 'arm_6_tool', 'sim_6_nominal', 'sim_6_measured',
        ])
        for definition in PROVISIONAL_DEFINITIONS:
            with self.subTest(build=definition.build_id):
                supplied = joints(len(definition.joint_names))
                plan = provisional_build(definition.build_id, revision='synthetic-rev', joints=supplied)
                self.assertIs(plan.definition, definition)
                self.assertEqual(plan.profile.joint_names(), definition.joint_names)
                self.assertEqual(plan.profile.backend, definition.backend)
                for original, configured in zip(supplied, plan.profile.joints):
                    self.assertEqual(original.mapping, configured.mapping)
                    self.assertEqual(original.motor_id, configured.motor_id)
                    self.assertIsNone(configured.actuator_identity)
                    self.assertEqual(configured.mapping_status, 'declared')
                    self.assertEqual(original.measurements, ())

    def test_missing_geometry_is_unknown_without_nominal_numeric_defaults(self):
        for definition in PROVISIONAL_DEFINITIONS:
            plan = provisional_build(definition.build_id, revision='synthetic-rev', joints=joints(len(definition.joint_names)))
            for joint in plan.profile.joints:
                for name, units in [('body_mass', 'kg'), ('link_length', 'm')]:
                    measurement = joint.measurement(name)
                    self.assertEqual(measurement.status, 'unknown')
                    self.assertEqual(measurement.units, units)
                    self.assertEqual(measurement.source, 'not measured for this build revision')
                    self.assertIsNone(measurement.value)
                    self.assertNotIn('value', measurement.to_dict())

    def test_roundtrip_does_not_upgrade_unknowns_including_measured_named_simulation(self):
        for definition in PROVISIONAL_DEFINITIONS:
            plan = provisional_build(definition.build_id, revision='synthetic-rev', joints=joints(len(definition.joint_names)))
            restored = parse_json(plan.profile.to_json())
            self.assertEqual(restored, plan.profile)
            self.assertEqual(BuildPlan(definition, restored), plan)
            self.assertTrue(all(value.status == 'unknown' for joint in restored.joints for value in joint.measurements))
            for obj in (plan, definition, restored):
                for name in ('calibrated', 'permits', 'allowed_actions'):
                    self.assertFalse(hasattr(obj, name))

    def test_explicit_declared_and_measured_values_keep_provenance(self):
        original = joints(2)
        values = (Measurement('body_mass', 'measured', 'kg', 'synthetic scale record', 0.5),
                  Measurement('link_length', 'declared', 'm', 'synthetic drawing', 0.35))
        supplied = (replace(original[0], measurements=values), original[1])
        plan = provisional_build('arm_j1_j2', revision='synthetic-rev', joints=supplied)
        self.assertEqual(plan.profile.joints[0].measurements, supplied[0].measurements)
        self.assertEqual(plan.profile.joints[1].measurement('body_mass').status, 'unknown')

    def test_simulation_and_physical_workflow_metadata_stay_distinct(self):
        for definition in PROVISIONAL_DEFINITIONS:
            if definition.backend == 'simulation':
                self.assertEqual(definition.mechanical_setup, MechanicalSetup.SIMULATED)
                self.assertEqual(definition.actions_when_commissioned, frozenset({Workflow.INSPECT, Workflow.SIMULATE}))
                self.assertNotIn(Workflow.ARM_JOINT_CONTROL, definition.actions_when_commissioned)
            else:
                self.assertEqual(definition.mechanical_setup, MechanicalSetup.MOUNTED_BASE)
                self.assertIn('link_geometry_verified', definition.required_evidence)
                self.assertNotIn(Workflow.SIMULATE, definition.actions_when_commissioned)
        self.assertIn('tool_and_camera_measured', ARM_6_TOOL.required_evidence)
        self.assertNotEqual(SIM_6_MEASURED.required_evidence, SIM_6_NOMINAL.required_evidence)

    def test_wrong_backends_names_and_dimensions_are_rejected(self):
        for definition in PROVISIONAL_DEFINITIONS:
            supplied = joints(len(definition.joint_names))
            plan = provisional_build(definition.build_id, revision='synthetic-rev', joints=supplied)
            for changed in (replace(plan.profile, backend='demo'), replace(plan.profile, robot='another-robot'), replace(plan.profile, build='bench_j1'),
                            replace(plan.profile, joints=plan.profile.joints[:-1]),
                            replace(plan.profile, joints=tuple(reversed(plan.profile.joints)))):
                with self.subTest(build=definition.build_id, changed=changed):
                    with self.assertRaises(ValueError):
                        BuildPlan(definition, changed)
            with self.assertRaises(ValueError):
                BuildPlan(replace(definition), plan.profile)

    def test_inputs_require_explicit_mapping_identity_revision_and_unique_bus_ids(self):
        for bad in ('bench_j1', 'unknown', ' arm_j1_j2 ', None):
            with self.assertRaises(ValueError):
                provisional_build(bad, revision='synthetic', joints=joints(2))
        for bad in (None, [], (None,)):
            with self.assertRaises(ValueError):
                provisional_build('arm_j1_j2', revision='synthetic', joints=bad)
        for missing in ('revision', 'joints'):
            arguments = dict(revision='synthetic', joints=joints(2))
            del arguments[missing]
            with self.assertRaises(TypeError):
                provisional_build('arm_j1_j2', **arguments)
        supplied = joints(2)
        with self.assertRaises(ValueError):
            provisional_build('arm_j1_j2', revision='synthetic', joints=(supplied[0], replace(supplied[1], motor_id=0)))
        with self.assertRaises(ValueError):
            provisional_build('arm_j1_j2', revision='', joints=supplied)

    def test_geometry_units_and_negative_known_values_are_rejected(self):
        supplied = joints(2)
        for name, units, value in [('body_mass', 'm', 1), ('link_length', 'kg', 1),
                                   ('body_mass', 'kg', -1), ('link_length', 'm', -1)]:
            with self.subTest(name=name, units=units, value=value):
                configured = replace(supplied[0], measurements=(Measurement(name, 'declared', units, 'synthetic', value),))
                with self.assertRaises(ValueError):
                    provisional_build('arm_j1_j2', revision='synthetic', joints=(configured, supplied[1]))

    def test_existing_j1_plan_also_rejects_simulation_backend(self):
        joint = joints(1)[0]
        plan = bench_j1(revision='synthetic', bus=joint.bus, motor_id=joint.motor_id,
                        mapping=joint.mapping, mapping_source=joint.mapping_source,
                        actuator_identity=None, actuator_identity_source=joint.actuator_identity_source)
        with self.assertRaises(ValueError):
            BuildPlan(BENCH_J1, replace(plan.profile, backend='simulation'))


if __name__ == '__main__':
    unittest.main()

```

## docs/provisional-builds.md
```
# Provisional later build profiles

D03 adds pure metadata for `arm_j1_j2` through `arm_j1_j2_j3_j4_j5_j6`, `arm_6_tool`, `sim_6_nominal` and `sim_6_measured`. Their canonical definitions bind the exact ordered joints and physical or simulation backend. Build names describe intended stages; they do not prove installation, calibration, clearance or readiness.

`provisional_build(build_id, revision=..., joints=...)` requires explicit D01 `JointEntry` objects with bus, ID, mapping and identity provenance. It fills missing `body_mass` (kg) and `link_length` (m) with unknown measurements that have no numeric value. It preserves supplied declared/measured values and their sources. No nominal mass, reduction, direction, zero or link length is silently supplied.

```python
from build_profile import JointEntry
from build_profiles import provisional_build
from joint_mapping import JointMap

# Synthetic configuration example; these numbers are not commissioned facts.
joints = tuple(JointEntry(
    name=f'J{index + 1}', bus='synthetic-bus', motor_id=index,
    actuator_identity=None, actuator_identity_source='not verified',
    encoder='unmarked', mapping=JointMap(1, 1, 0),
    mapping_source='synthetic example only', mapping_status='declared',
) for index in range(2))
plan = provisional_build('arm_j1_j2', revision='example-1', joints=joints)
assert plan.profile.joints[0].measurement('body_mass').status == 'unknown'
```

The `sim_6_measured` name targets a later measured simulation workflow. Unknown measurements stay unknown even in that profile; its definition requires assembled geometry and build-revision matching evidence. Neither simulation definition offers a physical control workflow. Physical definitions require link geometry, load, mounting, clearance and current-session reference evidence; `arm_6_tool` additionally requires measured tool/camera configuration. These are requirements, not verified facts or live permissions.

D04 handles absent joints and installed passive bodies. D05 binds calibration to revisions; D06 handles validation invalidation. This unit integrates none of those behaviors into the current runtime owner. No saved zero or movement authority is restored from JSON.

```

## exact diff
diff --git a/build_profiles.py b/build_profiles.py
index f266220..6e3c781 100644
--- a/build_profiles.py
+++ b/build_profiles.py
@@ -1,13 +1,14 @@
-from dataclasses import dataclass
+from dataclasses import dataclass, replace
 from enum import Enum
 
-from build_profile import BuildProfile, JointEntry
+from build_profile import BuildProfile, JointEntry, Measurement
 from joint_mapping import JointMap
 
 
 class MechanicalSetup(Enum):
     SECURED_BENCH = 'secured_bench'
     MOUNTED_BASE = 'mounted_base'
+    SIMULATED = 'simulated'
 
 
 class Workflow(Enum):
@@ -15,6 +16,7 @@ class Workflow(Enum):
     CALIBRATE = 'calibrate'
     BOUNDED_BENCH_TEST = 'bounded_bench_test'
     ARM_JOINT_CONTROL = 'arm_joint_control'
+    SIMULATE = 'simulate'
 
 
 @dataclass(frozen=True)
@@ -23,6 +25,8 @@ class BuildDefinition:
     mechanical_setup: MechanicalSetup
     required_evidence: frozenset[str]
     actions_when_commissioned: frozenset[Workflow]
+    backend: str
+    joint_names: tuple[str, ...]
 
 
 BENCH_J1 = BuildDefinition(
@@ -31,6 +35,7 @@ BENCH_J1 = BuildDefinition(
     frozenset({'adapter_and_wiring_inspected', 'actuator_identity_verified',
                'bench_secured', 'bench_clearance_verified', 'current_session_reference'}),
     frozenset({Workflow.INSPECT, Workflow.CALIBRATE, Workflow.BOUNDED_BENCH_TEST}),
+    'physical', ('J1',),
 )
 ARM_J1 = BuildDefinition(
     'arm_j1',
@@ -39,8 +44,35 @@ ARM_J1 = BuildDefinition(
                'base_mounting_verified', 'attached_load_verified',
                'arm_clearance_verified', 'current_session_reference'}),
     frozenset({Workflow.INSPECT, Workflow.CALIBRATE, Workflow.ARM_JOINT_CONTROL}),
+    'physical', ('J1',),
 )
-DEFINITIONS = (BENCH_J1, ARM_J1)
+LATER_PHYSICAL_BUILDS = tuple(
+    BuildDefinition(
+        'arm_' + '_'.join(f'j{index}' for index in range(1, count + 1)),
+        MechanicalSetup.MOUNTED_BASE,
+        ARM_J1.required_evidence | frozenset({'joint_mapping_verified', 'link_geometry_verified'}),
+        ARM_J1.actions_when_commissioned,
+        'physical', tuple(f'J{index}' for index in range(1, count + 1)),
+    )
+    for count in range(2, 7)
+)
+ARM_6_TOOL = BuildDefinition(
+    'arm_6_tool', MechanicalSetup.MOUNTED_BASE,
+    LATER_PHYSICAL_BUILDS[-1].required_evidence | frozenset({'tool_and_camera_measured'}),
+    ARM_J1.actions_when_commissioned, 'physical', LATER_PHYSICAL_BUILDS[-1].joint_names,
+)
+SIM_6_NOMINAL = BuildDefinition(
+    'sim_6_nominal', MechanicalSetup.SIMULATED,
+    frozenset({'provisional_geometry_disclosed'}), frozenset({Workflow.INSPECT, Workflow.SIMULATE}),
+    'simulation', LATER_PHYSICAL_BUILDS[-1].joint_names,
+)
+SIM_6_MEASURED = BuildDefinition(
+    'sim_6_measured', MechanicalSetup.SIMULATED,
+    frozenset({'assembled_geometry_measured', 'simulation_matches_build_revision'}),
+    SIM_6_NOMINAL.actions_when_commissioned, 'simulation', SIM_6_NOMINAL.joint_names,
+)
+PROVISIONAL_DEFINITIONS = (*LATER_PHYSICAL_BUILDS, ARM_6_TOOL, SIM_6_NOMINAL, SIM_6_MEASURED)
+DEFINITIONS = (BENCH_J1, ARM_J1, *PROVISIONAL_DEFINITIONS)
 
 
 @dataclass(frozen=True)
@@ -53,10 +85,14 @@ class BuildPlan:
             raise ValueError('profile must be a BuildProfile')
         if not any(self.definition is definition for definition in DEFINITIONS):
             raise ValueError('definition must be a canonical supported build definition')
+        if self.profile.robot != 'im6014-arm':
+            raise ValueError('profile robot must match im6014-arm build definitions')
         if self.profile.build != self.definition.build_id:
             raise ValueError('profile build identity does not match its workflow definition')
-        if self.profile.joint_names() != ('J1',):
-            raise ValueError('this build requires exactly the configured J1 joint')
+        if self.profile.backend != self.definition.backend:
+            raise ValueError('profile backend does not match its workflow definition')
+        if self.profile.joint_names() != self.definition.joint_names:
+            raise ValueError('profile joints do not match the exact ordered build joint names')
 
 
 def _j1_plan(definition, *, revision: str, bus: str, motor_id: int,
@@ -91,3 +127,27 @@ def arm_j1(*, revision: str, bus: str, motor_id: int, mapping: JointMap,
                     mapping=mapping, mapping_source=mapping_source,
                     actuator_identity=actuator_identity,
                     actuator_identity_source=actuator_identity_source)
+
+
+def provisional_build(build_id: str, *, revision: str, joints: tuple[JointEntry, ...]) -> BuildPlan:
+    definition = next((item for item in PROVISIONAL_DEFINITIONS if item.build_id == build_id), None)
+    if definition is None:
+        raise ValueError('build_id must name a supported provisional build')
+    if type(joints) is not tuple or any(not isinstance(joint, JointEntry) for joint in joints):
+        raise ValueError('joints must be a tuple of explicit JointEntry values')
+    configured = []
+    for joint in joints:
+        for name, units in (('body_mass', 'kg'), ('link_length', 'm')):
+            value = joint.measurement(name)
+            if value is not None and (value.units != units or (value.value is not None and value.value < 0)):
+                raise ValueError(f'{name} requires {units} units and a nonnegative value when known')
+        measurements = joint.measurements + tuple(
+            Measurement(name, 'unknown', units, 'not measured for this build revision')
+            for name, units in (('body_mass', 'kg'), ('link_length', 'm'))
+            if joint.measurement(name) is None
+        )
+        configured.append(replace(joint, measurements=measurements))
+    return BuildPlan(definition, BuildProfile(
+        schema_version=1, robot='im6014-arm', build=build_id,
+        backend=definition.backend, revision=revision, joints=tuple(configured),
+    ))
diff --git a/scripts/check_milestone.py b/scripts/check_milestone.py
index 539aa11..09d2fce 100644
--- a/scripts/check_milestone.py
+++ b/scripts/check_milestone.py
@@ -30,6 +30,7 @@ CHECKS = {
     'C16': 'test_station_independence',
     'D01': 'test_build_profile test_build_profile_boundary',
     'D02': 'test_build_profiles',
+    'D03': 'test_provisional_builds',
 }
 
 
diff --git a/scripts/check_offline_baseline.sh b/scripts/check_offline_baseline.sh
index f497ac0..11f9d38 100755
--- a/scripts/check_offline_baseline.sh
+++ b/scripts/check_offline_baseline.sh
@@ -78,7 +78,7 @@ run_logged() {
     return 0
 }
 
-files=(motor_dashboard.py motor_session.py spin_5rpm.py im6014_codec.py im6014_serial.py im6014_transport.py fake_serial.py test_spin_5rpm.py test_dashboard.py test_startup.py test_startup_verifier.py test_protocol.py test_serial_deadline.py evidence/boundary/vendor-crc-vectors.json USER_MANUAL.md joint_mapping.py test_joint_mapping.py test_numerical_inputs.py im6014_port.py test_port_config.py test_diagnostic_capture.py test_bus_owner.py joint_controller.py joint_records.py test_controller.py test_controller_targets.py test_controller_ownership.py test_controller_limits.py test_controller_velocity.py joint_reference.py test_controller_acceleration.py test_controller_lead.py test_controller_effort.py test_controller_faults.py station_timing.py test_station_timing.py arm_manual.py arm_dashboard.py test_station_independence.py build_profile.py test_build_profile.py test_build_profile_boundary.py build_profiles.py test_build_profiles.py)
+files=(motor_dashboard.py motor_session.py spin_5rpm.py im6014_codec.py im6014_serial.py im6014_transport.py fake_serial.py test_spin_5rpm.py test_dashboard.py test_startup.py test_startup_verifier.py test_protocol.py test_serial_deadline.py evidence/boundary/vendor-crc-vectors.json USER_MANUAL.md joint_mapping.py test_joint_mapping.py test_numerical_inputs.py im6014_port.py test_port_config.py test_diagnostic_capture.py test_bus_owner.py joint_controller.py joint_records.py test_controller.py test_controller_targets.py test_controller_ownership.py test_controller_limits.py test_controller_velocity.py joint_reference.py test_controller_acceleration.py test_controller_lead.py test_controller_effort.py test_controller_faults.py station_timing.py test_station_timing.py arm_manual.py arm_dashboard.py test_station_independence.py build_profile.py test_build_profile.py test_build_profile_boundary.py build_profiles.py test_build_profiles.py test_provisional_builds.py)
 for name in "${files[@]}"; do
     mkdir -p -- "$scratch/$(dirname -- "$name")" || exit 2
     if ! cp -- "$repo_dir/$name" "$scratch/$name" || ! cmp -s -- "$repo_dir/$name" "$scratch/$name"; then
@@ -98,7 +98,7 @@ cd -- "$scratch" || exit 2
 run_logged packet-tests env PYTHONDONTWRITEBYTECODE=1 "$python_path" test_spin_5rpm.py
 run_logged protocol-tests env PYTHONDONTWRITEBYTECODE=1 "$python_path" -m unittest -v test_protocol
 run_logged serial-deadline-tests env PYTHONDONTWRITEBYTECODE=1 "$python_path" -m unittest -v test_serial_deadline
-run_logged milestone-tests env PYTHONDONTWRITEBYTECODE=1 "$python_path" -m unittest -v test_joint_mapping test_numerical_inputs test_port_config test_diagnostic_capture test_bus_owner test_controller test_controller_targets test_controller_ownership test_controller_limits test_controller_velocity test_controller_acceleration test_controller_lead test_controller_effort test_controller_faults test_station_timing test_station_independence test_build_profile test_build_profile_boundary test_build_profiles
+run_logged milestone-tests env PYTHONDONTWRITEBYTECODE=1 "$python_path" -m unittest -v test_joint_mapping test_numerical_inputs test_port_config test_diagnostic_capture test_bus_owner test_controller test_controller_targets test_controller_ownership test_controller_limits test_controller_velocity test_controller_acceleration test_controller_lead test_controller_effort test_controller_faults test_station_timing test_station_independence test_build_profile test_build_profile_boundary test_build_profiles test_provisional_builds
 run_logged dashboard-tests env QT_QPA_PLATFORM=offscreen PYTHONDONTWRITEBYTECODE=1 "$python_path" test_dashboard.py
 for name in dashboard_preview.png dashboard_dark_preview.png dashboard_test_results.json; do
     if [[ -f "$scratch/$name" ]]; then
diff --git a/test_startup.py b/test_startup.py
index e3b3d96..212c7f1 100644
--- a/test_startup.py
+++ b/test_startup.py
@@ -110,7 +110,7 @@ def _cleanup_after_main(window, state):
 
 def _fingerprint(root):
     names = ("motor_dashboard.py", "motor_session.py", "spin_5rpm.py", "im6014_codec.py", "im6014_serial.py", "im6014_transport.py", "fake_serial.py", "USER_MANUAL.md",
-             "test_spin_5rpm.py", "test_dashboard.py", "test_startup.py", "test_startup_verifier.py", "test_protocol.py", "test_serial_deadline.py", "evidence/boundary/vendor-crc-vectors.json", "joint_mapping.py", "test_joint_mapping.py", "test_numerical_inputs.py", "im6014_port.py", "test_port_config.py", "test_diagnostic_capture.py", "test_bus_owner.py", "joint_controller.py", "joint_records.py", "test_controller.py", "test_controller_targets.py", "test_controller_ownership.py", "test_controller_limits.py", "test_controller_velocity.py", "joint_reference.py", "test_controller_acceleration.py", "test_controller_lead.py", "test_controller_effort.py", "test_controller_faults.py", "station_timing.py", "test_station_timing.py", "arm_manual.py", "arm_dashboard.py", "test_station_independence.py", "build_profile.py", "test_build_profile.py", "test_build_profile_boundary.py", "build_profiles.py", "test_build_profiles.py")
+             "test_spin_5rpm.py", "test_dashboard.py", "test_startup.py", "test_startup_verifier.py", "test_protocol.py", "test_serial_deadline.py", "evidence/boundary/vendor-crc-vectors.json", "joint_mapping.py", "test_joint_mapping.py", "test_numerical_inputs.py", "im6014_port.py", "test_port_config.py", "test_diagnostic_capture.py", "test_bus_owner.py", "joint_controller.py", "joint_records.py", "test_controller.py", "test_controller_targets.py", "test_controller_ownership.py", "test_controller_limits.py", "test_controller_velocity.py", "joint_reference.py", "test_controller_acceleration.py", "test_controller_lead.py", "test_controller_effort.py", "test_controller_faults.py", "station_timing.py", "test_station_timing.py", "arm_manual.py", "arm_dashboard.py", "test_station_independence.py", "build_profile.py", "test_build_profile.py", "test_build_profile_boundary.py", "build_profiles.py", "test_build_profiles.py", "test_provisional_builds.py")
     files = {}
     for name in names:
         path = root / name
