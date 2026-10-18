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
