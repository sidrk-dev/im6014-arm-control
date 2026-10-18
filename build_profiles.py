from dataclasses import dataclass
from enum import Enum

from build_profile import BuildProfile, JointEntry
from joint_mapping import JointMap


class MechanicalSetup(Enum):
    SECURED_BENCH = 'secured_bench'
    MOUNTED_BASE = 'mounted_base'


class Workflow(Enum):
    INSPECT = 'inspect'
    CALIBRATE = 'calibrate'
    BOUNDED_BENCH_TEST = 'bounded_bench_test'
    ARM_JOINT_CONTROL = 'arm_joint_control'


@dataclass(frozen=True)
class BuildDefinition:
    build_id: str
    mechanical_setup: MechanicalSetup
    required_evidence: frozenset[str]
    actions_when_commissioned: frozenset[Workflow]


BENCH_J1 = BuildDefinition(
    'bench_j1',
    MechanicalSetup.SECURED_BENCH,
    frozenset({'adapter_and_wiring_inspected', 'actuator_identity_verified',
               'bench_secured', 'bench_clearance_verified', 'current_session_reference'}),
    frozenset({Workflow.INSPECT, Workflow.CALIBRATE, Workflow.BOUNDED_BENCH_TEST}),
)
ARM_J1 = BuildDefinition(
    'arm_j1',
    MechanicalSetup.MOUNTED_BASE,
    frozenset({'adapter_and_wiring_inspected', 'actuator_identity_verified',
               'base_mounting_verified', 'attached_load_verified',
               'arm_clearance_verified', 'current_session_reference'}),
    frozenset({Workflow.INSPECT, Workflow.CALIBRATE, Workflow.ARM_JOINT_CONTROL}),
)
DEFINITIONS = (BENCH_J1, ARM_J1)


@dataclass(frozen=True)
class BuildPlan:
    definition: BuildDefinition
    profile: BuildProfile

    def __post_init__(self):
        if not isinstance(self.profile, BuildProfile):
            raise ValueError('profile must be a BuildProfile')
        if not any(self.definition is definition for definition in DEFINITIONS):
            raise ValueError('definition must be a canonical supported build definition')
        if self.profile.build != self.definition.build_id:
            raise ValueError('profile build identity does not match its workflow definition')
        if self.profile.joint_names() != ('J1',):
            raise ValueError('this build requires exactly the configured J1 joint')


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
