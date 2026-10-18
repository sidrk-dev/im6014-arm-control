"""Concrete D02 build profiles as immutable validated values.

The `build` string is the profile identity. `bench_j1` describes one J1
actuator under bench verification; `arm_j1` describes a mounted and calibrated
base. The two differ in mechanical readiness and therefore in allowed actions.
Mapping values here are nominal configuration, not measured calibration.
"""

from build_profile import BuildProfile, JointEntry
from joint_mapping import JointMap

ROBOT = "im6014-arm"
BACKEND = "physical"
SOURCE_REVISION = "e74ab42"

_J1_NOMINAL_MAPPING = JointMap(reduction=1, direction=1, zero_output_rad=0.0)


def _j1(readiness):
    return JointEntry(
        name="J1",
        bus="bus0",
        motor_id=1,
        encoder="rotor_incremental",
        mapping=_J1_NOMINAL_MAPPING,
        readiness=readiness,
    )


def bench_j1():
    return BuildProfile(
        schema_version=1,
        robot=ROBOT,
        build="bench_j1",
        backend=BACKEND,
        revision=SOURCE_REVISION,
        mechanical_readiness="bench",
        joints=(_j1("uncommissioned"),),
    )


def arm_j1():
    return BuildProfile(
        schema_version=1,
        robot=ROBOT,
        build="arm_j1",
        backend=BACKEND,
        revision=SOURCE_REVISION,
        mechanical_readiness="mounted",
        joints=(_j1("calibrated"),),
    )
