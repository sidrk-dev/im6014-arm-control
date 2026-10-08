"""Pure joint reference validation and canonical packet preparation."""

from im6014_codec import command, command_with_raw_position, decode_command, project_position_raw
from joint_contract import DecisionCode, JointSpec, Target
from joint_mapping import JointTarget, OutputSample

__all__ = ("prepare_reference_joint", "prepare_complete_reference")


def prepare_reference_joint(
    spec: JointSpec, target: JointTarget
) -> tuple[DecisionCode | None, JointTarget | None, bytes | None]:
    """Validate a request and bind its actual decoded command to canonical bytes."""
    if not isinstance(spec, JointSpec) or not isinstance(target, JointTarget):
        return DecisionCode.INVALID_ARGUMENT, None, None
    if target.q_rad < spec.q_min_rad or target.q_rad > spec.q_max_rad:
        return DecisionCode.POSITION_OUT_OF_BOUNDS, None, None

    try:
        output = spec.mapping.to_output(target)
        # Validate the original mapped request through the ordinary codec before
        # projecting its position onto an inward representable grid point.
        command(
            spec.motor_id,
            mode=1,
            rpm=output.rpm,
            kd=output.kd,
            torque=output.tau_nm,
            watchdog=True,
            position=output.q_rad,
            kp=output.kp,
        )
        raw_position = project_position_raw(
            target.q_rad,
            spec.q_min_rad,
            spec.q_max_rad,
            lambda output_q: spec.mapping.from_output(
                OutputSample(output_q, 0.0, 0.0)
            ).q_rad,
            spec.mapping.direction,
        )
    except (ArithmeticError, TypeError, ValueError):
        return DecisionCode.INVALID_ARGUMENT, None, None
    if raw_position is None:
        return DecisionCode.POSITION_OUT_OF_BOUNDS, None, None

    try:
        packet = command_with_raw_position(
            spec.motor_id,
            raw_position,
            mode=1,
            rpm=output.rpm,
            kd=output.kd,
            torque=output.tau_nm,
            watchdog=True,
            kp=output.kp,
        )
        decoded = decode_command(packet)
        actual_output = spec.mapping.from_output(
            OutputSample(
                decoded["position_rad"], decoded["rpm"], decoded["torque_nm"]
            )
        )
        gain_scale = spec.mapping.reduction ** 2
        actual_target = JointTarget(
            q_rad=actual_output.q_rad,
            dq_rad_s=actual_output.dq_rad_s,
            tau_nm=actual_output.tau_nm,
            kp=decoded["kp"] * gain_scale,
            kd=decoded["kd"] * gain_scale,
        )
    except (ArithmeticError, TypeError, ValueError):
        return DecisionCode.INVALID_ARGUMENT, None, None

    if not spec.q_min_rad <= actual_target.q_rad <= spec.q_max_rad:
        return DecisionCode.POSITION_OUT_OF_BOUNDS, None, None
    return None, actual_target, packet


def prepare_complete_reference(
    specs: tuple[JointSpec, ...], target: Target
) -> tuple[DecisionCode | None, tuple[JointTarget, ...] | None, tuple[bytes, ...] | None]:
    """Validate and prepare references and canonical packets for all joints atomically."""
    if not isinstance(target, Target) or len(target.joints) != len(specs):
        return DecisionCode.INVALID_ARGUMENT, None, None

    actual_targets: list[JointTarget] = []
    packets: list[bytes] = []

    for spec, joint_target in zip(specs, target.joints):
        code, actual_target, packet = prepare_reference_joint(spec, joint_target)
        if code is not None or actual_target is None or packet is None:
            return code or DecisionCode.INVALID_ARGUMENT, None, None
        actual_targets.append(actual_target)
        packets.append(packet)

    return None, tuple(actual_targets), tuple(packets)
