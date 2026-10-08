"""Pure joint reference validation and canonical packet preparation."""

from fractions import Fraction
import math
from im6014_codec import (
    command,
    command_with_raw_position,
    decode_command,
    project_position_raw,
    project_speed_raw,
    raw_to_position,
)
from joint_contract import (
    DecisionCode,
    JointSpec,
    JointTrajectoryReference,
    Target,
)
from joint_mapping import JointTarget, OutputSample

__all__ = (
    "prepare_reference_joint",
    "shape_reference_joint",
    "prepare_complete_reference",
    "shape_complete_reference",
)


def prepare_reference_joint(
    spec: JointSpec, target: JointTarget
) -> tuple[DecisionCode | None, JointTarget | None, bytes | None]:
    """Validate a request and bind its actual decoded command to canonical bytes."""
    if not isinstance(spec, JointSpec) or not isinstance(target, JointTarget):
        return DecisionCode.INVALID_ARGUMENT, None, None
    if target.q_rad < spec.q_min_rad or target.q_rad > spec.q_max_rad:
        return DecisionCode.POSITION_OUT_OF_BOUNDS, None, None
    if abs(target.dq_rad_s) > spec.v_max_rad_s:
        return DecisionCode.SPEED_OUT_OF_BOUNDS, None, None

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
        if raw_position is None:
            return DecisionCode.POSITION_OUT_OF_BOUNDS, None, None

        raw_rpm = project_speed_raw(
            target.dq_rad_s,
            -spec.v_max_rad_s,
            spec.v_max_rad_s,
            lambda output_rpm: spec.mapping.from_output(
                OutputSample(0.0, output_rpm, 0.0)
            ).dq_rad_s,
            spec.mapping.direction,
        )
        if raw_rpm is None:
            return DecisionCode.SPEED_OUT_OF_BOUNDS, None, None
    except (ArithmeticError, TypeError, ValueError):
        return DecisionCode.INVALID_ARGUMENT, None, None

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
            raw_rpm=raw_rpm,
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
    if abs(actual_target.dq_rad_s) > spec.v_max_rad_s:
        return DecisionCode.SPEED_OUT_OF_BOUNDS, None, None
    return None, actual_target, packet


def shape_reference_joint(
    spec: JointSpec,
    target: JointTarget,
    confirmed_ref: JointTrajectoryReference,
    now: float,
    max_reference_gap_s: float,
) -> tuple[
    DecisionCode | None,
    JointTarget | None,
    JointTrajectoryReference | None,
    bytes | None,
]:
    """Pure discrete reference step calculation under velocity bounds and quantization."""
    if (
        not isinstance(spec, JointSpec)
        or not isinstance(target, JointTarget)
        or not isinstance(confirmed_ref, JointTrajectoryReference)
    ):
        return DecisionCode.INVALID_ARGUMENT, None, None, None

    for val in (
        target.q_rad,
        target.dq_rad_s,
        target.tau_nm,
        target.kp,
        target.kd,
        now,
        max_reference_gap_s,
    ):
        if isinstance(val, bool) or not isinstance(val, (int, float)) or not math.isfinite(val):
            return DecisionCode.INVALID_ARGUMENT, None, None, None
    if max_reference_gap_s <= 0.0:
        return DecisionCode.INVALID_ARGUMENT, None, None, None

    if target.q_rad < spec.q_min_rad or target.q_rad > spec.q_max_rad:
        return DecisionCode.POSITION_OUT_OF_BOUNDS, None, None, None
    if abs(target.dq_rad_s) > spec.v_max_rad_s:
        return DecisionCode.SPEED_OUT_OF_BOUNDS, None, None, None

    h = now - confirmed_ref.sample_time_s
    if not math.isfinite(h) or h < 0.0:
        return DecisionCode.CONSTRAINT_VIOLATION, None, None, None
    if h > max_reference_gap_s:
        return DecisionCode.REFERENCE_GAP_EXCEEDED, None, None, None

    if h == 0.0:
        if not confirmed_ref.is_emitted:
            return DecisionCode.CONSTRAINT_VIOLATION, None, None, None
        if (
            target.q_rad != confirmed_ref.q_rad
            or target.dq_rad_s != confirmed_ref.encoded_velocity_rad_s
        ):
            return DecisionCode.CONSTRAINT_VIOLATION, None, None, None
        code, actual_target, packet = prepare_reference_joint(spec, target)
        if code is not None or actual_target is None or packet is None:
            return code or DecisionCode.CONSTRAINT_VIOLATION, None, None, None
        prop_ref = JointTrajectoryReference(
            q_rad=actual_target.q_rad,
            position_step_velocity_rad_s=0.0,
            encoded_velocity_rad_s=actual_target.dq_rad_s,
            sample_time_s=now,
            is_emitted=True,
        )
        return None, actual_target, prop_ref, packet

    # h > 0.0:
    v_max = spec.v_max_rad_s
    q_prev = confirmed_ref.q_rad

    v_pos_lo = (spec.q_min_rad - q_prev) / h
    v_pos_hi = (spec.q_max_rad - q_prev) / h
    v_lo = max(-v_max, v_pos_lo)
    v_hi = min(v_max, v_pos_hi)

    if v_lo > v_hi:
        return DecisionCode.CONSTRAINT_VIOLATION, None, None, None

    # Objective: v_desired = (goal.q - qprev)/h + goal.dq (one-step objective)
    v_desired = (target.q_rad - q_prev) / h + target.dq_rad_s
    v_proj = max(v_lo, min(v_hi, v_desired))
    q_cont_next = q_prev + v_proj * h

    q_next_lo = max(spec.q_min_rad, q_prev - v_max * h)
    q_next_hi = min(spec.q_max_rad, q_prev + v_max * h)
    if q_next_lo > q_next_hi:
        return DecisionCode.CONSTRAINT_VIOLATION, None, None, None

    try:
        output_req = spec.mapping.to_output(target)
        command(
            spec.motor_id,
            mode=1,
            rpm=output_req.rpm,
            kd=output_req.kd,
            torque=output_req.tau_nm,
            watchdog=True,
            position=output_req.q_rad,
            kp=output_req.kp,
        )
        raw_pos = project_position_raw(
            q_cont_next,
            q_next_lo,
            q_next_hi,
            lambda out_q: spec.mapping.from_output(OutputSample(out_q, 0.0, 0.0)).q_rad,
            spec.mapping.direction,
        )
    except (ArithmeticError, TypeError, ValueError):
        return DecisionCode.CONSTRAINT_VIOLATION, None, None, None

    if raw_pos is None:
        return DecisionCode.CONSTRAINT_VIOLATION, None, None, None

    output_q = raw_to_position(raw_pos)
    q_emitted = spec.mapping.from_output(OutputSample(output_q, 0.0, 0.0)).q_rad

    f = Fraction
    u_rat = (f(q_emitted) - f(q_prev)) / f(h)
    if abs(u_rat) > f(v_max):
        # Inward step if floating boundary rounding caused tiny excess
        step = -spec.mapping.direction if (q_emitted > q_prev) else spec.mapping.direction
        raw_candidate = raw_pos + step
        try:
            cand_out_q = raw_to_position(raw_candidate)
            cand_q = spec.mapping.from_output(OutputSample(cand_out_q, 0.0, 0.0)).q_rad
            cand_u = (f(cand_q) - f(q_prev)) / f(h)
            if abs(cand_u) <= f(v_max) and spec.q_min_rad <= cand_q <= spec.q_max_rad:
                raw_pos = raw_candidate
                q_emitted = cand_q
                u_rat = cand_u
            else:
                return DecisionCode.CONSTRAINT_VIOLATION, None, None, None
        except Exception:
            return DecisionCode.CONSTRAINT_VIOLATION, None, None, None

    # Deliberate hold vs quantization stall
    is_deliberate_hold = (target.q_rad == q_prev and target.dq_rad_s == 0.0)
    if not is_deliberate_hold and q_emitted == q_prev:
        return DecisionCode.CONSTRAINT_VIOLATION, None, None, None

    try:
        raw_rpm = project_speed_raw(
            target.dq_rad_s,
            -spec.v_max_rad_s,
            spec.v_max_rad_s,
            lambda out_rpm: spec.mapping.from_output(OutputSample(0.0, out_rpm, 0.0)).dq_rad_s,
            spec.mapping.direction,
        )
    except (ArithmeticError, TypeError, ValueError):
        return DecisionCode.CONSTRAINT_VIOLATION, None, None, None

    if raw_rpm is None:
        return DecisionCode.CONSTRAINT_VIOLATION, None, None, None

    try:
        packet = command_with_raw_position(
            spec.motor_id,
            raw_pos,
            mode=1,
            kd=output_req.kd,
            torque=output_req.tau_nm,
            watchdog=True,
            kp=output_req.kp,
            raw_rpm=raw_rpm,
        )
        decoded = decode_command(packet)
        actual_output = spec.mapping.from_output(
            OutputSample(decoded["position_rad"], decoded["rpm"], decoded["torque_nm"])
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
        return DecisionCode.CONSTRAINT_VIOLATION, None, None, None

    if abs(actual_target.dq_rad_s) > spec.v_max_rad_s:
        return DecisionCode.CONSTRAINT_VIOLATION, None, None, None
    if abs(float(u_rat)) > spec.v_max_rad_s:
        return DecisionCode.CONSTRAINT_VIOLATION, None, None, None
    if not (spec.q_min_rad <= actual_target.q_rad <= spec.q_max_rad):
        return DecisionCode.POSITION_OUT_OF_BOUNDS, None, None, None

    proposed_ref = JointTrajectoryReference(
        q_rad=actual_target.q_rad,
        position_step_velocity_rad_s=float(u_rat),
        encoded_velocity_rad_s=actual_target.dq_rad_s,
        sample_time_s=now,
        is_emitted=True,
    )
    return None, actual_target, proposed_ref, packet


def prepare_complete_reference(
    specs: tuple[JointSpec, ...],
    target: Target,
    confirmed_references: tuple[JointTrajectoryReference | None, ...] | None = None,
    now: float | None = None,
    max_reference_gap_s: float | None = None,
) -> tuple[
    DecisionCode | None,
    tuple[JointTarget, ...] | None,
    tuple[bytes, ...] | None,
    tuple[JointTrajectoryReference, ...] | None,
]:
    """Validate and prepare references and canonical packets for all joints atomically."""
    if not isinstance(target, Target) or len(target.joints) != len(specs):
        return DecisionCode.INVALID_ARGUMENT, None, None, None

    actual_targets: list[JointTarget] = []
    packets: list[bytes] = []
    proposed_references: list[JointTrajectoryReference] = []

    if confirmed_references is None:
        for spec, joint_target in zip(specs, target.joints):
            code, actual_target, packet = prepare_reference_joint(spec, joint_target)
            if code is not None or actual_target is None or packet is None:
                return code or DecisionCode.INVALID_ARGUMENT, None, None, None
            actual_targets.append(actual_target)
            packets.append(packet)
        return None, tuple(actual_targets), tuple(packets), None

    if len(confirmed_references) != len(specs) or now is None or max_reference_gap_s is None:
        return DecisionCode.INVALID_ARGUMENT, None, None, None

    for spec, joint_target, conf_ref in zip(specs, target.joints, confirmed_references):
        if conf_ref is None:
            return DecisionCode.INVALID_ARGUMENT, None, None, None
        code, actual_target, prop_ref, packet = shape_reference_joint(
            spec, joint_target, conf_ref, now, max_reference_gap_s
        )
        if code is not None or actual_target is None or prop_ref is None or packet is None:
            return code or DecisionCode.CONSTRAINT_VIOLATION, None, None, None
        actual_targets.append(actual_target)
        packets.append(packet)
        proposed_references.append(prop_ref)

    return None, tuple(actual_targets), tuple(packets), tuple(proposed_references)


shape_complete_reference = prepare_complete_reference
