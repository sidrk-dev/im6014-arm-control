"""Pure joint reference validation and canonical packet preparation."""

from dataclasses import dataclass
from fractions import Fraction
import math
from im6014_codec import (
    command,
    command_with_raw_position,
    decode_command,
    project_position_raw,
    project_speed_raw,
    raw_to_position,
    raw_to_rpm,
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
    "validate_target_vector",
    "verify_dispatch_claim",
    "ReferenceFailure",
    "ReferenceBatchResult",
)


@dataclass(frozen=True)
class ReferenceFailure:
    code: DecisionCode
    joint_index: int | None

    def __post_init__(self) -> None:
        if not isinstance(self.code, DecisionCode):
            raise ValueError("reference failure code must be a DecisionCode")
        if self.joint_index is not None and (
            type(self.joint_index) is not int or self.joint_index < 0
        ):
            raise ValueError("joint_index must be a nonnegative integer or None")


@dataclass(frozen=True)
class ReferenceBatchResult:
    joint_targets: tuple[JointTarget, ...] | None
    packets: tuple[bytes, ...] | None
    proposed_references: tuple[JointTrajectoryReference, ...] | None
    failures: tuple[ReferenceFailure, ...]

    def __post_init__(self) -> None:
        if type(self.failures) is not tuple or any(
            not isinstance(failure, ReferenceFailure) for failure in self.failures
        ):
            raise ValueError("failures must be a tuple of ReferenceFailure")
        if self.failures:
            if any(
                value is not None
                for value in (self.joint_targets, self.packets, self.proposed_references)
            ):
                raise ValueError("a failed batch cannot expose a partial plan")
            indices = [failure.joint_index for failure in self.failures]
            if len(indices) != len(set(indices)):
                raise ValueError("a batch can report at most one failure per joint")
            return

        if (
            type(self.joint_targets) is not tuple
            or type(self.packets) is not tuple
            or type(self.proposed_references) is not tuple
            or not self.joint_targets
            or len(self.joint_targets) != len(self.packets)
            or len(self.joint_targets) != len(self.proposed_references)
        ):
            raise ValueError("a successful batch requires complete parallel tuples")
        if any(not isinstance(target, JointTarget) for target in self.joint_targets):
            raise ValueError("every batch target must be a JointTarget")
        if any(type(packet) is not bytes for packet in self.packets):
            raise ValueError("every batch packet must be immutable bytes")
        if any(not isinstance(ref, JointTrajectoryReference) for ref in self.proposed_references):
            raise ValueError("every batch reference must be a JointTrajectoryReference")

    @property
    def ready(self) -> bool:
        return not self.failures


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


def validate_target_vector(
    specs: tuple[JointSpec, ...],
    target: Target,
    measured_positions: tuple[float, ...],
) -> tuple[ReferenceFailure, ...]:
    """Validate every admitted joint request before replacing the current goal."""
    if (
        not isinstance(target, Target)
        or len(target.joints) != len(specs)
        or len(measured_positions) != len(specs)
    ):
        return (ReferenceFailure(DecisionCode.INVALID_ARGUMENT, None),)

    failures: list[ReferenceFailure] = []
    for index, (spec, joint_target, q_meas) in enumerate(
        zip(specs, target.joints, measured_positions)
    ):
        code, _, _ = prepare_reference_joint(spec, joint_target)
        if code is not None:
            failures.append(ReferenceFailure(code, index))
            continue
        if isinstance(q_meas, bool) or not isinstance(q_meas, (int, float)):
            failures.append(ReferenceFailure(DecisionCode.INVALID_ARGUMENT, index))
            continue
        try:
            q_meas_float = float(q_meas)
        except (OverflowError, TypeError, ValueError):
            failures.append(ReferenceFailure(DecisionCode.INVALID_ARGUMENT, index))
            continue
        if not math.isfinite(q_meas_float):
            failures.append(ReferenceFailure(DecisionCode.INVALID_ARGUMENT, index))
            continue
        if (
            joint_target.q_rad < q_meas_float - spec.max_lead_rad
            or joint_target.q_rad > q_meas_float + spec.max_lead_rad
        ):
            failures.append(ReferenceFailure(DecisionCode.LEAD_OUT_OF_BOUNDS, index))
    return tuple(failures)


def shape_reference_joint(
    spec: JointSpec,
    target: JointTarget,
    confirmed_ref: JointTrajectoryReference,
    q_meas: float,
    now: float,
    max_reference_gap_s: float,
) -> tuple[
    DecisionCode | None,
    JointTarget | None,
    JointTrajectoryReference | None,
    bytes | None,
]:
    """Shape one discrete reference point and retain its exact emitted command."""
    if (
        not isinstance(spec, JointSpec)
        or not isinstance(target, JointTarget)
        or not isinstance(confirmed_ref, JointTrajectoryReference)
    ):
        return DecisionCode.INVALID_ARGUMENT, None, None, None

    normalized: list[float] = []
    for val in (
        target.q_rad,
        target.dq_rad_s,
        target.tau_nm,
        target.kp,
        target.kd,
        q_meas,
        now,
        max_reference_gap_s,
    ):
        if isinstance(val, bool) or not isinstance(val, (int, float)):
            return DecisionCode.INVALID_ARGUMENT, None, None, None
        try:
            numeric = float(val)
        except (OverflowError, TypeError, ValueError):
            return DecisionCode.INVALID_ARGUMENT, None, None, None
        if not math.isfinite(numeric):
            return DecisionCode.INVALID_ARGUMENT, None, None, None
        normalized.append(numeric)
    (
        goal_q,
        goal_dq,
        goal_tau,
        goal_kp,
        goal_kd,
        q_meas_val,
        now,
        max_reference_gap_s,
    ) = normalized
    if max_reference_gap_s <= 0.0:
        return DecisionCode.INVALID_ARGUMENT, None, None, None

    if goal_q < spec.q_min_rad or goal_q > spec.q_max_rad:
        return DecisionCode.POSITION_OUT_OF_BOUNDS, None, None, None
    if abs(goal_dq) > spec.v_max_rad_s:
        return DecisionCode.SPEED_OUT_OF_BOUNDS, None, None, None

    h = now - confirmed_ref.sample_time_s
    if not math.isfinite(h) or h < 0.0:
        return DecisionCode.CONSTRAINT_VIOLATION, None, None, None
    if h > max_reference_gap_s:
        return DecisionCode.REFERENCE_GAP_EXCEEDED, None, None, None

    if h == 0.0:
        if not confirmed_ref.is_emitted:
            return DecisionCode.CONSTRAINT_VIOLATION, None, None, None
        if target != confirmed_ref.requested_goal:
            return DecisionCode.CONSTRAINT_VIOLATION, None, None, None
        if (
            confirmed_ref.emitted_target is None
            or confirmed_ref.packet is None
        ):
            return DecisionCode.CONSTRAINT_VIOLATION, None, None, None
        if (
            confirmed_ref.q_rad < spec.q_min_rad
            or confirmed_ref.q_rad > spec.q_max_rad
        ):
            return DecisionCode.CONSTRAINT_VIOLATION, None, None, None
        if (
            confirmed_ref.q_rad < q_meas_val - spec.max_lead_rad
            or confirmed_ref.q_rad > q_meas_val + spec.max_lead_rad
        ):
            return DecisionCode.CONSTRAINT_VIOLATION, None, None, None
        if abs(confirmed_ref.encoded_velocity_rad_s) > spec.v_max_rad_s:
            return DecisionCode.CONSTRAINT_VIOLATION, None, None, None
        if abs(confirmed_ref.position_step_velocity_rad_s) > Fraction(spec.v_max_rad_s):
            return DecisionCode.CONSTRAINT_VIOLATION, None, None, None
        return (
            None,
            confirmed_ref.emitted_target,
            confirmed_ref,
            confirmed_ref.packet,
        )

    # Use exact rational arithmetic over the stored finite float-domain values.
    # The sample cadence is the actual float subtraction above, not a decimal
    # interpretation of either timestamp.
    try:
        h_exact = Fraction(h)
        q_prev_exact = Fraction(confirmed_ref.q_rad)
        v_prev_pos_exact = Fraction(confirmed_ref.position_step_velocity_rad_s)
        v_prev_enc_exact = Fraction(confirmed_ref.encoded_velocity_rad_s)
        q_min_exact = Fraction(spec.q_min_rad)
        q_max_exact = Fraction(spec.q_max_rad)
        v_max_exact = Fraction(spec.v_max_rad_s)
        a_max_exact = Fraction(spec.a_max_rad_s2)
        lead_exact = Fraction(spec.max_lead_rad)
        q_meas_exact = Fraction(q_meas_val)

        v_pos_lo = (q_min_exact - q_prev_exact) / h_exact
        v_pos_hi = (q_max_exact - q_prev_exact) / h_exact
        v_acc_lo = v_prev_pos_exact - a_max_exact * h_exact
        v_acc_hi = v_prev_pos_exact + a_max_exact * h_exact
        v_lead_lo = (q_meas_exact - lead_exact - q_prev_exact) / h_exact
        v_lead_hi = (q_meas_exact + lead_exact - q_prev_exact) / h_exact

        v_lo = max(-v_max_exact, v_pos_lo, v_acc_lo, v_lead_lo)
        v_hi = min(v_max_exact, v_pos_hi, v_acc_hi, v_lead_hi)
        if v_lo > v_hi:
            return DecisionCode.CONSTRAINT_VIOLATION, None, None, None

        # q and dq jointly define a one-step objective; q does not replace dq.
        v_desired = (Fraction(goal_q) - q_prev_exact) / h_exact + Fraction(goal_dq)
        v_projected = max(v_lo, min(v_hi, v_desired))
        q_continuous = q_prev_exact + v_projected * h_exact
        q_next_lo = q_prev_exact + v_lo * h_exact
        q_next_hi = q_prev_exact + v_hi * h_exact
        if q_next_lo > q_next_hi or not q_next_lo <= q_continuous <= q_next_hi:
            return DecisionCode.CONSTRAINT_VIOLATION, None, None, None

        q_wanted = float(q_continuous)
        q_lower = float(q_next_lo)
        q_upper = float(q_next_hi)
        if not all(math.isfinite(value) for value in (q_wanted, q_lower, q_upper)):
            return DecisionCode.CONSTRAINT_VIOLATION, None, None, None

        v_enc_lo = max(-v_max_exact, v_prev_enc_exact - a_max_exact * h_exact)
        v_enc_hi = min(v_max_exact, v_prev_enc_exact + a_max_exact * h_exact)
        if v_enc_lo > v_enc_hi:
            return DecisionCode.CONSTRAINT_VIOLATION, None, None, None

        v_enc_desired = Fraction(goal_dq)
        v_enc_projected = max(v_enc_lo, min(v_enc_hi, v_enc_desired))
        v_enc_wanted = float(v_enc_projected)
        v_enc_lower = float(v_enc_lo)
        v_enc_upper = float(v_enc_hi)
        if not all(math.isfinite(value) for value in (v_enc_wanted, v_enc_lower, v_enc_upper)):
            return DecisionCode.CONSTRAINT_VIOLATION, None, None, None
    except (ArithmeticError, TypeError, ValueError):
        return DecisionCode.CONSTRAINT_VIOLATION, None, None, None

    try:
        output_req = spec.mapping.to_output(
            JointTarget(goal_q, goal_dq, goal_tau, goal_kp, goal_kd)
        )
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
            q_wanted,
            q_lower,
            q_upper,
            lambda out_q: spec.mapping.from_output(OutputSample(out_q, 0.0, 0.0)).q_rad,
            spec.mapping.direction,
        )
    except (ArithmeticError, TypeError, ValueError):
        return DecisionCode.CONSTRAINT_VIOLATION, None, None, None

    if raw_pos is None:
        return DecisionCode.CONSTRAINT_VIOLATION, None, None, None

    def mapped_position(raw_position: int) -> float:
        return spec.mapping.from_output(
            OutputSample(raw_to_position(raw_position), 0.0, 0.0)
        ).q_rad

    try:
        q_emitted = mapped_position(raw_pos)
        q_emitted_exact = Fraction(q_emitted)
        position_step_velocity = (q_emitted_exact - q_prev_exact) / h_exact
        safe_position = (
            q_min_exact <= q_emitted_exact <= q_max_exact
            and (q_meas_exact - lead_exact) <= q_emitted_exact <= (q_meas_exact + lead_exact)
            and q_next_lo <= q_emitted_exact <= q_next_hi
        )
        safe_step = (
            abs(position_step_velocity) <= v_max_exact
            and abs(position_step_velocity - v_prev_pos_exact) <= a_max_exact * h_exact
        )
        if not safe_position or not safe_step:
            if q_emitted_exact < q_next_lo or position_step_velocity < v_lo:
                raw_step = spec.mapping.direction
            elif q_emitted_exact > q_next_hi or position_step_velocity > v_hi:
                raw_step = -spec.mapping.direction
            else:
                return DecisionCode.CONSTRAINT_VIOLATION, None, None, None
            raw_candidate = raw_pos + raw_step
            candidate_q = mapped_position(raw_candidate)
            candidate_q_exact = Fraction(candidate_q)
            candidate_velocity = (candidate_q_exact - q_prev_exact) / h_exact
            if not (
                q_min_exact <= candidate_q_exact <= q_max_exact
                and (q_meas_exact - lead_exact) <= candidate_q_exact <= (q_meas_exact + lead_exact)
                and q_next_lo <= candidate_q_exact <= q_next_hi
                and abs(candidate_velocity) <= v_max_exact
                and abs(candidate_velocity - v_prev_pos_exact) <= a_max_exact * h_exact
            ):
                return DecisionCode.CONSTRAINT_VIOLATION, None, None, None
            raw_pos = raw_candidate
            q_emitted = candidate_q
            q_emitted_exact = candidate_q_exact
            position_step_velocity = candidate_velocity
    except (ArithmeticError, TypeError, ValueError):
        return DecisionCode.CONSTRAINT_VIOLATION, None, None, None

    try:
        raw_rpm = project_speed_raw(
            v_enc_wanted,
            v_enc_lower,
            v_enc_upper,
            lambda out_rpm: spec.mapping.from_output(OutputSample(0.0, out_rpm, 0.0)).dq_rad_s,
            spec.mapping.direction,
        )
    except (ArithmeticError, TypeError, ValueError):
        return DecisionCode.CONSTRAINT_VIOLATION, None, None, None

    if raw_rpm is None:
        return DecisionCode.CONSTRAINT_VIOLATION, None, None, None

    def mapped_speed(raw_s: int) -> float:
        return spec.mapping.from_output(
            OutputSample(0.0, raw_to_rpm(raw_s), 0.0)
        ).dq_rad_s

    try:
        v_emitted_enc = mapped_speed(raw_rpm)
        v_emitted_enc_exact = Fraction(v_emitted_enc)
        safe_enc = (
            abs(v_emitted_enc_exact) <= v_max_exact
            and abs(v_emitted_enc_exact - v_prev_enc_exact) <= a_max_exact * h_exact
            and v_enc_lo <= v_emitted_enc_exact <= v_enc_hi
        )
        if not safe_enc:
            if v_emitted_enc_exact < v_enc_lo:
                raw_rpm_step = spec.mapping.direction
            elif v_emitted_enc_exact > v_enc_hi:
                raw_rpm_step = -spec.mapping.direction
            else:
                return DecisionCode.CONSTRAINT_VIOLATION, None, None, None
            raw_rpm_cand = raw_rpm + raw_rpm_step
            cand_v_enc = mapped_speed(raw_rpm_cand)
            cand_v_enc_exact = Fraction(cand_v_enc)
            if not (
                abs(cand_v_enc_exact) <= v_max_exact
                and abs(cand_v_enc_exact - v_prev_enc_exact) <= a_max_exact * h_exact
                and v_enc_lo <= cand_v_enc_exact <= v_enc_hi
            ):
                return DecisionCode.CONSTRAINT_VIOLATION, None, None, None
            raw_rpm = raw_rpm_cand
    except (ArithmeticError, TypeError, ValueError):
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

    actual_q_exact = Fraction(actual_target.q_rad)
    actual_position_step_velocity = (actual_q_exact - q_prev_exact) / h_exact
    actual_v_enc_exact = Fraction(actual_target.dq_rad_s)
    if not q_min_exact <= actual_q_exact <= q_max_exact:
        return DecisionCode.POSITION_OUT_OF_BOUNDS, None, None, None
    if not (q_meas_exact - lead_exact <= actual_q_exact <= q_meas_exact + lead_exact):
        return DecisionCode.CONSTRAINT_VIOLATION, None, None, None
    if not q_next_lo <= actual_q_exact <= q_next_hi:
        return DecisionCode.CONSTRAINT_VIOLATION, None, None, None
    if v_desired != 0 and actual_target.q_rad == confirmed_ref.q_rad:
        return DecisionCode.CONSTRAINT_VIOLATION, None, None, None
    if abs(actual_v_enc_exact) > v_max_exact:
        return DecisionCode.CONSTRAINT_VIOLATION, None, None, None
    if abs(actual_v_enc_exact - v_prev_enc_exact) > a_max_exact * h_exact:
        return DecisionCode.CONSTRAINT_VIOLATION, None, None, None
    if abs(actual_position_step_velocity) > v_max_exact:
        return DecisionCode.CONSTRAINT_VIOLATION, None, None, None
    if abs(actual_position_step_velocity - v_prev_pos_exact) > a_max_exact * h_exact:
        return DecisionCode.CONSTRAINT_VIOLATION, None, None, None

    proposed_ref = JointTrajectoryReference(
        q_rad=actual_target.q_rad,
        position_step_velocity_rad_s=actual_position_step_velocity,
        encoded_velocity_rad_s=actual_target.dq_rad_s,
        sample_time_s=now,
        is_emitted=True,
        requested_goal=JointTarget(goal_q, goal_dq, goal_tau, goal_kp, goal_kd),
        emitted_target=actual_target,
        packet=packet,
    )
    return None, actual_target, proposed_ref, packet


def prepare_complete_reference(
    specs: tuple[JointSpec, ...],
    target: Target,
    confirmed_references: tuple[JointTrajectoryReference, ...],
    measured_positions: tuple[float, ...],
    now: float,
    max_reference_gap_s: float,
) -> ReferenceBatchResult:
    """Shape all joints and report every failed index without exposing a partial plan."""
    if (
        type(specs) is not tuple
        or not specs
        or any(not isinstance(spec, JointSpec) for spec in specs)
        or not isinstance(target, Target)
        or len(target.joints) != len(specs)
        or type(confirmed_references) is not tuple
        or len(confirmed_references) != len(specs)
        or type(measured_positions) is not tuple
        or len(measured_positions) != len(specs)
    ):
        return ReferenceBatchResult(
            None, None, None, (ReferenceFailure(DecisionCode.INVALID_ARGUMENT, None),)
        )

    actual_targets: list[JointTarget] = []
    packets: list[bytes] = []
    proposed_references: list[JointTrajectoryReference] = []
    failures: list[ReferenceFailure] = []

    for index, (spec, joint_target, conf_ref, q_meas) in enumerate(
        zip(specs, target.joints, confirmed_references, measured_positions)
    ):
        code, actual_target, prop_ref, packet = shape_reference_joint(
            spec, joint_target, conf_ref, q_meas, now, max_reference_gap_s
        )
        if code is not None or actual_target is None or prop_ref is None or packet is None:
            failures.append(
                ReferenceFailure(code or DecisionCode.CONSTRAINT_VIOLATION, index)
            )
            continue
        actual_targets.append(actual_target)
        packets.append(packet)
        proposed_references.append(prop_ref)

    if failures:
        return ReferenceBatchResult(None, None, None, tuple(failures))
    return ReferenceBatchResult(
        tuple(actual_targets), tuple(packets), tuple(proposed_references), ()
    )


def verify_dispatch_claim(
    spec: JointSpec,
    prop_ref: JointTrajectoryReference,
    conf_ref: JointTrajectoryReference,
    plan_joint: JointTarget,
    packet: bytes,
    goal: JointTarget,
    q_meas: float,
    prepared_at_s: float,
) -> DecisionCode | None:
    """Validate one proposed joint reference against all closed constraints at the write gate."""
    if (
        not isinstance(spec, JointSpec)
        or not isinstance(prop_ref, JointTrajectoryReference)
        or not isinstance(conf_ref, JointTrajectoryReference)
        or not isinstance(plan_joint, JointTarget)
        or type(packet) is not bytes
        or not isinstance(goal, JointTarget)
    ):
        return DecisionCode.CONSTRAINT_VIOLATION

    if (
        prop_ref.requested_goal != goal
        or prop_ref.emitted_target != plan_joint
        or prop_ref.packet != packet
        or prop_ref.sample_time_s > prepared_at_s
    ):
        return DecisionCode.CONSTRAINT_VIOLATION

    if prop_ref.sample_time_s == conf_ref.sample_time_s:
        if prop_ref != conf_ref:
            return DecisionCode.CONSTRAINT_VIOLATION
    elif (
        prop_ref.sample_time_s != prepared_at_s
        or prop_ref.sample_time_s <= conf_ref.sample_time_s
    ):
        return DecisionCode.CONSTRAINT_VIOLATION

    try:
        h_ref = prop_ref.sample_time_s - conf_ref.sample_time_s
        h_ref_exact = Fraction(h_ref)
        a_max_exact = Fraction(spec.a_max_rad_s2)
        v_max_exact = Fraction(spec.v_max_rad_s)
        v_prev_pos = Fraction(conf_ref.position_step_velocity_rad_s)
        v_prev_enc = Fraction(conf_ref.encoded_velocity_rad_s)
        q_min_exact = Fraction(spec.q_min_rad)
        q_max_exact = Fraction(spec.q_max_rad)
        lead_exact = Fraction(spec.max_lead_rad)
        q_meas_exact = Fraction(q_meas)
        prop_q_exact = Fraction(prop_ref.q_rad)
        plan_q_exact = Fraction(plan_joint.q_rad)

        if h_ref > 0.0:
            actual_pos_vel = (prop_q_exact - Fraction(conf_ref.q_rad)) / h_ref_exact
            if prop_ref.position_step_velocity_rad_s != actual_pos_vel:
                return DecisionCode.CONSTRAINT_VIOLATION

        if (
            not q_min_exact <= prop_q_exact <= q_max_exact
            or not q_min_exact <= plan_q_exact <= q_max_exact
        ):
            return DecisionCode.POSITION_OUT_OF_BOUNDS

        if (
            abs(prop_ref.position_step_velocity_rad_s) > v_max_exact
            or abs(Fraction(prop_ref.encoded_velocity_rad_s)) > v_max_exact
            or abs(Fraction(plan_joint.dq_rad_s)) > v_max_exact
        ):
            return DecisionCode.SPEED_OUT_OF_BOUNDS

        if prop_ref.sample_time_s > conf_ref.sample_time_s:
            if (
                abs(prop_ref.position_step_velocity_rad_s - v_prev_pos) > a_max_exact * h_ref_exact
                or abs(Fraction(prop_ref.encoded_velocity_rad_s) - v_prev_enc) > a_max_exact * h_ref_exact
            ):
                return DecisionCode.CONSTRAINT_VIOLATION

        if (
            prop_q_exact < q_meas_exact - lead_exact
            or prop_q_exact > q_meas_exact + lead_exact
            or plan_q_exact < q_meas_exact - lead_exact
            or plan_q_exact > q_meas_exact + lead_exact
        ):
            return DecisionCode.CONSTRAINT_VIOLATION
    except (ArithmeticError, TypeError, ValueError):
        return DecisionCode.CONSTRAINT_VIOLATION

    return None
