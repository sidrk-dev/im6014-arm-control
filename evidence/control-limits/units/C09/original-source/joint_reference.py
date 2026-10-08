"""Pure reference and encoding calculation for joint position limits.

Owns pure feasible-interval calculation, monotonic binary search for inward
representable motor position codes, and canonical command packet binding.
Owns no clock, observations, mutable state, or transport.
"""

from im6014_codec import (
    RAW_POSITION_MAX,
    RAW_POSITION_MIN,
    RPM_MAX,
    RPM_MIN,
    command,
    raw_to_position,
)
from joint_contract import DecisionCode, JointSpec, Target
from joint_mapping import JointTarget, OutputSample

__all__ = (
    "find_inward_position_raw",
    "prepare_reference_joint",
    "prepare_complete_reference",
)


def find_inward_position_raw(spec: JointSpec, wanted_q_rad: float) -> int | None:
    """Find the raw position code in the motor grid that produces an inward decoded joint position.

    Returns the raw integer position code within [RAW_POSITION_MIN, RAW_POSITION_MAX]
    such that spec.q_min_rad <= decoded_q_rad <= spec.q_max_rad, choosing the code
    closest to wanted_q_rad. Returns None if no code produces a joint position within [q_min_rad, q_max_rad].
    """
    direction = spec.mapping.direction
    if direction == 1:
        k_min = RAW_POSITION_MIN
        k_max = RAW_POSITION_MAX
        raw_from_k = lambda k: k
    else:
        k_min = -RAW_POSITION_MAX
        k_max = -RAW_POSITION_MIN
        raw_from_k = lambda k: -k

    def joint_q(k: int) -> float:
        raw_pos = raw_from_k(k)
        pos_out = raw_to_position(raw_pos)
        return spec.mapping.from_output(OutputSample(pos_out, 0.0, 0.0)).q_rad

    # 1. Binary search for a: first k in [k_min, k_max] where joint_q(k) >= spec.q_min_rad
    left = k_min
    right = k_max + 1
    while left < right:
        mid = (left + right) // 2
        if joint_q(mid) >= spec.q_min_rad:
            right = mid
        else:
            left = mid + 1
    a = left
    if a > k_max:
        return None

    # 2. Binary search for b: largest k in [k_min, k_max] where joint_q(k) <= spec.q_max_rad
    left = k_min
    right = k_max + 1
    while left < right:
        mid = (left + right) // 2
        if joint_q(mid) > spec.q_max_rad:
            right = mid
        else:
            left = mid + 1
    b = left - 1
    if b < k_min or a > b:
        return None

    # [a, b] is non-empty. Every k in [a, b] satisfies q_min_rad <= joint_q(k) <= q_max_rad.
    # 3. Find k in [a, b] minimizing abs(joint_q(k) - wanted_q_rad).
    left = a
    right = b + 1
    while left < right:
        mid = (left + right) // 2
        if joint_q(mid) >= wanted_q_rad:
            right = mid
        else:
            left = mid + 1
    p = left

    if p > b:
        chosen_k = b
    elif p == a:
        chosen_k = a
    else:
        # Both p and p - 1 are in [a, b]
        diff_p = abs(joint_q(p) - wanted_q_rad)
        diff_prev = abs(joint_q(p - 1) - wanted_q_rad)
        if diff_prev < diff_p:
            chosen_k = p - 1
        elif diff_p < diff_prev:
            chosen_k = p
        else:
            chosen_k = p if abs(p - (a + b) / 2) <= abs((p - 1) - (a + b) / 2) else p - 1

    return raw_from_k(chosen_k)


def prepare_reference_joint(
    spec: JointSpec, target: JointTarget
) -> tuple[DecisionCode | None, JointTarget | None, bytes | None]:
    """Validate target and compute inward representable reference and canonical packet."""
    if not isinstance(target, JointTarget):
        return DecisionCode.INVALID_ARGUMENT, None, None

    # Inclusive position bounds check on requested target
    if target.q_rad < spec.q_min_rad or target.q_rad > spec.q_max_rad:
        return DecisionCode.POSITION_OUT_OF_BOUNDS, None, None

    # Validate output mapping
    try:
        output = spec.mapping.to_output(target)
    except ValueError:
        return DecisionCode.INVALID_ARGUMENT, None, None

    # Check bench limits on mapped output values
    if not (RPM_MIN <= output.rpm <= RPM_MAX):
        return DecisionCode.INVALID_ARGUMENT, None, None
    if not (0.0 <= output.kd <= 6.0 and 0.0 <= output.kp <= 30.0 and -3.5 <= output.tau_nm <= 3.5):
        return DecisionCode.INVALID_ARGUMENT, None, None

    # Find inward representable position code
    raw_pos = find_inward_position_raw(spec, target.q_rad)
    if raw_pos is None:
        return DecisionCode.POSITION_OUT_OF_BOUNDS, None, None

    pos_out_rad = raw_to_position(raw_pos)
    actual_q_rad = spec.mapping.from_output(OutputSample(pos_out_rad, 0.0, 0.0)).q_rad
    if not (spec.q_min_rad <= actual_q_rad <= spec.q_max_rad):
        return DecisionCode.POSITION_OUT_OF_BOUNDS, None, None

    try:
        packet = command(
            spec.motor_id,
            mode=1,
            rpm=output.rpm,
            kd=output.kd,
            torque=output.tau_nm,
            watchdog=True,
            position=pos_out_rad,
            kp=output.kp,
        )
    except ValueError:
        return DecisionCode.INVALID_ARGUMENT, None, None

    actual_target = JointTarget(
        q_rad=actual_q_rad,
        dq_rad_s=target.dq_rad_s,
        tau_nm=target.tau_nm,
        kp=target.kp,
        kd=target.kd,
    )
    return None, actual_target, packet


def prepare_complete_reference(
    specs: tuple[JointSpec, ...], target: Target
) -> tuple[DecisionCode | None, tuple[JointTarget, ...] | None, tuple[bytes, ...] | None]:
    """Validate and prepare references and canonical packets for all joints atomically."""
    if len(target.joints) != len(specs):
        return DecisionCode.INVALID_ARGUMENT, None, None

    actual_targets: list[JointTarget] = []
    packets: list[bytes] = []

    for spec, joint_target in zip(specs, target.joints):
        code, actual_jt, packet = prepare_reference_joint(spec, joint_target)
        if code is not None or actual_jt is None or packet is None:
            return (code or DecisionCode.POSITION_OUT_OF_BOUNDS), None, None
        actual_targets.append(actual_jt)
        packets.append(packet)

    return None, tuple(actual_targets), tuple(packets)
