import math

RATIO = 38.0 / 3.0
TAU = math.tau
POS_SCALE = (RATIO * 32768.0) / TAU
SPEED_SCALE = (RATIO * 64.0) / TAU  # rad/s to raw velocity units

def quantize_inward(lo: float, hi: float, scale: float) -> tuple[float, float] | None:
    if not (math.isfinite(lo) and math.isfinite(hi)) or lo > hi:
        return None
    raw_lo = math.ceil(lo * scale)
    raw_hi = math.floor(hi * scale)
    if raw_lo > raw_hi:
        return None
    return raw_lo / scale, raw_hi / scale

def run_proof():
    # 1. Counterexample: Naive rounding exceeds upper position limit
    q_max = 1.0
    raw_ideal = q_max * POS_SCALE
    naive_q = round(raw_ideal) / POS_SCALE
    assert naive_q > q_max, f"Naive rounding failed to overshoot: {naive_q} <= {q_max}"

    # Inward rounding strictly respects q_max
    inward_q = quantize_inward(-10.0, q_max, POS_SCALE)
    assert inward_q is not None and inward_q[1] <= q_max

    # 2. Inward selection with negative mapping (direction = -1, reduction = 2.0)
    direction, reduction = -1, 2.0
    q_min_j, q_max_j = -1.0, 0.5
    # Output position: q_out = direction * reduction * q_j
    out_a = direction * reduction * q_min_j  # +2.0
    out_b = direction * reduction * q_max_j  # -1.0
    out_lo, out_hi = min(out_a, out_b), max(out_a, out_b)
    grid_out = quantize_inward(out_lo, out_hi, POS_SCALE)
    assert grid_out is not None

    # Map grid boundaries back to joint space
    j_candidates = (grid_out[0] / (direction * reduction), grid_out[1] / (direction * reduction))
    j_min_dec, j_max_dec = min(j_candidates), max(j_candidates)
    assert j_min_dec >= q_min_j and j_max_dec <= q_max_j

    # 3. Discrete reference step rule with h > 0, V, A, lead, and position bounds
    h = 0.005
    q_prev = 0.1
    v_step_prev = 0.5
    V = 1.5
    A = 20.0
    lead = 0.02
    q_meas = 0.102
    q_min, q_max = -0.5, 0.5

    # Feasible step velocity intersection
    v_lo = max(-V, v_step_prev - A * h, (q_min - q_prev) / h, (q_meas - lead - q_prev) / h)
    v_hi = min(V, v_step_prev + A * h, (q_max - q_prev) / h, (q_meas + lead - q_prev) / h)
    assert v_lo <= v_hi

    # Feasible joint position range
    q_next_lo = q_prev + v_lo * h
    q_next_hi = q_prev + v_hi * h
    q_grid = quantize_inward(q_next_lo, q_next_hi, POS_SCALE)
    assert q_grid is not None

    # Pick representable position (e.g. upper inward bound)
    q_wire = q_grid[1]
    v_step_actual = (q_wire - q_prev) / h

    # Validate all invariants
    assert -V <= v_step_actual <= V
    assert v_step_prev - A * h - 1e-12 <= v_step_actual <= v_step_prev + A * h + 1e-12
    assert q_min <= q_wire <= q_max
    assert abs(q_wire - q_meas) <= lead + 1e-12

    # 4. Infeasible/unrepresentable interval withdraws motion (returns None)
    # Contradictory interval (lo > hi)
    empty_interval = quantize_inward(0.105, 0.101, POS_SCALE)
    assert empty_interval is None

    # Interval narrower than one quantization step
    step = 1.0 / POS_SCALE
    narrow_empty = quantize_inward(0.1 + 0.1 * step, 0.1 + 0.8 * step, POS_SCALE)
    assert narrow_empty is None

if __name__ == "__main__":
    run_proof()
