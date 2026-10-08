"""Pure discrete protection policy and transition mathematics for joints.

This module owns pure thresholds, bounded state records, and transition
arithmetic for temperature and continuous effort protection. It owns no mutable
core state, clock, or hardware transport.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from joint_contract import DecisionCode, JointSpec, MotorHealth
    from joint_mapping import JointSample

__all__ = (
    "JointProtectionState",
    "MeasuredViolation",
    "ProtectionPolicy",
    "check_joint_limits",
    "transition_protection",
)


@dataclass(frozen=True)
class ProtectionPolicy:
    driver_trip_c: float
    driver_reset_c: float
    winding_trip_c: float
    winding_reset_c: float
    continuous_effort_nm: float
    continuous_dwell_s: float
    recovery_dwell_s: float

    def __post_init__(self) -> None:
        for name, val in (
            ("driver_trip_c", self.driver_trip_c),
            ("driver_reset_c", self.driver_reset_c),
            ("winding_trip_c", self.winding_trip_c),
            ("winding_reset_c", self.winding_reset_c),
        ):
            if isinstance(val, bool) or not isinstance(val, (int, float)):
                raise ValueError(f"{name} must be a finite real number")
            try:
                f_val = float(val)
            except (OverflowError, TypeError, ValueError) as exc:
                raise ValueError(f"{name} must be finite") from exc
            if not math.isfinite(f_val):
                raise ValueError(f"{name} must be finite")
            object.__setattr__(self, name, f_val)

        if self.driver_reset_c >= self.driver_trip_c:
            raise ValueError("driver_reset_c must be strictly less than driver_trip_c")
        if self.winding_reset_c >= self.winding_trip_c:
            raise ValueError("winding_reset_c must be strictly less than winding_trip_c")

        for name, val in (
            ("continuous_effort_nm", self.continuous_effort_nm),
            ("continuous_dwell_s", self.continuous_dwell_s),
            ("recovery_dwell_s", self.recovery_dwell_s),
        ):
            if isinstance(val, bool) or not isinstance(val, (int, float)):
                raise ValueError(f"{name} must be a finite positive real number")
            try:
                f_val = float(val)
            except (OverflowError, TypeError, ValueError) as exc:
                raise ValueError(f"{name} must be finite") from exc
            if not math.isfinite(f_val) or f_val <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
            object.__setattr__(self, name, f_val)


@dataclass(frozen=True)
class JointProtectionState:
    overload_duration_s: float = 0.0
    recovery_duration_s: float = 0.0
    last_overload_time_s: float | None = None
    last_recovery_time_s: float | None = None

    def reset_recovery(self) -> "JointProtectionState":
        return JointProtectionState(
            overload_duration_s=self.overload_duration_s,
            recovery_duration_s=0.0,
            last_overload_time_s=self.last_overload_time_s,
            last_recovery_time_s=None,
        )

    def reset_timing_gap(self) -> "JointProtectionState":
        return JointProtectionState(
            overload_duration_s=self.overload_duration_s,
            recovery_duration_s=0.0,
            last_overload_time_s=None,
            last_recovery_time_s=None,
        )



@dataclass(frozen=True)
class MeasuredViolation:
    position_limit: bool = False
    speed_limit: bool = False
    effort_limit: bool = False
    driver_temperature_limit: bool = False
    winding_temperature_limit: bool = False
    continuous_effort_limit: bool = False

    @property
    def has_violation(self) -> bool:
        return (
            self.position_limit
            or self.speed_limit
            or self.effort_limit
            or self.driver_temperature_limit
            or self.winding_temperature_limit
            or self.continuous_effort_limit
        )

    @property
    def primary_decision_code(self) -> DecisionCode | None:
        from joint_contract import DecisionCode

        if self.position_limit:
            return DecisionCode.POSITION_OUT_OF_BOUNDS
        if self.speed_limit:
            return DecisionCode.SPEED_OUT_OF_BOUNDS
        if self.driver_temperature_limit or self.winding_temperature_limit:
            return DecisionCode.TEMPERATURE_OUT_OF_BOUNDS
        if self.effort_limit or self.continuous_effort_limit:
            return DecisionCode.EFFORT_OUT_OF_BOUNDS
        return None


def check_joint_limits(
    spec: JointSpec,
    sample: JointSample,
    health: MotorHealth,
) -> MeasuredViolation:
    pos_out = sample.q_rad < spec.q_min_rad or sample.q_rad > spec.q_max_rad
    speed_out = abs(sample.dq_rad_s) > spec.v_max_rad_s
    effort_out = abs(sample.tau_nm) > spec.max_effort_nm
    driver_temp = health.driver_c >= spec.protection.driver_trip_c
    winding_temp = health.winding_c >= spec.protection.winding_trip_c
    return MeasuredViolation(
        position_limit=pos_out,
        speed_limit=speed_out,
        effort_limit=effort_out,
        driver_temperature_limit=driver_temp,
        winding_temperature_limit=winding_temp,
        continuous_effort_limit=False,
    )


def transition_protection(
    spec: JointSpec,
    sample: JointSample,
    health: MotorHealth,
    state: JointProtectionState,
    received_at: float,
    max_age_s: float,
    recovery_authorized: bool,
) -> tuple[MeasuredViolation, JointProtectionState, bool]:
    base = check_joint_limits(spec, sample, health)

    is_overload = abs(sample.tau_nm) > spec.protection.continuous_effort_nm
    if is_overload:
        if state.last_overload_time_s is not None:
            dt = received_at - state.last_overload_time_s
            if 0.0 <= dt <= max_age_s:
                new_overload_duration = state.overload_duration_s + dt
            else:
                new_overload_duration = state.overload_duration_s
        else:
            new_overload_duration = state.overload_duration_s
        new_last_overload_time = received_at
    else:
        new_overload_duration = 0.0
        new_last_overload_time = None

    continuous_fault = new_overload_duration >= spec.protection.continuous_dwell_s

    is_recovery_candidate = (
        health.mode == 0
        and health.healthy
        and not base.has_violation
        and not is_overload
        and health.driver_c <= spec.protection.driver_reset_c
        and health.winding_c <= spec.protection.winding_reset_c
    )

    if recovery_authorized and is_recovery_candidate:
        if state.last_recovery_time_s is not None:
            dt = received_at - state.last_recovery_time_s
            if 0.0 <= dt <= max_age_s:
                new_recovery_duration = state.recovery_duration_s + dt
                new_last_recovery_time = received_at
            else:
                new_recovery_duration = 0.0
                new_last_recovery_time = received_at
        else:
            new_recovery_duration = 0.0
            new_last_recovery_time = received_at
    else:
        new_recovery_duration = 0.0
        new_last_recovery_time = None

    recovery_satisfied = is_recovery_candidate and (
        new_recovery_duration >= spec.protection.recovery_dwell_s
    )

    violation = MeasuredViolation(
        position_limit=base.position_limit,
        speed_limit=base.speed_limit,
        effort_limit=base.effort_limit or continuous_fault,
        driver_temperature_limit=base.driver_temperature_limit,
        winding_temperature_limit=base.winding_temperature_limit,
        continuous_effort_limit=continuous_fault,
    )

    next_state = JointProtectionState(
        overload_duration_s=new_overload_duration,
        recovery_duration_s=new_recovery_duration,
        last_overload_time_s=new_last_overload_time,
        last_recovery_time_s=new_last_recovery_time,
    )

    return violation, next_state, recovery_satisfied
