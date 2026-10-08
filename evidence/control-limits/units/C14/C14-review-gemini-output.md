[
  {
    "file": "joint_protection.py",
    "function": "transition_protection",
    "line": 128,
    "severity": "high",
    "evidence": "transition_protection accumulates dwell by adding dt = received_at - state.last_overload_time_s (and last_recovery_time_s) whenever 0 <= dt <= max_age_s. If two observations arrive separated by a communication gap up to max_age_s (e.g. 0.49s), the entire unobserved gap is credited as continuous dwell. Additionally, communication_failed, _record_dispatch_failure, and _housekeeping stale feedback detection in JointCore do not reset last_recovery_time_s or last_overload_time_s, allowing gaps across communication failures and stale periods to be credited as sustained dwell.",
    "fix": "Reset timing gaps and recovery timers on all joints when communication fails, dispatch fails, or observations become stale. In transition_protection, do not credit unobserved gaps across missing or discontinuous samples toward dwell accumulation."
  },
  {
    "file": "joint_controller.py",
    "function": "JointCore.checked",
    "line": 423,
    "severity": "high",
    "evidence": "In JointCore.checked(), upon check success, self._protection_states is unconditionally overwritten with [JointProtectionState() for _ in self._specs]. This resets overload_duration_s to 0.0, completely erasing confirmed unsafe continuous effort overload history when performing a check in STOPPED or READY state without a tripped fault. The specification requires that missing data or check operations cannot discard confirmed unsafe history.",
    "fix": "Preserve accumulated overload_duration_s in JointCore.checked() by calling s.reset_recovery() rather than reinitializing to default JointProtectionState(), only resetting overload_duration_s when authorized continuous effort recovery dwell has been satisfied."
  },
  {
    "file": "joint_protection.py",
    "function": "transition_protection",
    "line": 127,
    "severity": "high",
    "evidence": "new_overload_duration and new_recovery_duration accumulate dt using binary floating-point addition (state.overload_duration_s + dt). Due to floating-point representation error (such as 0.1 + 0.2 = 0.30000000000000004), accumulated float durations exceed continuous_dwell_s or recovery_dwell_s prematurely before the physical dwell period has elapsed. Furthermore, continuous_effort_nm threshold comparison uses float inequalities subject to mapping roundoff rather than exact rational representation.",
    "fix": "Use exact rational arithmetic (Fraction) for dwell duration accumulation and effort limit comparisons to eliminate floating-point rounding errors causing premature overload trips or premature fault recoveries."
  },
  {
    "file": "joint_protection.py",
    "function": "JointProtectionState",
    "line": 60,
    "severity": "medium",
    "evidence": "JointProtectionState lacks a __post_init__ method and performs no validation on its fields, allowing nonfinite, negative, or non-numeric values. In transition_protection, duration histories can become nonfinite, and overload_duration_s and recovery_duration_s accumulate indefinitely without finite bounds, violating the requirement for finite bounded timers.",
    "fix": "Implement __post_init__ on JointProtectionState to validate that all duration fields are finite, nonnegative real numbers and timestamps are finite or None. Enforce finite bounds on timer accumulation in transition_protection."
  },
  {
    "file": "joint_protection.py",
    "function": "transition_protection",
    "line": 180,
    "severity": "high",
    "evidence": "In MeasuredViolation, effort_limit is set to 'base.effort_limit or continuous_fault'. Similarly, in JointCore._latch_joint_fault (joint_controller.py:249), effort_limit is set to 'existing.effort_limit or effort_limit or continuous_effort_limit'. This conflates peak instantaneous effort limit violation with continuous effort dwell limit violation, wrongly latching peak JointFault.effort_limit when only continuous dwell tripped.",
    "fix": "In transition_protection, keep effort_limit=base.effort_limit without ORing continuous_fault. In JointCore._latch_joint_fault, keep effort_limit=existing.effort_limit or effort_limit without ORing continuous_effort_limit."
  },
  {
    "file": "joint_controller.py",
    "function": "JointCore.checked",
    "line": 417,
    "severity": "medium",
    "evidence": "When JointCore.checked() fails because below_recovery is False (temperature or effort exceeds reset thresholds), it returns DecisionCode.FAULT_LATCHED without resetting the recovery dwell timers. The contract specifies: 'If any joint telemetry is stale or exceeds reset thresholds during the recovery window, the recovery dwell timer resets and checked() rejects with DecisionCode.FAULT_LATCHED'.",
    "fix": "Reset recovery dwell timers across all joints (self._protection_states = [s.reset_recovery() for s in self._protection_states]) before returning DecisionCode.FAULT_LATCHED when any joint exceeds reset thresholds or telemetry is stale."
  },
  {
    "file": "joint_protection.py",
    "function": "transition_protection",
    "line": 142,
    "severity": "high",
    "evidence": "In transition_protection, when is_overload is False, new_overload_duration is immediately reset to 0.0 and continuous_fault becomes False. Once continuous dwell has tripped and latched a fault, a single observation below continuous_effort_nm prematurely clears the overload violation flag before the system has completed the full recovery dwell window under reset thresholds.",
    "fix": "Maintain the continuous effort violation flag while the fault remains latched, only clearing the violation and timer once authorized recovery dwell below reset thresholds has completed."
  },
  {
    "file": "joint_controller.py",
    "function": "JointCore._housekeeping",
    "line": 277,
    "severity": "medium",
    "evidence": "JointCore._housekeeping only calls check_joint_limits, which always sets continuous_effort_limit=False and does not advance or evaluate continuous effort dwell. During periodic tick() calls in READY, ARMED, or MOVING states, continuous effort overload dwell is never evaluated or tripped if no new receive() calls arrive.",
    "fix": "Evaluate continuous effort protection and advance dwell in _housekeeping using the current monotonic time, latching continuous_effort_limit if continuous dwell expires during housekeeping."
  }
]