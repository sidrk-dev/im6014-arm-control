```json
[
  {
    "file": "joint_controller.py",
    "function": "dispatch_result",
    "line": 686,
    "severity": "high",
    "evidence": "In dispatch_result, when result is WriteResult.COMPLETE, housekeeping = self._housekeeping(now) is evaluated at line 673. If feedback has aged out, a joint has exceeded measured limits, or the admitted target has expired, _housekeeping latches a joint fault and transitions state to State.FAULT or State.STOPPED. The condition 'if still_current and housekeeping is None:' at line 681 fails, correctly preventing self._next_dispatch_index from advancing. However, the method unconditionally falls through to line 686 and returns self._accepted(). This erroneously reports an accepted dispatch decision when the core has latched a fault or the goal has expired.",
    "fix": "Check housekeeping and self._state before returning; if housekeeping is not None or self._faults or self._state not in (State.ARMED, State.MOVING), return self._rejected(housekeeping or DecisionCode.FAULT_LATCHED)."
  },
  {
    "file": "joint_reference.py",
    "function": "shape_reference_joint",
    "line": 480,
    "severity": "high",
    "evidence": "Line 480 contains 'if v_desired != 0 and actual_target.q_rad == confirmed_ref.q_rad: return DecisionCode.CONSTRAINT_VIOLATION, None, None, None'. When a small non-zero velocity goal or small sample interval h produces a position delta smaller than the actuator's discrete position quantization resolution (~2.4 µrad), raw_pos does not advance and actual_target.q_rad remains equal to confirmed_ref.q_rad. Treating this normal physical/representation quantization outcome as a CONSTRAINT_VIOLATION fails trajectory preparation and causes prepare_refresh to latch a constraint fault on the joint when no limit was violated.",
    "fix": "Remove line 480 or permit actual_target.q_rad == confirmed_ref.q_rad when the continuous projected position delta |q_continuous - q_prev| is smaller than one quantization step."
  },
  {
    "file": "joint_controller.py",
    "function": "prepare_refresh",
    "line": 535,
    "severity": "medium",
    "evidence": "Lines 535-537 execute 'if gap > self._max_reference_gap_s: self.stop(); return None'. This pre-check silently calls self.stop() and transitions to State.STOPPED without latching a fault. Downstream, shape_reference_joint specifically checks 'if h > max_reference_gap_s: return DecisionCode.REFERENCE_GAP_EXCEEDED' (line 354), and prepare_refresh has fault-latching logic for prepared.failures (lines 545-562). The pre-check intercepts gap violations beforehand, suppressing the error code and making DecisionCode.REFERENCE_GAP_EXCEEDED in shape_reference_joint unreachable dead code.",
    "fix": "Remove the premature gap check in lines 535-537 so gap validation is handled uniformly through prepare_complete_reference and shape_reference_joint, or explicitly latch constraint_fault when gap > self._max_reference_gap_s before stopping."
  },
  {
    "file": "joint_controller.py",
    "function": "claim_dispatch",
    "line": 619,
    "severity": "medium",
    "evidence": "In claim_dispatch, lines 619-624 check 'if conf_ref is not None: gap = ...'. If conf_ref is None, execution bypasses this block and calls verify_dispatch_claim(..., conf_ref=conf_ref, ...) at line 630. Inside verify_dispatch_claim, line 527 enforces 'not isinstance(conf_ref, JointTrajectoryReference)', returning DecisionCode.CONSTRAINT_VIOLATION. Lines 644-647 then latch constraint_fault=True and fault the joint. In contrast, prepare_refresh cleanly returns None when conf_ref is None.",
    "fix": "In claim_dispatch, check 'if conf_ref is None: return False' immediately at line 619 before proceeding to verify_dispatch_claim."
  },
  {
    "file": "joint_reference.py",
    "function": "verify_dispatch_claim",
    "line": 538,
    "severity": "medium",
    "evidence": "Lines 538-548 check: 'if prop_ref.sample_time_s == conf_ref.sample_time_s: if prop_ref != conf_ref: return DecisionCode.CONSTRAINT_VIOLATION elif (prop_ref.sample_time_s != prepared_at_s or prop_ref.sample_time_s <= conf_ref.sample_time_s): return DecisionCode.CONSTRAINT_VIOLATION'. In the first branch (where sample_time_s matches conf_ref), it does not verify whether prepared_at_s == conf_ref.sample_time_s. If time has advanced (prepared_at_s > conf_ref.sample_time_s), a plan proposing the old reference with stale sample time is accepted instead of being rejected.",
    "fix": "In verify_dispatch_claim, require prop_ref.sample_time_s == prepared_at_s in all cases, or ensure that if prop_ref.sample_time_s == conf_ref.sample_time_s, prepared_at_s also equals conf_ref.sample_time_s."
  },
  {
    "file": "joint_reference.py",
    "function": "shape_reference_joint",
    "line": 456,
    "severity": "medium",
    "evidence": "In shape_reference_joint (lines 456-466), packet = command_with_raw_position(...) passes raw_pos, mode=1, kd=output_req.kd, torque=output_req.tau_nm, watchdog=True, kp=output_req.kp, raw_rpm=raw_rpm, but omits the rpm parameter. In command_with_raw_position, rpm defaults to 0 and line 144 checks RPM_MIN <= rpm <= RPM_MAX. By omitting rpm, continuous speed bounds validation on output_req.rpm is bypassed, creating an inconsistency with prepare_reference_joint (line 218) where rpm=output.rpm is explicitly passed.",
    "fix": "Pass rpm=output_req.rpm to command_with_raw_position in shape_reference_joint line 456."
  },
  {
    "file": "im6014_codec.py",
    "function": "feedback",
    "line": 253,
    "severity": "medium",
    "evidence": "In feedback(), line 253 returns 'timeout=m >> 7'. In Python, m >> 7 evaluates to int 0 or 1. However, MotorHealth.__post_init__ strictly checks 'if type(self.timeout) is not bool: raise ValueError(\"timeout must be a bool\")'. Passing the raw dict output of feedback() into MotorHealth fails this strict type assertion.",
    "fix": "Convert the timeout flag to a boolean in feedback(): 'timeout=bool(m & 0x80)' or 'timeout=bool(m >> 7)'."
  },
  {
    "file": "joint_controller.py",
    "function": "prepare_refresh",
    "line": 511,
    "severity": "low",
    "evidence": "Lines 511-526 iterate over self._specs and self._observations to verify joint.valid, pos_out, speed_out, and effort_out. Because prepare_refresh requires State.ARMED or State.MOVING, self._housekeeping(now) at line 504 already performed these exact checks on all joints and latched any faults. Lines 511-526 are redundant dead code. Additionally, line 525 redundantly calls self.stop() after _latch_joint_fault (which already sets _state = State.FAULT). The identical redundant loops exist in claim_dispatch lines 601-617 and submit_target lines 508-511.",
    "fix": "Remove the duplicate observation validity and bounds checks in prepare_refresh, claim_dispatch, and submit_target, relying on _housekeeping(now)."
  },
  {
    "file": "joint_reference.py",
    "function": "prepare_reference_joint",
    "line": 189,
    "severity": "low",
    "evidence": "Lines 189-199 in prepare_reference_joint and lines 431-440 in shape_reference_joint invoke command(...) and discard the return value before invoking command_with_raw_position. command() constructs a full 20-byte packet and computes a CRC-32 purely for parameter checking, which command_with_raw_position repeats. In shape_reference_joint, calling command() with output_req.q_rad checks raw codec position range on the unshaped goal position rather than the trajectory step.",
    "fix": "Eliminate the discarded command() calls and perform direct range validation on output parameters before calling command_with_raw_position."
  }
]
```