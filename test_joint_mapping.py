from dataclasses import FrozenInstanceError
import math
import unittest

import im6014_codec as codec
from joint_mapping import JointMap, JointSample, JointTarget, OutputSample, OutputTarget


class B11JointMapping(unittest.TestCase):
    def test_n3_negative_direction_anchor(self):
        mapping = JointMap(reduction=3, direction=-1, zero_output_rad=0.25)
        output = mapping.to_output(JointTarget(-0.5, 2, 12, 36, 9))
        self.assertAlmostEqual(output.q_rad, 1.75, places=12)
        self.assertAlmostEqual(output.rpm, -57.29577951308232, places=12)
        self.assertAlmostEqual(output.tau_nm, -4.0, places=12)
        self.assertAlmostEqual(output.kp, 4.0, places=12)
        self.assertAlmostEqual(output.kd, 1.0, places=12)

    def test_n6_positive_direction_anchor(self):
        mapping = JointMap(reduction=6, direction=1, zero_output_rad=-0.75)
        output = mapping.to_output(JointTarget(0.25, -1 / 3, -12, 72, 18))
        self.assertAlmostEqual(output.q_rad, 0.75, places=12)
        self.assertAlmostEqual(output.rpm, -19.098593171027442, places=12)
        self.assertAlmostEqual(output.tau_nm, -2.0, places=12)
        self.assertAlmostEqual(output.kp, 2.0, places=12)
        self.assertAlmostEqual(output.kd, 0.5, places=12)

    def test_round_trip_for_ratios_directions_and_offsets(self):
        for reduction in (1, 3, 6):
            for direction in (-1, 1):
                with self.subTest(reduction=reduction, direction=direction):
                    mapping = JointMap(reduction, direction, 0.37)
                    target = JointTarget(-0.42, 1.25, -2.5, 8, 3)
                    output_target = mapping.to_output(target)
                    joint_again = mapping.from_output(
                        OutputSample(output_target.q_rad, output_target.rpm, output_target.tau_nm)
                    )
                    self.assertAlmostEqual(joint_again.q_rad, target.q_rad, places=12)
                    self.assertAlmostEqual(joint_again.dq_rad_s, target.dq_rad_s, places=12)
                    self.assertAlmostEqual(joint_again.tau_nm, target.tau_nm, places=12)

                    output_sample = OutputSample(-0.81, 43.25, 0.72)
                    joint_sample = mapping.from_output(output_sample)
                    output_again = mapping.to_output(
                        JointTarget(joint_sample.q_rad, joint_sample.dq_rad_s,
                                    joint_sample.tau_nm, 0, 0)
                    )
                    self.assertAlmostEqual(output_again.q_rad, output_sample.q_rad, places=12)
                    self.assertAlmostEqual(output_again.rpm, output_sample.rpm, places=12)
                    self.assertAlmostEqual(output_again.tau_nm, output_sample.tau_nm, places=12)

    def test_torque_speed_power_invariant(self):
        target = JointTarget(-0.3, 2.4, -1.7, 4, 0.8)
        for direction in (-1, 1):
            mapping = JointMap(reduction=6, direction=direction, zero_output_rad=0.21)
            output = mapping.to_output(target)
            joint_power = target.tau_nm * target.dq_rad_s
            output_power = output.tau_nm * output.rpm * math.tau / 60.0
            self.assertAlmostEqual(output_power, joint_power, places=12)

    def test_mapping_does_not_clip_speed_and_codec_still_rejects_it(self):
        mapping = JointMap(reduction=1, direction=1, zero_output_rad=0)
        output = mapping.to_output(JointTarget(0, 10_000, 0, 0, 0))
        self.assertGreater(abs(output.rpm), codec.RPM_MAX)
        with self.assertRaises(ValueError):
            codec.command(
                motor_id=0,
                mode=1,
                rpm=output.rpm,
                kd=output.kd,
                torque=output.tau_nm,
                watchdog=True,
                position=output.q_rad,
                kp=output.kp,
            )

    def test_mapping_keeps_excessive_torque_for_codec_rejection(self):
        output = JointMap(3, -1, .25).to_output(JointTarget(-.5, 2, 12, 36, 9))
        self.assertEqual(output.tau_nm, -4)
        with self.assertRaises(ValueError):
            codec.command(0, mode=1, rpm=output.rpm, torque=output.tau_nm,
                          position=output.q_rad, kp=output.kp, kd=output.kd)

    def test_zero_vectors_round_trip(self):
        for reduction in (1, 3, 6):
            for direction in (-1, 1):
                mapping = JointMap(reduction, direction, .37)
                output = mapping.to_output(JointTarget(0, 0, 0, 0, 0))
                self.assertEqual(output.kp, 0)
                self.assertEqual(output.kd, 0)
                self.assertEqual(mapping.from_output(OutputSample(
                    output.q_rad, output.rpm, output.tau_nm)), JointSample(0, 0, 0))

    def test_dataclasses_are_frozen(self):
        values = [
            (JointMap(3, 1, 0.2), "reduction"),
            (JointTarget(0, 0, 0, 1, 1), "q_rad"),
            (OutputTarget(0, 0, 0, 1, 1), "rpm"),
            (OutputSample(0, 0, 0), "tau_nm"),
            (JointSample(0, 0, 0), "dq_rad_s"),
        ]
        for value, field in values:
            with self.subTest(type=type(value).__name__), self.assertRaises(FrozenInstanceError):
                setattr(value, field, 1)

    def test_numeric_fields_reject_invalid_types_nonfinite_and_huge_integers(self):
        invalid = [True, False, "1", None, 1j, float("nan"), float("inf"), -float("inf"), 10 ** 1000]
        constructors = [
            lambda value: JointMap(value, 1, 0),
            lambda value: JointMap(1, 1, value),
            lambda value: JointTarget(value, 0, 0, 0, 0),
            lambda value: JointTarget(0, value, 0, 0, 0),
            lambda value: JointTarget(0, 0, value, 0, 0),
            lambda value: JointTarget(0, 0, 0, value, 0),
            lambda value: JointTarget(0, 0, 0, 0, value),
            lambda value: OutputTarget(value, 0, 0, 0, 0),
            lambda value: OutputTarget(0, value, 0, 0, 0),
            lambda value: OutputTarget(0, 0, value, 0, 0),
            lambda value: OutputTarget(0, 0, 0, value, 0),
            lambda value: OutputTarget(0, 0, 0, 0, value),
            lambda value: OutputSample(value, 0, 0),
            lambda value: OutputSample(0, value, 0),
            lambda value: OutputSample(0, 0, value),
            lambda value: JointSample(value, 0, 0),
            lambda value: JointSample(0, value, 0),
            lambda value: JointSample(0, 0, value),
        ]
        for constructor in constructors:
            for value in invalid:
                with self.subTest(constructor=constructor, value=repr(value)), self.assertRaises(ValueError):
                    constructor(value)

    def test_direction_and_reduction_validation(self):
        for direction in (True, False, 0, -2, 1.0, -1.0, "1"):
            with self.subTest(direction=direction), self.assertRaises(ValueError):
                JointMap(1, direction, 0)
        for reduction in (0, -1, -0.5):
            with self.subTest(reduction=reduction), self.assertRaises(ValueError):
                JointMap(reduction, 1, 0)

    def test_gains_must_be_nonnegative(self):
        for constructor in (
            lambda value: JointTarget(0, 0, 0, value, 0),
            lambda value: JointTarget(0, 0, 0, 0, value),
            lambda value: OutputTarget(0, 0, 0, value, 0),
            lambda value: OutputTarget(0, 0, 0, 0, value),
        ):
            with self.subTest(constructor=constructor), self.assertRaises(ValueError):
                constructor(-0.1)

    def test_arithmetic_underflow_and_overflow_raise_value_error(self):
        tiny = JointMap(reduction=1e-320, direction=1, zero_output_rad=0)
        with self.assertRaises(ValueError):
            tiny.to_output(JointTarget(1e-10, 0, 0, 0, 0))

        huge = JointMap(reduction=1e308, direction=1, zero_output_rad=0)
        with self.assertRaises(ValueError):
            huge.to_output(JointTarget(1e308, 0, 0, 0, 0))

        with self.assertRaises(ValueError):
            huge.from_output(OutputSample(0, 1e308, 0))


if __name__ == "__main__":
    unittest.main()
