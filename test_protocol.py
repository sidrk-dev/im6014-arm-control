"""Offline IM6014 communication-boundary acceptance checks."""
import math
import unittest

import spin_5rpm as s

CAPTURED = bytes.fromhex("fcee801d1b48000000006cc4db000000000000000000f179457d")
NEGATIVE = bytes.fromhex("fcee10fbc89600f6a0fd0040fbff0110008000e01234c9a85ecc")
POSITIVE = bytes.fromhex("fcee10fbc896000a600200c004000110008000e012348ddd0a74")


class B01Encoder(unittest.TestCase):
    def test_pre_extraction_golden_commands(self):
        cases = [
            ({"motor_id": 0}, "feee80000000000000000000000000001d99fb6e"),
            ({"motor_id": 14}, "feee8e000000000000000000000000007da3a05b"),
            ({"motor_id": 0, "watchdog": False}, "feee000000000000000000000000000080462079"),
            ({"motor_id": 0, "mode": 1, "rpm": 5, "kd": 6}, "feee9000000044000000000000007b071f113ea7"),
            ({"motor_id": 3, "mode": 1, "rpm": -5, "kd": 1, "torque": -.2,
              "position": -math.pi / 2, "kp": 3}, "feee9300d8ffbcffab6afeffef003f013c754064"),
            ({"motor_id": 1, "mode": 1, "rpm": 60}, "feee910000002b0300000000000000006bad05c6"),
            ({"motor_id": 1, "mode": 1, "rpm": -60}, "feee91000000d5fc0000000000000000b16eeecc"),
        ]
        for values, expected in cases:
            with self.subTest(values=values):
                self.assertEqual(s.command(**values).hex(), expected)
                self.assertEqual(len(s.command(**values)), 20)

    def test_single_pure_codec_implementation(self):
        import im6014_codec as codec
        self.assertIs(s.command, codec.command)
        self.assertIs(s.feedback, codec.feedback)
        self.assertIs(s.crc32, codec.crc32)
        import subprocess
        import sys
        result = subprocess.run(
            [sys.executable, "-B", "-c",
             "import sys; sys.modules['serial'] = None; "
             "sys.modules['PySide6'] = None; import im6014_codec as c; "
             "assert c.command(0).hex() == 'feee80000000000000000000000000001d99fb6e'"],
            capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)


class B02Decoder(unittest.TestCase):
    def test_captured_physical_stop_fields(self):
        expected = dict(id=0, mode=0, timeout=1, driver_c=29, winding_c=27,
                        voltage=36.0, torque_nm=0.0, rpm=0.0, errors=0, warnings=0,
                        position_rad=218.02713164107216, raw=CAPTURED.hex())
        result = s.feedback(CAPTURED, 0)
        self.assertEqual(set(result), set(expected))
        for key, value in expected.items():
            with self.subTest(key=key):
                if key == "position_rad":
                    self.assertAlmostEqual(result[key], value, places=12)
                else:
                    self.assertEqual(result[key], value)

    def test_generated_signed_and_unsigned_fields(self):
        for raw, sign in [(NEGATIVE, -1), (POSITIVE, 1)]:
            with self.subTest(sign=sign):
                result = s.feedback(raw, 0)
                self.assertEqual((result["id"], result["mode"], result["timeout"]), (0, 1, 0))
                self.assertEqual((result["driver_c"], result["winding_c"], result["voltage"]), (-5, 200, 75.0))
                self.assertEqual((result["errors"], result["warnings"]), (2147487745, 7))
                self.assertEqual(result["rpm"], sign * 45.0)
                self.assertAlmostEqual(result["torque_nm"], sign * 12.666666666666666, places=12)
                self.assertAlmostEqual(result["position_rad"], sign * 4.71238898038469, places=12)
                self.assertEqual(result["raw"], raw.hex())


if __name__ == "__main__":
    unittest.main(verbosity=2)
