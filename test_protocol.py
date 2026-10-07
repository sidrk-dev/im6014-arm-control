"""Offline IM6014 communication-boundary acceptance checks."""
import math
import unittest

import spin_5rpm as s


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


if __name__ == "__main__":
    unittest.main(verbosity=2)
