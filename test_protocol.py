"""Offline IM6014 communication-boundary acceptance checks."""
import math
import json
from pathlib import Path
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


class B03Checksum(unittest.TestCase):
    def test_each_payload_and_checksum_bit_rejected(self):
        for offset in range(2, 26):
            for bit in range(8):
                with self.subTest(offset=offset, bit=bit):
                    damaged = bytearray(CAPTURED)
                    damaged[offset] ^= 1 << bit
                    with self.assertRaisesRegex(RuntimeError, "CRC"):
                        s.feedback(bytes(damaged), 0)

    def test_256_independent_vendor_vectors(self):
        path = Path(__file__).parent / "evidence/boundary/vendor-crc-vectors.json"
        records = json.loads(path.read_text())
        self.assertEqual(len(records["vectors"]), 256)
        for index, item in enumerate(records["vectors"]):
            with self.subTest(index=index):
                self.assertEqual(s.crc32(bytes.fromhex(item["inputHex"])), int(item["crcHex"], 16))
        self.assertEqual(s.crc32(CAPTURED[2:22]), 0x7d4579f1)

    def test_incomplete_checksum_words_rejected(self):
        for size in [1, 2, 3, 5, 6, 7, 17, 21]:
            with self.subTest(size=size), self.assertRaises(ValueError):
                s.crc32(bytes(size))


class B04Framing(unittest.TestCase):
    def test_short_oversized_and_coalesced_decoder_inputs(self):
        invalid = [CAPTURED[:n] for n in range(26)]
        invalid += [CAPTURED + bytes(n) for n in [1, 2, 26, 1000]]
        invalid += [CAPTURED + CAPTURED]
        for raw in invalid:
            with self.subTest(length=len(raw)), self.assertRaisesRegex(RuntimeError, "length/header"):
                s.feedback(raw, 0)

    def test_every_unrelated_header_byte_rejected(self):
        for offset, valid in [(0, 0xfc), (1, 0xee)]:
            for value in range(256):
                if value == valid:
                    continue
                with self.subTest(offset=offset, value=value):
                    raw = bytearray(CAPTURED)
                    raw[offset] = value
                    with self.assertRaisesRegex(RuntimeError, "length/header"):
                        s.feedback(bytes(raw), 0)

    def test_unsupported_modes_with_valid_vendor_crc_rejected(self):
        frames = ['fceea01d1b48000000006cc4db0000000000000000003f69a18e', 'fceeb01d1b48000000006cc4db000000000000000000586153f7', 'fceec01d1b48000000006cc4db000000000000000000da454c9e', 'fceed01d1b48000000006cc4db000000000000000000bd4dbee7', 'fceee01d1b48000000006cc4db0000000000000000001455a86d', 'fceef01d1b48000000006cc4db000000000000000000735d5a14']
        for raw in frames:
            with self.subTest(raw=raw), self.assertRaisesRegex(RuntimeError, "ID/mode"):
                s.feedback(bytes.fromhex(raw), 0)


class B05Fake(unittest.TestCase):
    def test_record_reset_and_chunk_remainder(self):
        from fake_serial import FakeClock, FakeSerial
        clock = FakeClock()
        bus = FakeSerial(responses=[(b"abc", b"de"), (b"new",)], clock=clock)
        mutable = bytearray(b"request")
        self.assertEqual(bus.write(mutable), 7)
        mutable[:] = b"changed"
        self.assertEqual(bus.writes, (b"request",))
        self.assertEqual(bus.read(2), b"ab")
        self.assertEqual(bus.read(2), b"cd")
        bus.reset_input_buffer()
        self.assertEqual(bus.in_waiting, 0)
        bus.write(b"second")
        self.assertEqual(bus.read(99), b"new")
        self.assertEqual(bus.writes, (b"request", b"second"))

    def test_delays_timeouts_and_write_controls(self):
        from fake_serial import FakeClock, FakeChunk, FakeSerial
        clock = FakeClock()
        bus = FakeSerial(responses=[(FakeChunk(b"ab", .03),)], clock=clock, timeout=.01)
        bus.write(b"x")
        self.assertEqual(bus.read(2), b"")
        self.assertAlmostEqual(clock.monotonic(), .01)
        bus.timeout = .05
        self.assertEqual(bus.read(1), b"a")
        self.assertAlmostEqual(clock.monotonic(), .03)
        self.assertEqual(bus.read(1), b"b")
        bus.write_result = 0
        self.assertEqual(bus.write(b"y"), 0)
        bus.write_error = OSError("write failed")
        with self.assertRaisesRegex(OSError, "write failed"):
            bus.write(b"z")
        bus.read_error = OSError("read failed")
        with self.assertRaisesRegex(OSError, "read failed"):
            bus.read(1)
        bus.reset_error = OSError("reset failed")
        with self.assertRaisesRegex(OSError, "reset failed"):
            bus.reset_input_buffer()

    def test_partial_write_preserves_next_successful_reply(self):
        from fake_serial import FakeSerial
        bus = FakeSerial(responses=[(b"reply",)], write_result=0)
        self.assertEqual(bus.write(b"x"), 0)
        self.assertEqual(bus.read(5), b"")
        bus.write_result = None
        self.assertEqual(bus.write(b"x"), 1)
        self.assertEqual(bus.read(5), b"reply")

    def test_empty_nonblocking_read_advances_fake_clock(self):
        from fake_serial import FakeClock, FakeSerial
        clock = FakeClock()
        bus = FakeSerial(clock=clock)
        self.assertEqual(bus.read(1), b"")
        self.assertGreater(clock.monotonic(), 0)


class B06Fragmented(unittest.TestCase):
    def test_each_split_and_byte_at_a_time(self):
        from fake_serial import FakeClock, FakeSerial
        from unittest.mock import patch
        cases = [(CAPTURED[:n], b"", CAPTURED[n:]) for n in range(27)]
        cases.append(tuple(bytes([b]) for b in CAPTURED))
        for chunks in cases:
            clock = FakeClock()
            bus = FakeSerial(responses=[chunks], clock=clock)
            with self.subTest(chunks=chunks), patch.object(s.time, "monotonic", clock.monotonic):
                result = s.exchange(bus, s.command(0), 0)
                self.assertEqual(result["raw"], CAPTURED.hex())
                self.assertEqual(bus.writes, (s.command(0),))

    def test_each_truncated_prefix_expires(self):
        from fake_serial import FakeClock, FakeSerial
        from unittest.mock import patch
        for size in range(26):
            clock = FakeClock()
            bus = FakeSerial(responses=[(CAPTURED[:size],)], clock=clock)
            with self.subTest(size=size), patch.object(s.time, "monotonic", clock.monotonic):
                with self.assertRaisesRegex(RuntimeError, "deadline"):
                    s.exchange(bus, s.command(0), 0, .002)
                self.assertEqual(len(bus.writes), 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
