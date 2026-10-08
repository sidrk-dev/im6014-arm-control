import contextlib
import io
import math
import struct
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import im6014_codec as codec
import im6014_serial as transport
import motor_session
import spin_5rpm
from joint_mapping import JointMap, JointTarget


CAPTURED_STOP = bytes.fromhex(
    'fcee801d1b48000000006cc4db000000000000000000f179457d'
)


class B12CodecNumerics(unittest.TestCase):
    def test_motor_ids_modes_and_watchdog_use_exact_types(self):
        for value in (True, False, 1.0, '1', None, -1, 15):
            with self.subTest(field='motor_id', value=value), self.assertRaises(ValueError):
                codec.command(value)
        for value in (True, False, 0.0, 1.0, '1', None, -1, 2):
            with self.subTest(field='mode', value=value), self.assertRaises(ValueError):
                codec.command(0, mode=value)
        for value in (0, 1, 2, None, 'true'):
            with self.subTest(field='watchdog', value=value), self.assertRaises(ValueError):
                codec.command(0, watchdog=value)

    def test_controls_reject_bad_types_huge_ints_and_nonfinite_values(self):
        fields = ('rpm', 'kd', 'torque', 'position', 'kp')
        for field in fields:
            for value in (True, False, '1', None, 1j, 10 ** 1000):
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    codec.command(0, mode=1, **{field: value})
            for value in (math.nan, math.inf, -math.inf):
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    codec.command(0, mode=1, **{field: value})

    def test_existing_control_bounds_and_signed_packet_endpoints(self):
        invalid = (
            ('rpm', codec.RPM_MIN - 1), ('rpm', codec.RPM_MAX + 1),
            ('kd', -0.001), ('kd', 6.001), ('kp', -0.001), ('kp', 30.001),
            ('torque', -3.501), ('torque', 3.501),
            ('position', codec.POSITION_MIN - 1), ('position', codec.POSITION_MAX + 1),
        )
        for field, value in invalid:
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                codec.command(0, mode=1, **{field: value})

        low_speed = codec.command(0, mode=1, rpm=codec.RPM_MIN)
        high_speed = codec.command(0, mode=1, rpm=codec.RPM_MAX)
        self.assertEqual(struct.unpack('<h', low_speed[6:8])[0], -32768)
        self.assertEqual(struct.unpack('<h', high_speed[6:8])[0], 32767)
        low_position = codec.command(0, mode=1, position=codec.POSITION_MIN)
        high_position = codec.command(0, mode=1, position=codec.POSITION_MAX)
        self.assertEqual(struct.unpack('<i', low_position[8:12])[0], -2147483648)
        self.assertEqual(struct.unpack('<i', high_position[8:12])[0], 2147483647)

    def test_exact_existing_golden_packets_are_unchanged(self):
        self.assertEqual(
            codec.command(0).hex(),
            'feee80000000000000000000000000001d99fb6e',
        )
        self.assertEqual(
            codec.command(0, 1, rpm=5, kd=6).hex(),
            'feee9000000044000000000000007b071f113ea7',
        )

    def test_feedback_rejects_bad_expected_ids_and_buffer_types_deliberately(self):
        for value in (True, False, 0.0, '0', None, -1, 15):
            with self.subTest(expected_id=value), self.assertRaises(ValueError):
                codec.feedback(CAPTURED_STOP, value)
        for value in (None, 'frame', [1, 2], 26, object()):
            with self.subTest(raw_type=type(value).__name__), self.assertRaises(ValueError):
                codec.feedback(value, 0)

        self.assertEqual(codec.feedback(bytearray(CAPTURED_STOP), 0)['id'], 0)
        self.assertEqual(codec.feedback(memoryview(CAPTURED_STOP), 0)['id'], 0)
        with self.assertRaisesRegex(RuntimeError, 'length/header'):
            codec.feedback(CAPTURED_STOP[:-1], 0)
        with self.assertRaisesRegex(RuntimeError, 'ID/mode'):
            codec.feedback(CAPTURED_STOP, 1)

    def test_mapping_preserves_large_target_and_codec_rejects_it(self):
        mapping = JointMap(reduction=1, direction=1, zero_output_rad=0)
        output = mapping.to_output(JointTarget(0, 10_000, 0, 0, 0))
        self.assertGreater(output.rpm, codec.RPM_MAX)
        with self.assertRaisesRegex(ValueError, 'Speed cannot be encoded'):
            codec.command(0, mode=1, rpm=output.rpm)


class B12SessionInputs(unittest.TestCase):
    def test_dashboard_accepts_numeric_strings_and_rejects_bool_or_nonfinite(self):
        values = dict(kind='Speed', rpm=5, kd=1, ff=.16, torque=0, duration=5,
                      position_deg=10, kp=3, speed_limit=0, travel_limit=0)
        values['position_reference'] = 'Relative'
        as_strings = {key: str(value) if key != 'position_reference' else value
                      for key, value in values.items()}
        self.assertEqual(motor_session.validate_test(as_strings)['rpm'], 5.0)

        numeric_fields = ('rpm', 'kd', 'ff', 'torque', 'duration', 'position_deg',
                          'kp', 'speed_limit', 'travel_limit')
        for field in numeric_fields:
            for value in (True, False, math.nan, math.inf, -math.inf):
                candidate = dict(values, **{field: value})
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    motor_session.validate_test(candidate)

    def test_connect_rejects_non_integer_baud_before_opening(self):
        for baud in (True, 4000000.0, 6000000.0, '4000000', None):
            with self.subTest(baud=baud), self.assertRaises(ValueError):
                motor_session.MotorSession._handle(
                    SimpleNamespace(bus=None), 'connect', {'id': 0, 'baud': baud}
                )


class B12SerialInputs(unittest.TestCase):
    class NoIoBus:
        def __init__(self):
            self.reset_calls = 0
            self.write_calls = 0

        def reset_input_buffer(self):
            self.reset_calls += 1

        def write(self, packet):
            self.write_calls += 1
            return len(packet)

    def test_invalid_ids_and_deadlines_do_no_bus_io(self):
        invalid_ids = (True, False, 0.0, '0', None, -1, 15)
        invalid_deadlines = (True, False, '0.05', None, 0, -0.1,
                             math.nan, math.inf, -math.inf, 10 ** 1000)
        for motor_id in invalid_ids:
            bus = self.NoIoBus()
            with self.subTest(motor_id=motor_id), self.assertRaises(ValueError):
                transport.exchange(bus, b'one megabyte not required here', motor_id)
            self.assertEqual((bus.reset_calls, bus.write_calls), (0, 0))
        for deadline in invalid_deadlines:
            bus = self.NoIoBus()
            with self.subTest(deadline=deadline), self.assertRaises(ValueError):
                transport.exchange(bus, b'one megabyte not required here', 0, deadline)
            self.assertEqual((bus.reset_calls, bus.write_calls), (0, 0))


class B12CliAndZeroSpeed(unittest.TestCase):
    def test_invalid_cli_codec_input_is_argparse_error_before_file_or_serial(self):
        stderr = io.StringIO()
        argv = ['spin_5rpm.py', '--rpm', 'nan', '--log', '/tmp/b12-invalid-run.json']
        with patch.object(sys, 'argv', argv), patch.object(spin_5rpm.serial, 'Serial') as serial_open, \
                patch.object(Path, 'write_text') as write_file, \
                contextlib.redirect_stderr(stderr):
            with self.assertRaises(SystemExit) as raised:
                spin_5rpm.main()
        self.assertEqual(raised.exception.code, 2)
        self.assertIn('finite', stderr.getvalue().lower())
        self.assertNotIn('traceback', stderr.getvalue().lower())
        serial_open.assert_not_called()
        write_file.assert_not_called()

    def test_negative_cli_feedforward_fails_before_encoder(self):
        stderr = io.StringIO()
        argv = ['spin_5rpm.py', '--torque-ff', '-0.1', '--log', '/tmp/b12-negative-ff.json']
        with patch.object(sys, 'argv', argv), patch.object(spin_5rpm, 'command') as encode, \
                patch.object(spin_5rpm.serial, 'Serial') as serial_open, \
                patch.object(Path, 'write_text') as write_file, \
                contextlib.redirect_stderr(stderr):
            with self.assertRaises(SystemExit) as raised:
                spin_5rpm.main()
        self.assertEqual(raised.exception.code, 2)
        self.assertIn('Friction compensation must be nonnegative', stderr.getvalue())
        self.assertNotIn('traceback', stderr.getvalue().lower())
        encode.assert_not_called()
        serial_open.assert_not_called()
        write_file.assert_not_called()

    def test_zero_rpm_encodes_zero_feedforward_and_still_checks_measured_mean(self):
        class FakeClock:
            def __init__(self):
                self.now = 0.0

            def monotonic(self):
                return self.now

            def sleep(self, seconds):
                self.now += seconds

        def run_with_measured_rpm(measured_rpm):
            clock = FakeClock()
            packets = []
            replies = 0

            def exchange(_bus, packet, _motor_id):
                nonlocal replies
                packets.append(packet)
                replies += 1
                mode = 0 if replies <= 2 else 1
                return dict(mode=mode, errors=0, warnings=0, timeout=0,
                            voltage=36.0, driver_c=25, winding_c=25,
                            torque_nm=0.0, rpm=measured_rpm, position_rad=0.0)

            args = SimpleNamespace(id=0, speed_limit=0, arm=True, duration=2.0, ack_timeout=False,
                                   rpm=0.0, kd=6.0, torque_ff=0.16, travel_limit=0)
            with patch.object(spin_5rpm, 'time', SimpleNamespace(
                    monotonic=clock.monotonic, sleep=clock.sleep)), \
                    patch.object(spin_5rpm, 'exchange', exchange):
                spin_5rpm.run(object(), args, {'samples': []})
            return packets

        packets = run_with_measured_rpm(0.0)
        foc_torque_fields = [struct.unpack('<h', packet[4:6])[0]
                             for packet in packets if (packet[2] >> 4) & 7 == 1]
        self.assertTrue(foc_torque_fields)
        self.assertEqual(set(foc_torque_fields), {0})

        clock = FakeClock()
        replies = 0
        def drifting_exchange(_bus, _packet, _motor_id):
            nonlocal replies
            replies += 1
            mode = 0 if replies <= 2 else 1
            return dict(mode=mode, errors=0, warnings=0, timeout=0,
                        voltage=36.0, driver_c=25, winding_c=25,
                        torque_nm=0.0, rpm=1.0, position_rad=0.0)

        args = SimpleNamespace(id=0, speed_limit=0, arm=True, duration=2.0, ack_timeout=False,
                               rpm=0.0, kd=6.0, torque_ff=0.16, travel_limit=0)
        with patch.object(spin_5rpm, 'time', SimpleNamespace(
                monotonic=clock.monotonic, sleep=clock.sleep)), \
                patch.object(spin_5rpm, 'exchange', drifting_exchange):
            with self.assertRaisesRegex(RuntimeError, 'did not reach the requested speed'):
                spin_5rpm.run(object(), args, {'samples': []})


if __name__ == '__main__':
    unittest.main()
