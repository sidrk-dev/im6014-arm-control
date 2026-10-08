import json
from array import array
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import im6014_serial as transport
import motor_session
import spin_5rpm
from fake_serial import FakeClock, FakeSerial


STOP_REPLY = bytes.fromhex('fcee801d1b48000000006cc4db000000000000000000f179457d')
FOC_REPLY = bytes.fromhex('fcee10fbc896000a600200c004000110008000e012348ddd0a74')
BAD_STOP_REPLY = STOP_REPLY[:-1] + bytes([STOP_REPLY[-1] ^ 1])


def _capture(capture, exchange_fn, bus, packet, *, source='fixture', identity='unit-test-fixture',
             connection=0, port='', baud=6000000):
    return transport.capture_exchange(
        capture, exchange_fn, bus, packet, 0,
        source=source, identity=identity, connection=connection, port=port, baud=baud,
    )


def _healthy_stop():
    import struct
    body = struct.pack('<BbBBhhiIHBB', 0, 35, 30, 72, 0, 0, 0, 0, 0, 0, 0)
    return b'\xfc\xee' + body + struct.pack('<I', spin_5rpm.crc32(body))


class DiagnosticCapture(unittest.TestCase):
    def test_dashboard_counts_typed_failures_without_message_matching(self):
        import threading
        cases = (
            (transport.TransactionTimeoutError('changed wording'), 1, 0),
            (transport.InvalidFeedbackError('changed wording', rejected=2, crc_rejected=2), 1, 2),
            (transport.InvalidFeedbackError('changed wording', rejected=1), 1, 0),
            (TimeoutError('changed wording'), 1, 0),
            (OSError('CRC deadline unrelated'), 0, 0),
        )
        for error, timeouts, crc in cases:
            with self.subTest(error=type(error).__name__, text=str(error)):
                session = object.__new__(motor_session.MotorSession)
                session.lock = threading.Lock()
                session.stop_requested = threading.Event()
                session.closing = threading.Event()
                session.bus = object()
                session.packet_capture = transport.PacketCapture()
                session.data = dict(id=0, tx=0, demo=True, port='',
                                    baud=6000000, connection=1,
                                    crc_errors=0, reply_timeouts=0)
                with patch.object(motor_session, 'exchange', side_effect=error):
                    with self.assertRaises(type(error)) as raised:
                        session._transact()
                self.assertIs(raised.exception, error)
                self.assertEqual(session.data['reply_timeouts'], timeouts)
                self.assertEqual(session.data['crc_errors'], crc)

    def test_captured_stop_fixture_and_foc_fixture_keep_raw_and_decoded_fields(self):
        cases = [
            (spin_5rpm.command(0), STOP_REPLY, dict(
                id=0, mode=0, timeout=1, driver_c=29, winding_c=27, voltage=36.0,
                torque_nm=0.0, rpm=0.0, position_rad=218.02713164107216,
                errors=0, warnings=0,
            )),
            (spin_5rpm.command(0, 1, rpm=5, kd=6), FOC_REPLY, dict(
                id=0, mode=1, timeout=0, driver_c=-5, winding_c=200, voltage=75.0,
                torque_nm=12.666666666666666, rpm=45.0,
                position_rad=4.71238898038469, errors=2147487745, warnings=7,
            )),
        ]
        for request, reply, expected in cases:
            clock = FakeClock()
            bus = FakeSerial(responses=[reply], clock=clock)
            capture = transport.PacketCapture(clock=clock.monotonic)
            with self.subTest(reply=reply.hex()), patch.object(transport, '_monotonic', clock.monotonic):
                decoded = _capture(
                    capture, spin_5rpm.exchange, bus, request,
                    source='fixture', identity='protocol-fixture', connection=1,
                    port='', baud=6000000,
                )
            record = capture.snapshot()['records'][0]
            self.assertEqual(bus.writes, (request,))
            self.assertEqual(record['request_hex'], request.hex())
            self.assertEqual(record['reply_hex'], reply.hex())
            self.assertEqual(record['decoded_fields'], expected)
            self.assertEqual(record['decoded_fields'], {k: v for k, v in decoded.items() if k != 'raw'})
            self.assertEqual((record['source'], record['identity']), ('fixture', 'protocol-fixture'))
            self.assertEqual(record['outcome'], 'OK')
            self.assertLessEqual(record['started_s'], record['ended_s'])

    def test_bad_end_clock_preserves_exchange_result_and_error(self):
        for end in (float('nan'), float('inf'), .5, 10 ** 1000):
            for failed in (False, True):
                with self.subTest(end=repr(end), failed=failed):
                    values = iter((1.0, end))
                    capture = transport.PacketCapture(clock=lambda: next(values))
                    original = OSError('injected driver failure')
                    result = {'raw': STOP_REPLY, 'id': 0}
                    def exchange(*args):
                        if failed:
                            raise original
                        return result
                    if failed:
                        with self.assertRaises(OSError) as raised:
                            _capture(capture, exchange, object(), spin_5rpm.command(0))
                        self.assertIs(raised.exception, original)
                    else:
                        self.assertIs(_capture(capture, exchange, object(),
                                               spin_5rpm.command(0)), result)
                    record = capture.snapshot()['records'][0]
                    self.assertIsNone(record['ended_s'])
                    self.assertEqual(record['outcome'], 'DRIVER_FAILURE' if failed else 'OK')

    def test_capture_metadata_is_validated_before_exchange(self):
        for invalid in (
            dict(source='unknown'), dict(source='physical', port=''),
            dict(identity=''), dict(connection=-1), dict(connection=True),
            dict(connection=0x80000000), dict(baud=115200), dict(baud=True),
        ):
            with self.subTest(invalid=invalid):
                called = []
                capture = transport.PacketCapture()
                kwargs = dict(source='fixture', identity='fixture', connection=0,
                              port='', baud=6000000)
                kwargs.update(invalid)
                with self.assertRaises(ValueError):
                    transport.capture_exchange(
                        capture, lambda *args: called.append(args), object(),
                        b'packet', 0, **kwargs)
                self.assertEqual(called, [])
                self.assertEqual(capture.snapshot()['records'], [])
        for capacity in (0, -1, 257, True, 1.5, '1'):
            with self.subTest(capacity=capacity), self.assertRaises(ValueError):
                transport.PacketCapture(capacity)

    def test_ring_capacity_strings_and_snapshots_are_bounded_and_owned(self):
        clock = FakeClock()
        capture = transport.PacketCapture(clock=clock.monotonic)
        response = spin_5rpm.feedback(STOP_REPLY, 0)

        def exchange(bus, packet, motor_id):
            clock.advance(0.001)
            return response

        for _ in range(260):
            _capture(capture, exchange, None, b'R' * 20,
                     source='fixture', identity='I' * 200, connection=5,
                     port='P' * 200, baud=6000000)
        first = capture.snapshot()
        self.assertEqual((first['capacity'], first['dropped'], len(first['records'])), (256, 4, 256))
        self.assertEqual(len(first['records'][0]['identity']), 96)
        self.assertEqual(len(first['records'][0]['port']), 160)
        first['records'][0]['decoded_fields']['id'] = 99
        first['records'].clear()
        second = capture.snapshot()
        self.assertEqual(len(second['records']), 256)
        self.assertEqual(second['records'][0]['decoded_fields']['id'], 0)

    def test_megabyte_request_and_reply_are_truncated_before_storage(self):
        packet = b'R' * 1_000_000
        seen = []
        capture = transport.PacketCapture()
        decoded = spin_5rpm.feedback(FOC_REPLY, 0)

        def exchange(bus, request, motor_id):
            seen.append(len(request))
            return dict(decoded, raw='ab' * 1_000_000, settings={'secret': 'FAKE_SECRET'})

        _capture(capture, exchange, None, packet)
        record = capture.snapshot()['records'][0]
        self.assertEqual(seen, [1_000_000])
        self.assertEqual(len(bytes.fromhex(record['request_hex'])), 20)
        self.assertEqual(len(bytes.fromhex(record['reply_hex'])), 26)
        dump = json.dumps(capture.snapshot())
        self.assertLessEqual(len(dump), 2000)
        self.assertNotIn('FAKE_SECRET', dump)
        self.assertNotIn('settings', record)

    def test_memoryview_packet_and_reply_limits_are_measured_in_bytes(self):
        packet = memoryview(array('I', [1] * 20))
        reply = memoryview(array('I', [2] * 20))
        capture = transport.PacketCapture()

        def exchange(bus, request, motor_id):
            return {'raw': reply, 'id': 0, 'mode': 0, 'timeout': 0}

        _capture(capture, exchange, None, packet)
        record = capture.snapshot()['records'][0]
        self.assertEqual(len(bytes.fromhex(record['request_hex'])), 20)
        self.assertEqual(len(bytes.fromhex(record['reply_hex'])), 26)

    def test_malformed_exception_counters_are_ignored_and_large_counts_clamped(self):
        capture = transport.PacketCapture()
        malformed = transport.TransactionTimeoutError('FAKE_SECRET_exception_text')
        malformed.rejected_frames = 'FAKE_SECRET_counter'
        malformed.crc_rejected_frames = 10 ** 1000
        malformed.tail = 'FAKE_SECRET_tail'

        def fail_malformed(bus, packet, motor_id):
            raise malformed

        with self.assertRaises(transport.TransactionTimeoutError):
            _capture(capture, fail_malformed, None, b'request')
        record = capture.snapshot()['records'][0]
        self.assertEqual((record['rejected_frames'], record['crc_rejected_frames']), (0, 0))
        self.assertEqual(record['tail_hex'], '')
        self.assertNotIn('FAKE_SECRET', json.dumps(capture.snapshot()))

        capture = transport.PacketCapture()
        large = transport.TransactionTimeoutError(
            'bounded count', rejected=10 ** 1000, crc_rejected=10 ** 1000,
        )

        def fail_large(bus, packet, motor_id):
            raise large

        with self.assertRaises(transport.TransactionTimeoutError):
            _capture(capture, fail_large, None, b'request')
        record = capture.snapshot()['records'][0]
        self.assertEqual((record['rejected_frames'], record['crc_rejected_frames']),
                         (0x7FFFFFFF, 0x7FFFFFFF))

    def test_timeout_invalid_feedback_and_secret_driver_errors_have_fixed_codes(self):
        clock = FakeClock()
        capture = transport.PacketCapture(clock=clock.monotonic)
        bus = FakeSerial(clock=clock)

        def timed_exchange(fake_bus, packet, motor_id):
            return transport.exchange(fake_bus, packet, motor_id, 0.002)

        with patch.object(transport, '_monotonic', clock.monotonic):
            with self.assertRaises(transport.TransactionTimeoutError):
                _capture(capture, timed_exchange, bus, spin_5rpm.command(0))
        timeout_record = capture.snapshot()['records'][0]
        self.assertEqual(timeout_record['outcome'], 'TIMEOUT')

        clock = FakeClock()
        capture = transport.PacketCapture(clock=clock.monotonic)
        bus = FakeSerial(responses=[BAD_STOP_REPLY], clock=clock)
        with patch.object(transport, '_monotonic', clock.monotonic):
            with self.assertRaises(transport.InvalidFeedbackError):
                _capture(capture, timed_exchange, bus, spin_5rpm.command(0))
        invalid_record = capture.snapshot()['records'][0]
        self.assertEqual(invalid_record['outcome'], 'INVALID_FEEDBACK')
        self.assertEqual((invalid_record['rejected_frames'], invalid_record['crc_rejected_frames']), (1, 1))
        self.assertLessEqual(len(bytes.fromhex(invalid_record['tail_hex'])), 25)

        secret = 'FAKE_SECRET_exception_text'
        settings = {'api_key': 'FAKE_SECRET_settings'}
        clock = FakeClock()
        capture = transport.PacketCapture(clock=clock.monotonic)
        bus = FakeSerial(clock=clock, write_error=RuntimeError(secret))
        with patch.object(transport, '_monotonic', clock.monotonic):
            with self.assertRaisesRegex(RuntimeError, 'FAKE_SECRET_exception_text'):
                _capture(capture, transport.exchange, bus, spin_5rpm.command(0))
        dump = json.dumps(capture.snapshot())
        self.assertEqual(capture.snapshot()['records'][0]['outcome'], 'DRIVER_FAILURE')
        self.assertNotIn(secret, dump)
        self.assertNotIn(settings['api_key'], dump)
        self.assertNotIn('settings', dump)

    def test_motor_session_snapshot_exports_demo_capture_without_sharing(self):
        with patch.object(motor_session.threading.Thread, 'start'):
            session = motor_session.MotorSession()
        session.bus = object()
        session.data.update(demo=True, connection=7, id=0, baud=6000000)
        feedback = spin_5rpm.feedback(STOP_REPLY, 0)
        with patch.object(motor_session, 'exchange', return_value=feedback) as lookup:
            session._transact()
        lookup.assert_called_once_with(session.bus, spin_5rpm.command(0), 0)
        snapshot = session.snapshot()
        self.assertEqual(snapshot['packet_capture']['records'][0]['source'], 'demo')
        self.assertEqual(snapshot['packet_capture']['records'][0]['identity'], 'demo-simulator')
        self.assertEqual(snapshot['packet_capture']['records'][0]['port'], '')
        json.dumps(snapshot)
        snapshot['packet_capture']['records'][0]['decoded_fields']['id'] = 99
        self.assertEqual(session.snapshot()['packet_capture']['records'][0]['decoded_fields']['id'], 0)

    def test_cli_run_log_capture_is_opt_in_and_uses_patched_exchange_lookup(self):
        args = SimpleNamespace(id=0, speed_limit=0, ack_timeout=False, arm=False)
        log = {'samples': []}
        clock = FakeClock()
        capture = transport.PacketCapture(clock=clock.monotonic)
        fixture = spin_5rpm.feedback(_healthy_stop(), 0)
        with patch.object(spin_5rpm, 'exchange', return_value=fixture) as lookup:
            spin_5rpm.run(
                object(), args, log, capture,
                capture_source='fixture', capture_identity='cli-test-fixture',
                capture_connection=2, capture_port='', capture_baud=6000000,
            )
        self.assertEqual(lookup.call_count, 2)
        self.assertEqual(len(log['packet_capture']['records']), 2)
        self.assertEqual(log['packet_capture']['records'][0]['source'], 'fixture')

        legacy_log = {'samples': []}
        with patch.object(spin_5rpm, 'exchange', return_value=fixture) as lookup:
            spin_5rpm.run(object(), args, legacy_log)
        self.assertEqual(lookup.call_count, 2)
        self.assertNotIn('packet_capture', legacy_log)


if __name__ == '__main__':
    unittest.main(verbosity=2)
