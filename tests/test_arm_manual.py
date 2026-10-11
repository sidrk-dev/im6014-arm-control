"""Unit tests for single-motor J1 manual arm backend.

Tests file-only authored. Validates lifecycle, wire history, mapping,
authentication, single-use tickets, sequence ordering, device change,
pre-write Stop, queue bounds, teardown fault, expiry loss, limits, and
virtual vs physical isolation.
"""

import math
import time
import unittest

from arm_manual import (
    ArmManualSession,
    DEFAULT_SETTINGS,
    DemoSerial,
    FEEDBACK_MAX_AGE_S,
    INGRESS_LEASE_S,
    MAX_ACCEL_RAD_S2,
    MAX_MEASURED_LEAD_RAD,
    MAX_MEASURED_SPEED_RPM,
    MAX_QUEUE,
    MAX_RATE_RPM,
    MAX_RECEIPTS_PER_SESSION,
    MAX_SESSIONS,
    MAX_TICKETS,
    SCHEDULE_GAP_MAX_S,
    TICKET_EXPIRY_S,
)
from im6014_codec import command, crc32, decode_command, feedback, position_to_raw


class ManualClock:

    def __init__(self, start=1000.0):
        self._now = float(start)

    def __call__(self):
        return self._now

    def advance(self, dt):
        self._now += float(dt)
        return self._now


class FakeBus:

    is_open = True
    in_waiting = 26
    timeout = 0.005
    write_timeout = 0.05

    def __init__(self, responses=None, clock=time.monotonic):
        self.clock = clock
        self.responses = list(responses or [])
        self.writes = []
        self._read_buf = b''
        self.fail_close = False

    def reset_input_buffer(self):
        self._read_buf = b''

    def write(self, packet):
        self.writes.append(bytes(packet))
        if self.responses:
            self._read_buf = self.responses.pop(0)
        return len(packet)

    def read(self, size):
        data = self._read_buf[:size]
        self._read_buf = self._read_buf[size:]
        return data

    def close(self):
        if self.fail_close:
            raise OSError('Injected serial close failure')
        self.is_open = False


def _wait_receipt(session, sid, op_id, timeout=2.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        r = session.receipt(sid, op_id)
        if r['status'] in ('complete', 'rejected'):
            return r
        time.sleep(0.01)
    raise TimeoutError(f'Operation {op_id} did not complete within {timeout}s')


class TestArmManualLifecycle(unittest.TestCase):

    def test_lifecycle(self):
        clk = ManualClock(100.0)
        session = ArmManualSession(physical_opt_in=False, clock=clk)
        try:
            snap = session.snapshot()
            self.assertEqual(snap['state'], 'Disconnected')
            self.assertFalse(snap['connected'])
            self.assertFalse(snap['armed'])

            sid = session.create_session()
            self.assertIsInstance(sid, str)
            self.assertTrue(len(sid) >= 16)

            op = session.request(sid, 'connect', {'demo': True, 'port': '', 'baud': 6000000, 'id': 0})
            r = _wait_receipt(session, sid, op)
            self.assertEqual(r['status'], 'complete')

            snap = session.snapshot()
            self.assertTrue(snap['connected'])
            self.assertTrue(snap['demo'])
            self.assertEqual(snap['state'], 'Timeout — disarmed')

            op = session.request(sid, 'ack', {'supported': True})
            r = _wait_receipt(session, sid, op)
            self.assertEqual(r['status'], 'complete')

            snap = session.snapshot()
            self.assertEqual(snap['state'], 'Ready · Stop')

            op = session.request(
                sid,
                'calibrate',
                {'known_angle_deg': 0.0, 'direction': 1, 'lower_deg': -90.0, 'upper_deg': 90.0},
            )
            r = _wait_receipt(session, sid, op)
            self.assertEqual(r['status'], 'complete')

            snap = session.snapshot()
            self.assertTrue(snap['calibrated'])

            op = session.request(
                sid,
                'configure',
                {'kp': 3.0, 'kd': 0.05, 'max_rpm': 0.5, 'effort_limit_nm': 1.0},
            )
            r = _wait_receipt(session, sid, op)
            self.assertEqual(r['status'], 'complete')

            op = session.request(sid, 'check', {})
            self.assertEqual(_wait_receipt(session, sid, op)['status'], 'complete')

            op = session.request(sid, 'arm', {'supported': True})
            r = _wait_receipt(session, sid, op)
            self.assertEqual(r['status'], 'complete')

            snap = session.snapshot()
            self.assertTrue(snap['armed'])
            self.assertEqual(snap['state'], 'Armed · motor stopped')

            t1 = session.issue_ticket(sid)
            session.submit(sid, t1, seq=1, deadman=False, axes=[0.0, 0.0, 0.0], device='kb1')

            t2 = session.issue_ticket(sid)
            session.submit(sid, t2, seq=2, deadman=True, axes=[0.5, 0.0, 0.0], device='kb1')

            time.sleep(0.05)
            snap = session.snapshot()
            self.assertEqual(snap['state'], 'Jogging')

            session.stop(sid, 'Operator manual pause')
            snap = session.snapshot()
            self.assertFalse(snap['armed'])
            self.assertEqual(snap['state'], 'Stopped · re-check required')

            op = session.request(sid, 'disconnect', {})
            r = _wait_receipt(session, sid, op)
            self.assertEqual(r['status'], 'complete')

            retired = session.retire_session(sid)
            self.assertTrue(retired)

            disconnected = session.snapshot()
            self.assertFalse(disconnected['connected'])
            self.assertFalse(disconnected['stop_confirmed'])
            self.assertIsNone(disconnected['latest'])
            self.assertIsNone(disconnected['joints'][0]['mode'])
            self.assertIsNone(disconnected['joints'][0]['position_rad'])

            closed = session.close()
            self.assertTrue(closed)
        finally:
            session.close()


class TestArmManualWireHistory(unittest.TestCase):

    def test_actual_wire_history(self):
        clk = ManualClock(100.0)
        session = ArmManualSession(physical_opt_in=False, clock=clk)
        try:
            sid = session.create_session()
            op = session.request(sid, 'connect', {'demo': True, 'port': '', 'baud': 6000000, 'id': 0})
            _wait_receipt(session, sid, op)

            op = session.request(sid, 'ack', {'supported': True})
            _wait_receipt(session, sid, op)

            op = session.request(
                sid,
                'calibrate',
                {'known_angle_deg': 0.0, 'direction': 1, 'lower_deg': -90.0, 'upper_deg': 90.0},
            )
            _wait_receipt(session, sid, op)

            op = session.request(sid, 'check', {})
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'arm', {'supported': True})
            _wait_receipt(session, sid, op)

            inconsistent = session.issue_ticket(sid)
            with self.assertRaisesRegex(ValueError, 'Motion intent'):
                session.submit(sid, inconsistent, seq=1, deadman=False, axes=[0.2, 0.0, 0.0], device='dev1')
            pre_neutral = session.issue_ticket(sid)
            with self.assertRaisesRegex(ValueError, 'Neutral input required'):
                session.submit(sid, pre_neutral, seq=1, deadman=True, axes=[0.2, 0.0, 0.0], device='dev1')
            self.assertTrue(session.require_neutral)
            self.assertFalse(session.deadman_active)

            t1 = session.issue_ticket(sid)
            session.submit(sid, t1, seq=2, deadman=False, axes=[0.0, 0.0, 0.0], device='dev1')
            self.assertFalse(session.require_neutral)
            t2 = session.issue_ticket(sid)
            session.submit(sid, t2, seq=3, deadman=True, axes=[0.2, 0.0, 0.0], device='dev1')

            time.sleep(0.15)

            capture = session.packet_capture.snapshot()
            records = capture['records']
            self.assertTrue(len(records) >= 3)

            for r in records:
                req_bytes = bytes.fromhex(r['request_hex'])
                self.assertEqual(req_bytes[:2], b'\xfe\xee')
                self.assertEqual(req_bytes[2] & 0x0F, 0)
                decoded = decode_command(req_bytes)
                self.assertEqual(decoded['motor_id'], 0)

                reply_bytes = bytes.fromhex(r['reply_hex'])
                self.assertEqual(reply_bytes[:2], b'\xfc\xee')
                fb = feedback(reply_bytes, 0)
                self.assertEqual(fb['id'], 0)

            mode1_records = [r for r in records if (bytes.fromhex(r['request_hex'])[2] >> 4) & 7 == 1]
            self.assertTrue(len(mode1_records) >= 2)
            first_foc = mode1_records[0]
            first_decoded = decode_command(bytes.fromhex(first_foc['request_hex']))
            self.assertEqual(first_decoded['raw_kp'], 0)
            self.assertEqual(first_decoded['raw_kd'], 0)
            self.assertEqual(first_decoded['raw_rpm'], 0)
            self.assertTrue(all(bytes.fromhex(record['request_hex'])[2] & 0x80
                                for record in mode1_records))
            self.assertTrue(any(
                decode_command(bytes.fromhex(record['request_hex']))['raw_kp'] > 0
                for record in mode1_records[2:]
            ), 'A live deadman press must emit a PD jog packet after FOC entry')
            jog_positions = [
                decode_command(bytes.fromhex(record['request_hex']))['raw_position']
                for record in mode1_records[2:]
            ]
            self.assertTrue(any(position != first_decoded['raw_position']
                                for position in jog_positions))

            session.close()
        finally:
            session.close()


class TestArmManualMapping(unittest.TestCase):

    def test_multi_turn_relative_zero_direction_positive(self):
        clk = ManualClock(100.0)
        session = ArmManualSession(physical_opt_in=False, clock=clk)
        try:
            sid = session.create_session()
            op = session.request(sid, 'connect', {'demo': True, 'port': '', 'baud': 6000000, 'id': 0})
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'ack', {'supported': True})
            _wait_receipt(session, sid, op)

            session.bus.states[0]['pos'] = 150.0

            known_deg = 30.0
            known_rad = math.radians(known_deg)
            op = session.request(
                sid,
                'calibrate',
                {'known_angle_deg': known_deg, 'direction': 1, 'lower_deg': -90.0, 'upper_deg': 90.0},
            )
            _wait_receipt(session, sid, op)

            expected_zero = 150.0 - known_rad
            self.assertAlmostEqual(session.q_zero, expected_zero, places=4)

            snap = session.snapshot()
            self.assertAlmostEqual(snap['joints'][0]['position_rad'], known_rad, places=4)
            self.assertAlmostEqual(snap['latest']['position_rad'], 150.0, places=4)
        finally:
            session.close()

    def test_direction_negative(self):
        clk = ManualClock(100.0)
        session = ArmManualSession(physical_opt_in=False, clock=clk)
        try:
            sid = session.create_session()
            op = session.request(sid, 'connect', {'demo': True, 'port': '', 'baud': 6000000, 'id': 0})
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'ack', {'supported': True})
            _wait_receipt(session, sid, op)

            session.bus.states[0]['pos'] = 50.0

            known_deg = -15.0
            known_rad = math.radians(known_deg)
            op = session.request(
                sid,
                'calibrate',
                {'known_angle_deg': known_deg, 'direction': -1, 'lower_deg': -90.0, 'upper_deg': 90.0},
            )
            _wait_receipt(session, sid, op)

            expected_zero = 50.0 - (-1) * known_rad
            self.assertAlmostEqual(session.q_zero, expected_zero, places=4)

            snap = session.snapshot()
            self.assertAlmostEqual(snap['joints'][0]['position_rad'], known_rad, places=4)
        finally:
            session.close()

    def test_invalid_calibration_bounds(self):
        clk = ManualClock(100.0)
        session = ArmManualSession(physical_opt_in=False, clock=clk)
        try:
            sid = session.create_session()
            op = session.request(sid, 'connect', {'demo': True, 'port': '', 'baud': 6000000, 'id': 0})
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'ack', {'supported': True})
            _wait_receipt(session, sid, op)

            op = session.request(
                sid,
                'calibrate',
                {'known_angle_deg': 0.0, 'direction': 1, 'lower_deg': -95.0, 'upper_deg': 90.0},
            )
            r = _wait_receipt(session, sid, op)
            self.assertEqual(r['status'], 'rejected')

            op = session.request(
                sid,
                'calibrate',
                {'known_angle_deg': 0.0, 'direction': 1, 'lower_deg': -90.0, 'upper_deg': 95.0},
            )
            r = _wait_receipt(session, sid, op)
            self.assertEqual(r['status'], 'rejected')

            op = session.request(
                sid,
                'calibrate',
                {'known_angle_deg': 0.0, 'direction': 1, 'lower_deg': 50.0, 'upper_deg': 40.0},
            )
            r = _wait_receipt(session, sid, op)
            self.assertEqual(r['status'], 'rejected')

            op = session.request(
                sid,
                'calibrate',
                {'known_angle_deg': 60.0, 'direction': 1, 'lower_deg': -45.0, 'upper_deg': 45.0},
            )
            r = _wait_receipt(session, sid, op)
            self.assertEqual(r['status'], 'rejected')

            op = session.request(
                sid,
                'calibrate',
                {'known_angle_deg': 0.0, 'direction': 0, 'lower_deg': -45.0, 'upper_deg': 45.0},
            )
            r = _wait_receipt(session, sid, op)
            self.assertEqual(r['status'], 'rejected')
        finally:
            session.close()


class TestArmManualUnknownSession(unittest.TestCase):

    def test_unknown_session_calls(self):
        clk = ManualClock(100.0)
        session = ArmManualSession(physical_opt_in=False, clock=clk)
        try:
            bad_sid = 'nonexistent-session-token'
            with self.assertRaises(ValueError):
                session.request(bad_sid, 'check', {})
            with self.assertRaises(ValueError):
                session.receipt(bad_sid, 'op1')
            with self.assertRaises(ValueError):
                session.issue_ticket(bad_sid)
            with self.assertRaises(ValueError):
                session.submit(bad_sid, 'ticket', 1, False, [0, 0, 0], 'dev')
            with self.assertRaises(ValueError):
                session.stop(bad_sid, 'Stop')
            self.assertFalse(session.retire_session(bad_sid))
        finally:
            session.close()

    def test_demo_and_physical_opt_in_require_exact_bools(self):
        with self.assertRaises(ValueError):
            ArmManualSession(physical_opt_in=1)
        session = ArmManualSession(physical_opt_in=False)
        try:
            sid = session.create_session()
            op = session.request(sid, 'connect', {'demo': 1, 'port': '', 'baud': 6000000, 'id': 0})
            result = _wait_receipt(session, sid, op)
            self.assertEqual(result['status'], 'rejected')
            self.assertIn('exact bool', result['reason'])
        finally:
            session.close()

    def test_observer_cannot_modify_owner(self):
        clk = ManualClock(100.0)
        session = ArmManualSession(physical_opt_in=False, clock=clk)
        try:
            owner_sid = session.create_session()
            obs_sid = session.create_session()

            op = session.request(owner_sid, 'connect', {'demo': True, 'port': '', 'baud': 6000000, 'id': 0})
            _wait_receipt(session, owner_sid, op)
            op = session.request(owner_sid, 'ack', {'supported': True})
            _wait_receipt(session, owner_sid, op)
            op = session.request(
                owner_sid,
                'calibrate',
                {'known_angle_deg': 0.0, 'direction': 1, 'lower_deg': -90.0, 'upper_deg': 90.0},
            )
            _wait_receipt(session, owner_sid, op)
            op = session.request(owner_sid, 'check', {})
            _wait_receipt(session, owner_sid, op)
            op = session.request(owner_sid, 'arm', {'supported': True})
            _wait_receipt(session, owner_sid, op)

            self.assertFalse(session.retire_session(owner_sid),
                             'A connected owner cannot retire and release its lease')

            observer_actions = (
                ('disconnect', {}), ('check', {}), ('ack', {'supported': True}),
                ('calibrate', {
                    'known_angle_deg': 0.0, 'direction': 1,
                    'lower_deg': -90.0, 'upper_deg': 90.0,
                }),
                ('configure', {
                    'kp': 5.0, 'kd': 0.1, 'max_rpm': 0.5,
                    'effort_limit_nm': 1.0,
                }),
                ('arm', {'supported': True}), ('release', {}),
            )
            for action, values in observer_actions:
                op = session.request(obs_sid, action, values)
                self.assertEqual(session.receipt(obs_sid, op)['status'], 'rejected', action)

            op = session.request(
                obs_sid,
                'configure',
                {'kp': 5.0, 'kd': 0.1, 'max_rpm': 0.5, 'effort_limit_nm': 1.0},
            )
            r = session.receipt(obs_sid, op)
            self.assertEqual(r['status'], 'rejected')

            with self.assertRaises(ValueError):
                session.issue_ticket(obs_sid)

            t_owner = session.issue_ticket(owner_sid)
            with self.assertRaises(ValueError):
                session.submit(obs_sid, t_owner, 1, True, [0, 0, 0], 'dev')
            self.assertTrue(session.retire_session(obs_sid))
        finally:
            session.close()


class TestArmManualTicketsAndDevice(unittest.TestCase):

    def test_ticketed_connect_can_reserve_initial_owner(self):
        clk = ManualClock(100.0)
        session = ArmManualSession(physical_opt_in=False, clock=clk)
        try:
            sid = session.create_session()
            ticket = session.issue_ticket(sid)
            op = session.request(
                sid, 'connect',
                {'demo': True, 'port': '', 'baud': 6000000, 'id': 0},
                ticket=ticket,
            )
            self.assertEqual(_wait_receipt(session, sid, op)['status'], 'complete')
            self.assertTrue(session.snapshot()['connected'])
            replay = session.request(
                sid, 'connect',
                {'demo': True, 'port': '', 'baud': 6000000, 'id': 0},
                ticket=ticket,
            )
            self.assertEqual(session.receipt(sid, replay)['status'], 'rejected')
        finally:
            session.close()

    def test_ticket_replay_and_stale(self):
        clk = ManualClock(100.0)
        session = ArmManualSession(physical_opt_in=False, clock=clk)
        try:
            sid = session.create_session()
            op = session.request(sid, 'connect', {'demo': True, 'port': '', 'baud': 6000000, 'id': 0})
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'ack', {'supported': True})
            _wait_receipt(session, sid, op)
            op = session.request(
                sid,
                'calibrate',
                {'known_angle_deg': 0.0, 'direction': 1, 'lower_deg': -90.0, 'upper_deg': 90.0},
            )
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'check', {})
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'arm', {'supported': True})
            _wait_receipt(session, sid, op)

            t1 = session.issue_ticket(sid)
            session.submit(sid, t1, seq=1, deadman=False, axes=[0, 0, 0], device='dev1')

            with self.assertRaises(ValueError):
                session.submit(sid, t1, seq=2, deadman=False, axes=[0, 0, 0], device='dev1')

            t2 = session.issue_ticket(sid)
            clk.advance(0.251)
            with self.assertRaises(ValueError):
                session.submit(sid, t2, seq=3, deadman=False, axes=[0, 0, 0], device='dev1')
        finally:
            session.close()

    def test_ticketed_action_is_single_use_and_expires_at_issuance(self):
        clk = ManualClock(100.0)
        session = ArmManualSession(physical_opt_in=False, clock=clk)
        try:
            sid = session.create_session()
            op = session.request(sid, 'connect', {'demo': True, 'port': '', 'baud': 6000000, 'id': 0})
            _wait_receipt(session, sid, op)

            ticket = session.issue_ticket(sid)
            op = session.request(sid, 'check', {}, ticket=ticket)
            self.assertEqual(_wait_receipt(session, sid, op)['status'], 'complete')
            replay = session.request(sid, 'check', {}, ticket=ticket)
            self.assertEqual(session.receipt(sid, replay)['status'], 'rejected')

            expired = session.issue_ticket(sid)
            clk.advance(TICKET_EXPIRY_S + 0.001)
            late = session.request(sid, 'check', {}, ticket=expired)
            self.assertEqual(session.receipt(sid, late)['status'], 'rejected')
            self.assertIn('expired', session.receipt(sid, late)['reason'].lower())
        finally:
            session.close()

    def test_ticketed_action_cannot_gain_time_while_queued(self):
        clk = ManualClock(100.0)
        session = ArmManualSession(physical_opt_in=False, clock=clk)
        try:
            sid = session.create_session()
            op = session.request(sid, 'connect', {'demo': True, 'port': '', 'baud': 6000000, 'id': 0})
            _wait_receipt(session, sid, op)

            with session.lock:
                ticket = session.issue_ticket(sid)
                delayed = session.request(sid, 'check', {}, ticket=ticket)
                self.assertEqual(session.receipt(sid, delayed)['status'], 'pending')
                clk.advance(TICKET_EXPIRY_S + 0.001)

            result = _wait_receipt(session, sid, delayed)
            self.assertEqual(result['status'], 'rejected')
            self.assertIn('expired', result['reason'].lower())
        finally:
            session.close()

    def test_stop_rejects_queued_pre_stop_action_ticket(self):
        clk = ManualClock(100.0)
        session = ArmManualSession(physical_opt_in=False, clock=clk)
        try:
            sid = session.create_session()
            op = session.request(sid, 'connect', {'demo': True, 'port': '', 'baud': 6000000, 'id': 0})
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'ack', {'supported': True})
            _wait_receipt(session, sid, op)
            op = session.request(
                sid, 'calibrate', {
                    'known_angle_deg': 0.0, 'direction': 1,
                    'lower_deg': -90.0, 'upper_deg': 90.0,
                },
            )
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'check', {})
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'arm', {'supported': True})
            _wait_receipt(session, sid, op)

            with session.lock:
                ticket = session.issue_ticket(sid)
                delayed = session.request(sid, 'check', {}, ticket=ticket)
                self.assertEqual(session.receipt(sid, delayed)['status'], 'pending')
                session.stop(sid, 'Stop before queued action dispatch')

            result = _wait_receipt(session, sid, delayed)
            self.assertEqual(result['status'], 'rejected')
            self.assertIn('cancelled', result['reason'].lower())
            replay = session.request(sid, 'check', {}, ticket=ticket)
            self.assertEqual(session.receipt(sid, replay)['status'], 'rejected')
            self.assertFalse(session.snapshot()['armed'])
        finally:
            session.close()

    def test_ticket_pool_is_bounded_and_prunes_expired(self):
        clk = ManualClock(100.0)
        session = ArmManualSession(physical_opt_in=False, clock=clk)
        try:
            sid = session.create_session()
            op = session.request(sid, 'connect', {'demo': True, 'port': '', 'baud': 6000000, 'id': 0})
            _wait_receipt(session, sid, op)
            tickets = [session.issue_ticket(sid) for _ in range(MAX_TICKETS)]
            with self.assertRaises(RuntimeError):
                session.issue_ticket(sid)
            clk.advance(TICKET_EXPIRY_S + 0.001)
            self.assertIsInstance(session.issue_ticket(sid), str)
            self.assertEqual(len(session.tickets), 1)
            self.assertTrue(tickets)
        finally:
            session.close()

    def test_sequence_must_be_increasing(self):
        clk = ManualClock(100.0)
        session = ArmManualSession(physical_opt_in=False, clock=clk)
        try:
            sid = session.create_session()
            op = session.request(sid, 'connect', {'demo': True, 'port': '', 'baud': 6000000, 'id': 0})
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'ack', {'supported': True})
            _wait_receipt(session, sid, op)
            op = session.request(
                sid,
                'calibrate',
                {'known_angle_deg': 0.0, 'direction': 1, 'lower_deg': -90.0, 'upper_deg': 90.0},
            )
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'check', {})
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'arm', {'supported': True})
            _wait_receipt(session, sid, op)

            t1 = session.issue_ticket(sid)
            session.submit(sid, t1, seq=5, deadman=False, axes=[0, 0, 0], device='dev1')

            t2 = session.issue_ticket(sid)
            with self.assertRaises(ValueError):
                session.submit(sid, t2, seq=5, deadman=False, axes=[0, 0, 0], device='dev1')

            t3 = session.issue_ticket(sid)
            with self.assertRaises(ValueError):
                session.submit(sid, t3, seq=4, deadman=False, axes=[0, 0, 0], device='dev1')
        finally:
            session.close()

    def test_device_change_drops_authority(self):
        clk = ManualClock(100.0)
        session = ArmManualSession(physical_opt_in=False, clock=clk)
        try:
            sid = session.create_session()
            op = session.request(sid, 'connect', {'demo': True, 'port': '', 'baud': 6000000, 'id': 0})
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'ack', {'supported': True})
            _wait_receipt(session, sid, op)
            op = session.request(
                sid,
                'calibrate',
                {'known_angle_deg': 0.0, 'direction': 1, 'lower_deg': -90.0, 'upper_deg': 90.0},
            )
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'check', {})
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'arm', {'supported': True})
            _wait_receipt(session, sid, op)

            t1 = session.issue_ticket(sid)
            session.submit(sid, t1, seq=1, deadman=False, axes=[0, 0, 0], device='kb1')

            t2 = session.issue_ticket(sid)
            session.submit(sid, t2, seq=2, deadman=True, axes=[0.2, 0, 0], device='kb1')

            t3 = session.issue_ticket(sid)
            session.submit(sid, t3, seq=3, deadman=True, axes=[0.2, 0, 0], device='xbox1')

            snap = session.snapshot()
            self.assertFalse(snap['armed'])
            self.assertEqual(snap['state'], 'Stopped · re-check required')
        finally:
            session.close()


class TestArmManualStopBeforeWrite(unittest.TestCase):

    def test_stop_cancels_mode1(self):
        clk = ManualClock(100.0)
        session = ArmManualSession(physical_opt_in=False, clock=clk)
        try:
            sid = session.create_session()
            op = session.request(sid, 'connect', {'demo': True, 'port': '', 'baud': 6000000, 'id': 0})
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'ack', {'supported': True})
            _wait_receipt(session, sid, op)
            op = session.request(
                sid,
                'calibrate',
                {'known_angle_deg': 0.0, 'direction': 1, 'lower_deg': -90.0, 'upper_deg': 90.0},
            )
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'check', {})
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'arm', {'supported': True})
            _wait_receipt(session, sid, op)

            session.stop(sid, 'Immediate E-stop')
            snap = session.snapshot()
            self.assertFalse(snap['armed'])
            self.assertTrue(session.stop_requested.is_set())
        finally:
            session.close()


class TestArmManualQueueBounds(unittest.TestCase):

    def test_sessions_bounded_16(self):
        session = ArmManualSession(physical_opt_in=False)
        try:
            sids = [session.create_session() for _ in range(MAX_SESSIONS)]
            self.assertEqual(len(sids), 16)
            with self.assertRaises(RuntimeError):
                session.create_session()
        finally:
            session.close()

    def test_request_queue_bounded_64(self):
        session = ArmManualSession(physical_opt_in=False)
        try:
            sid = session.create_session()
            with session.lock:
                for i in range(MAX_QUEUE):
                    session.jobs.put((0, sid, f'op_{i}', 'check', {}))
                op_overflow = session.request(sid, 'check', {})
                r = session.receipt(sid, op_overflow)
                self.assertEqual(r['status'], 'rejected')
                self.assertEqual(r['reason'], 'Queue full')
        finally:
            session.close()

    def test_receipts_per_session_bounded_256(self):
        session = ArmManualSession(physical_opt_in=False)
        try:
            sid = session.create_session()
            for i in range(300):
                session._record_receipt(sid, f'op_{i}', 'complete', '')
            self.assertEqual(len(session.receipts[sid]), MAX_RECEIPTS_PER_SESSION)
        finally:
            session.close()


class TestArmManualTeardownFault(unittest.TestCase):

    def test_teardown_close_failure(self):
        clk = ManualClock(100.0)
        session = ArmManualSession(physical_opt_in=False, clock=clk)
        try:
            sid = session.create_session()
            op = session.request(sid, 'connect', {'demo': True, 'port': '', 'baud': 6000000, 'id': 0})
            _wait_receipt(session, sid, op)

            session.bus.fail_close = True

            op = session.request(sid, 'disconnect', {})
            result = _wait_receipt(session, sid, op)
            self.assertEqual(result['status'], 'rejected')

            snap = session.snapshot()
            self.assertTrue(snap['connected'])
            self.assertIsNotNone(session.bus)
            self.assertEqual(snap['state'], 'Fault — serial close unconfirmed')

            session.bus.fail_close = False
            op = session.request(sid, 'disconnect', {})
            self.assertEqual(_wait_receipt(session, sid, op)['status'], 'complete')
            self.assertFalse(session.snapshot()['connected'])
        finally:
            session.close()


class TestArmManualExpiryAndLoss(unittest.TestCase):

    def test_ingress_lease_loss(self):
        clk = ManualClock(100.0)
        session = ArmManualSession(physical_opt_in=False, clock=clk)
        try:
            sid = session.create_session()
            op = session.request(sid, 'connect', {'demo': True, 'port': '', 'baud': 6000000, 'id': 0})
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'ack', {'supported': True})
            _wait_receipt(session, sid, op)
            op = session.request(
                sid,
                'calibrate',
                {'known_angle_deg': 0.0, 'direction': 1, 'lower_deg': -90.0, 'upper_deg': 90.0},
            )
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'check', {})
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'arm', {'supported': True})
            _wait_receipt(session, sid, op)

            t1 = session.issue_ticket(sid)
            session.submit(sid, t1, seq=1, deadman=False, axes=[0, 0, 0], device='dev1')
            t2 = session.issue_ticket(sid)
            session.submit(sid, t2, seq=2, deadman=True, axes=[0.1, 0, 0], device='dev1')

            time.sleep(0.03)
            clk.advance(0.300)
            time.sleep(0.03)

            snap = session.snapshot()
            self.assertFalse(snap['armed'])
            self.assertEqual(snap['state'], 'Fault — disarmed')
            self.assertIn('Feedback stale', snap['reason'])
        finally:
            session.close()

    def test_stale_feedback_fault(self):
        clk = ManualClock(100.0)
        session = ArmManualSession(physical_opt_in=False, clock=clk)
        try:
            sid = session.create_session()
            op = session.request(sid, 'connect', {'demo': True, 'port': '', 'baud': 6000000, 'id': 0})
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'ack', {'supported': True})
            _wait_receipt(session, sid, op)
            op = session.request(
                sid,
                'calibrate',
                {'known_angle_deg': 0.0, 'direction': 1, 'lower_deg': -90.0, 'upper_deg': 90.0},
            )
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'check', {})
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'arm', {'supported': True})
            _wait_receipt(session, sid, op)

            clk.advance(0.150)
            time.sleep(0.03)

            snap = session.snapshot()
            self.assertTrue(snap['state'].startswith('Fault'))
        finally:
            session.close()


class TestArmManualLimitsAndGains(unittest.TestCase):

    def test_configure_validation(self):
        clk = ManualClock(100.0)
        session = ArmManualSession(physical_opt_in=False, clock=clk)
        try:
            sid = session.create_session()
            op = session.request(sid, 'connect', {'demo': True, 'port': '', 'baud': 6000000, 'id': 0})
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'ack', {'supported': True})
            _wait_receipt(session, sid, op)

            op = session.request(
                sid,
                'configure',
                {'kp': 0.0, 'kd': 0.05, 'max_rpm': 0.5, 'effort_limit_nm': 1.0},
            )
            r = _wait_receipt(session, sid, op)
            self.assertEqual(r['status'], 'rejected')

            op = session.request(
                sid,
                'configure',
                {'kp': 35.0, 'kd': 0.05, 'max_rpm': 0.5, 'effort_limit_nm': 1.0},
            )
            r = _wait_receipt(session, sid, op)
            self.assertEqual(r['status'], 'rejected')

            op = session.request(
                sid,
                'configure',
                {'kp': 3.0, 'kd': 7.0, 'max_rpm': 0.5, 'effort_limit_nm': 1.0},
            )
            r = _wait_receipt(session, sid, op)
            self.assertEqual(r['status'], 'rejected')

            op = session.request(
                sid,
                'configure',
                {'kp': 3.0, 'kd': 0.05, 'max_rpm': 100.1, 'effort_limit_nm': 1.0},
            )
            r = _wait_receipt(session, sid, op)
            self.assertEqual(r['status'], 'rejected')

            op = session.request(
                sid,
                'configure',
                {'kp': 3.0, 'kd': 0.05, 'max_rpm': 0.5, 'effort_limit_nm': 15.1},
            )
            r = _wait_receipt(session, sid, op)
            self.assertEqual(r['status'], 'rejected')
        finally:
            session.close()

    def test_overload_exposure_retained(self):
        clk = ManualClock(100.0)
        session = ArmManualSession(physical_opt_in=False, clock=clk)
        try:
            sid = session.create_session()
            op = session.request(sid, 'connect', {'demo': True, 'port': '', 'baud': 6000000, 'id': 0})
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'ack', {'supported': True})
            _wait_receipt(session, sid, op)
            op = session.request(
                sid,
                'calibrate',
                {'known_angle_deg': 0.0, 'direction': 1, 'lower_deg': -90.0, 'upper_deg': 90.0},
            )
            _wait_receipt(session, sid, op)

            session._record_overload('Simulated severe overload')

            op = session.request(sid, 'arm', {'supported': True})
            r = _wait_receipt(session, sid, op)
            self.assertEqual(r['status'], 'rejected')
            self.assertIn('Check again', r['reason'])
            self.assertEqual(session.protection_fault, 'Simulated severe overload')

            op = session.request(sid, 'disconnect', {})
            _wait_receipt(session, sid, op)

            op = session.request(sid, 'connect', {'demo': True, 'port': '', 'baud': 6000000, 'id': 0})
            _wait_receipt(session, sid, op)

            self.assertIsNotNone(session.protection_fault)
            self.assertTrue(len(session.protection_history) > 0)
        finally:
            session.close()


class TestArmManualVirtualVsPhysical(unittest.TestCase):

    def test_physical_without_opt_in_rejected(self):
        session = ArmManualSession(physical_opt_in=False)
        try:
            sid = session.create_session()
            op = session.request(
                sid,
                'connect',
                {'demo': False, 'port': '/dev/ttyUSB0', 'baud': 6000000, 'id': 0},
            )
            r = _wait_receipt(session, sid, op)
            self.assertEqual(r['status'], 'rejected')
            self.assertIn('physical_opt_in', r['reason'])
        finally:
            session.close()

    def test_physical_invalid_id_or_baud_rejected(self):
        session = ArmManualSession(physical_opt_in=True)
        try:
            sid = session.create_session()
            op = session.request(
                sid,
                'connect',
                {'demo': False, 'port': '/dev/ttyUSB0', 'baud': 6000000, 'id': 1},
            )
            r = _wait_receipt(session, sid, op)
            self.assertEqual(r['status'], 'rejected')

            op = session.request(
                sid,
                'connect',
                {'demo': False, 'port': '/dev/ttyUSB0', 'baud': 4000000, 'id': 0},
            )
            r = _wait_receipt(session, sid, op)
            self.assertEqual(r['status'], 'rejected')
        finally:
            session.close()

    def test_virtual_joints_in_snapshot(self):
        clk = ManualClock(100.0)
        session = ArmManualSession(physical_opt_in=False, clock=clk)
        try:
            sid = session.create_session()
            op = session.request(sid, 'connect', {'demo': True, 'port': '', 'baud': 6000000, 'id': 0})
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'ack', {'supported': True})
            _wait_receipt(session, sid, op)
            op = session.request(
                sid,
                'calibrate',
                {'known_angle_deg': 0.0, 'direction': 1, 'lower_deg': -90.0, 'upper_deg': 90.0},
            )
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'check', {})
            _wait_receipt(session, sid, op)
            op = session.request(sid, 'arm', {'supported': True})
            _wait_receipt(session, sid, op)

            t1 = session.issue_ticket(sid)
            session.submit(sid, t1, seq=1, deadman=False, axes=[0, 0, 0], device='dev1')

            t2 = session.issue_ticket(sid)
            session.submit(sid, t2, seq=2, deadman=True, axes=[0.0, 0.5, -0.5], device='dev1')

            time.sleep(0.05)
            snap = session.snapshot()
            joints = snap['joints']
            self.assertEqual(len(joints), 3)

            self.assertEqual(joints[0]['name'], 'J1')
            self.assertIn('voltage', joints[0])
            self.assertIn('driver_temp', joints[0])
            self.assertIn('winding_temp', joints[0])
            self.assertIn('errors', joints[0])
            self.assertIn('warnings', joints[0])
            self.assertIn('torque_nm', joints[0])

            self.assertEqual(joints[1]['name'], 'J2')
            self.assertEqual(joints[1]['source'], 'virtual')
            self.assertNotIn('voltage', joints[1])
            self.assertNotIn('errors', joints[1])

            self.assertEqual(joints[2]['name'], 'J3')
            self.assertEqual(joints[2]['source'], 'virtual')
            self.assertNotIn('voltage', joints[2])
            self.assertNotIn('errors', joints[2])
        finally:
            session.close()


if __name__ == '__main__':
    unittest.main()


class TestObservedMotorTransitions(unittest.TestCase):
    def test_immediate_foc_reply_still_requires_zero_effort_entry(self):
        from unittest.mock import patch
        import struct

        class ImmediateModeSerial(DemoSerial):
            def write(self, packet):
                count = super().write(packet)
                if self.data:
                    body = bytearray(self.data[2:-4])
                    body[0] = (body[0] & 0x8F) | (packet[2] & 0x70)
                    self.data = b'\xfc\xee' + body + struct.pack('<I', crc32(body))
                return count

        with patch('arm_manual.DemoSerial', ImmediateModeSerial):
            TestArmManualWireHistory.test_actual_wire_history(self)

    def test_owner_observation_does_not_grant_observer_authority(self):
        session = ArmManualSession(physical_opt_in=False)
        try:
            owner, observer = session.create_session(), session.create_session()
            self.assertFalse(session.controls_motor(owner))
            op = session.request(owner, 'connect', {'demo': True, 'port': '', 'baud': 6000000, 'id': 0})
            self.assertEqual(_wait_receipt(session, owner, op)['status'], 'complete')
            self.assertTrue(session.controls_motor(owner))
            self.assertFalse(session.controls_motor(observer))
            self.assertEqual(session.input_sequence(observer), -1)
            with self.assertRaises(ValueError):
                session.controls_motor('unknown')
        finally:
            session.close()
