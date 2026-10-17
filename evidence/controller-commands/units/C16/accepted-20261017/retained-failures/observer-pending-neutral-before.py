"""Actual owner lifecycle under fake dashboard failures; no physical I/O."""

from contextlib import ExitStack
import http.client
import json
import threading
import time
import unittest
from unittest.mock import Mock, patch

import arm_dashboard
from arm_dashboard import ArmDashboardHandler, ArmDashboardServer, run_dashboard
from arm_manual import ArmManualSession, INGRESS_LEASE_S
from im6014_codec import decode_command


class StationIndependenceTests(unittest.TestCase):
    def setUp(self):
        self.guards = ExitStack()
        self.addCleanup(self.guards.close)
        self.serial = self.guards.enter_context(patch(
            'serial.Serial', side_effect=AssertionError('physical serial forbidden')))
        self.ports = self.guards.enter_context(patch(
            'arm_dashboard.enumerate_ports', side_effect=AssertionError('port discovery forbidden')))
        self.guards.enter_context(patch.object(ArmDashboardHandler, '_get_serial_ports', return_value=[]))
        self.managed = self.guards.enter_context(patch(
            'arm_manual.open_managed_port', side_effect=AssertionError('physical owner forbidden')))
        self.arm = ArmManualSession(physical_opt_in=False)
        self.addCleanup(self.arm.close)
        self.sid = self.arm.create_session()

    def tearDown(self):
        self.serial.assert_not_called()
        self.ports.assert_not_called()
        self.managed.assert_not_called()

    def wait_for(self, condition):
        end = time.monotonic() + 2
        while time.monotonic() < end:
            if condition():
                return
            time.sleep(.005)
        self.fail('fake owner did not reach the expected state')

    def action(self, action, values=None):
        ticket = self.arm.issue_ticket(self.sid) if action == 'arm' else None
        op = self.arm.request(self.sid, action, values or {}, ticket=ticket)
        self.wait_for(lambda: self.arm.receipt(self.sid, op)['status'] != 'pending')
        receipt = self.arm.receipt(self.sid, op)
        self.assertEqual(receipt['status'], 'complete', receipt)

    def ready(self, profile='arm_j1'):
        self.action('connect', {'demo': True, 'profile_id': profile})
        self.action('ack', {'supported': True})
        names = ('J1', 'J2', 'J3') if profile == 'arm_j1_j2_j3' else ('J1',)
        for name in names:
            self.action('check')
            self.action('calibrate', {'joint': name, 'known_angle_deg': 0, 'direction': 1,
                                      'lower_deg': -80, 'upper_deg': 80})
        self.action('check')
        self.action('arm', {'supported': True})

    def moving(self, profile='arm_j1'):
        self.ready(profile)
        bus = self.arm.bus
        packets = []
        write = bus.write
        def record(packet):
            packets.append((threading.current_thread(), decode_command(packet)))
            return write(packet)
        bus.write = record
        self.arm.submit(self.sid, self.arm.issue_ticket(self.sid), 1, False, [0, 0, 0], 'keyboard')
        self.arm.submit(self.sid, self.arm.issue_ticket(self.sid), 2, True, [.2, 0, 0], 'keyboard')
        self.wait_for(lambda: any(p['mode'] == 1 for _, p in packets))
        return bus, packets

    def test_bind_failure_closes_the_implicitly_created_owner(self):
        owners = []
        def create(**kwargs):
            owner = ArmManualSession(physical_opt_in=False)
            owners.append(owner)
            return owner
        try:
            with patch.object(arm_dashboard, 'ArmManualSession', side_effect=create), \
                    patch.object(arm_dashboard, 'ArmDashboardServer', side_effect=OSError('fake bind failure')):
                with self.assertRaisesRegex(OSError, 'fake bind failure'):
                    run_dashboard(port=0)
            self.assertEqual(len(owners), 1)
            self.assertFalse(owners[0].thread.is_alive())
            self.assertTrue(owners[0].closed.is_set())
        finally:
            for owner in owners:
                owner.close()

    def test_bind_failure_closes_the_supplied_owner_without_constructing_another(self):
        with patch.object(arm_dashboard, 'ArmManualSession', side_effect=AssertionError('second owner')), \
                patch.object(arm_dashboard, 'ArmDashboardServer', side_effect=OSError('fake bind failure')):
            with self.assertRaisesRegex(OSError, 'fake bind failure'):
                run_dashboard(port=0, session=self.arm)
        self.assertFalse(self.arm.thread.is_alive())

    def test_http_loop_failure_stops_all_three_ids_and_closes_the_actual_owner(self):
        bus, packets = self.moving('arm_j1_j2_j3')
        server = Mock(bound_port=0)
        server.serve_forever.side_effect = RuntimeError('fake dashboard crash')
        with patch.object(arm_dashboard, 'ArmManualSession', side_effect=AssertionError('second owner')), \
                patch.object(arm_dashboard, 'ArmDashboardServer', return_value=server):
            with self.assertRaisesRegex(RuntimeError, 'fake dashboard crash'):
                run_dashboard(port=0, session=self.arm)
        server.server_close.assert_called_once()
        self.assertEqual({p['motor_id'] for _, p in packets if p['mode'] == 0}, {0, 1, 2})
        self.assertFalse(self.arm.thread.is_alive())
        self.assertFalse(bus.is_open)
        self.assertIsNone(self.arm.bus)
        self.assertFalse(self.arm.snapshot()['armed'])
        self.assertEqual(packets[-1][1]['mode'], 0)
        self.assertTrue(all(owner is self.arm.thread for owner, _ in packets))

    def test_server_cleanup_failure_cannot_skip_actual_owner_close(self):
        bus, packets = self.moving()
        server = Mock(bound_port=0)
        server.server_close.side_effect = OSError('fake server cleanup failure')
        with patch.object(arm_dashboard, 'ArmDashboardServer', return_value=server):
            with self.assertRaisesRegex(OSError, 'fake server cleanup failure'):
                run_dashboard(port=0, session=self.arm)
        self.assertFalse(self.arm.thread.is_alive())
        self.assertFalse(bus.is_open)
        self.assertEqual(packets[-1][1]['mode'], 0)

    def test_keyboard_interrupt_uses_the_same_stop_and_close_path(self):
        self.ready()
        bus = self.arm.bus
        server = Mock(bound_port=0)
        server.serve_forever.side_effect = KeyboardInterrupt
        with patch.object(arm_dashboard, 'ArmDashboardServer', return_value=server):
            run_dashboard(port=0, session=self.arm)
        self.assertFalse(self.arm.thread.is_alive())
        self.assertFalse(bus.is_open)
        self.assertFalse(self.arm.snapshot()['armed'])

    def start_http(self):
        server = ArmDashboardServer(('127.0.0.1', 0), self.arm)
        worker = threading.Thread(target=lambda: server.serve_forever(poll_interval=.005))
        worker.start()
        self.addCleanup(worker.join, 1)
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        return server

    def get_state(self, server):
        connection = http.client.HTTPConnection('127.0.0.1', server.bound_port, timeout=1)
        try:
            connection.request('GET', '/api/state')
            response = connection.getresponse()
            self.assertEqual(response.status, 200)
            return json.loads(response.read())
        finally:
            connection.close()

    def test_http_restart_preserves_one_owner_and_cannot_renew_stale_input(self):
        bus, packets = self.moving()
        owner = self.arm.thread
        ticket = self.arm.issue_ticket(self.sid)
        old = self.start_http()
        self.assertTrue(self.get_state(old)['connected'])
        old.shutdown()
        old.server_close()
        self.wait_for(lambda: not self.arm.snapshot()['armed'])
        self.wait_for(lambda: self.arm.snapshot()['stop_confirmed'])
        self.assertTrue(owner.is_alive())
        self.assertIs(self.arm.bus, bus)
        boundary = len(packets)
        with patch.object(arm_dashboard, 'ArmManualSession', side_effect=AssertionError('second owner')):
            new = self.start_http()
            for _ in range(3):
                data = self.get_state(new)
                self.assertTrue(data['connected'])
                self.assertFalse(data['armed'])
            with self.assertRaises(ValueError):
                self.arm.submit(self.sid, ticket, 3, True, [.2, 0, 0], 'keyboard')
            time.sleep(INGRESS_LEASE_S + .02)
            self.assertFalse(self.get_state(new)['armed'])
        self.assertIs(self.arm.thread, owner)
        self.assertTrue(all(thread is owner for thread, _ in packets))
        self.assertTrue(all(packet['mode'] == 0 for _, packet in packets[boundary:]))

    def test_new_http_observer_cannot_inherit_control_or_rearm_stopped_owner(self):
        self.ready()
        ticket = self.arm.issue_ticket(self.sid)
        self.wait_for(lambda: not self.arm.snapshot()['armed'])
        server = self.start_http()
        connection = http.client.HTTPConnection('127.0.0.1', server.bound_port, timeout=1)
        try:
            body = b'{}'
            connection.request('POST', '/api/session', body=body,
                               headers={'Content-Type': 'application/json',
                                        'Origin': f'http://127.0.0.1:{server.bound_port}'})
            response = connection.getresponse()
            self.assertEqual(response.status, 200)
            new_sid = json.loads(response.read())['session']
        finally:
            connection.close()
        self.assertNotEqual(new_sid, self.sid)
        self.assertFalse(self.get_state(server)['armed'])
        with self.assertRaises(ValueError):
            self.arm.submit(new_sid, self.arm.issue_ticket(new_sid), 1, True, [.2, 0, 0], 'keyboard')
        with self.assertRaises(ValueError):
            self.arm.submit(self.sid, ticket, 1, True, [.2, 0, 0], 'keyboard')


if __name__ == '__main__':
    unittest.main()
