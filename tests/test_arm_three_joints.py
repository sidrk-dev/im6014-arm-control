import time
import unittest

from arm_manual import ArmManualSession
from im6014_codec import decode_command


class ThreeJointTests(unittest.TestCase):
    def setUp(self):
        self.arm = ArmManualSession(physical_opt_in=False)
        self.sid = self.arm.create_session()

    def tearDown(self):
        self.arm.close()

    def action(self, name, values=None, expected='complete'):
        ticket = self.arm.issue_ticket(self.sid) if name == 'arm' else None
        op = self.arm.request(self.sid, name, values or {}, ticket=ticket)
        until = time.monotonic() + 2
        while time.monotonic() < until:
            receipt = self.arm.receipt(self.sid, op)
            if receipt['status'] != 'pending':
                self.assertEqual(receipt['status'], expected, receipt)
                return receipt
            time.sleep(.005)
        self.fail('Action did not settle')

    def connect(self):
        self.action('connect', {'demo': True, 'profile_id': 'arm_j1_j2_j3'})
        self.action('ack', {'supported': True})

    def zero(self, name, known=0, direction=1):
        self.action('check')
        self.action('calibrate', {'joint': name, 'known_angle_deg': known,
                                'direction': direction, 'lower_deg': -80, 'upper_deg': 80})

    def ready(self):
        self.connect()
        for name in ('J1', 'J2', 'J3'):
            self.zero(name)
        self.action('check')

    def test_partial_zero_visible_and_all_required(self):
        self.connect()
        self.zero('J2', known=20, direction=-1)
        snap = self.arm.snapshot()
        self.assertFalse(snap['calibrated'])
        self.assertTrue(snap['joints'][1]['calibrated'])
        self.assertAlmostEqual(snap['joints'][1]['position_rad'], 20 * 3.141592653589793 / 180)
        self.action('check')
        self.assertIn('J1', self.action('arm', {'supported': True}, 'rejected')['reason'])

    def test_neutral_poll_refreshes_all_three_and_jog_writes_all_ids(self):
        self.ready()
        self.action('arm', {'supported': True})
        for seq in range(1, 14):
            self.arm.submit(self.sid, self.arm.issue_ticket(self.sid), seq, False, [0, 0, 0], 'keyboard')
            time.sleep(.015)
        self.assertTrue(self.arm.snapshot()['armed'])
        self.assertTrue(all(j['feedback_age_s'] < .1 for j in self.arm.snapshot()['joints']))
        for seq in range(14, 24):
            self.arm.submit(self.sid, self.arm.issue_ticket(self.sid), seq, True, [.3, -.3, .3], 'keyboard')
            time.sleep(.01)
        self.assertTrue(self.arm.snapshot()['armed'], self.arm.snapshot()['reason'])
        self.assertTrue(all(j.reference_history for j in self.arm.configured_joints))
        self.arm.stop(self.sid, 'test Stop')
        until = time.monotonic() + 1
        while not self.arm.snapshot()['stop_confirmed'] and time.monotonic() < until:
            time.sleep(.005)
        self.assertTrue(all(j['stop_confirmed'] for j in self.arm.snapshot()['joints']))

    def test_streaming_input_does_not_cancel_multijoint_entry_or_updates(self):
        self.ready()
        self.action('arm', {'supported': True})
        self.arm.submit(self.sid, self.arm.issue_ticket(self.sid), 1, False, [0, 0, 0], 'keyboard')
        original = self.arm.bus.write

        def write(packet):
            if decode_command(packet)['mode'] == 1:
                time.sleep(.006)
            return original(packet)

        self.arm.bus.write = write
        for seq in range(2, 70):
            self.arm.submit(
                self.sid, self.arm.issue_ticket(self.sid), seq, True, [.3, .3, .3], 'keyboard'
            )
            time.sleep(.004)

        self.assertTrue(self.arm.snapshot()['armed'], self.arm.snapshot()['reason'])
        self.assertTrue(all(j.foc_entered for j in self.arm.configured_joints))
        self.assertTrue(all(len(j.reference_history) > 2 for j in self.arm.configured_joints))
        self.assertTrue(all(j.last_reference_raw is not None for j in self.arm.configured_joints))

    def test_stop_attempts_other_ids_when_j2_missing(self):
        self.ready()
        bus = self.arm.bus
        original = bus.write
        sent = []
        def write(packet):
            sent.append(decode_command(packet))
            return original(packet)
        bus.write = write
        del bus.states[1]
        self.arm.stop(self.sid, 'missing J2')
        time.sleep(.2)
        stops = {p['motor_id'] for p in sent if p['mode'] == 0}
        self.assertEqual(stops, {0, 1, 2})
        self.assertFalse(self.arm.snapshot()['stop_confirmed'])
        self.assertFalse(self.arm.snapshot()['armed'])
        self.assertEqual(self.arm.snapshot()['joints'][1]['stop_result'], 'unconfirmed')

    def test_reconnect_invalidates_all_zeros(self):
        self.ready()
        self.action('disconnect')
        self.action('connect', {'demo': True, 'profile_id': 'arm_j1_j2_j3'})
        self.assertFalse(any(j['calibrated'] for j in self.arm.snapshot()['joints']))

    def test_fault_on_j2_disarms_all(self):
        self.ready()
        self.action('arm', {'supported': True})
        self.arm.submit(self.sid, self.arm.issue_ticket(self.sid), 1, False, [0, 0, 0], 'keyboard')
        self.arm.bus.states[1]['errors'] = 1
        time.sleep(.09)
        self.assertFalse(self.arm.snapshot()['armed'])
        self.assertTrue(self.arm.protection_fault)
        self.assertTrue(all(j['stop_confirmed'] for j in self.arm.snapshot()['joints']))

    def test_entered_joint_cannot_fall_back_to_stop_before_next_write(self):
        self.ready()
        self.action('arm', {'supported': True})
        self.arm.submit(self.sid, self.arm.issue_ticket(self.sid), 1, False, [0, 0, 0], 'keyboard')
        sent = []
        original = self.arm.bus.write
        def write(packet):
            decoded = decode_command(packet)
            sent.append(decoded)
            if decoded['mode'] == 1 and decoded['motor_id'] == 1:
                self.arm.joint_states['J1'].latest['mode'] = 0
            return original(packet)
        self.arm.bus.write = write
        self.arm.submit(self.sid, self.arm.issue_ticket(self.sid), 2, True, [.3, .3, .3], 'keyboard')
        time.sleep(.09)
        self.assertFalse(self.arm.snapshot()['armed'])
        self.assertFalse(any(p['mode'] == 1 and p['motor_id'] == 2 for p in sent))

    def test_unexpected_foc_exit_cannot_automatically_reenter(self):
        self.ready()
        self.action('arm', {'supported': True})
        self.arm.submit(self.sid, self.arm.issue_ticket(self.sid), 1, False, [0, 0, 0], 'keyboard')
        original = self.arm._tick
        injected = []
        def tick():
            if not injected and all(j.foc_entered for j in self.arm.configured_joints):
                injected.append(True)
                self.arm.joint_states['J1'].latest['mode'] = 0
            original()
        self.arm._tick = tick
        self.arm.submit(self.sid, self.arm.issue_ticket(self.sid), 2, True, [.3, .3, .3], 'keyboard')
        time.sleep(.09)
        self.assertTrue(injected)
        self.assertFalse(self.arm.snapshot()['armed'])
        self.assertIn('left FOC', self.arm.snapshot()['reason'])

    def test_profile_fixed_while_connected(self):
        self.connect()
        self.action('select_profile', {'profile_id': 'arm_j1'}, 'rejected')
        self.assertEqual(self.arm.snapshot()['profile_id'], 'arm_j1_j2_j3')

    def test_physical_support_requires_bool(self):
        self.arm.physical_opt_in = True
        for value in (False, 1, 'true', None):
            self.action('connect', {'demo': False, 'profile_id': 'arm_j1_j2_j3',
                                   'port': '/dev/not-a-device', 'support_confirmed': value}, 'rejected')
        self.assertFalse(self.arm.snapshot()['connected'])

    def test_ack_restores_watchdog_for_every_timed_out_joint(self):
        self.action('connect', {'demo': True, 'profile_id': 'arm_j1_j2_j3'})
        sent = []
        original = self.arm.bus.write
        def write(packet):
            sent.append(decode_command(packet))
            return original(packet)
        self.arm.bus.write = write
        self.action('ack', {'supported': True})
        for motor_id in range(3):
            writes = [p for p in sent if p['motor_id'] == motor_id]
            off = next(i for i, p in enumerate(writes) if not p['watchdog'])
            self.assertTrue(any(p['watchdog'] for p in writes[off + 1:]))
            self.assertTrue(all(p['mode'] == 0 for p in writes))
        self.assertTrue(all(j['timeout'] == 0 for j in self.arm.snapshot()['joints']))

    def test_joint_settings_and_drafts_are_independent(self):
        self.connect()
        self.action('configure', {'joint': 'J3', 'kp': 2, 'kd': .1, 'max_rpm': .5, 'effort_limit_nm': .7})
        snap = self.arm.snapshot()
        self.assertEqual(snap['joints'][2]['settings']['kp'], 2)
        self.assertEqual(snap['joints'][0]['settings']['kp'], 15)
        draft = self.arm.edit_parameters(self.sid, {'lower_deg': -1e100, 'upper_deg': 1e100}, 'J2')
        self.assertEqual(draft['upper_deg'], 1e100)
        self.assertEqual(self.arm.snapshot()['joints'][1]['settings']['upper_deg'], 90)

    def test_virtual_joint_draft_never_adds_physical_authority(self):
        draft = self.arm.edit_parameters(self.sid, {'lower_deg': -1e308, 'upper_deg': 1e308}, 'J3')
        self.assertEqual(draft['upper_deg'], 1e308)
        self.assertEqual([j.spec.motor_id for j in self.arm.configured_joints], [0])
        self.action('configure', {'joint': 'J3', 'kp': 2, 'kd': .1, 'max_rpm': .5,
                                  'effort_limit_nm': .7}, 'rejected')
        self.assertFalse(self.arm.snapshot()['connected'])

    def test_disconnected_profile_selection_does_not_own_bus(self):
        self.action('select_profile', {'profile_id': 'arm_j1_j2_j3'})
        self.assertIsNone(self.arm.owner)
        other = self.arm.create_session()
        self.sid = other
        self.action('connect', {'demo': True, 'profile_id': 'arm_j1_j2_j3'})
        self.assertEqual(self.arm.owner, other)

    def test_virtual_axes_do_not_energize_stopped_j1(self):
        self.action('connect', {'demo': True})
        self.action('ack', {'supported': True})
        self.zero('J1')
        self.action('check')
        self.action('arm', {'supported': True})
        self.arm.submit(self.sid, self.arm.issue_ticket(self.sid), 1, False, [0, 0, 0], 'keyboard')
        for seq in range(2, 12):
            self.arm.submit(self.sid, self.arm.issue_ticket(self.sid), seq, True, [0, .3, .3], 'keyboard')
            time.sleep(.01)
        self.assertTrue(self.arm.snapshot()['armed'])
        self.assertEqual(self.arm.latest['mode'], 0)
        self.assertFalse(self.arm.reference_history)
        self.assertGreater(self.arm.q_j2, 0)
