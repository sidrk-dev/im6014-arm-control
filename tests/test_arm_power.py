import unittest
from unittest.mock import patch

from arm_manual import ArmManualSession, DEFAULT_SETTINGS
from spin_5rpm import healthy


def feedback(*, rpm=0.0, torque_nm=0.0, **changes):
    result = {
        'id': 0,
        'mode': 1,
        'errors': 0,
        'warnings': 0,
        'timeout': False,
        'voltage': 36.0,
        'driver_c': 25.0,
        'winding_c': 25.0,
        'torque_nm': torque_nm,
        'rpm': rpm,
        'position_rad': 0.0,
    }
    result.update(changes)
    return result


class ArmPowerEnvelopeTests(unittest.TestCase):
    def setUp(self):
        self.session = ArmManualSession()

    def tearDown(self):
        self.session.close()

    def test_configuration_accepts_requested_envelope_but_defaults_stay_conservative(self):
        configured = self.session._normalize_action('configure', {
            'kp': 15.0,
            'kd': 3.0,
            'max_rpm': 100.0,
            'effort_limit_nm': 15.0,
        })
        self.assertEqual(configured['max_rpm'], 100.0)
        self.assertEqual(configured['effort_limit_nm'], 15.0)
        self.assertEqual(self.session.joint_states['J1'].settings, DEFAULT_SETTINGS)

        for key, value in (('max_rpm', 100.1), ('effort_limit_nm', 15.1)):
            invalid = dict(configured, **{key: value})
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.session._normalize_action('configure', invalid)

    def test_measured_speed_trip_tracks_joint_command_with_bounded_margin(self):
        joint = self.session.joint_states['J1']
        joint.settings.update(max_rpm=100.0, effort_limit_nm=15.0)
        self.session._check_motion_reply(joint, feedback(rpm=105.0, torque_nm=14.5), 0.0)
        with self.assertRaisesRegex(RuntimeError, r'configured trip 120\.000 rpm'):
            self.session._check_motion_reply(joint, feedback(rpm=120.1), 0.0)

        joint.settings['max_rpm'] = 0.5
        self.session._check_motion_reply(joint, feedback(rpm=5.4), 0.0)
        with self.assertRaisesRegex(RuntimeError, r'configured trip 5\.500 rpm'):
            self.session._check_motion_reply(joint, feedback(rpm=5.6), 0.0)

    def test_joint_feedback_uses_configured_effort_and_keeps_other_health_gates(self):
        joint = self.session.joint_states['J1']
        joint.settings['effort_limit_nm'] = 15.0
        self.session._validate_feedback(feedback(torque_nm=14.9), 1, joint=joint)

        for key, value in (
            ('errors', 1),
            ('warnings', 1),
            ('timeout', True),
            ('voltage', 23.9),
            ('driver_c', 70.1),
            ('winding_c', 90.1),
        ):
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                self.session._validate_feedback(feedback(**{key: value}), 1, joint=joint)

    def test_input_loss_keeps_existing_fault_state_after_confirmed_stop(self):
        with self.session.lock:
            self.session.data['connected'] = True
            self.session.data['state'] = 'Fault — disarmed'
            self.session._latch_stop_locked('Motor feedback fault')
        with patch.object(self.session, '_stop', return_value=True):
            self.session._trip_loss('Input ticket lease expired')
        self.assertEqual(self.session.data['state'], 'Fault — disarmed')
        self.assertEqual(self.session.data['reason'], 'Motor feedback fault')
        self.assertTrue(self.session.stop_requested.is_set())

    def test_stale_ui_stop_keeps_first_motion_fault_and_still_latches_stop(self):
        sid = self.session.create_session()
        first_cause = 'Measured speed -1.628 rpm exceeds configured trip 0.500 rpm'
        with self.session.lock:
            self.session.data['connected'] = True
            self.session.data['state'] = 'Fault — disarmed'
            self.session._latch_stop_locked(first_cause)
            generation = self.session.generation

        self.session.stop(sid, 'input_ticket_or_submit_failed')

        self.assertEqual(self.session.data['reason'], first_cause)
        self.assertTrue(self.session.data['state'].startswith('Fault —'))
        self.assertGreater(self.session.generation, generation)
        self.assertTrue(self.session.stop_requested.is_set())


class HealthyTorqueDefaultTests(unittest.TestCase):
    def test_default_torque_gate_remains_four_nm_and_can_be_explicitly_raised(self):
        sample = feedback(torque_nm=4.1)
        with self.assertRaises(RuntimeError):
            healthy(sample, 1)
        healthy(sample, 1, torque_limit_nm=15.0)
        for limit in (0.0, 15.1, float('inf')):
            with self.subTest(limit=limit), self.assertRaises(ValueError):
                healthy(sample, 1, torque_limit_nm=limit)


if __name__ == '__main__':
    unittest.main()
