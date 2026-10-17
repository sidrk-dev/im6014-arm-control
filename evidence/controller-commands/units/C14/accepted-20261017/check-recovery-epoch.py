import unittest
from test_controller_faults import ProtectionTests
from joint_records import DecisionCode


class RecoveryEpochProbe(ProtectionTests):
    def test_new_acknowledged_check_does_not_inherit_recovery_credit(self):
        core, clock = self.make_core()
        self.overload(core, clock)
        self.assertTrue(core.begin_check(acknowledge_fault=True))
        for t in (1.25, 1.5):
            self.observe(core, clock, t, effort=0, mode=0)
        self.assertEqual(core.checked().code, DecisionCode.RECOVERY_PENDING)
        core.stop()
        self.assertTrue(core.begin_check(acknowledge_fault=True))
        for t in (1.75, 2):
            self.observe(core, clock, t, effort=0, mode=0)
            self.assertEqual(core.checked().code, DecisionCode.RECOVERY_PENDING)
        self.observe(core, clock, 2.25, effort=0, mode=0)
        self.assertTrue(core.checked())
        self.assertIsNone(core.snapshot().ownership)


suite = unittest.TestSuite([RecoveryEpochProbe('test_new_acknowledged_check_does_not_inherit_recovery_credit')])
result = unittest.TextTestRunner(verbosity=2).run(suite)
raise SystemExit(not result.wasSuccessful())
