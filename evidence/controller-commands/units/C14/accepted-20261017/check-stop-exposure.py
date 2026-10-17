import unittest
from test_controller_faults import ProtectionTests


class RepeatedControlActions(ProtectionTests):
    def test_repeated_stop_does_not_erase_measured_time(self):
        core, clock = self.make_core()
        for i in range(9):
            self.observe(core, clock, i * .25)
            core.stop()
        self.assertTrue(any(f.persistent_effort for f in core.snapshot().faults))

    def test_repeated_check_does_not_erase_measured_time(self):
        core, clock = self.make_core(arm=False)
        for i in range(9):
            core.stop()
            core.begin_check(acknowledge_fault=True)
            self.observe(core, clock, i * .25, mode=0)
        self.assertTrue(any(f.persistent_effort for f in core.snapshot().faults))


suite=unittest.TestSuite(RepeatedControlActions(n) for n in (
    'test_repeated_stop_does_not_erase_measured_time',
    'test_repeated_check_does_not_erase_measured_time',
))
result=unittest.TextTestRunner(verbosity=2).run(suite)
raise SystemExit(not result.wasSuccessful())
