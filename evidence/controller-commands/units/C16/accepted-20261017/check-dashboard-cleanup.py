import unittest
from unittest.mock import Mock,patch
import arm_dashboard
from arm_manual import ArmManualSession

class CleanupChecks(unittest.TestCase):
    def test_failed_server_construction_closes_its_owner(self):
        owners=[]
        def owner(**kwargs):
            value=ArmManualSession(physical_opt_in=False);owners.append(value);return value
        try:
            with patch.object(arm_dashboard,'ArmManualSession',side_effect=owner), \
                    patch.object(arm_dashboard,'ArmDashboardServer',side_effect=OSError('fake bind failure')):
                with self.assertRaisesRegex(OSError,'fake bind failure'):
                    arm_dashboard.run_dashboard(port=0)
            self.assertEqual(len(owners),1)
            self.assertFalse(owners[0].thread.is_alive())
        finally:
            for value in owners:value.close()

    def test_server_cleanup_failure_cannot_skip_owner_close(self):
        arm=ArmManualSession(physical_opt_in=False)
        server=Mock(bound_port=0)
        server.server_close.side_effect=OSError('fake server cleanup failure')
        try:
            with patch.object(arm_dashboard,'ArmDashboardServer',return_value=server):
                with self.assertRaisesRegex(OSError,'fake server cleanup failure'):
                    arm_dashboard.run_dashboard(port=0,session=arm)
            self.assertFalse(arm.thread.is_alive())
        finally:
            arm.close()

if __name__=='__main__':unittest.main(verbosity=2)
