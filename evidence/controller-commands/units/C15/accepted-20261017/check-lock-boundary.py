import argparse,importlib.util,sys,unittest
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('runtime',type=Path);p.add_argument('tests',type=Path);a=p.parse_args()
sys.path.insert(0,str(a.runtime.resolve()))
spec=importlib.util.spec_from_file_location('timing_boundary_checks',a.tests.resolve())
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
r=unittest.TextTestRunner(verbosity=2).run(unittest.TestSuite([m.ArmTimingTests('test_exchange_latency_excludes_post_exchange_lock_wait')]))
raise SystemExit(not r.wasSuccessful())
