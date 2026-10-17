import argparse,cProfile,json,pstats,threading,time
from collections import Counter
from pathlib import Path
from unittest.mock import patch

import arm_manual
from arm_manual import ArmManualSession
from im6014_codec import decode_command


def run(profile_path=None):
    calls=Counter();modes=Counter();original=arm_manual.capture_exchange
    def counted(*args,**kwargs):
        decoded=decode_command(args[3]);modes[decoded['mode']]+=1
        result=original(*args,**kwargs);calls[decoded['id']]+=1
        return result
    original_loop=ArmManualSession._loop
    def profiled_loop(self):
        profiler=cProfile.Profile()
        try:return profiler.runcall(original_loop,self)
        finally:profiler.dump_stats(str(profile_path))
    started=time.monotonic()
    with patch('arm_manual.capture_exchange',counted):
        with patch.object(ArmManualSession,'_loop',profiled_loop if profile_path else original_loop):
            arm=ArmManualSession(physical_opt_in=False)
            try:
                sid=arm.create_session()
                op=arm.request(sid,'connect',{'demo':True,'profile_id':'arm_j1_j2_j3'})
                deadline=time.monotonic()+3
                while arm.receipt(sid,op)['status']=='pending' and time.monotonic()<deadline:time.sleep(.002)
                receipt=arm.receipt(sid,op);assert receipt['status']=='complete',receipt
                deadline=time.monotonic()+3
                while sum(calls.values())<60 and time.monotonic()<deadline:time.sleep(.002)
                assert sum(calls.values())>=60
                assert not arm.snapshot()['armed']
            finally:arm.close()
            data=arm.snapshot()['timing']
    assert not arm.thread.is_alive()
    assert set(calls)=={0,1,2} and modes.keys()=={0}
    assert data['transaction_samples']==sum(calls.values())
    assert data['transaction_failures']==0 and data['measurement_errors']==0
    assert data['loop_samples']>0 and data['loop_period_s'] is not None
    return {'wall_s':time.monotonic()-started,'successful_mode0_by_id':dict(calls),
            'all_packet_modes':dict(modes),'timing':data,'physicalIO':'none;DemoSerialonly',
            'profiled':profile_path is not None}


if __name__=='__main__':
    root=Path('/tmp/c15-codex-evidence-20261017');rows=[]
    for _ in range(5):rows.append(run())
    (root/'five-runs.json').write_text(json.dumps(rows,indent=2)+'\n')
    profiled=run(root/'owner-profile.pstats')
    (root/'profiled-run.json').write_text(json.dumps(profiled,indent=2)+'\n')
    with (root/'owner-profile.txt').open('w') as out:
        pstats.Stats(str(root/'owner-profile.pstats'),stream=out).sort_stats('cumtime').print_stats(12)
    print(json.dumps({'unprofiledRuns':5,'allSuccessful':True,'transactions':[r['timing']['transaction_samples'] for r in rows],'profiledSeparate':True}))
