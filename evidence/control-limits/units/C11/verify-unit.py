"""Parent-only offline checks for one retained, settled implementation copy."""
from pathlib import Path
import argparse,datetime,hashlib,json,os,subprocess
p=argparse.ArgumentParser();p.add_argument('stem');p.add_argument('--repro',action='append',default=[]);a=p.parse_args()
r=Path(__file__).resolve().parent;unit=a.stem.split('-')[0]
assert unit in ['C'+str(n).zfill(2) for n in range(7,17)]
assert all(c.isalnum() or c=='-' for c in a.stem)
w=r/(a.stem+'-workbench');assert w.is_dir()
py='/home/sid/Documents/code/im6014Workbench/.venv/bin/python'
out=r/(a.stem+'-parent-checks')/datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ');out.mkdir(parents=True)
def fingerprint():
 paths=sorted(w.glob('*.py'))+sorted((w/'scripts').glob('*'))+sorted((w/'docs').glob('*.md'))
 return {str(x.relative_to(w)):hashlib.sha256(x.read_bytes()).hexdigest() for x in paths if x.is_file()}
assert len((w/'joint_controller.py').read_text().splitlines()) < 1000, 'Core size constraint exceeded; inspect domain structure before acceptance'
before=fingerprint();env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',QT_QPA_PLATFORM='offscreen');checks=[]
def run(name,args):
 with (out/(name+'.log')).open('w') as f:
  x=subprocess.run(args,cwd=w,env=env,stdout=f,stderr=subprocess.STDOUT)
 checks.append({'name':name,'argv':args,'exitCode':x.returncode,'log':str(out/(name+'.log'))})
run('targeted',[py,'-B','scripts/check_milestone.py',unit,'--python',py,'--evidence-root',str(out/'targeted')])
for n,path in enumerate(a.repro):
 q=Path(path).resolve();assert q.is_relative_to(r) and q.is_file()
 run('parent-repro-'+str(n),[py,'-B',str(q),str(w)])
run('baseline',[str(w/'scripts/check_offline_baseline.sh'),py,str(out/'baseline')])
after=fingerprint();d={'stem':a.stem,'unit':unit,'cwd':str(w),'baseHead':subprocess.check_output(['git','rev-parse','HEAD'],cwd=w,text=True).strip(),'sourceHashes':before,'sourceUnchanged':before==after,'checks':checks,'physicalIO':'none; audited fake I/O and isolated PTY fixtures only','visualInspection':'pending; parent must inspect four baseline captures','passedChecks':before==after and all(x['exitCode']==0 for x in checks)}
(out/'result.json').write_text(json.dumps(d,indent=2)+'\n');(r/(a.stem+'-latest-checks')).write_text(str(out)+'\n');print(json.dumps({'result':str(out/'result.json'),'passedChecks':d['passedChecks'],'checks':[{k:v for k,v in x.items() if k!='argv'} for x in checks]},indent=2))
raise SystemExit(0 if d['passedChecks'] else 1)
