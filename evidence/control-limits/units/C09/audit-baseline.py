from pathlib import Path
import argparse,hashlib,json,re
p=argparse.ArgumentParser();p.add_argument('repo',type=Path);p.add_argument('run',type=Path);args=p.parse_args()
r=args.repo.resolve();b=args.run.resolve();rows=(b/'checks.tsv').read_text().splitlines()
expected={'interpreter','packet-tests','protocol-tests','serial-deadline-tests','milestone-tests','dashboard-tests','startup-verifier-tests','startup-real','startup-demo'}
checks={}
for row in rows[1:]:
 name,code=row.split('\t');assert name not in checks;checks[name]=int(code)
assert set(checks)==expected and all(code==0 for code in checks.values()),checks
counts={}
for name in ('protocol-tests','serial-deadline-tests','milestone-tests','startup-verifier-tests'):
 text=(b/(name+'.log')).read_text();matches=re.findall(r'Ran (\d+) tests? in ',text)
 assert len(matches)==1 and '\nOK\n' in text,(name,text[-300:]);counts[name]=int(matches[0])
for name in ('startup-real','startup-demo'):
 d=json.loads((b/name/'startup-result.json').read_text())
 assert d['exit_code']==0 and d['callback_completed'] and not d['main_exception'] and not d['callback_errors'],name
 for path,fp in d['source_fingerprint'].items():
  f=r/path;assert f.stat().st_size==fp['bytes'] and hashlib.sha256(f.read_bytes()).hexdigest()==fp['sha256'],path
ui=json.loads((b/'dashboard_test_results.json').read_text());assert ui['motor_io_performed'] is False
print(json.dumps({'baselineGroups':len(checks),'counts':counts,'dashboardGroups':len(ui['checks']),'sourceHashesMatch':True,'dashboardMotorIO':False,'visualInspection':'parent must inspect four captures separately'},indent=2))
