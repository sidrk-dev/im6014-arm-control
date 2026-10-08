"""Parent retains settled unit source and raw checks for PR review."""
from pathlib import Path
import argparse,gzip,hashlib,json,shutil
p=argparse.ArgumentParser();p.add_argument('unit');p.add_argument('--stem',action='append',required=True);a=p.parse_args()
r=Path(__file__).resolve().parent
assert a.unit in ['C'+str(n).zfill(2) for n in range(7,17)]
out=r/'workbench/evidence/control-limits/units'/a.unit;out.mkdir(parents=True,exist_ok=True)
def copy(source,dest):
 assert source.is_file() and not source.is_symlink()
 dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,dest)
 assert hashlib.sha256(source.read_bytes()).digest()==hashlib.sha256(dest.read_bytes()).digest()
for stem in a.stem:
 assert stem==a.unit or stem.startswith(a.unit+'-')
 assert all(c.isalnum() or c=='-' for c in stem)
 w=r/(stem+'-workbench');assert w.is_dir()
 manifest=json.loads((r/(stem+'-files.json')).read_text())
 for item in manifest['files']:
  rel=Path(item['path']);assert not rel.is_absolute() and '..' not in rel.parts
  src=w/rel;assert src.resolve().is_relative_to(w.resolve())
  assert hashlib.sha256(src.read_bytes()).hexdigest()==item['sha256']
  copy(src,out/(stem+'-source')/rel)
 for src in sorted(r.glob(stem+'-*')):
  if src.is_file() and not src.is_symlink():
   if src.suffix in ('.patch','.md') and src.stat().st_size>60000:
    dest=out/(src.name+'.gz');dest.write_bytes(gzip.compress(src.read_bytes(),mtime=0))
   else:copy(src,out/src.name)
 checks=r/(stem+'-parent-checks')
 if checks.exists():
  shutil.copytree(checks,out/(stem+'-parent-checks'),dirs_exist_ok=True,
                  ignore=shutil.ignore_patterns('.source-*','__pycache__'))
for name in ('parent-operations.json','verify-unit.py','audit-baseline.py',
             'reference-oracle.py','audit-retained-tests.py','audit-required-constructors.py','build-review-packet.py','integrate-checked-unit.py','preserve-unit.py'):
 copy(r/name,out/name)
print(json.dumps({'unit':a.unit,'retainedStems':a.stem,'path':str(out)}))
