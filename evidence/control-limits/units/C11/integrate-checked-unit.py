"""Parent-only copy of explicitly inspected source after exact-copy checks."""
from pathlib import Path
import hashlib,json,shutil,subprocess,sys
r=Path(__file__).resolve().parent;stem=sys.argv[1]
assert all(c.isalnum() or c=='-' for c in stem)
w=r/(stem+'-workbench');parent=r/'workbench'
manifest=json.loads((r/(stem+'-files.json')).read_text())
review=json.loads((r/(stem+'-parent-acceptance.json')).read_text())
checks=Path((r/(stem+'-latest-checks')).read_text().strip())
result=json.loads((checks/'result.json').read_text())
assert review['sourceInspected'] is True and review['visualCapturesInspected']==4
assert review['baselineAuditPassed'] is True and review['accepted'] is True
assert result['passedChecks'] is True and result['sourceUnchanged'] is True
assert review['checkResult']==str(checks/'result.json')
assert review['manifestSha256']==hashlib.sha256((r/(stem+'-files.json')).read_bytes()).hexdigest()
head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=parent,text=True).strip()
assert head==manifest['baseHead']==result['baseHead']
for path,sha in result['sourceHashes'].items():
 assert hashlib.sha256((w/path).read_bytes()).hexdigest()==sha,path
for item in manifest['files']:
 p=Path(item['path']);assert not p.is_absolute() and '..' not in p.parts and '.git' not in p.parts
 src=w/p;dst=parent/p
 assert src.resolve().is_relative_to(w.resolve()) and dst.resolve().is_relative_to(parent.resolve())
 assert not src.is_symlink() and not dst.is_symlink()
 assert item['exists'] is True,'Deletion needs a separately explicit parent operation'
 assert hashlib.sha256(src.read_bytes()).hexdigest()==item['sha256'],str(p)
for item in manifest['files']:
 p=Path(item['path']);dst=parent/p;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(w/p,dst)
 assert hashlib.sha256(dst.read_bytes()).hexdigest()==item['sha256']
print(json.dumps({'integrated':stem,'baseHead':head,'files':[x['path'] for x in manifest['files']],'sourceMatchesCheckedCopy':True},indent=2))
