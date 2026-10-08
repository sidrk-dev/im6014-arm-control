"""Parent creates an isolated author copy with active source visible."""
from pathlib import Path
import hashlib,json,subprocess,sys,shutil
r=Path(__file__).resolve().parent;stem=sys.argv[1]
assert all(c.isalnum() or c=='-' for c in stem)
assert stem.split('-')[0] in ['C'+str(n).zfill(2) for n in range(7,17)]
parent=r/'workbench';copy=r/(stem+'-workbench');assert not copy.exists()
def git(cwd,*args):return subprocess.check_output(['git',*args],cwd=cwd,text=True).strip()
head=git(parent,'rev-parse','HEAD')
subprocess.run(['git','clone','--quiet','--no-hardlinks',str(parent),str(copy)],check=True)
git(copy,'sparse-checkout','init','--no-cone')
git(copy,'sparse-checkout','set','--no-cone','/*','!/evidence/','/evidence/boundary/vendor-crc-vectors.json')
assert git(copy,'rev-parse','HEAD')==head
assert not git(copy,'status','--porcelain')
assert sorted(str(p.relative_to(copy/'evidence')) for p in (copy/'evidence').rglob('*') if p.is_file())==['boundary/vendor-crc-vectors.json']
assert (copy/'evidence/boundary/vendor-crc-vectors.json').read_bytes()==(parent/'evidence/boundary/vendor-crc-vectors.json').read_bytes()
hashes={};metadata_overrides=[]
for source in sorted(parent.glob('*.py'))+sorted((parent/'scripts').glob('*'))+sorted((parent/'docs').glob('*.md')):
 if not source.is_file():continue
 relative=source.relative_to(parent);target=copy/relative
 assert target.is_file() and not target.is_symlink()
 sha=hashlib.sha256(source.read_bytes()).hexdigest()
 if hashlib.sha256(target.read_bytes()).hexdigest()!=sha:
  assert str(relative) in {'docs/resume-c07-c16.md','docs/c07-c16-workflow.md','docs/decisions.tsv','docs/milestones.tsv'},relative
  shutil.copyfile(source,target);metadata_overrides.append(str(relative))
 assert hashlib.sha256(target.read_bytes()).hexdigest()==sha,relative
 hashes[str(relative)]=sha
result={'stem':stem,'baseHead':head,'cwd':str(copy),'sourceHashes':hashes,
 'evidenceRetention':'Complete evidence and Git history retained in parent; sparse copy exposes active source and docs',
 'gitStatus':git(copy,'status','--porcelain'),'parentMetadataOverrides':metadata_overrides,'physicalIO':'none'}
(r/(stem+'-copy.json')).write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='sourceHashes'}))
