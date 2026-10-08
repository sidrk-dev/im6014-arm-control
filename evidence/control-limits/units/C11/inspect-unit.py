from pathlib import Path
import hashlib,json,subprocess,sys
r=Path(__file__).resolve().parent
unit=sys.argv[1]
assert unit.split('-')[0] in ['C'+str(n).zfill(2) for n in range(7,17)]
assert all(c.isalnum() or c == '-' for c in unit)
w=r/(unit+'-workbench'); parent=r/'workbench'
def git(*args): return subprocess.check_output(['git',*args],cwd=w,text=True)
base=git('rev-parse','HEAD').strip()
assert base==subprocess.check_output(['git','rev-parse','HEAD'],cwd=parent,text=True).strip(), 'Parent changed while writer ran'
changed=set(git('diff','--name-only').splitlines())|set(git('ls-files','--others','--exclude-standard').splitlines())
copy_record=r/(unit+'-copy.json')
parent_metadata=[]
if copy_record.exists():
 copy_data=json.loads(copy_record.read_text())
 assert copy_data['baseHead']==base
 for relative in copy_data.get('parentMetadataOverrides',[]):
  assert relative in {'docs/resume-c07-c16.md','docs/c07-c16-workflow.md','docs/decisions.tsv','docs/milestones.tsv'}
  assert hashlib.sha256((w/relative).read_bytes()).hexdigest()==copy_data['sourceHashes'][relative], 'Worker changed parent-owned metadata'
  changed.discard(relative);parent_metadata.append(relative)

for name in changed:
 p=Path(name)
 assert not p.is_absolute() and '..' not in p.parts and '.git' not in p.parts
 assert (w/p).resolve().is_relative_to(w.resolve()), 'Symlink outside assignment'
 assert not (w/p).is_symlink(), 'New symlink needs separate inspection'
files=[]
for name in sorted(changed):
 p=w/name; files.append({'path':name,'exists':p.exists(),'bytes':p.stat().st_size if p.exists() else None,'sha256':hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else None})
(r/(unit+'-diff.patch')).write_bytes(subprocess.check_output(['git','diff','--binary'],cwd=w))
(r/(unit+'-files.json')).write_text(json.dumps({'baseHead':base,'files':files,'parentMetadataOverridesExcluded':parent_metadata},indent=2)+'\n')
print(json.dumps({'baseHead':base,'files':files,'parentMetadataOverridesExcluded':parent_metadata},indent=2))
