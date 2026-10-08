"""Parent preserves exact proof bytes while reducing generated PR diff noise."""
from pathlib import Path
import gzip,hashlib,json,sys
repo=Path(sys.argv[1]).resolve();root=repo/'evidence/control-limits';assert root.is_dir() and not root.is_symlink()
index=root/'compressed-proof-index.json';prior=json.loads(index.read_text()) if index.exists() else {'format':'gzip with exact decompressed SHA256','entries':[]};rows=prior['entries'];known={x['originalPath'] for x in rows}
proof_logs=[]
for p in sorted(root.rglob('parent-repro-*.log')):
 if p.stat().st_size<=60000:continue
 try:data=json.loads(p.read_bytes())
 except (ValueError,UnicodeError):continue
 if isinstance(data,dict) and data.get('passed') is True and isinstance(data.get('positiveStepCertificates'),int) and isinstance(data.get('samples'),list):
  assert data['positiveStepCertificates']==len(data['samples'])
  proof_logs.append(p)
for p in sorted(list(root.rglob('*.json'))+proof_logs):
 if p==index or p.stat().st_size<=60000:continue
 assert p.is_file() and not p.is_symlink() and p.resolve().is_relative_to(root.resolve())
 relative=str(p.relative_to(repo));assert relative not in known
 # Unit receipts and acceptance/check metadata stay individually readable.
 if any(word in p.name for word in ['receipt','acceptance','result','source','manifest','files','operation']):continue
 raw=p.read_bytes();json.loads(raw);packed=gzip.compress(raw,mtime=0);archive=p.with_name(p.name+'.gz');assert not archive.exists()
 archive.write_bytes(packed);assert gzip.decompress(archive.read_bytes())==raw
 rows.append({'originalPath':relative,'archivePath':str(archive.relative_to(repo)),'originalBytes':len(raw),'compressedBytes':len(packed),'sha256':hashlib.sha256(raw).hexdigest()});p.unlink();known.add(relative)
index.write_text(json.dumps(prior,indent=2)+'\n')
for row in rows:
 archive=repo/row['archivePath'];raw=gzip.decompress(archive.read_bytes());assert len(raw)==row['originalBytes'] and hashlib.sha256(raw).hexdigest()==row['sha256'];json.loads(raw)
print(json.dumps({'archivesVerified':len(rows),'originalBytes':sum(x['originalBytes'] for x in rows),'compressedBytes':sum(x['compressedBytes'] for x in rows),'sourceChanges':False,'allOriginalBytesRecoverable':True}))
