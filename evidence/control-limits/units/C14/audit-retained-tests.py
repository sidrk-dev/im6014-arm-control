"""Parent reports existing unittest method retention across a settled source migration."""
from pathlib import Path
import ast, json, subprocess, sys
root=Path(sys.argv[1]).resolve()
def methods(text):
    tree=ast.parse(text)
    return {c.name+'.'+n.name for c in tree.body if isinstance(c,ast.ClassDef)
            for n in c.body if isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef)) and n.name.startswith('test_')}
rows=[]
for file in sorted(root.glob('test_*.py')):
    old=subprocess.run(['git','show','HEAD:'+file.name],cwd=root,text=True,capture_output=True)
    if old.returncode:
        continue
    before=methods(old.stdout); after=methods(file.read_text())
    rows.append({'file':file.name,'before':len(before),'after':len(after),
                 'removed':sorted(before-after),'added':sorted(after-before)})
report={'scope':str(root),'removedMethods':[{'file':r['file'],'method':m} for r in rows for m in r['removed']],
        'files':rows,'limitation':'Method retention does not prove assertions retained; parent source inspection and actual checks remain required.'}
print(json.dumps(report,indent=2))
raise SystemExit(bool(report['removedMethods']))
