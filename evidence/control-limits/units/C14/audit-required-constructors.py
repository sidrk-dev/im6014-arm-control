"""Parent static audit of required direct dataclass constructor calls; no imports or I/O devices."""
from pathlib import Path
import ast,json,sys
root=Path(sys.argv[1]).resolve()
records=[('joint_contract.py',{'JointSpec','MotorHealth'})]
for value in sys.argv[2:]:
 module,name=value.split(':',1)
 assert module.endswith('.py') and '/' not in module and name.isidentifier()
 records.append((module,{name}))
required={};order={}
for module,names in records:
 tree=ast.parse((root/module).read_text());found=set()
 for c in tree.body:
  if not isinstance(c,ast.ClassDef) or c.name not in names:continue
  found.add(c.name);assert c.name not in required
  required[c.name]={x.target.id for x in c.body if isinstance(x,ast.AnnAssign) and isinstance(x.target,ast.Name) and x.value is None}
  order[c.name]=[x.target.id for x in c.body if isinstance(x,ast.AnnAssign) and isinstance(x.target,ast.Name)]
 assert found==names,(module,names-found)
rows=[];bad=[];unknown=[]
for p in sorted(root.glob('*.py')):
 tree=ast.parse(p.read_text());parent={child:n for n in ast.walk(tree) for child in ast.iter_child_nodes(n)}
 for n in ast.walk(tree):
  if not isinstance(n,ast.Call) or not isinstance(n.func,ast.Name) or n.func.id not in required:continue
  name=n.func.id;dynamic=any(isinstance(x,ast.Starred) for x in n.args) or any(k.arg is None for k in n.keywords)
  supplied=set(order[name][:len(n.args)])|{k.arg for k in n.keywords if k.arg is not None};missing=sorted(required[name]-supplied)
  ancestor=n;intentional=False
  while ancestor in parent:
   ancestor=parent[ancestor]
   if isinstance(ancestor,ast.With):
    for item in ancestor.items:
     call=item.context_expr
     if isinstance(call,ast.Call) and isinstance(call.func,ast.Attribute) and call.func.attr in {'assertRaises','assertRaisesRegex'} and call.args:
      names={q.id for q in ast.walk(call.args[0]) if isinstance(q,ast.Name)}
      intentional|='TypeError' in names
  row={'file':p.name,'line':n.lineno,'constructor':name,'missing':missing,'intentionalTypeError':intentional,'dynamicArguments':dynamic}
  rows.append(row)
  if dynamic:unknown.append(row)
  elif missing and not intentional:bad.append(row)
print(json.dumps({'requiredFields':{k:sorted(v) for k,v in required.items()},'directCalls':len(rows),'incompleteNonTypeErrorCalls':bad,'dynamicCallsNeedInspection':unknown,'passedStaticAudit':not bad and not unknown,'calls':rows},indent=2))
raise SystemExit(bool(bad or unknown))
