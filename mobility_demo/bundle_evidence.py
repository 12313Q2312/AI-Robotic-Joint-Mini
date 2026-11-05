"""Package fixed, sanitized engineering evidence; originals remain untouched."""
from pathlib import Path
import hashlib
import json
import zipfile
from analyze import ROOT,evidence_path
from cushion import selected_results

def build(archive_name='evidence_bundle_v02.zip'):
    archive=ROOT/archive_name
    if archive.parent.resolve()!=ROOT.resolve() or not archive.name.endswith('.zip'):raise ValueError('Use a zip filename within this module')
    if archive.exists():raise FileExistsError('Delivery archive already exists; choose a new version instead of replacing it')
    entries={}
    for case in json.loads((ROOT/'delivery_cases.json').read_text(encoding='utf-8')):
        name=case['run']
        for suffix in ('.csv','.events.jsonl','.completed.json','.source.cpp','.policy.hpp','.PlanGate.hpp','.RiskMonitor.hpp'):
            path=evidence_path('runs/'+name+suffix);entries['runs/'+path.name]=path.read_bytes()
        raw=json.loads(evidence_path(f'runs/{name}.json').read_text(encoding='utf-8'))
        safe={k:raw[k] for k in ('scenario','mode','run','exit_code','elapsed_wall_s','evidence','source_sha256','completion') if k in raw}
        entries[f'runs/{name}.json']=(json.dumps(safe,indent=2)+'\n').encode()
        entries[f'worlds/{name}.wbt']=evidence_path(f'worlds/{name}.wbt').read_bytes()
    component=selected_results()
    for path in component.iterdir():
        if path.is_file():entries[f'cushion/{component.name}/{path.name}']=path.read_bytes()
    manifest={'schema_version':1,'scope':'synthetic_engineering_evidence_only','files':{p:{'sha256':hashlib.sha256(raw).hexdigest(),'bytes':len(raw)} for p,raw in entries.items()}}
    with zipfile.ZipFile(archive,'x',compression=zipfile.ZIP_DEFLATED,compresslevel=9) as z:
        for path,raw in sorted(entries.items()):z.writestr(path,raw)
        z.writestr('manifest.json',json.dumps(manifest,indent=2))
    outer={'archive':archive.name,'sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),'bytes':archive.stat().st_size,'files':len(entries),'contains_private_raw_logs':False}
    (ROOT/'evidence_bundle.manifest.json').write_text(json.dumps(outer,indent=2),encoding='utf-8')
    print(json.dumps(outer))

if __name__=='__main__':build()
