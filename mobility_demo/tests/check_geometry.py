import csv
import json
from pathlib import Path
import shutil
import subprocess
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from analyze import evidence_path,FRAME_KEYS
samples=[]
fixture=Path(__file__).with_name('geometry_samples.json')
try:
    for name in ('validation_flat_assist_04','validation_flat_off_04','validation_stairs_04','validation_stairs_03'):
        with evidence_path(f'runs/{name}.csv').open() as f:rows=list(csv.DictReader(f))
        for row in rows[::max(1,len(rows)//30)]:
            samples.append({'run':name,'frame':[float(row.get(k,0)) for k in FRAME_KEYS],'feet':[[float(row[f'{side}_foot_{axis}_m']) for axis in ('x','y','z')] for side in ('r','l')]})
except FileNotFoundError:
    samples=json.loads(fixture.read_text(encoding='utf-8'))['samples']
node=shutil.which('node')
if not node:raise SystemExit('Node.js required to validate rendered geometry')
r=subprocess.run([node,str(Path(__file__).with_name('geometry_test.js'))],input=json.dumps(samples),capture_output=True,text=True)
print(r.stdout+r.stderr);raise SystemExit(r.returncode)
