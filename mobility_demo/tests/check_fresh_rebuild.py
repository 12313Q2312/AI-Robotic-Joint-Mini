"""Prove committed artifacts can be regenerated without the private runs directory."""
from pathlib import Path
import json
import shutil
import subprocess
import sys
import tempfile

root=Path(__file__).resolve().parents[1]
scratch=root/'results/test-temporaries';scratch.mkdir(parents=True,exist_ok=True)
sources=['analyze.py','build_demo.py','cushion.py','report.py','unpack_evidence.py','delivery_cases.json','cushion_delivery.json','MODEL_AND_IMPLEMENTATION.md','evidence_bundle_v02.zip','evidence_bundle.manifest.json']
with tempfile.TemporaryDirectory(dir=scratch,prefix='fresh-rebuild-') as tmp:
    target=Path(tmp).resolve()
    if not target.is_relative_to(scratch.resolve()):raise RuntimeError('Unexpected temporary checkout path')
    for name in sources:shutil.copy2(root/name,target/name)
    (target/'viewer').mkdir();(target/'results').mkdir()
    for command in ('unpack_evidence.py','build_demo.py','report.py'):
        result=subprocess.run([sys.executable,command],cwd=target,capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=60)
        if result.returncode:raise RuntimeError(command+'\n'+result.stdout+result.stderr)
    decision=json.loads((target/'results/acceptance_decision.json').read_text(encoding='utf-8'))
    if not decision['flat_pass'] or not decision['effort_pass'] or not decision['stairs_pass']:raise AssertionError('Fresh evidence changed the reviewed 04 verdict')
    if 'validation_stairs_03' not in (target/'viewer/demo_data.js').read_text(encoding='utf-8'):raise AssertionError('Historical failure was omitted')
    if not (target/'viewer/demo_data.js').is_file():raise AssertionError('Viewer data not rebuilt')
    report={'passed':True,'method':'isolated_directory_no_private_runs','raw_evidence_bundle_verified':True,'metrics_regenerated':True,'stairs_success_and_prior_failure_preserved':True,'browser_qa_in_fresh_directory':'not_claimed'}
    (root/'results/fresh_rebuild_qa.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print('PASS fresh checkout rebuilt metrics, report and viewer from immutable evidence')
