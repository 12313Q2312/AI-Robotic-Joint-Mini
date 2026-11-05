"""Offline software/evidence checks; does not launch motion, ROS or Webots."""
from pathlib import Path
import json
import os
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parent
def verify(software_only=False):
    output=ROOT/'results';output.mkdir(exist_ok=True)
    compiler=os.environ.get('CXX') or shutil.which('g++') or shutil.which('clang++')
    node=shutil.which('node')
    commands=[('python_behavior_tests',[sys.executable,'-m','unittest','discover','-s','tests','-p','test_*.py','-v'])]
    if compiler:commands.extend([('cpp_compile',[compiler,'-std=c++17','-O2','-Wall','-Wextra','-Iinclude','tests/core_test.cpp','-o','results/core_test.exe']),('cpp_behavior_groups',[str(ROOT/'results/core_test.exe')])])
    if node:
        commands.extend((f'javascript_syntax_{name}',[node,'--check','viewer/'+name]) for name in ('app.js','geometry.js','demo_data.js'))
        commands.append(('actual_state_geometry',[sys.executable,'tests/check_geometry.py']))
    if not software_only:commands.append(('fresh_evidence_rebuild',[sys.executable,'tests/check_fresh_rebuild.py']))
    env=dict(os.environ,PYTHONUTF8='1',PYTHONDONTWRITEBYTECODE='1')
    if compiler:env['PATH']=str(Path(compiler).parent)+os.pathsep+env.get('PATH','')
    checks=[]
    for name,command in commands:
        r=subprocess.run(command,cwd=ROOT,capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=120,env=env)
        checks.append({'name':name,'exit_code':r.returncode,'passed':r.returncode==0,'output':r.stdout+r.stderr})
        print(('PASS ' if r.returncode==0 else 'FAIL ')+name,flush=True)
        if name=='cpp_compile' and r.returncode:break
    report={'scope':'software and selected geometry samples' if software_only else 'offline software, immutable evidence and viewer geometry; not clinical/mechanical certification','full_evidence_rebuild_requested':not software_only,'passed':bool(compiler and node) and all(c['passed'] for c in checks),'checks':checks,'cpp_available':bool(compiler),'node_available':bool(node),'counts_are_not_additive':True}
    (output/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    return report

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--software-only',action='store_true');args=parser.parse_args()
    raise SystemExit(0 if verify(args.software_only)['passed'] else 1)
