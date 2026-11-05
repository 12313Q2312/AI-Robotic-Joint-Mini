"""Compile and run a named local scenario, preserving each run's logs."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import time
from prepare import ROOT,generate

def run(scenario,mode,name,timeout=90,capture=False):
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,60}',name):raise ValueError('Use a short alphanumeric run name.')
    if scenario not in ('stand','flat','stairs','stop','push-small','push-large'):raise ValueError('Unknown scenario.')
    (ROOT/'runs').mkdir(exist_ok=True)
    if list((ROOT/'runs').glob(name+'.*')):raise ValueError('Run name already exists; preserve prior evidence.')
    if list((ROOT/'evidence/runs').glob(name+'.*')):raise ValueError('Run name belongs to preserved delivery evidence.')
    for archive_path in ROOT.glob('evidence_bundle*.zip'):
        import zipfile
        with zipfile.ZipFile(archive_path) as archive:
            if any(member.startswith('runs/'+name+'.') for member in archive.namelist()):raise ValueError('Run name belongs to immutable archived evidence.')
    home=Path(os.environ.get('WEBOTS_HOME','G:/Webots'))
    compiler=Path(os.environ.get('CXX') or shutil.which('g++') or 'H:/MSYS2/mingw64/bin/g++.exe')
    binary=ROOT/'controllers/mobility/mobility.exe'
    env=dict(os.environ,WEBOTS_HOME=str(home),PYTHONUTF8='1')
    env['PATH']=str(compiler.parent)+os.pathsep+str(home/'msys64/mingw64/bin')+os.pathsep+env.get('PATH','')
    command=[str(compiler),'-std=c++17','-O2','-Wall','-Wextra','-Iinclude','-I'+str(home/'include/controller/c'),'controllers/mobility/mobility.cpp','-L'+str(home/'lib/controller'),'-lController','-o','controllers/mobility/mobility.exe']
    compiled=subprocess.run(command,cwd=ROOT,capture_output=True,text=True,env=env,timeout=60)
    if compiled.returncode:
        report={'scenario':scenario,'mode':mode,'run':name,'exit_code':compiled.returncode,'evidence':'compilation_failed_no_simulation','stdout':compiled.stdout,'stderr':compiled.stderr}
        (ROOT/'runs'/f'{name}.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(report,ensure_ascii=True));return report
    world=generate(scenario,mode=='assist',name,mode!='no-device',capture)
    command=[str(home/'msys64/mingw64/bin/webots.exe'),'--batch','--mode=fast','--minimize','--stdout','--stderr','--port=1241',str(world)]
    if not capture:command.insert(4,'--no-rendering')
    startup=subprocess.STARTUPINFO();startup.dwFlags|=subprocess.STARTF_USESHOWWINDOW;startup.wShowWindow=0
    started=time.monotonic()
    source_manifest={}
    for path in (ROOT/'controllers/mobility/mobility.cpp',*sorted((ROOT/'include').glob('*.hpp')),ROOT/'prepare.py',world):
        source_manifest[path.relative_to(ROOT).as_posix()]=hashlib.sha256(path.read_bytes()).hexdigest()
    (ROOT/'runs'/f'{name}.source.cpp').write_bytes((ROOT/'controllers/mobility/mobility.cpp').read_bytes())
    (ROOT/'runs'/f'{name}.policy.hpp').write_bytes((ROOT/'include/AssistPolicy.hpp').read_bytes())
    for header in (ROOT/'include').glob('*.hpp'):
        if header.name!='AssistPolicy.hpp':(ROOT/'runs'/f'{name}.{header.name}').write_bytes(header.read_bytes())
    process=subprocess.Popen(command,cwd=ROOT,env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,encoding='utf-8',errors='replace',creationflags=subprocess.CREATE_NO_WINDOW,startupinfo=startup)
    try:
        stdout,stderr=process.communicate(timeout=timeout)
        report={'scenario':scenario,'mode':mode,'run':name,'exit_code':process.returncode,'elapsed_wall_s':time.monotonic()-started,'stdout':stdout,'stderr':stderr,'evidence':'actual_webots_simulation'}
    except subprocess.TimeoutExpired:
        subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],capture_output=True,creationflags=subprocess.CREATE_NO_WINDOW,timeout=10)
        stdout,stderr=process.communicate(timeout=10)
        report={'scenario':scenario,'mode':mode,'run':name,'exit_code':None,'elapsed_wall_s':time.monotonic()-started,'stdout':stdout,'stderr':stderr,'evidence':'blocked_or_timeout'}
    report['source_sha256']=source_manifest
    marker=ROOT/'runs'/f'{name}.completed.json'
    report['completion']=json.loads(marker.read_text(encoding='utf-8')) if marker.exists() else None
    if report['exit_code']==0 and not (report['completion'] or {}).get('completed'):
        report['exit_code']=1;report['evidence']='unexpected_exit_not_completed'
    (ROOT/'runs'/f'{name}.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=True))
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--scenario',default='stand');p.add_argument('--mode',choices=['assist','off','no-device'],default='assist');p.add_argument('--run',required=True);p.add_argument('--timeout',type=float,default=90)
    p.add_argument('--capture',action='store_true')
    a=p.parse_args();r=run(a.scenario,a.mode,a.run,a.timeout,a.capture);raise SystemExit(0 if r['exit_code']==0 else 1)
