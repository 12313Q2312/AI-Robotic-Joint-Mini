"""Open the review without installing software or touching another service."""
from pathlib import Path
from datetime import datetime
import argparse
import http.client
import json
import subprocess
import sys
import time
import webbrowser

ROOT=Path(__file__).resolve().parent
def health(port):
    conn=http.client.HTTPConnection('127.0.0.1',port,timeout=1)
    try:
        conn.request('GET','/api/health');response=conn.getresponse()
        if response.status!=200:raise RuntimeError('Port is occupied by another service')
        result=json.loads(response.read())
        if result.get('app')!='joint-mini-mobility-review':raise RuntimeError('Port is occupied by another service')
        return True
    except (ConnectionRefusedError,TimeoutError,OSError):return False
    finally:conn.close()

def launch(port,no_open=False):
    if (ROOT/'evidence_bundle.manifest.json').is_file():
        from unpack_evidence import unpack
        unpack()
    if not health(port):
        local=ROOT/'.local';local.mkdir(exist_ok=True)
        logfile=local/f'server_{datetime.now():%Y%m%d_%H%M%S}.log'
        startup=subprocess.STARTUPINFO();startup.dwFlags|=subprocess.STARTF_USESHOWWINDOW;startup.wShowWindow=0
        with logfile.open('wb') as output:
            process=subprocess.Popen([sys.executable,str(ROOT/'serve_demo.py'),'--port',str(port)],cwd=ROOT,stdin=subprocess.DEVNULL,stdout=output,stderr=subprocess.STDOUT,startupinfo=startup,creationflags=subprocess.CREATE_NO_WINDOW)
        (local/'server-state.json').write_text(json.dumps({'pid':process.pid,'port':port,'app':'joint-mini-mobility-review'}),encoding='utf-8')
        for _ in range(50):
            if health(port):break
            if process.poll() is not None:raise RuntimeError('Review server could not start; see '+str(logfile))
            time.sleep(.2)
        else:raise RuntimeError('Review server did not become available')
    url=f'http://127.0.0.1:{port}/viewer/index.html'
    if not no_open:webbrowser.open(url)
    print(json.dumps({'url':url,'ready':True}))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--port',type=int,default=8787);p.add_argument('--no-open',action='store_true');a=p.parse_args();launch(a.port,a.no_open)
