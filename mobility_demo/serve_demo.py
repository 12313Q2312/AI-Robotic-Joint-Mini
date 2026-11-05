"""Serve only this local review artifact on loopback; no control/hardware API."""
from http.server import SimpleHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
import argparse
import json

ROOT=Path(__file__).resolve().parent
class Handler(SimpleHTTPRequestHandler):
    def __init__(self,*args,**kwargs):super().__init__(*args,directory=str(ROOT),**kwargs)
    def do_GET(self):
        if self.path=='/api/health':
            body=json.dumps({'app':'joint-mini-mobility-review','mode':'recorded-simulation-playback'}).encode()
            self.send_response(200);self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body);return
        if self.path=='/':self.send_response(302);self.send_header('Location','/viewer/index.html');self.end_headers();return
        if any(p.startswith('.') for p in self.path.split('?')[0].split('/') if p):self.send_error(404);return
        super().do_GET()
    def list_directory(self,path):self.send_error(404,'Directory listing disabled')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--port',type=int,default=8787);a=p.parse_args()
    server=ThreadingHTTPServer(('127.0.0.1',a.port),Handler)
    print(f'Local review: http://127.0.0.1:{a.port}/viewer/index.html',flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:server.server_close()
