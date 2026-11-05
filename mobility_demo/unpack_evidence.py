"""Verify and unpack the versioned evidence without overwriting existing files."""
from pathlib import Path
import hashlib
import json
import zipfile

ROOT=Path(__file__).resolve().parent
def unpack():
    outer=json.loads((ROOT/'evidence_bundle.manifest.json').read_text())
    archive=(ROOT/outer['archive']).resolve()
    if archive.parent!=ROOT.resolve() or archive.suffix!='.zip':raise ValueError('Unexpected evidence archive path')
    if hashlib.sha256(archive.read_bytes()).hexdigest()!=outer['sha256']:raise ValueError('Evidence archive fingerprint mismatch')
    target=ROOT/'evidence';target.mkdir(exist_ok=True)
    with zipfile.ZipFile(archive) as z:
        manifest=json.loads(z.read('manifest.json'))
        for name,meta in manifest['files'].items():
            path=(target/name).resolve()
            if not path.is_relative_to(target.resolve()):raise ValueError('Evidence member outside intended directory')
            raw=z.read(name)
            if len(raw)!=meta['bytes'] or hashlib.sha256(raw).hexdigest()!=meta['sha256']:raise ValueError('Evidence member mismatch')
            if path.exists():
                if path.read_bytes()!=raw:raise FileExistsError('Existing evidence differs; nothing will overwrite it: '+name)
            else:path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(raw)
        manifest_path=target/'manifest.json'
        if not manifest_path.exists():manifest_path.write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    print('Verified evidence available: '+str(target))

if __name__=='__main__':unpack()
