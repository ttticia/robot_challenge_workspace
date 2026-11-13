#!/usr/bin/env python3
"""Bundle portable source/model assets; exclude environments and machine builds."""
from pathlib import Path
import hashlib
import json
import tarfile
ROOT=Path(__file__).resolve().parents[1]
files=[]
for name in ('src','scripts','tests','assets','vendor','docs','README.md','LICENSE','requirements-sim.txt'):
    p=ROOT/name
    candidates=p.rglob('*') if p.is_dir() else [p]
    files.extend(f for f in candidates if f.is_file() and '__pycache__' not in f.parts and not f.name.endswith('.pyc'))
manifest={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}
(ROOT/'dist').mkdir(exist_ok=True)
(ROOT/'dist/checksums.json').write_text(json.dumps(manifest,indent=2)+'\n')
with tarfile.open(ROOT/'dist/lite3-maze-source.tar.gz','w:gz') as tar:
    for p in files:tar.add(p,arcname=Path('lite3-maze')/p.relative_to(ROOT))
    tar.add(ROOT/'dist/checksums.json',arcname='lite3-maze/checksums.json')
print('Created dist/lite3-maze-source.tar.gz')
