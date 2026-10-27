#!/usr/bin/python3
"""发布前逐一检查ELF；不能只看文件名/static后缀。"""
import argparse, hashlib, subprocess
from pathlib import Path
parser=argparse.ArgumentParser();parser.add_argument('files',type=Path,nargs='+');args=parser.parse_args()
for path in args.files:
    data=path.read_bytes();assert data[:4]==b'\x7fELF',path
    ph=subprocess.check_output(['readelf','-l',str(path)],text=True)
    dynamic=subprocess.check_output(['readelf','-d',str(path)],text=True)
    assert 'INTERP' not in ph and '(NEEDED)' not in dynamic, str(path)+' is dynamically linked'
    print(hashlib.sha256(data).hexdigest()+'  '+str(path))
