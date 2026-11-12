#!/usr/bin/env python3
"""Fetch the pinned manufacturer-independent COCO person detector weights."""
import argparse,hashlib,urllib.request
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--output',default=str(Path.home()/'challenge_ws/models/yolov5n.onnx'));a=p.parse_args()
url='https://github.com/ultralytics/yolov5/releases/download/v6.0/yolov5n.onnx'
expected='6c686f5c56c5da87c6445a2f7bd2dc2d6b664325368d9de1fd7c05100b9e442e'
out=Path(a.output).expanduser();out.parent.mkdir(parents=True,exist_ok=True)
data=urllib.request.urlopen(url,timeout=60).read()
if hashlib.sha256(data).hexdigest()!=expected:raise RuntimeError('Model checksum differs from pinned YOLOv5 v6.0 asset')
part=out.with_suffix('.tmp');part.write_bytes(data);part.replace(out);print(out)
