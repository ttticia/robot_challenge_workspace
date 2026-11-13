#!/usr/bin/env python3
"""Sample recorded RGB frames through the actual CPU model, without live commands."""
import argparse
import json
from pathlib import Path
import sqlite3
import time
import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage
from lite3_maze.object_detector import Detector

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('bag',type=Path)
p.add_argument('--output',type=Path,default=Path('docs/validation/yolo_bag.json'))
a=p.parse_args();root=Path(__file__).resolve().parents[1]
model=Detector(root/'src/lite3_maze/models/yolov5n.onnx')
db=sqlite3.connect(a.bag.resolve().as_uri()+'?mode=ro',uri=True)
rows=db.execute("SELECT timestamp,data FROM messages WHERE topic_id=(SELECT id FROM topics WHERE name='/camera/color/image_raw/compressed') ORDER BY timestamp")
next_stamp=0;frames=person_frames=0;times=[];best=None;best_frame=None
for stamp,raw in rows:
 if stamp<next_stamp:continue
 next_stamp=stamp+1_000_000_000
 msg=deserialize_message(raw,CompressedImage)
 frame=cv2.imdecode(np.frombuffer(msg.data,np.uint8),cv2.IMREAD_COLOR)
 start=time.monotonic();objects=model.detect(frame);times.append(time.monotonic()-start);frames+=1
 people=[o for o in objects if o['label']=='person']
 if people:
  person_frames+=1
  score=max(o['confidence'] for o in people)
  if best is None or score>best['confidence']:
   best=dict(confidence=score,frame_id=msg.header.frame_id,stamp_ns=stamp,people=people)
   best_frame=frame.copy()
result=dict(sample_interval_sec=1.,frames_processed=frames,frames_with_person_detections=person_frames,
    mean_inference_seconds=float(np.mean(times)),maximum_inference_seconds=max(times),best_detection=best,
    note='Model outputs on recorded images; not a measurement of detection accuracy or 3D location.')
a.output.write_text(json.dumps(result,indent=2)+'\n')
if best is not None:
 for d in best['people']:
  x1,y1,x2,y2=map(int,d['box']);cv2.rectangle(best_frame,(x1,y1),(x2,y2),(40,240,60),2)
 cv2.imwrite(str(a.output.with_suffix('.jpg')),best_frame)
print(json.dumps(result,indent=2));db.close()
