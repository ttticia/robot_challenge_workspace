"""CPU YOLOv5 ONNX inference and geometry; no ROS or motion commands."""
from pathlib import Path
import cv2
import numpy as np


def letterbox(image,size=640):
    h,w=image.shape[:2]
    scale=min(size/w,size/h)
    nw,nh=int(round(w*scale)),int(round(h*scale))
    left,top=(size-nw)//2,(size-nh)//2
    padded=np.full((size,size,3),114,dtype=np.uint8)
    padded[top:top+nh,left:left+nw]=cv2.resize(image,(nw,nh))
    blob=np.ascontiguousarray(padded[:,:,::-1].transpose(2,0,1)[None],dtype=np.float32)/255.
    return blob,scale,(left,top)


def person_boxes(predictions,image_shape,scale,padding,confidence=.5,iou=.45):
    rows=np.asarray(predictions)
    if rows.ndim==3:rows=rows[0]
    if rows.ndim!=2 or rows.shape[1]!=85:raise ValueError('Expected COCO YOLOv5 output [N,85]')
    selected=rows[(rows[:,4]*rows[:,5]>=confidence)&(np.argmax(rows[:,5:],axis=1)==0)]
    boxes=[];scores=[]
    h,w=image_shape[:2];left,top=padding
    for row in selected:
        x,y,bw,bh=map(float,row[:4])
        x1=float(np.clip((x-bw/2-left)/scale,0,w));y1=float(np.clip((y-bh/2-top)/scale,0,h))
        x2=float(np.clip((x+bw/2-left)/scale,0,w));y2=float(np.clip((y+bh/2-top)/scale,0,h))
        if not np.isfinite([x1,y1,x2,y2]).all() or x2<=x1 or y2<=y1:continue
        boxes.append([x1,y1,x2-x1,y2-y1]);scores.append(float(row[4]*row[5]))
    if not boxes:return []
    indices=np.asarray(cv2.dnn.NMSBoxes(boxes,scores,confidence,iou)).reshape(-1)
    return [dict(label='person',confidence=scores[i],bbox_xyxy=[boxes[i][0],boxes[i][1],boxes[i][0]+boxes[i][2],boxes[i][1]+boxes[i][3]]) for i in indices]


class PeopleDetector:
    def __init__(self,model,confidence=.5):
        import onnxruntime as ort
        model=Path(model).expanduser()
        if not model.is_file():raise FileNotFoundError('People model missing: '+str(model)+'. Run scripts/download_people_model.py.')
        options=ort.SessionOptions();options.intra_op_num_threads=2;options.inter_op_num_threads=1
        self.session=ort.InferenceSession(str(model),sess_options=options,providers=['CPUExecutionProvider'])
        self.input=self.session.get_inputs()[0].name
        self.confidence=confidence

    def detect(self,image):
        blob,scale,padding=letterbox(image)
        prediction=self.session.run(None,{self.input:blob})[0]
        return person_boxes(prediction,image.shape,scale,padding,self.confidence)
