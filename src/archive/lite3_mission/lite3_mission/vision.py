"""Image/QR and lab-client parsing helpers, with no ROS or network side effects."""
import json
import cv2
import numpy as np


def bgr_image(data,width,height,step,encoding):
    channels=3 if encoding in ('rgb8','bgr8') else 1 if encoding=='mono8' else 0
    if not channels or step<width*channels or len(data)<step*height:
        raise ValueError('Unsupported image encoding or invalid row layout')
    rows=np.frombuffer(data,dtype=np.uint8,count=step*height).reshape(height,step)
    pixels=rows[:,:width*channels].reshape(height,width,channels).copy()
    if encoding=='rgb8':return cv2.cvtColor(pixels,cv2.COLOR_RGB2BGR)
    if encoding=='mono8':return cv2.cvtColor(pixels,cv2.COLOR_GRAY2BGR)
    return pixels


def qr_position(corners,intrinsics,distortion,size):
    if not .02<=size<=.5:raise ValueError('QR size must be the measured black-symbol side length, 0.02–0.5 m')
    image=np.asarray(corners,dtype=np.float64).reshape(4,2)
    matrix=np.asarray(intrinsics,dtype=np.float64).reshape(3,3)
    if not np.isfinite(image).all() or not np.isfinite(matrix).all() or matrix[0,0]<=0 or matrix[1,1]<=0:
        raise ValueError('Invalid camera calibration or QR corners')
    half=size/2
    object_points=np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]],dtype=np.float64)
    d=np.asarray(distortion,dtype=float) if len(distortion) else None
    result=cv2.solvePnPGeneric(object_points,image,matrix,d,flags=cv2.SOLVEPNP_IPPE_SQUARE)
    valid=[]
    if result[0]:
        for rvec,tvec in zip(result[1],result[2]):
            position=tvec.reshape(3)
            projected,_=cv2.projectPoints(object_points,rvec,tvec,matrix,d)
            error=float(np.sqrt(np.mean(np.sum((projected.reshape(4,2)-image)**2,axis=1))))
            if np.isfinite(position).all() and .08<position[2]<8. and error<=3.:
                valid.append((error,position.tolist()))
    if not valid:raise ValueError('No valid positive-depth QR pose within reprojection tolerance')
    return min(valid,key=lambda value:value[0])


def parse_client_output(text):
    decoder=json.JSONDecoder()
    for index,char in enumerate(text):
        if char!='{':continue
        try:value,_=decoder.raw_decode(text[index:])
        except ValueError:continue
        if not isinstance(value,dict) or 'success' not in value:continue
        if value.get('success') is not True:raise ValueError('Scene service rejected request: '+str(value.get('error')))
        result=value.get('result')
        if isinstance(result,str):result=json.loads(result)
        if not isinstance(result,dict):raise ValueError('Scene result is not structured JSON')
        confidence=result.get('confidence',0.)
        if isinstance(confidence,bool) or not isinstance(confidence,(float,int)) or not np.isfinite(confidence) or not 0<=confidence<=1:
            raise ValueError('Invalid scene confidence')
        label=result.get('room_type','unknown')
        if label not in ('kitchen','living_room','bedroom','bathroom','other','unknown'):
            raise ValueError('Unknown room label')
        present=result.get('survivor_present',False)
        if not isinstance(present,bool):raise ValueError('survivor_present must be a boolean')
        return dict(room_type=label,survivor_present=present,confidence=float(confidence),evidence=result.get('evidence',[]))
    raise ValueError('No lab scene-service JSON response found')
