import numpy as np
from team_perception.people import letterbox,person_boxes


def test_people_unpad_clip_and_nms_preserve_only_person_class():
    image=np.zeros((480,640,3),np.uint8);blob,scale,pad=letterbox(image)
    assert blob.shape==(1,3,640,640) and pad==(0,80)
    rows=np.zeros((3,85),np.float32)
    rows[:,:4]=[320,320,100,200];rows[:,4]=.9
    rows[0,5]=.9;rows[1,5]=.8;rows[2,7]=.99
    boxes=person_boxes(rows,image.shape,scale,pad)
    assert len(boxes)==1
    assert np.allclose(boxes[0]['bbox_xyxy'],[270,140,370,340])
    assert boxes[0]['label']=='person'


def test_empty_people_predictions_are_a_valid_empty_frame():
    assert person_boxes(np.zeros((1,0,85)),(480,640,3),1.,(0,80))==[]
