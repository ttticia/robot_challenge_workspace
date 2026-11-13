"""CPU YOLOv5 ONNX inference. No network downloads at runtime."""
import cv2
import numpy as np

COCO = ('person bicycle car motorcycle airplane bus train truck boat traffic_light fire_hydrant '
        'stop_sign parking_meter bench bird cat dog horse sheep cow elephant bear zebra giraffe '
        'backpack umbrella handbag tie suitcase frisbee skis snowboard sports_ball kite baseball_bat '
        'baseball_glove skateboard surfboard tennis_racket bottle wine_glass cup fork knife spoon bowl '
        'banana apple sandwich orange broccoli carrot hot_dog pizza donut cake chair couch potted_plant '
        'bed dining_table toilet tv laptop mouse remote keyboard cell_phone microwave oven toaster sink '
        'refrigerator book clock vase scissors teddy_bear hair_drier toothbrush').split()


class Detector:
    def __init__(self, path, confidence=.4):
        import onnxruntime as ort
        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        self.session = ort.InferenceSession(str(path), sess_options=options, providers=['CPUExecutionProvider'])
        self.input = self.session.get_inputs()[0]
        self.dtype = np.float16 if self.input.type == 'tensor(float16)' else np.float32
        self.confidence = confidence

    def detect(self, image):
        h, w = image.shape[:2]
        scale = min(640/w, 640/h)
        nw, nh = round(w*scale), round(h*scale)
        left, top = (640-nw)//2, (640-nh)//2
        canvas = np.full((640, 640, 3), 114, np.uint8)
        canvas[top:top+nh, left:left+nw] = cv2.resize(image, (nw, nh))
        tensor = np.ascontiguousarray(canvas[:, :, ::-1].transpose(2, 0, 1)[None], dtype=self.dtype)/self.dtype(255.)
        predictions = self.session.run(None, {self.input.name: tensor})[0][0].astype(np.float32)
        ids = predictions[:, 5:].argmax(axis=1)
        scores = predictions[:, 4]*predictions[np.arange(len(predictions)), ids+5]
        keep = scores >= self.confidence
        predictions, ids, scores = predictions[keep], ids[keep], scores[keep]
        boxes = predictions[:, :4].copy()
        boxes[:, :2] -= boxes[:, 2:]/2
        boxes[:, [0, 2]] /= scale
        boxes[:, [1, 3]] /= scale
        boxes[:, 0] -= left/scale
        boxes[:, 1] -= top/scale
        output = []
        for category in np.unique(ids):
            indices = np.flatnonzero(ids == category)
            selected = cv2.dnn.NMSBoxes(boxes[indices].tolist(), scores[indices].tolist(), self.confidence, .45)
            for j in np.asarray(selected, dtype=int).reshape(-1):
                i = indices[j]
                x, y, bw, bh = boxes[i]
                x1, y1, x2, y2 = max(0., x), max(0., y), min(float(w), x+bw), min(float(h), y+bh)
                if x2 > x1 and y2 > y1:
                    output.append(dict(label=COCO[int(ids[i])], confidence=float(scores[i]),
                                       box=[float(x1), float(y1), float(x2), float(y2)]))
        return output


def scene_events(detections):
    """Evidence-based room candidates; COCO people are possible survivors only."""
    cues = {'bed': 'bedroom', 'toilet': 'bathroom', 'couch': 'living_room',
            'microwave': 'kitchen', 'oven': 'kitchen', 'refrigerator': 'kitchen', 'sink': 'kitchen'}
    events = []
    rooms = {}
    for d in detections:
        if d['label'] == 'person':
            events.append(dict(kind='survivor', label='possible_survivor', confidence=d['confidence'],
                               box=d['box'], evidence='COCO person detection; mannequin recognition is unvalidated'))
        room = cues.get(d['label'])
        if room:
            rooms.setdefault(room, []).append(d)
    for room, objects in rooms.items():
        events.append(dict(kind='room', label=room, confidence=max(d['confidence'] for d in objects)*.8,
                           evidence='Room candidate from visible objects: '+', '.join(sorted({d['label'] for d in objects}))))
    return events
