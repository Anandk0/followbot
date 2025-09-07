import numpy as np, cv2
try:
    import tflite_runtime.interpreter as tflite
except ImportError:
    import tensorflow.lite as tflite

class MoveNetDetector:
    def __init__(self, model_path:str, input_size:int=192, score_threshold:float=0.3):
        self.input_size = input_size
        self.score_threshold = score_threshold
        self.interp = tflite.Interpreter(model_path=model_path, num_threads=2)
        self.interp.allocate_tensors()
        self.in_idx = self.interp.get_input_details()[0]['index']
        self.out_idx = self.interp.get_output_details()[0]['index']

    def _pre(self, frame_bgr):
        h, w = frame_bgr.shape[:2]
        s = self.input_size
        img = cv2.resize(frame_bgr, (s, s))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.uint8)[None, ...]
        return img, (w, h)

    def infer(self, frame_bgr):
        """
        Returns: dict or None
          dict = { 'bbox': (x,y,w,h), 'center': (cx,cy), 'score': float }
        """
        inp, (w, h) = self._pre(frame_bgr)
        self.interp.set_tensor(self.in_idx, inp)
        self.interp.invoke()
        out = self.interp.get_tensor(self.out_idx)  # [1,1,17,3] -> [y,x,score]
        kps = out[0,0,:,:]  # 17x3
        scores = kps[:,2]
        
        # Check if enough keypoints are detected with good confidence
        valid_kps = scores >= self.score_threshold
        if np.sum(valid_kps) < 5:  # Need at least 5 keypoints for human
            return None
            
        # Validate human-like structure (head, shoulders, hips)
        nose, left_eye, right_eye = scores[0], scores[1], scores[2]
        left_shoulder, right_shoulder = scores[5], scores[6]
        left_hip, right_hip = scores[11], scores[12]
        
        # Must have head (nose or eyes) and torso (shoulders or hips)
        has_head = (nose > self.score_threshold) or (left_eye > self.score_threshold) or (right_eye > self.score_threshold)
        has_torso = (left_shoulder > self.score_threshold) or (right_shoulder > self.score_threshold) or (left_hip > self.score_threshold) or (right_hip > self.score_threshold)
        
        if not (has_head and has_torso):
            return None

        xs = (kps[valid_kps,1] * w).clip(0, w-1)
        ys = (kps[valid_kps,0] * h).clip(0, h-1)
        x1, x2 = np.min(xs), np.max(xs)
        y1, y2 = np.min(ys), np.max(ys)
        bw, bh = (x2-x1), (y2-y1)
        
        # Human proportions check
        if bw < 20 or bh < 40 or bh/bw < 1.2:  # Humans are taller than wide
            return None
            
        cx, cy = int((x1+x2)/2), int((y1+y2)/2)
        return {'bbox': (int(x1), int(y1), int(bw), int(bh)),
                'center': (cx, cy), 'score': float(np.mean(scores[valid_kps]))}

