import numpy as np, cv2
try:
    import tflite_runtime.interpreter as tflite
except ImportError:
    import tensorflow.lite as tflite

class MoveNetDetector:
    def __init__(self, model_path:str, input_size:int=192, score_threshold:float=0.2):
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
        if (scores < self.score_threshold).all():
            return None

        xs = (kps[:,1] * w).clip(0, w-1)
        ys = (kps[:,0] * h).clip(0, h-1)
        x1, x2 = np.min(xs), np.max(xs)
        y1, y2 = np.min(ys), np.max(ys)
        bw, bh = (x2-x1), (y2-y1)
        if bw < 10 or bh < 10:
            return None
        cx, cy = int((x1+x2)/2), int((y1+y2)/2)
        return {'bbox': (int(x1), int(y1), int(bw), int(bh)),
                'center': (cx, cy), 'score': float(np.mean(scores))}

