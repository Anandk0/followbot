import cv2
import mediapipe as mp
import numpy as np

class MediaPipeCamera:
    def __init__(self, width=320, height=240, fps=30):
        self.width = width
        self.height = height
        self.fps = fps
        
        # Initialize MediaPipe camera
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_pose = mp.solutions.pose
        
        # Setup camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise RuntimeError("Cannot open camera")
            
        # Set camera properties for speed
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # Try to set format for speed
        try:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
        except:
            pass
            
        print(f"MediaPipe camera initialized: {width}x{height}@{fps}fps")
    
    def read_frame(self):
        """Read a frame from camera"""
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return None
        return frame
    
    def release(self):
        """Release camera resources"""
        if self.cap:
            self.cap.release()
    
    def is_opened(self):
        """Check if camera is opened"""
        return self.cap.isOpened()