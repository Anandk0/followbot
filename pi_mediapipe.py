import cv2
import mediapipe as mp
import serial
import time
import math

class FollowBot:
    def __init__(self, serial_port='/dev/ttyUSB0', baud=115200):
        # MediaPipe setup
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            enable_segmentation=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils
        
        # Serial connection to ESP8266
        try:
            self.ser = serial.Serial(serial_port, baud, timeout=0.1)
            print(f"Connected to ESP8266 on {serial_port}")
        except:
            print(f"Failed to connect to {serial_port}")
            self.ser = None
        
        # Control parameters
        self.target_area = 15000  # Target person area (pixels)
        self.center_deadzone = 50  # Pixels from center
        self.base_speed = 40
        self.max_speed = 80
        self.turn_speed = 60
        
        # Safety
        self.min_distance = 30.0  # cm
        self.last_distance = 999.0
        
    def get_person_bbox(self, results, frame_width, frame_height):
        """Extract person bounding box from MediaPipe pose landmarks"""
        if not results.pose_landmarks:
            return None
            
        landmarks = results.pose_landmarks.landmark
        
        # Get all visible landmarks
        x_coords = []
        y_coords = []
        
        for lm in landmarks:
            if lm.visibility > 0.5:  # Only use visible landmarks
                x_coords.append(int(lm.x * frame_width))
                y_coords.append(int(lm.y * frame_height))
        
        if len(x_coords) < 4:  # Need minimum landmarks
            return None
            
        # Calculate bounding box
        x1, x2 = min(x_coords), max(x_coords)
        y1, y2 = min(y_coords), max(y_coords)
        
        # Add padding
        padding = 20
        x1 = max(0, x1 - padding)
        y1 = max(0, y1 - padding)
        x2 = min(frame_width, x2 + padding)
        y2 = min(frame_height, y2 + padding)
        
        width = x2 - x1
        height = y2 - y1
        area = width * height
        center_x = x1 + width // 2
        center_y = y1 + height // 2
        
        return {
            'bbox': (x1, y1, width, height),
            'center': (center_x, center_y),
            'area': area
        }
    
    def calculate_motor_speeds(self, person_data, frame_width):
        """Calculate left and right motor speeds based on person position"""
        if not person_data:
            return 0, 0, "NO PERSON"
            
        center_x, center_y = person_data['center']
        area = person_data['area']
        
        # Calculate horizontal error (person position relative to center)
        frame_center = frame_width // 2
        error_x = center_x - frame_center
        
        # Check if person is centered
        if abs(error_x) < self.center_deadzone:
            # Person is centered, check distance
            if area > self.target_area:
                # Too close - back up
                return -self.base_speed, -self.base_speed, "BACKING UP"
            elif area < self.target_area * 0.6:
                # Too far - move forward
                return self.base_speed, self.base_speed, "MOVING FORWARD"
            else:
                # Good distance - stop
                return 0, 0, "GOOD DISTANCE"
        else:
            # Person not centered - turn towards them
            turn_intensity = min(abs(error_x) / (frame_width // 4), 1.0)
            turn_speed = int(self.turn_speed * turn_intensity)
            
            if error_x > 0:  # Person on right
                return turn_speed, -turn_speed, "TURNING RIGHT"
            else:  # Person on left
                return -turn_speed, turn_speed, "TURNING LEFT"
    
    def send_motor_command(self, left_speed, right_speed):
        """Send motor command to ESP8266"""
        if self.ser:
            cmd = f"MOVE:{left_speed},{right_speed}\n"
            self.ser.write(cmd.encode())
    
    def stop_motors(self):
        """Stop all motors"""
        if self.ser:
            self.ser.write(b"STOP\n")
    
    def read_distance(self):
        """Read distance from ESP8266"""
        if self.ser and self.ser.in_waiting:
            try:
                line = self.ser.readline().decode().strip()
                if line.startswith("DIST:"):
                    self.last_distance = float(line[5:])
            except:
                pass
        return self.last_distance
    
    def draw_info(self, frame, person_data, status, distance):
        """Draw information on frame"""
        # Draw person bounding box
        if person_data:
            x, y, w, h = person_data['bbox']
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Draw center point
            center_x, center_y = person_data['center']
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
            
            # Draw area info
            cv2.putText(frame, f"Area: {person_data['area']}", 
                       (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Draw center line
        h, w = frame.shape[:2]
        cv2.line(frame, (w//2, 0), (w//2, h), (255, 0, 0), 1)
        
        # Draw status and distance
        cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"Distance: {distance:.1f}cm", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    def run(self):
        """Main loop"""
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        print("Starting FollowBot...")
        
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    continue
                
                # Convert BGR to RGB for MediaPipe
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = self.pose.process(rgb_frame)
                
                # Get person data
                h, w = frame.shape[:2]
                person_data = self.get_person_bbox(results, w, h)
                
                # Read distance sensor
                distance = self.read_distance()
                
                # Safety check - stop if obstacle too close
                if distance < self.min_distance:
                    self.stop_motors()
                    status = f"OBSTACLE {distance:.1f}cm"
                    left_speed = right_speed = 0
                else:
                    # Calculate motor speeds
                    left_speed, right_speed, status = self.calculate_motor_speeds(person_data, w)
                    self.send_motor_command(left_speed, right_speed)
                
                # Draw pose landmarks
                if results.pose_landmarks:
                    self.mp_draw.draw_landmarks(frame, results.pose_landmarks, 
                                              self.mp_pose.POSE_CONNECTIONS)
                
                # Draw info overlay
                self.draw_info(frame, person_data, status, distance)
                
                # Show frame
                cv2.imshow('FollowBot', frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        finally:
            self.stop_motors()
            cap.release()
            cv2.destroyAllWindows()
            if self.ser:
                self.ser.close()

if __name__ == "__main__":
    # Update serial port for your system
    # Windows: 'COM3', 'COM4', etc.
    # Linux/Pi: '/dev/ttyUSB0', '/dev/ttyACM0', etc.
    bot = FollowBot(serial_port='/dev/ttyUSB0')
    bot.run()