import cv2

def draw_vis(frame, bbox, status):
    if bbox:
        x,y,w,h = bbox
        cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
        cx, cy = x+w//2, y+h//2
        cv2.circle(frame, (cx,cy), 5, (0,0,255), -1)
    cv2.putText(frame, status, (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

