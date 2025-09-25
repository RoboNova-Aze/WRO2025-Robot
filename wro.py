import cv2
import numpy as np
import time

# ===========================
# HSV Thresholds (kalibrlənə bilər)
# ===========================
HSV_CONFIG = {
    "red_lower1": np.array([0, 120, 70]),
    "red_upper1": np.array([10, 255, 255]),
    "red_lower2": np.array([170,120,70]),
    "red_upper2": np.array([180,255,255]),
    "green_lower": np.array([40, 50, 50]),
    "green_upper": np.array([90, 255, 255])
}

# ===========================
# Object Detection
# ===========================
def detect_object(frame, color):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    if color == "red":
        mask1 = cv2.inRange(hsv, HSV_CONFIG["red_lower1"], HSV_CONFIG["red_upper1"])
        mask2 = cv2.inRange(hsv, HSV_CONFIG["red_lower2"], HSV_CONFIG["red_upper2"])
        mask = cv2.bitwise_or(mask1, mask2)
    else:  # green
        mask = cv2.inRange(hsv, HSV_CONFIG["green_lower"], HSV_CONFIG["green_upper"])

    # Morphology to remove noise
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) > 500:  # minimum area threshold
            x, y, w, h = cv2.boundingRect(c)
            center = (x + w//2, y + h//2)
            # Draw rectangle and center
            cv2.rectangle(frame, (x,y), (x+w, y+h), (0,255,0), 2)
            cv2.circle(frame, center, 5, (255,0,0), -1)
            return center
    return None

# ===========================
# Main Loop
# ===========================
cap = cv2.VideoCapture(0)
cap.set(3, 320)  # width
cap.set(4, 240)  # height

prev_time = time.time()
fps = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    red_center = detect_object(frame, "red")
    green_center = detect_object(frame, "green")

    # FPS calculation
    curr_time = time.time()
    fps = 0.9*fps + 0.1*(1/(curr_time-prev_time))
    prev_time = curr_time

    # Overlay info
    cv2.putText(frame, f"FPS: {int(fps)}", (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)
    
    # Show frame
    cv2.imshow("Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
