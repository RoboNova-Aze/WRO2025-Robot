import cv2
import numpy as np
import time
import argparse
import serial
import math

# --- Global Configuration and Constants ---

# Define the acceptable colors and their steering rules
COLOR_CONFIG = {
    'red': {
        'hsv_min': np.array([0, 100, 100]),
        'hsv_max': np.array([10, 255, 255]),
        'pass_side': 'RIGHT',  # Pass on the right
        'turn_direction': 'LEFT',  # Turn left to pass on the right
    },
    'green': {
        'hsv_min': np.array([35, 100, 100]),
        'hsv_max': np.array([85, 255, 255]),
        'pass_side': 'LEFT',  # Pass on the left
        'turn_direction': 'RIGHT',  # Turn right to pass on the left
    },
    'red_wrap': { # Handle the wrap-around HUE for red (170-180)
        'hsv_min': np.array([170, 100, 100]),
        'hsv_max': np.array([180, 255, 255]),
    }
}

# Physical properties of the columns (for simple distance estimation / filtering)
COLUMN_HEIGHT_CM = 10.0
# Assumed vertical field of view (VFOV) in degrees for simple focal length calculation
# This value will need tuning based on the specific camera.
ASSUMED_VFOV = 45.0
# Width of the desired pass gap relative to the center (normalized to image width, 0.0 to 1.0)
TARGET_GAP_OFFSET = 0.2

# Serial communication settings
SERIAL_PORT = '/dev/ttyACM0'  # Use '/dev/ttyUSB0' or 'COM3' etc. for Raspberry Pi/Desktop
BAUD_RATE = 9600
SERIAL_TIMEOUT = 0.1

class ColorCalibrator:
    """
    Handles interactive HSV range calibration using OpenCV Trackbars.
    This runs in a separate loop for setup/debugging.
    """
    def __init__(self, color_name):
        self.color_name = color_name
        self.window_name = f'HSV Calibration: {color_name}'
        cv2.namedWindow(self.window_name)
        
        # Initialize default values (using red's primary range as a template)
        h_min, s_min, v_min = COLOR_CONFIG[color_name].get('hsv_min', np.array([0, 100, 100]))
        h_max, s_max, v_max = COLOR_CONFIG[color_name].get('hsv_max', np.array([10, 255, 255]))

        cv2.createTrackbar('H Min', self.window_name, h_min, 180, self._set_h_min)
        cv2.createTrackbar('H Max', self.window_name, h_max, 180, self._set_h_max)
        cv2.createTrackbar('S Min', self.window_name, s_min, 255, self._set_s_min)
        cv2.createTrackbar('S Max', self.window_name, s_max, 255, self._set_s_max)
        cv2.createTrackbar('V Min', self.window_name, v_min, 255, self._set_v_min)
        cv2.createTrackbar('V Max', self.window_name, v_max, 255, self._set_v_max)

    def _set_h_min(self, val): COLOR_CONFIG[self.color_name]['hsv_min'][0] = val
    def _set_h_max(self, val): COLOR_CONFIG[self.color_name]['hsv_max'][0] = val
    def _set_s_min(self, val): COLOR_CONFIG[self.color_name]['hsv_min'][1] = val
    def _set_s_max(self, val): COLOR_CONFIG[self.color_name]['hsv_max'][1] = val
    def _set_v_min(self, val): COLOR_CONFIG[self.color_name]['hsv_min'][2] = val
    def _set_v_max(self, val): COLOR_CONFIG[self.color_name]['hsv_max'][2] = val

    def get_hsv_ranges(self):
        """Returns the current HSV min/max arrays."""
        return (COLOR_CONFIG[self.color_name]['hsv_min'], COLOR_CONFIG[self.color_name]['hsv_max'])

class ColumnDetector:
    """
    Performs real-time detection, filtering, and calculation for a single color.
    """
    def __init__(self, color_name, config):
        self.color_name = color_name
        self.config = config

    def _calculate_distance_cm(self, pixel_height, frame_height):
        """
        Approximates the distance (in cm) based on pixel height and assumed VFOV.
        This uses simple pinhole camera geometry, assuming the column height is constant.
        Distance D = (H * f) / h, where H is real height, h is pixel height, f is focal length.
        focal length 'f' is estimated from VFOV and frame height.
        """
        if pixel_height < 5:
            return 999.0 # Effectively infinity
        
        # Estimate focal length (f_pix = frame_height / (2 * tan(VFOV/2)))
        f_pix = frame_height / (2 * math.tan(math.radians(ASSUMED_VFOV / 2)))
        
        # Distance (D) = (Real Height * Estimated Focal Length) / Pixel Height
        distance = (COLUMN_HEIGHT_CM * f_pix) / pixel_height
        
        return distance

    def detect(self, frame, hsv_frame):
        """
        Detects columns of the configured color in the given frame.
        Returns a list of detected objects (x, y, w, h, distance_cm).
        """
        detected_columns = []
        hsv_min = self.config['hsv_min']
        hsv_max = self.config['hsv_max']
        frame_h, frame_w = frame.shape[:2]

        # 1. Create HSV Mask
        mask = cv2.inRange(hsv_frame, hsv_min, hsv_max)
        
        # Handle red wrap-around (if color is red and red_wrap is defined)
        if self.color_name == 'red' and 'red_wrap' in COLOR_CONFIG:
            mask2 = cv2.inRange(hsv_frame, COLOR_CONFIG['red_wrap']['hsv_min'], COLOR_CONFIG['red_wrap']['hsv_max'])
            mask = cv2.bitwise_or(mask, mask2)
        
        # Morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 2. Find Contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            # Filter 1: Area filtering (avoid small noise)
            if area < 500: # Minimum acceptable pixel area
                continue

            # Get bounding box (x, y, w, h)
            x, y, w, h = cv2.boundingRect(contour)
            
            # Filter 2: Shape filtering (aspect ratio for cubes/columns)
            aspect_ratio = w / float(h)
            # A tall column should have a small aspect ratio (w/h < 1)
            # We look for something tall, not necessarily wide.
            if aspect_ratio > 1.0 or aspect_ratio < 0.2: 
                continue

            # Calculate centroid (position)
            center_x = x + w // 2
            center_y = y + h // 2
            
            # Calculate approximate distance
            distance_cm = self._calculate_distance_cm(h, frame_h)

            detected_columns.append({
                'color': self.color_name,
                'bbox': (x, y, w, h),
                'centroid': (center_x, center_y),
                'distance_cm': distance_cm,
                'area': area
            })
            
        return detected_columns

class SteeringController:
    """
    Decides the optimal steering command based on detected columns.
    """
    def __init__(self, frame_width):
        self.frame_width = frame_width
        self.center_x = frame_width / 2

    def calculate_command(self, columns):
        """
        Calculates the required steering angle and motor speed.
        Returns (steering_angle_normalized, motor_speed_normalized, confidence_score, debug_msg).
        Steering: -1.0 (Full Left) to 1.0 (Full Right).
        Motor: 0.0 (Stop) to 1.0 (Full Speed).
        """
        if not columns:
            # No columns detected, assume straight line movement
            return 0.0, 0.4, 0.0, "STRAIGHT (No Obstacle)"

        # Sort columns by distance (closest first)
        columns.sort(key=lambda c: c['distance_cm'])
        closest_column = columns[0]
        
        color = closest_column['color']
        centroid_x = closest_column['centroid'][0]
        distance_cm = closest_column['distance_cm']
        
        config = COLOR_CONFIG[color]
        pass_side = config['pass_side']
        turn_direction = config['turn_direction']

        # 1. Calculate Target Gap Position
        # Normalized position of the column center (-1.0 to 1.0 from center)
        normalized_column_pos = (centroid_x - self.center_x) / self.center_x
        
        # Calculate the required target position based on which side we need to pass
        if pass_side == 'RIGHT':
            # We want the column to be to our left (normalized position should be NEGATIVE)
            # We target a normalized position of -TARGET_GAP_OFFSET for the column centroid.
            target_pos = -TARGET_GAP_OFFSET
        else: # pass_side == 'LEFT'
            # We want the column to be to our right (normalized position should be POSITIVE)
            # We target a normalized position of +TARGET_GAP_OFFSET for the column centroid.
            target_pos = TARGET_GAP_OFFSET

        # 2. Calculate Error and Steering Command
        error = target_pos - normalized_column_pos
        
        # Proportional control (P-controller) for steering
        # Simple P gain (K_p). Tune this value (e.g., 0.5 to 1.5)
        P_GAIN = 1.0 
        steering_normalized = np.clip(P_GAIN * error, -1.0, 1.0) # Clamp between -1.0 (Left) and 1.0 (Right)

        # 3. Calculate Motor Speed (Slow down when close)
        # Minimum speed is 0.2, maximum is 0.5
        min_speed = 0.2
        max_speed = 0.5
        # Scale speed based on inverse distance (closer = slower)
        # Use a distance threshold (e.g., 100 cm)
        speed_factor = np.clip(distance_cm / 100.0, 0.2, 1.0)
        motor_normalized = min_speed + (max_speed - min_speed) * speed_factor

        # 4. Calculate Confidence Score
        # Confidence is higher when the object is closer and larger (more certain detection)
        # Normalized inverse distance (1.0 = very close/high confidence, 0.0 = far/low confidence)
        confidence = np.clip(1.0 - (distance_cm / 200.0), 0.0, 1.0) # Assume 200cm is the max range
        
        debug_msg = f"TARGET: {color} @ {distance_cm:.1f}cm. Pass {pass_side}. Steer: {turn_direction} ({steering_normalized:.2f})"
        
        return steering_normalized, motor_normalized, confidence, debug_msg


class PerformanceMonitor:
    """Tracks FPS and provides frame skipping logic."""
    def __init__(self, target_ms=100):
        self.target_ms = target_ms  # Target processing time per frame
        self.last_time = time.time()
        self.frame_count = 0
        self.fps = 0.0
        self.skip_frame = False

    def start_frame(self):
        """Called at the start of the frame loop."""
        self.frame_count += 1
        current_time = time.time()
        
        # Calculate FPS every 10 frames
        if self.frame_count % 10 == 0:
            elapsed = current_time - self.last_time
            if elapsed > 0:
                self.fps = 10 / elapsed
            self.last_time = current_time
            
        # Check if we should skip the next frame (simple high-load check)
        self.skip_frame = (self.fps > 0) and (1000 / self.fps > self.target_ms * 1.5)

    def get_status(self):
        """Returns the current FPS and skip status."""
        status_color = (0, 255, 0) if self.fps >= (1000 / self.target_ms) else (0, 0, 255)
        warning = " (SKIPPING)" if self.skip_frame else ""
        return f"FPS: {self.fps:.1f}{warning}", status_color

class RobotCommander:
    """Handles serial communication to the Arduino."""
    def __init__(self, port, baud_rate, is_mock=False):
        self.is_mock = is_mock
        self.serial_connection = None
        self.port = port
        self.baud_rate = baud_rate
        
        print(f"Commander initialized. Mock mode: {self.is_mock}")
        if not self.is_mock:
            try:
                self.serial_connection = serial.Serial(port, baud_rate, timeout=SERIAL_TIMEOUT)
                print(f"Serial connection established on {port} at {baud_rate}.")
            except serial.SerialException as e:
                print(f"WARNING: Could not open serial port {port}. Running in mock mode. Error: {e}")
                self.is_mock = True

    def send_command(self, steering_normalized, motor_normalized):
        """
        Sends a command string to the Arduino using the format: S<float>,M<float>\n
        Steering: -1.0 to 1.0. Motor: 0.0 to 1.0.
        """
        # Format the command string
        command = f"S{steering_normalized:.2f},M{motor_normalized:.2f}\n"
        
        if self.is_mock:
            # Mock mode: print the command to the console
            print(f"MOCK SERIAL TX: {command.strip()}")
        else:
            # Real mode: send via serial
            try:
                self.serial_connection.write(command.encode('utf-8'))
            except serial.SerialException as e:
                print(f"ERROR: Serial write failed: {e}")
                self.serial_connection.close() # Attempt to close and let main loop handle re-init if needed
                
class MainApp:
    """Main application class to orchestrate all components and the mission."""
    def __init__(self, camera_id=0, mock_serial=False, calibration_mode=None):
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            print(f"ERROR: Cannot open camera {camera_id}")
            exit()
            
        # Set frame resolution for consistency
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.monitor = PerformanceMonitor()
        self.commander = RobotCommander(SERIAL_PORT, BAUD_RATE, mock_serial)
        self.steerer = SteeringController(self.frame_width)
        
        # Initialize detectors for Red and Green
        self.detectors = {
            'red': ColumnDetector('red', COLOR_CONFIG['red']),
            'green': ColumnDetector('green', COLOR_CONFIG['green']),
        }

        # Mission State
        self.laps_completed = 0
        self.mission_state = "RACE_LAPS" # RACE_LAPS, FIND_MAGENTA, STOP
        self.MAGENTA_HSV_MIN = np.array([130, 50, 100]) # Approx magenta (H: 130-160)
        self.MAGENTA_HSV_MAX = np.array([160, 255, 255])
        self.magenta_column_detected = False

        if calibration_mode:
            self.run_calibration(calibration_mode)
        else:
            self.run_racing_loop()

    def _process_frame(self, frame):
        """Processes a single frame for detection and command generation."""
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        all_columns = []
        
        # 1. Detection Phase
        for name, detector in self.detectors.items():
            all_columns.extend(detector.detect(frame, hsv_frame))

        # 2. Mission Logic & Steering Command
        steering_norm, motor_norm, confidence, debug_msg = 0.0, 0.0, 0.0, "System Ready"

        if self.mission_state == "RACE_LAPS":
            # Find the closest column (Red or Green)
            steering_norm, motor_norm, confidence, debug_msg = self.steerer.calculate_command(all_columns)

            # Simple Lap Counter (TBD: Needs real external sensor/GPS to count laps)
            # For demonstration, we'll assume a lap is completed after X seconds or some event.
            if self.monitor.frame_count > 1000 and self.laps_completed < 3: # Example threshold
                 self.laps_completed += 1
                 print(f"*** LAP {self.laps_completed} COMPLETED ***")
                 if self.laps_completed >= 3:
                     self.mission_state = "FIND_MAGENTA"
                     print("MISSION STATE CHANGE: FIND_MAGENTA")
        
        elif self.mission_state == "FIND_MAGENTA":
            # Look for magenta columns
            magenta_mask = cv2.inRange(hsv_frame, self.MAGENTA_HSV_MIN, self.MAGENTA_HSV_MAX)
            magenta_contours, _ = cv2.findContours(magenta_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            largest_magenta = None
            max_area = 0
            
            for contour in magenta_contours:
                area = cv2.contourArea(contour)
                if area > 1000 and area > max_area:
                    max_area = area
                    largest_magenta = cv2.boundingRect(contour)

            if largest_magenta:
                x, y, w, h = largest_magenta
                center_x = x + w // 2
                
                # Center on magenta column area
                error = (center_x - self.steerer.center_x) / self.steerer.center_x
                P_GAIN = 1.0 
                steering_norm = np.clip(P_GAIN * error, -1.0, 1.0)
                motor_norm = 0.3 # Slow speed for approach
                debug_msg = f"FIND MAGENTA: Centered @ {center_x}. W:{w}"
                
                # Stop condition: Magenta column occupies a large part of the screen width/height
                if h > self.frame_height * 0.7 or w > self.frame_width * 0.7:
                     self.mission_state = "STOP"
                     print("MISSION STATE CHANGE: STOP")
            else:
                 # Default straight line search
                 steering_norm, motor_norm = 0.0, 0.3
                 debug_msg = "FIND MAGENTA: Searching..."

        elif self.mission_state == "STOP":
            steering_norm, motor_norm = 0.0, 0.0 # Full stop
            debug_msg = "MISSION COMPLETE. STOP."

        # 3. Command Execution
        self.commander.send_command(steering_norm, motor_norm)

        # 4. Visual Debug Overlay
        frame = self._draw_overlay(frame, all_columns, steering_norm, motor_norm, confidence, debug_msg)
        
        return frame

    def _draw_overlay(self, frame, columns, steering_norm, motor_norm, confidence, debug_msg):
        """Adds visual annotations to the frame for debugging."""
        
        # Draw frame center
        center_x = self.frame_width // 2
        cv2.line(frame, (center_x, 0), (center_x, self.frame_height), (255, 255, 255), 1)

        for col in columns:
            x, y, w, h = col['bbox']
            center_x, center_y = col['centroid']
            
            # Determine BGR color for overlay
            if col['color'] == 'red':
                color = (0, 0, 255)
            elif col['color'] == 'green':
                color = (0, 255, 0)
            else:
                color = (255, 255, 255) # White fallback

            # Bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            # Centroid
            cv2.circle(frame, (center_x, center_y), 5, color, -1)
            # Info text
            text = f"{col['color'].upper()} D:{col['distance_cm']:.1f}cm"
            cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Performance and Command Overlay
        fps_text, fps_color = self.monitor.get_status()
        cv2.putText(frame, fps_text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, fps_color, 2)
        cv2.putText(frame, f"Mission: {self.mission_state} | Laps: {self.laps_completed}/3", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(frame, f"Steering: {steering_norm:.2f} | Motor: {motor_norm:.2f}", (10, self.frame_height - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
        cv2.putText(frame, f"Confidence: {confidence:.2f}", (10, self.frame_height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(frame, f"Debug: {debug_msg}", (200, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        return frame

    def run_racing_loop(self):
        """The main video processing and racing loop."""
        print("Starting Racing Loop. Press 'q' to quit.")
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter('race_log.avi', fourcc, 10.0, (self.frame_width, self.frame_height))

        while self.cap.isOpened():
            self.monitor.start_frame()
            
            # Skip frames under heavy load
            if self.monitor.skip_frame:
                self.cap.grab() # Quickly skip frame acquisition
                continue

            ret, frame = self.cap.read()
            if not ret:
                print("Error reading frame from camera.")
                break

            processed_frame = self._process_frame(frame)
            
            # Display and record
            cv2.imshow('Robot Vision', processed_frame)
            out.write(processed_frame)

            # Exit on 'q' press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        out.release()
        cv2.destroyAllWindows()
        print("Racing loop ended. Video log saved as race_log.avi.")


    def run_calibration(self, color_name):
        """Interactive HSV calibration mode."""
        if color_name not in COLOR_CONFIG:
            print(f"ERROR: Cannot calibrate '{color_name}'. Must be one of: {list(COLOR_CONFIG.keys())}")
            return

        calibrator = ColorCalibrator(color_name)
        print(f"Starting Calibration for {color_name}. Adjust sliders and press 'q' to save and exit.")

        while self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                break

            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            hsv_min, hsv_max = calibrator.get_hsv_ranges()
            
            # Create the mask and show it
            mask = cv2.inRange(hsv_frame, hsv_min, hsv_max)
            # Apply the mask to the original frame (for visual confirmation)
            res = cv2.bitwise_and(frame, frame, mask=mask)
            
            cv2.imshow('Original Frame', frame)
            cv2.imshow(calibrator.window_name, res)
            
            # Exit on 'q' press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print(f"Calibration saved for {color_name}:")
                print(f"HSV MIN: {hsv_min.tolist()}")
                print(f"HSV MAX: {hsv_max.tolist()}")
                break

        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Autonomous Racing Robot Computer Vision System.")
    parser.add_argument('--camera', type=int, default=0, help='Camera ID (e.g., 0 for built-in, 1 for USB).')
    parser.add_argument('--mock-serial', action='store_true', help='Run without actual serial connection (prints commands to console).')
    parser.add_argument('--calibrate', type=str, choices=['red', 'green'], help='Run in HSV calibration mode for the specified color (e.g., --calibrate red).')
    
    args = parser.parse_args()

    try:
        MainApp(args.camera, args.mock_serial, args.calibrate)
    except Exception as e:
        print(f"FATAL ERROR: {e}")
    finally:
        print("Application closed.")
