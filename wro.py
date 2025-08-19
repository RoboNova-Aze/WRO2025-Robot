#!/usr/bin/env python3
"""
Autonomous Robot Car Competition System
Designed for Raspberry Pi 4 - WRO Future Engineers Competition
Handles Open Challenge, Obstacle Challenge, and Parallel Parking
"""

import RPi.GPIO as GPIO
import cv2
import numpy as np
import time
import threading
from enum import Enum
from dataclasses import dataclass
from typing import Tuple, Optional, List
import logging
from picamera2 import Picamera2
from libcamera import controls

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class CompetitionMode(Enum):
    """Competition mode enumeration"""
    OPEN_CHALLENGE = "open"
    OBSTACLE_CHALLENGE = "obstacle"

class RobotState(Enum):
    """Robot state enumeration"""
    IDLE = "idle"
    RUNNING = "running"
    PARKING = "parking"
    FINISHED = "finished"
    ERROR = "error"

@dataclass
class PillarDetection:
    """Data class for pillar detection results"""
    color: str  # 'red', 'green', or 'none'
    position: Tuple[int, int]  # (x, y) center coordinates
    confidence: float  # detection confidence
    distance: float  # estimated distance

# ==================== PIN CONFIGURATION ====================
class PinConfig:
    """GPIO pin configuration"""
    # IR Sensors
    IR_LEFT = 18
    IR_CENTER = 24
    IR_RIGHT = 25
    
    # Motors (using L298N driver)
    MOTOR_LEFT_FORWARD = 20
    MOTOR_LEFT_BACKWARD = 21
    MOTOR_LEFT_PWM = 12
    
    MOTOR_RIGHT_FORWARD = 19
    MOTOR_RIGHT_BACKWARD = 26
    MOTOR_RIGHT_PWM = 13
    
    # Start button
    START_BUTTON = 2
    
    # Status LED
    STATUS_LED = 16

# ==================== HARDWARE CONTROL MODULES ====================

class IRSensorModule:
    """IR sensor management for line following"""
    
    def __init__(self):
        self.pins = [PinConfig.IR_LEFT, PinConfig.IR_CENTER, PinConfig.IR_RIGHT]
        self.setup_pins()
        
    def setup_pins(self):
        """Initialize IR sensor pins"""
        for pin in self.pins:
            GPIO.setup(pin, GPIO.IN)
        logger.info("IR sensors initialized")
    
    def read_sensors(self) -> Tuple[bool, bool, bool]:
        """
        Read all IR sensors
        Returns: (left_detected, center_detected, right_detected)
        True = line detected (LOW signal), False = no line (HIGH signal)
        """
        left = not GPIO.input(PinConfig.IR_LEFT)
        center = not GPIO.input(PinConfig.IR_CENTER)
        right = not GPIO.input(PinConfig.IR_RIGHT)
        return (left, center, right)
    
    def get_line_position(self) -> str:
        """
        Determine line position relative to robot
        Returns: 'left', 'center', 'right', 'none', 'multiple'
        """
        left, center, right = self.read_sensors()
        
        if center and not left and not right:
            return 'center'
        elif left and not center and not right:
            return 'left'
        elif right and not center and not left:
            return 'right'
        elif left and center:
            return 'slight_left'
        elif right and center:
            return 'slight_right'
        elif not any([left, center, right]):
            return 'none'
        else:
            return 'multiple'

class MotorController:
    """Motor control for movement and steering"""
    
    def __init__(self):
        self.setup_pins()
        self.left_pwm = None
        self.right_pwm = None
        self.setup_pwm()
        
    def setup_pins(self):
        """Initialize motor control pins"""
        motor_pins = [
            PinConfig.MOTOR_LEFT_FORWARD, PinConfig.MOTOR_LEFT_BACKWARD,
            PinConfig.MOTOR_RIGHT_FORWARD, PinConfig.MOTOR_RIGHT_BACKWARD
        ]
        for pin in motor_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        
        GPIO.setup(PinConfig.MOTOR_LEFT_PWM, GPIO.OUT)
        GPIO.setup(PinConfig.MOTOR_RIGHT_PWM, GPIO.OUT)
        
    def setup_pwm(self):
        """Initialize PWM for speed control"""
        self.left_pwm = GPIO.PWM(PinConfig.MOTOR_LEFT_PWM, 1000)  # 1kHz frequency
        self.right_pwm = GPIO.PWM(PinConfig.MOTOR_RIGHT_PWM, 1000)
        self.left_pwm.start(0)
        self.right_pwm.start(0)
        logger.info("Motor controller initialized")
    
    def set_motor_speed(self, left_speed: int, right_speed: int):
        """
        Set motor speeds (-100 to 100)
        Positive = forward, Negative = backward
        """
        # Left motor
        if left_speed > 0:
            GPIO.output(PinConfig.MOTOR_LEFT_FORWARD, GPIO.HIGH)
            GPIO.output(PinConfig.MOTOR_LEFT_BACKWARD, GPIO.LOW)
        elif left_speed < 0:
            GPIO.output(PinConfig.MOTOR_LEFT_FORWARD, GPIO.LOW)
            GPIO.output(PinConfig.MOTOR_LEFT_BACKWARD, GPIO.HIGH)
        else:
            GPIO.output(PinConfig.MOTOR_LEFT_FORWARD, GPIO.LOW)
            GPIO.output(PinConfig.MOTOR_LEFT_BACKWARD, GPIO.LOW)
        
        # Right motor
        if right_speed > 0:
            GPIO.output(PinConfig.MOTOR_RIGHT_FORWARD, GPIO.HIGH)
            GPIO.output(PinConfig.MOTOR_RIGHT_BACKWARD, GPIO.LOW)
        elif right_speed < 0:
            GPIO.output(PinConfig.MOTOR_RIGHT_FORWARD, GPIO.LOW)
            GPIO.output(PinConfig.MOTOR_RIGHT_BACKWARD, GPIO.HIGH)
        else:
            GPIO.output(PinConfig.MOTOR_RIGHT_FORWARD, GPIO.LOW)
            GPIO.output(PinConfig.MOTOR_RIGHT_BACKWARD, GPIO.LOW)
        
        # Set PWM duty cycles
        self.left_pwm.ChangeDutyCycle(abs(left_speed))
        self.right_pwm.ChangeDutyCycle(abs(right_speed))
    
    def stop(self):
        """Stop all motors"""
        self.set_motor_speed(0, 0)
    
    def move_forward(self, speed: int = 60):
        """Move forward at specified speed"""
        self.set_motor_speed(speed, speed)
    
    def turn_left(self, speed: int = 40):
        """Turn left"""
        self.set_motor_speed(-speed, speed)
    
    def turn_right(self, speed: int = 40):
        """Turn right"""
        self.set_motor_speed(speed, -speed)
    
    def slight_left(self, speed: int = 60):
        """Slight left adjustment while moving forward"""
        self.set_motor_speed(speed - 20, speed)
    
    def slight_right(self, speed: int = 60):
        """Slight right adjustment while moving forward"""
        self.set_motor_speed(speed, speed - 20)

class CameraModule:
    """Raspberry Pi Camera Module 2 and OpenCV processing for pillar detection"""
    
    def __init__(self):
        self.picam2 = None
        self.camera_config = None
        self.setup_camera()
        
        # Color ranges for pillar detection (HSV)
        self.red_lower = np.array([0, 120, 70])
        self.red_upper = np.array([10, 255, 255])
        self.red_lower2 = np.array([170, 120, 70])
        self.red_upper2 = np.array([180, 255, 255])
        
        self.green_lower = np.array([40, 60, 60])
        self.green_upper = np.array([80, 255, 255])
        
        # Magenta for parking lot detection
        self.magenta_lower = np.array([140, 60, 60])
        self.magenta_upper = np.array([170, 255, 255])
        
    def setup_camera(self):
        """Initialize Raspberry Pi Camera Module 2"""
        try:
            self.picam2 = Picamera2()
            
            # Configure camera for optimal performance
            self.camera_config = self.picam2.create_preview_configuration(
                main={"format": 'XRGB8888', "size": (640, 480)},
                controls={"FrameRate": 30}
            )
            
            self.picam2.configure(self.camera_config)
            
            # Set camera controls for competition environment
            self.picam2.set_controls({
                "AeEnable": True,  # Auto exposure
                "AwbEnable": True,  # Auto white balance
                "AwbMode": controls.AwbModeEnum.Auto,
                "ExposureTime": 10000,  # 10ms exposure (adjust for lighting)
                "AnalogueGain": 1.0,
                "Brightness": 0.0,
                "Contrast": 1.0,
                "Saturation": 1.0
            })
            
            self.picam2.start()
            
            # Allow camera to warm up
            time.sleep(2)
            
            logger.info("Raspberry Pi Camera Module 2 initialized")
            
        except Exception as e:
            logger.error(f"Pi Camera initialization failed: {e}")
            self.picam2 = None
    
    def capture_frame(self) -> Optional[np.ndarray]:
        """Capture a frame from Pi Camera"""
        if self.picam2 is None:
            return None
        
        try:
            # Capture frame as numpy array
            frame = self.picam2.capture_array()
            
            # Convert from XRGB8888 to BGR for OpenCV
            if frame.shape[2] == 4:  # XRGB format
                frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            
            return frame
            
        except Exception as e:
            logger.error(f"Frame capture failed: {e}")
            return None
    
    def detect_pillars(self, frame: np.ndarray) -> List[PillarDetection]:
        """
        Detect red and green pillars in the frame
        Returns list of detected pillars
        """
        if frame is None:
            return []
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detections = []
        
        # Red pillar detection (two ranges due to HSV wrap-around)
        red_mask1 = cv2.inRange(hsv, self.red_lower, self.red_upper)
        red_mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
        red_mask = red_mask1 + red_mask2
        
        # Green pillar detection
        green_mask = cv2.inRange(hsv, self.green_lower, self.green_upper)
        
        # Process red pillars
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in red_contours:
            area = cv2.contourArea(contour)
            if area > 100:  # Minimum area threshold
                x, y, w, h = cv2.boundingRect(contour)
                center_x, center_y = x + w//2, y + h//2
                confidence = min(area / 2000.0, 1.0)  # Normalize confidence
                distance = self.estimate_distance(area)
                
                detections.append(PillarDetection('red', (center_x, center_y), confidence, distance))
        
        # Process green pillars
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in green_contours:
            area = cv2.contourArea(contour)
            if area > 100:  # Minimum area threshold
                x, y, w, h = cv2.boundingRect(contour)
                center_x, center_y = x + w//2, y + h//2
                confidence = min(area / 2000.0, 1.0)
                distance = self.estimate_distance(area)
                
                detections.append(PillarDetection('green', (center_x, center_y), confidence, distance))
        
        return detections
    
    def detect_parking_lot(self, frame: np.ndarray) -> bool:
        """
        Detect magenta parking lot markers
        Returns True if parking area detected
        """
        if frame is None:
            return False
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        magenta_mask = cv2.inRange(hsv, self.magenta_lower, self.magenta_upper)
        
        contours, _ = cv2.findContours(magenta_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Significant area for parking lot
                return True
        
        return False
    
    def estimate_distance(self, area: float) -> float:
        """
        Estimate distance to pillar based on detected area
        Returns distance in arbitrary units
        """
        if area > 0:
            return max(1.0, 10000.0 / area)  # Inverse relationship
        return 100.0  # Default far distance
    
    def cleanup(self):
        """Release camera resources"""
        if self.camera:
            self.camera.release()

# ==================== NAVIGATION AND CONTROL ====================

class LineFollower:
    """Line following algorithm for Open Challenge"""
    
    def __init__(self, ir_sensor: IRSensorModule, motor_controller: MotorController):
        self.ir_sensor = ir_sensor
        self.motor_controller = motor_controller
        self.base_speed = 65
        self.turn_speed = 45
        
    def follow_line(self):
        """
        Execute line following based on IR sensor readings
        """
        line_position = self.ir_sensor.get_line_position()
        
        if line_position == 'center':
            self.motor_controller.move_forward(self.base_speed)
        elif line_position == 'slight_left':
            self.motor_controller.slight_left(self.base_speed)
        elif line_position == 'slight_right':
            self.motor_controller.slight_right(self.base_speed)
        elif line_position == 'left':
            self.motor_controller.turn_left(self.turn_speed)
        elif line_position == 'right':
            self.motor_controller.turn_right(self.turn_speed)
        elif line_position == 'none':
            # Lost line - continue straight briefly then search
            self.motor_controller.move_forward(30)
        else:  # multiple sensors
            self.motor_controller.move_forward(self.base_speed)

class ObstacleHandler:
    """Handle obstacle avoidance with pillars"""
    
    def __init__(self, camera: CameraModule, motor_controller: MotorController):
        self.camera = camera
        self.motor_controller = motor_controller
        self.avoidance_distance = 50.0  # Distance threshold for avoidance
        
    def process_obstacles(self, frame: np.ndarray) -> bool:
        """
        Process pillar obstacles and execute avoidance
        Returns True if obstacle handling was performed
        """
        detections = self.camera.detect_pillars(frame)
        
        if not detections:
            return False
        
        # Find closest pillar
        closest = min(detections, key=lambda d: d.distance)
        
        if closest.distance > self.avoidance_distance:
            return False  # Too far to worry about
        
        frame_center = 320  # Assuming 640px width
        pillar_x = closest.position[0]
        
        if closest.color == 'red':
            # Red pillar: pass on the right
            if pillar_x < frame_center:
                # Pillar on left, move right
                self.motor_controller.turn_right(40)
            else:
                # Already on correct side, continue
                self.motor_controller.move_forward(50)
                
        elif closest.color == 'green':
            # Green pillar: pass on the left  
            if pillar_x > frame_center:
                # Pillar on right, move left
                self.motor_controller.turn_left(40)
            else:
                # Already on correct side, continue
                self.motor_controller.move_forward(50)
        
        time.sleep(0.1)  # Brief avoidance maneuver
        return True

class ParkingSystem:
    """Parallel parking implementation"""
    
    def __init__(self, camera: CameraModule, motor_controller: MotorController, ir_sensor: IRSensorModule):
        self.camera = camera
        self.motor_controller = motor_controller
        self.ir_sensor = ir_sensor
        
    def execute_parallel_parking(self):
        """
        Execute parallel parking maneuver
        """
        logger.info("Starting parallel parking sequence")
        
        # Step 1: Locate parking space (magenta markers)
        parking_found = False
        search_time = time.time()
        
        while not parking_found and (time.time() - search_time) < 10:
            frame = self.camera.capture_frame()
            if frame is not None:
                parking_found = self.camera.detect_parking_lot(frame)
            
            if not parking_found:
                # Continue following line slowly while searching
                self.motor_controller.move_forward(30)
                time.sleep(0.1)
        
        if not parking_found:
            logger.warning("Parking lot not found")
            return False
        
        # Step 2: Position for parking
        logger.info("Parking lot detected, positioning for entry")
        
        # Move past the parking space
        self.motor_controller.move_forward(40)
        time.sleep(1.0)
        
        # Step 3: Reverse parallel parking maneuver
        # Turn right while reversing
        self.motor_controller.set_motor_speed(-35, -15)
        time.sleep(1.2)
        
        # Straighten while continuing reverse
        self.motor_controller.set_motor_speed(-30, -30)
        time.sleep(0.8)
        
        # Turn left while reversing to straighten
        self.motor_controller.set_motor_speed(-15, -35)
        time.sleep(1.0)
        
        # Final positioning
        self.motor_controller.set_motor_speed(-25, -25)
        time.sleep(0.5)
        
        # Stop
        self.motor_controller.stop()
        
        logger.info("Parallel parking completed")
        return True

# ==================== MAIN ROBOT CONTROLLER ====================

class RobotController:
    """Main robot control system"""
    
    def __init__(self):
        self.state = RobotState.IDLE
        self.mode = CompetitionMode.OPEN_CHALLENGE
        self.lap_count = 0
        self.max_laps = 3
        self.running = False
        
        # Initialize hardware modules
        self.ir_sensor = IRSensorModule()
        self.motor_controller = MotorController()
        self.camera = CameraModule()
        
        # Initialize navigation modules
        self.line_follower = LineFollower(self.ir_sensor, self.motor_controller)
        self.obstacle_handler = ObstacleHandler(self.camera, self.motor_controller)
        self.parking_system = ParkingSystem(self.camera, self.motor_controller, self.ir_sensor)
        
        # Setup button and LED
        self.setup_controls()
        
        # Competition timing
        self.start_time = None
        self.max_time = 180  # 3 minutes in seconds
        
    def setup_controls(self):
        """Setup start button and status LED"""
        GPIO.setup(PinConfig.START_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(PinConfig.STATUS_LED, GPIO.OUT)
        GPIO.output(PinConfig.STATUS_LED, GPIO.LOW)
        
        # Button interrupt
        GPIO.add_event_detect(PinConfig.START_BUTTON, GPIO.FALLING, 
                            callback=self.start_button_pressed, bouncetime=300)
    
    def start_button_pressed(self, channel):
        """Handle start button press"""
        if self.state == RobotState.IDLE:
            logger.info("Start button pressed - Beginning competition")
            self.start_competition()
    
    def start_competition(self):
        """Start the competition sequence"""
        self.state = RobotState.RUNNING
        self.running = True
        self.start_time = time.time()
        self.lap_count = 0
        GPIO.output(PinConfig.STATUS_LED, GPIO.HIGH)
        
        # Start main control thread
        control_thread = threading.Thread(target=self.main_control_loop)
        control_thread.daemon = True
        control_thread.start()
    
    def main_control_loop(self):
        """Main control loop for robot operation"""
        logger.info(f"Starting {self.mode.value} challenge")
        
        try:
            while self.running and self.lap_count < self.max_laps:
                # Check time limit
                if time.time() - self.start_time > self.max_time:
                    logger.warning("Time limit reached")
                    break
                
                # Get camera frame for obstacle detection
                frame = None
                if self.mode == CompetitionMode.OBSTACLE_CHALLENGE:
                    frame = self.camera.capture_frame()
                
                # Primary navigation logic
                if self.mode == CompetitionMode.OPEN_CHALLENGE:
                    self.line_follower.follow_line()
                    
                elif self.mode == CompetitionMode.OBSTACLE_CHALLENGE:
                    # Try obstacle avoidance first, fall back to line following
                    if frame is not None and not self.obstacle_handler.process_obstacles(frame):
                        self.line_follower.follow_line()
                    else:
                        self.line_follower.follow_line()
                
                # Lap counting logic (simplified - based on returning to start area)
                self.check_lap_completion()
                
                time.sleep(0.05)  # 20Hz control loop
            
            # Competition finished - attempt parking if obstacle challenge
            if self.mode == CompetitionMode.OBSTACLE_CHALLENGE:
                self.state = RobotState.PARKING
                self.parking_system.execute_parallel_parking()
            
            self.finish_competition()
            
        except Exception as e:
            logger.error(f"Error in main control loop: {e}")
            self.state = RobotState.ERROR
            self.motor_controller.stop()
    
    def check_lap_completion(self):
        """
        Check if a lap has been completed
        This is simplified - in real implementation, you'd use more sophisticated
        position tracking or specific markers
        """
        # Simple lap detection based on line sensor patterns or timing
        # This is a placeholder - implement based on your track layout
        pass
    
    def finish_competition(self):
        """Finish competition and stop robot"""
        self.running = False
        self.state = RobotState.FINISHED
        self.motor_controller.stop()
        GPIO.output(PinConfig.STATUS_LED, GPIO.LOW)
        logger.info(f"Competition finished. Laps completed: {self.lap_count}")
    
    def emergency_stop(self):
        """Emergency stop function"""
        self.running = False
        self.state = RobotState.ERROR
        self.motor_controller.stop()
        GPIO.output(PinConfig.STATUS_LED, GPIO.LOW)
        logger.warning("Emergency stop activated")
    
    def set_competition_mode(self, mode: CompetitionMode):
        """Set competition mode"""
        self.mode = mode
        logger.info(f"Competition mode set to: {mode.value}")
    
    def cleanup(self):
        """Cleanup resources"""
        self.running = False
        self.motor_controller.stop()
        self.camera.cleanup()
        GPIO.cleanup()

# ==================== MAIN EXECUTION ====================

def main():
    """Main execution function"""
    logger.info("Initializing Autonomous Robot Car System")
    
    # Initialize GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    try:
        # Create robot controller
        robot = RobotController()
        
        # Set competition mode (change as needed)
        # robot.set_competition_mode(CompetitionMode.OPEN_CHALLENGE)
        robot.set_competition_mode(CompetitionMode.OBSTACLE_CHALLENGE)
        
        logger.info("Robot ready. Press start button to begin competition.")
        logger.info("Press Ctrl+C to exit")
        
        # Keep main thread alive
        while True:
            time.sleep(1)
            
            # Optional: Add status reporting
            if robot.running:
                elapsed = time.time() - robot.start_time
                logger.info(f"Running... Elapsed: {elapsed:.1f}s, Laps: {robot.lap_count}")
                
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
    finally:
        # Cleanup
        if 'robot' in locals():
            robot.cleanup()
        GPIO.cleanup()
        logger.info("Shutdown complete")

if __name__ == "__main__":
    main()
