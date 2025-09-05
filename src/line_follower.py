#!/usr/bin/env python3
"""
Autonomous Racing Robot - IR Line Following System
==================================================

This module implements a robust line-following system using 3 IR sensors
for autonomous racing competitions. Designed for Raspberry Pi 4 with
differential drive motors.

Features:
- Configurable GPIO pin mapping and sensor thresholds
- Robust line-following logic with combination handling
- Calibration routine for sensor tuning
- State machine for lap tracking
- Safety mechanisms and emergency stops
- Integration hooks for camera vision system

Author: AI Assistant
License: MIT
"""

import logging
import time
import threading
import json
import yaml
from enum import Enum
from dataclasses import dataclass, asdict
from typing import Dict, Tuple, Optional, Callable
from abc import ABC, abstractmethod

try:
    import RPi.GPIO as GPIO
except ImportError:
    logging.warning("RPi.GPIO not available - running in simulation mode")
    GPIO = None


class RobotState(Enum):
    """Robot operational states"""
    IDLE = "idle"
    CALIBRATING = "calibrating"
    FOLLOWING_LINE = "following_line"
    TURNING = "turning"
    EMERGENCY_STOP = "emergency_stop"
    COMPLETED = "completed"
    REPAIR_MODE = "repair_mode"


class SensorReading(Enum):
    """IR sensor reading states"""
    ON_LINE = True
    OFF_LINE = False


@dataclass
class IRSensorConfig:
    """Configuration for IR sensors"""
    left_pin: int = 18
    center_pin: int = 23
    right_pin: int = 24
    threshold_low: float = 0.3
    threshold_high: float = 0.7
    sample_rate: int = 100  # Hz
    moving_average_window: int = 5


@dataclass
class MotorConfig:
    """Configuration for motor control"""
    left_enable_pin: int = 12
    left_dir_pin1: int = 16
    left_dir_pin2: int = 20
    right_enable_pin: int = 13
    right_dir_pin1: int = 19
    right_dir_pin2: int = 21
    base_speed: int = 70
    max_speed: int = 100
    turn_speed: int = 50
    correction_factor: float = 0.8


@dataclass
class NavigationConfig:
    """Configuration for navigation logic"""
    lap_target: int = 3
    encoder_counts_per_lap: int = 1000  # Configurable based on track size
    turn_duration: float = 0.5
    correction_duration: float = 0.2
    watchdog_timeout: float = 5.0
    repair_allowed: bool = True


class MotorController(ABC):
    """Abstract base class for motor control"""
    
    @abstractmethod
    def set_speed(self, left_speed: int, right_speed: int) -> None:
        """Set motor speeds (-100 to 100)"""
        pass
    
    @abstractmethod
    def stop(self) -> None:
        """Stop all motors immediately"""
        pass
    
    @abstractmethod
    def turn_left(self, duration: float = 0.5) -> None:
        """Execute left turn"""
        pass
    
    @abstractmethod
    def turn_right(self, duration: float = 0.5) -> None:
        """Execute right turn"""
        pass
    
    @abstractmethod
    def pivot(self, angle: float) -> None:
        """Pivot turn by specified angle (degrees)"""
        pass


class GPIOMotorController(MotorController):
    """GPIO-based motor controller implementation"""
    
    def __init__(self, config: MotorConfig):
        self.config = config
        self.logger = logging.getLogger(__name__ + '.MotorController')
        self._setup_gpio()
        
    def _setup_gpio(self) -> None:
        """Initialize GPIO pins for motor control"""
        if GPIO is None:
            self.logger.warning("GPIO not available - using simulation mode")
            return
            
        GPIO.setmode(GPIO.BCM)
        
        # Setup motor pins
        motor_pins = [
            self.config.left_enable_pin, self.config.left_dir_pin1, self.config.left_dir_pin2,
            self.config.right_enable_pin, self.config.right_dir_pin1, self.config.right_dir_pin2
        ]
        
        for pin in motor_pins:
            GPIO.setup(pin, GPIO.OUT)
            
        # Setup PWM for enable pins
        self.left_pwm = GPIO.PWM(self.config.left_enable_pin, 1000)
        self.right_pwm = GPIO.PWM(self.config.right_enable_pin, 1000)
        self.left_pwm.start(0)
        self.right_pwm.start(0)
        
        self.logger.info("Motor GPIO initialized")
    
    def set_speed(self, left_speed: int, right_speed: int) -> None:
        """Set motor speeds with direction control"""
        if GPIO is None:
            self.logger.debug(f"Simulated motor speeds: L={left_speed}, R={right_speed}")
            return
            
        # Clamp speeds to valid range
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))
        
        # Set left motor
        if left_speed >= 0:
            GPIO.output(self.config.left_dir_pin1, GPIO.HIGH)
            GPIO.output(self.config.left_dir_pin2, GPIO.LOW)
            self.left_pwm.ChangeDutyCycle(abs(left_speed))
        else:
            GPIO.output(self.config.left_dir_pin1, GPIO.LOW)
            GPIO.output(self.config.left_dir_pin2, GPIO.HIGH)
            self.left_pwm.ChangeDutyCycle(abs(left_speed))
            
        # Set right motor
        if right_speed >= 0:
            GPIO.output(self.config.right_dir_pin1, GPIO.HIGH)
            GPIO.output(self.config.right_dir_pin2, GPIO.LOW)
            self.right_pwm.ChangeDutyCycle(abs(right_speed))
        else:
            GPIO.output(self.config.right_dir_pin1, GPIO.LOW)
            GPIO.output(self.config.right_dir_pin2, GPIO.HIGH)
            self.right_pwm.ChangeDutyCycle(abs(right_speed))
            
        self.logger.debug(f"Motor speeds set: L={left_speed}, R={right_speed}")
    
    def stop(self) -> None:
        """Stop all motors"""
        self.set_speed(0, 0)
        self.logger.info("Motors stopped")
    
    def turn_left(self, duration: float = 0.5) -> None:
        """Execute left turn"""
        self.set_speed(-self.config.turn_speed, self.config.turn_speed)
        time.sleep(duration)
        self.stop()
        
    def turn_right(self, duration: float = 0.5) -> None:
        """Execute right turn"""
        self.set_speed(self.config.turn_speed, -self.config.turn_speed)
        time.sleep(duration)
        self.stop()
        
    def pivot(self, angle: float) -> None:
        """Pivot turn by angle (simplified implementation)"""
        duration = abs(angle) / 90.0 * 0.5  # Rough estimation
        if angle > 0:
            self.turn_right(duration)
        else:
            self.turn_left(duration)
    
    def cleanup(self) -> None:
        """Cleanup GPIO resources"""
        if GPIO is not None:
            self.left_pwm.stop()
            self.right_pwm.stop()
            GPIO.cleanup()


class IRSensorArray:
    """Manages 3 IR sensors for line detection"""
    
    def __init__(self, config: IRSensorConfig):
        self.config = config
        self.logger = logging.getLogger(__name__ + '.IRSensorArray')
        self._readings_history = {
            'left': [],
            'center': [],
            'right': []
        }
        self._setup_gpio()
        
    def _setup_gpio(self) -> None:
        """Initialize GPIO pins for IR sensors"""
        if GPIO is None:
            self.logger.warning("GPIO not available - using simulation mode")
            return
            
        sensor_pins = [self.config.left_pin, self.config.center_pin, self.config.right_pin]
        for pin in sensor_pins:
            GPIO.setup(pin, GPIO.IN)
            
        self.logger.info("IR sensor GPIO initialized")
    
    def read_sensors(self) -> Dict[str, bool]:
        """Read current state of all IR sensors"""
        if GPIO is None:
            # Simulation mode - return dummy values
            import random
            return {
                'left': random.choice([True, False]),
                'center': random.choice([True, False]),
                'right': random.choice([True, False])
            }
        
        readings = {
            'left': GPIO.input(self.config.left_pin),
            'center': GPIO.input(self.config.center_pin),
            'right': GPIO.input(self.config.right_pin)
        }
        
        # Apply moving average filter
        for sensor, value in readings.items():
            history = self._readings_history[sensor]
            history.append(float(value))
            
            # Maintain window size
            if len(history) > self.config.moving_average_window:
                history.pop(0)
            
            # Convert to boolean based on threshold
            avg = sum(history) / len(history)
            readings[sensor] = avg > self.config.threshold_high
        
        self.logger.debug(f"Sensor readings: {readings}")
        return readings
    
    def calibrate(self) -> Dict[str, Tuple[float, float]]:
        """Interactive calibration routine"""
        self.logger.info("Starting IR sensor calibration...")
        
        calibration_data = {}
        sensors = ['left', 'center', 'right']
        pins = [self.config.left_pin, self.config.center_pin, self.config.right_pin]
        
        for sensor, pin in zip(sensors, pins):
            print(f"\nCalibrating {sensor} sensor (GPIO {pin})")
            
            input("Place sensor OFF the line and press Enter...")
            off_line_readings = []
            for _ in range(50):
                if GPIO is not None:
                    off_line_readings.append(GPIO.input(pin))
                else:
                    off_line_readings.append(0)
                time.sleep(0.02)
            
            input("Place sensor ON the line and press Enter...")
            on_line_readings = []
            for _ in range(50):
                if GPIO is not None:
                    on_line_readings.append(GPIO.input(pin))
                else:
                    on_line_readings.append(1)
                time.sleep(0.02)
            
            off_avg = sum(off_line_readings) / len(off_line_readings)
            on_avg = sum(on_line_readings) / len(on_line_readings)
            
            threshold = (off_avg + on_avg) / 2
            calibration_data[sensor] = (off_avg, on_avg, threshold)
            
            print(f"{sensor}: OFF={off_avg:.3f}, ON={on_avg:.3f}, THRESHOLD={threshold:.3f}")
        
        self.logger.info("Calibration completed")
        return calibration_data


class LineFollower:
    """Main line following controller"""
    
    def __init__(self, config_path: str = "config.yaml"):
        self.logger = logging.getLogger(__name__ + '.LineFollower')
        self.config_path = config_path
        self.load_config()
        
        # Initialize components
        self.sensors = IRSensorArray(self.ir_config)
        self.motor_controller = GPIOMotorController(self.motor_config)
        
        # State management
        self.state = RobotState.IDLE
        self.lap_count = 0
        self.encoder_count = 0
        self.emergency_stop_flag = threading.Event()
        self.repair_used = False
        
        # Threading
        self.main_thread = None
        self.watchdog_thread = None
        self._running = False
        
        # Integration hooks
        self.vision_callback: Optional[Callable] = None
        
    def load_config(self) -> None:
        """Load configuration from file"""
        try:
            with open(self.config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            self.ir_config = IRSensorConfig(**config.get('ir_sensors', {}))
            self.motor_config = MotorConfig(**config.get('motors', {}))
            self.nav_config = NavigationConfig(**config.get('navigation', {}))
            
            self.logger.info(f"Configuration loaded from {self.config_path}")
            
        except FileNotFoundError:
            self.logger.warning("Config file not found, using defaults")
            self.ir_config = IRSensorConfig()
            self.motor_config = MotorConfig()
            self.nav_config = NavigationConfig()
            
    def save_config(self) -> None:
        """Save current configuration to file"""
        config = {
            'ir_sensors': asdict(self.ir_config),
            'motors': asdict(self.motor_config),
            'navigation': asdict(self.nav_config)
        }
        
        with open(self.config_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)
            
        self.logger.info(f"Configuration saved to {self.config_path}")
    
    def set_vision_callback(self, callback: Callable) -> None:
        """Set callback for vision system integration"""
        self.vision_callback = callback
        self.logger.info("Vision system callback registered")
    
    def emergency_stop(self) -> None:
        """Trigger emergency stop"""
        self.emergency_stop_flag.set()
        self.motor_controller.stop()
        self.state = RobotState.EMERGENCY_STOP
        self.logger.critical("EMERGENCY STOP ACTIVATED")
    
    def reset_after_repair(self) -> None:
        """Reset system after mechanical/electronic repair"""
        if not self.repair_used:
            self.emergency_stop_flag.clear()
            self.state = RobotState.IDLE
            self.repair_used = True
            self.logger.info("System reset after repair")
        else:
            self.logger.warning("Repair already used - cannot reset")
    
    def _determine_action(self, sensor_readings: Dict[str, bool]) -> str:
        """Determine action based on sensor readings"""
        left, center, right = sensor_readings['left'], sensor_readings['center'], sensor_readings['right']
        
        # Line following logic
        if center and not left and not right:
            return "straight"
        elif left and center and not right:
            return "slight_right"
        elif right and center and not left:
            return "slight_left"
        elif left and not center and not right:
            return "turn_right"
        elif right and not center and not left:
            return "turn_left"
        elif left and center and right:
            # All sensors on line - intersection or thick line
            return "straight"
        elif not left and not center and not right:
            # No line detected - emergency situation
            return "search"
        else:
            # Default case
            return "straight"
    
    def _execute_action(self, action: str) -> None:
        """Execute motor action based on decision"""
        base_speed = self.motor_config.base_speed
        correction = int(base_speed * self.motor_config.correction_factor)
        
        if action == "straight":
            self.motor_controller.set_speed(base_speed, base_speed)
        elif action == "slight_left":
            self.motor_controller.set_speed(base_speed - correction, base_speed)
        elif action == "slight_right":
            self.motor_controller.set_speed(base_speed, base_speed - correction)
        elif action == "turn_left":
            self.motor_controller.set_speed(-correction, base_speed)
        elif action == "turn_right":
            self.motor_controller.set_speed(base_speed, -correction)
        elif action == "search":
            # Lost line - slow search pattern
            self.motor_controller.set_speed(20, -20)
        
        self.logger.debug(f"Action executed: {action}")
    
    def _update_lap_count(self) -> None:
        """Update lap count based on encoder or virtual detection"""
        # Simplified implementation - increment encoder count
        self.encoder_count += 1
        
        if self.encoder_count >= self.nav_config.encoder_counts_per_lap:
            self.lap_count += 1
            self.encoder_count = 0
            self.logger.info(f"Lap {self.lap_count} completed")
            
            if self.lap_count >= self.nav_config.lap_target:
                self.state = RobotState.COMPLETED
                self.motor_controller.stop()
                self.logger.info("All laps completed!")
    
    def _watchdog_thread(self) -> None:
        """Watchdog thread for safety monitoring"""
        last_heartbeat = time.time()
        
        while self._running:
            current_time = time.time()
            
            if current_time - last_heartbeat > self.nav_config.watchdog_timeout:
                self.logger.error("Watchdog timeout - triggering emergency stop")
                self.emergency_stop()
                break
            
            # Reset heartbeat if robot is moving
            if self.state == RobotState.FOLLOWING_LINE:
                last_heartbeat = current_time
                
            time.sleep(1.0)
    
    def _main_loop(self) -> None:
        """Main line following loop"""
        sample_delay = 1.0 / self.ir_config.sample_rate
        
        while self._running and not self.emergency_stop_flag.is_set():
            if self.state == RobotState.FOLLOWING_LINE:
                # Read sensors
                sensor_readings = self.sensors.read_sensors()
                
                # Determine action
                action = self._determine_action(sensor_readings)
                
                # Execute action
                self._execute_action(action)
                
                # Update navigation
                self._update_lap_count()
                
                # Vision system integration
                if self.vision_callback:
                    try:
                        vision_command = self.vision_callback(sensor_readings)
                        if vision_command:
                            self.logger.debug(f"Vision command received: {vision_command}")
                            # Process vision commands here
                    except Exception as e:
                        self.logger.error(f"Vision callback error: {e}")
                
                # Check completion
                if self.state == RobotState.COMPLETED:
                    break
            
            time.sleep(sample_delay)
        
        # Clean shutdown
        self.motor_controller.stop()
        self.logger.info("Main loop terminated")
    
    def start_following(self) -> None:
        """Start the line following operation"""
        if self.state != RobotState.IDLE:
            self.logger.warning(f"Cannot start - current state: {self.state}")
            return
        
        self.state = RobotState.FOLLOWING_LINE
        self._running = True
        
        # Start threads
        self.main_thread = threading.Thread(target=self._main_loop)
        self.watchdog_thread = threading.Thread(target=self._watchdog_thread)
        
        self.main_thread.start()
        self.watchdog_thread.start()
        
        self.logger.info("Line following started")
    
    def stop_following(self) -> None:
        """Stop the line following operation"""
        self._running = False
        self.motor_controller.stop()
        
        if self.main_thread and self.main_thread.is_alive():
            self.main_thread.join(timeout=2.0)
        
        if self.watchdog_thread and self.watchdog_thread.is_alive():
            self.watchdog_thread.join(timeout=2.0)
        
        self.state = RobotState.IDLE
        self.logger.info("Line following stopped")
    
    def calibrate_sensors(self) -> None:
        """Run sensor calibration routine"""
        if self.state != RobotState.IDLE:
            self.logger.warning("Cannot calibrate while running")
            return
        
        self.state = RobotState.CALIBRATING
        calibration_data = self.sensors.calibrate()
        
        # Update configuration with calibration results
        for sensor, (off_val, on_val, threshold) in calibration_data.items():
            if sensor == 'left':
                self.ir_config.threshold_low = min(off_val, on_val)
                self.ir_config.threshold_high = threshold
        
        self.save_config()
        self.state = RobotState.IDLE
        
        self.logger.info("Sensor calibration completed and saved")
    
    def get_status(self) -> Dict:
        """Get current system status"""
        return {
            'state': self.state.value,
            'lap_count': self.lap_count,
            'encoder_count': self.encoder_count,
            'emergency_stop': self.emergency_stop_flag.is_set(),
            'repair_used': self.repair_used,
            'running': self._running
        }
    
    def cleanup(self) -> None:
        """Cleanup resources"""
        self.stop_following()
        self.motor_controller.cleanup()
        if GPIO is not None:
            GPIO.cleanup()
        self.logger.info("System cleanup completed")


def setup_logging(level: str = "INFO") -> None:
    """Setup logging configuration"""
    logging.basicConfig(
        level=getattr(logging, level.upper()),
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('line_follower.log'),
            logging.StreamHandler()
        ]
    )


def main():
    """Main entry point for standalone operation"""
    setup_logging()
    logger = logging.getLogger(__name__)
    
    try:
        # Create line follower instance
        robot = LineFollower()
        
        # Setup signal handlers for graceful shutdown
        import signal
        
        def signal_handler(signum, frame):
            logger.info("Shutdown signal received")
            robot.cleanup()
            exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        # Interactive menu
        while True:
            print("\n=== Line Following Robot Control ===")
            print("1. Start line following")
            print("2. Stop line following")
            print("3. Calibrate sensors")
            print("4. Emergency stop")
            print("5. Reset after repair")
            print("6. Show status")
            print("7. Exit")
            
            choice = input("Enter choice: ").strip()
            
            if choice == "1":
                robot.start_following()
            elif choice == "2":
                robot.stop_following()
            elif choice == "3":
                robot.calibrate_sensors()
            elif choice == "4":
                robot.emergency_stop()
            elif choice == "5":
                robot.reset_after_repair()
            elif choice == "6":
                status = robot.get_status()
                print(f"Status: {json.dumps(status, indent=2)}")
            elif choice == "7":
                break
            else:
                print("Invalid choice")
        
        robot.cleanup()
        
    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
    finally:
        if GPIO is not None:
            GPIO.cleanup()


if __name__ == "__main__":
    main()
