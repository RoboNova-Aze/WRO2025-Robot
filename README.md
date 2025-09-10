# Autonomous Racing Robot - WRO 2025 Future Engineers

A comprehensive autonomous racing robot system built for the World Robot Olympiad 2025 Future Engineers competition. The robot uses computer vision for pillar detection and IR sensors for line following to navigate the track autonomously.

## 🎯 Competition Overview

### Open Challenge
- Complete 3 laps autonomously
- No traffic signs on track  
- Wall contact prohibited (one repair allowed)
- 3-minute time limit

### Obstacle Challenge
- Complete 3 laps with pillar navigation
- **Red pillars**: Pass on the RIGHT side
- **Green pillars**: Pass on the LEFT side  
- Finish with parallel parking maneuver
- Pillars must not be moved

## 🏁 Track Specifications

- **Mat size**: 3200×3200 mm
- **Inner track**: 3000×3000 mm
- **Wall height**: 100 mm (black walls)
- **Starting zone**: 200×500 mm
- **Traffic pillars**: 50×50×100 mm
  - Red RGB: (238, 39, 55)
  - Green RGB: (68, 214, 44)
- **Parking lot**: 200 mm width × 1.5× robot length

## 🔧 Hardware Components

### Core Components
- **Raspberry Pi 5** (4GB RAM) - Main controller
- **Micro SD Card** (64GB) - OS and program storage
- **Raspberry Pi Camera** - Real-time vision system

### Motors & Control
- **2× Yellow TT DC Gear Motors** - Differential drive
- **Black plastic wheels** - Traction and movement
- **L298N Motor Driver** - Motor control interface
- **MG90 Servo Motor** - Precise front-wheel steering control

### Power System
- **3× LiPo Batteries** (1S, 3.7V, 1500mAh) - Connected in series (11.1V)
- **LM2596 DC-DC Converter** - Voltage regulation
- **Power Switch** - Safe system control

### Sensors
- **3× IR Sensors** - Line detection and wall avoidance
  - Left sensor (GPIO 18)
  - Center sensor (GPIO 23)  
  - Right sensor (GPIO 24)

## 📁 Project Structure

```
WRO2025-Robot/
├── README.md                 # Project documentation
├── LICENSE                   # Project license
├── .gitignore               # Git ignore rules
├── 2025-WRO-Future-Engineers.txt  # Competition guidelines
├── component-list.md.txt     # Hardware components documentation
├── src/                     # Source code
│   ├── vision_pillars.py    # Camera-based pillar detection
│   ├── line_follower.py     # IR sensor line following system  
│   ├── servo.py            # Servo motor control and steering
│   ├── mainservo.py        # Main servo integration example
│   ├── config.yaml         # System configuration
│   └── placeholder.txt     # Development notes
├── circuit-diagram/         # Electronic schematics
├── design-of-rc-car/       # Mechanical design files
├── robot-photos/           # Robot assembly photos
├── team-photos/            # Team member photos
├── video/                  # Demonstration videos
├── commit-log/             # Development history
└── other/                  # Additional resources
```

## 🚀 Quick Start

### 1. Hardware Setup

**Power Connections:**
```
LiPo Batteries (11.1V) → LM2596 Converter → Raspberry Pi 5V
                      └→ L298N Motor Driver → Motors
```

**GPIO Pin Mapping:**
```
IR Sensors:
- Left: GPIO 18
- Center: GPIO 23  
- Right: GPIO 24

Motors (L298N):
- Left Enable: GPIO 12
- Left Dir 1: GPIO 16
- Left Dir 2: GPIO 20
- Right Enable: GPIO 13
- Right Dir 1: GPIO 19
- Right Dir 2: GPIO 21

Servo Motor:
- Control Pin: GPIO 2 (PWM)
- Power: 5V from Raspberry Pi
- Ground: Common ground
```

### 2. Software Installation

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install dependencies
sudo apt install python3-pip python3-opencv python3-yaml -y

# Install Python packages
pip3 install picamera2 numpy opencv-python pyyaml pyzmq RPi.GPIO

# Clone/download project files
# Copy all .py files and config.yaml to robot
```

### 3. Configuration

Edit `config.yaml` to match your hardware setup:

```yaml
# Servo configuration
servo:
  pin: 2                    # GPIO pin for servo control
  freq_hz: 50              # PWM frequency (standard servo)
  center_us: 1500          # Center position pulse width
  min_us: 600              # Minimum pulse width
  max_us: 2400             # Maximum pulse width
  deg_min: -50.0           # Minimum angle (left)
  deg_max: 50.0            # Maximum angle (right)
  slew_deg_per_s: 400.0    # Maximum turn rate
  failsafe_ms: 300         # Safety timeout

# Turning limits
turning:
  sharp_left: -25.0        # Default left turn angle
  sharp_right: 25.0        # Default right turn angle
  max_left: -50.0          # Maximum left angle
  max_right: 50.0          # Maximum right angle

# IR sensor pins
ir_sensors:
  left_pin: 18
  center_pin: 23
  right_pin: 24

# Motor control
motors:
  base_speed: 70
  max_speed: 100
  turn_speed: 50
```

### 4. Sensor Calibration

**IR Sensor Calibration:**
```bash
python3 line_follower.py
# Choose option 3 to calibrate sensors
# Follow on-screen prompts to calibrate each sensor
```

**Camera Calibration:**
```bash
python3 vision_pillars.py
# Adjust HSV color ranges for lighting conditions
# Test pillar detection in preview mode
```

**Servo Calibration:**
```bash
python3 servo.py
# Test servo movement and verify steering directions
# Adjust center_us value if needed for straight alignment
```

## 🎮 Running the Robot

### Servo Control Test
```bash
python3 servo.py
# Tests servo sweep from -50° to +50°
# Verifies steering linkage and direction
```

### Main Servo Integration
```bash
python3 mainservo.py
# Example of servo integration with perception systems
# Shows proper initialization and control loop
```

### Line Following Only (Open Challenge)
```bash
python3 line_follower.py
# Interactive menu will appear
# Choose option 1 to start line following
```

### Full Vision System (Obstacle Challenge)
```bash
python3 vision_pillars.py
# Real-time pillar detection with preview
# Press 'q' to stop safely
```

### Integrated System
```bash
# Run complete autonomous system
python3 mainservo.py
# Integrates vision, line following, and steering
```

## 🔧 System Architecture

### Servo Control System (`servo.py`)

**Features:**
- **PWM-based Control**: 50Hz PWM signal generation for standard servos
- **Angle Interface**: Direct degree-based control (-50° to +50°)
- **Slew Rate Limiting**: Prevents jerky movements (400°/sec default)
- **Failsafe Protection**: Stops servo if control loop stalls
- **Configuration-driven**: All parameters loaded from config.yaml

**Servo Control Pipeline:**
1. Angle command received in degrees
2. Slew rate limiting applied for smooth motion
3. Angle converted to PWM pulse width (600-2400μs)
4. PWM duty cycle calculated and applied
5. Watchdog monitors for control loop health

**Steering Commands:**
- **Negative angles**: Left steering (e.g., -25° = left turn)
- **Positive angles**: Right steering (e.g., +25° = right turn)
- **Zero degrees**: Straight ahead (center position)

### Turning Limiter (`TurningLimiter` class)

**Intelligent Turn Management:**
- **Default Turns**: Uses ±25° for standard left/right commands
- **Computed Angles**: If vision system computes >25°, uses computed value
- **Safety Limits**: Never exceeds ±50° maximum steering angle
- **Direction Enforcement**: Ensures turn direction matches command

### Main Servo Integration (`mainservo.py`)

**Usage Example:**
```python
from servo import make_servo_and_limiter

# Initialize servo and turning limiter
servo, limiter = make_servo_and_limiter()

# Main control loop
while running:
    # Get perception data (vision, IR sensors, etc.)
    angle, lTurn, rTurn = get_steering_command()
    
    # Apply turning limits and safety checks
    angle_cmd = limiter.apply(angle, lTurn, rTurn)
    
    # Command servo and update
    servo.set_deg(angle_cmd)
    servo.tick()  # Call at 200-300 Hz
    
    time.sleep(0.005)  # ~200 Hz loop

# Cleanup on exit
servo.center()
servo.close()
```

### Vision System (`vision_pillars.py`)

**Features:**
- Real-time Picamera2 integration with OpenCV fallback
- HSV-based color detection for red/green pillars
- Morphological filtering to reduce false positives
- Configurable detection parameters
- ZeroMQ communication for integration
- Live preview with detection visualization

**Detection Pipeline:**
1. Frame capture from Picamera2
2. Gaussian blur preprocessing  
3. HSV color space conversion
4. Color-based segmentation
5. Morphological operations (erosion/dilation)
6. Contour analysis with quality filters
7. Pillar classification and steering commands

**Steering Commands:**
- `pillar_left` - Green pillar detected, pass on left
- `pillar_right` - Red pillar detected, pass on right  
- `no_pillar` - Clear path ahead
- `straight` - Continue current path

### Line Following (`line_follower.py`)

**Features:**
- 3-sensor IR array for line detection
- Robust sensor reading with moving average filter
- State machine for competition phases
- Safety systems (emergency stop, watchdog timer)
- One-time repair functionality
- Configurable motor control abstraction

**Line Following Logic:**
- **Center only**: Go straight
- **Left + Center**: Slight right correction
- **Right + Center**: Slight left correction  
- **Left only**: Sharp right turn
- **Right only**: Sharp left turn
- **All sensors**: Intersection/wide line - continue straight
- **No sensors**: Lost line - search pattern

## ⚙️ Configuration Reference

### Servo Configuration
```yaml
servo:
  pin: 2                    # GPIO pin (BCM numbering)
  freq_hz: 50              # PWM frequency (Hz)
  center_us: 1500          # Center pulse width (microseconds)
  min_us: 600              # Minimum pulse width
  max_us: 2400             # Maximum pulse width
  deg_min: -50.0           # Minimum angle (degrees)
  deg_max: 50.0            # Maximum angle (degrees)
  slew_deg_per_s: 400.0    # Maximum turn rate (deg/sec)
  failsafe_ms: 300         # Watchdog timeout (milliseconds)
```

### Turning Configuration
```yaml
turning:
  sharp_left: -25.0        # Default left turn angle
  sharp_right: 25.0        # Default right turn angle
  max_left: -50.0          # Maximum left steering limit
  max_right: 50.0          # Maximum right steering limit
```

### Color Detection (HSV Ranges)
```yaml
colors:
  # Red pillar (handles HSV wrap-around)
  red_lower: [0, 120, 120]
  red_upper: [10, 255, 255]
  red_lower2: [170, 120, 120]
  red_upper2: [180, 255, 255]
  
  # Green pillar  
  green_lower: [40, 120, 120]
  green_upper: [80, 255, 255]
```

### Detection Quality Filters
```yaml
detection:
  min_area: 100          # Minimum pillar area (pixels)
  max_area: 50000        # Maximum pillar area (pixels)
  min_aspect_ratio: 0.3  # Width/height ratio limits
  max_aspect_ratio: 3.0
  expected_pillar_ratio: 2.0  # Expected height/width ratio
  min_solidity: 0.5      # Shape quality (area/convex hull)
```

### Motor Control
```yaml
motors:
  base_speed: 70         # Normal driving speed (0-100)
  max_speed: 100         # Maximum speed limit
  turn_speed: 50         # Speed during turns
  correction_factor: 0.8 # Line following correction strength
```

## 🛠️ Troubleshooting

### Common Issues

**Servo not responding:**
```bash
# Check servo wiring
# Power: 5V (red wire)
# Ground: GND (brown/black wire)  
# Signal: GPIO 2 (orange/yellow wire)

# Test servo manually
python3 servo.py
```

**Servo jittery or unstable:**
- Check power supply stability (servo draws significant current)
- Verify PWM frequency is 50Hz
- Adjust slew rate in config.yaml
- Ensure proper grounding

**Steering direction reversed:**
- Swap servo horn attachment 180°, or
- Modify deg_min/deg_max values in config:
```yaml
servo:
  deg_min: -50.0  # Change to 50.0
  deg_max: 50.0   # Change to -50.0
```

**Camera not working:**
```bash
# Enable camera interface
sudo raspi-config
# Interface Options → Camera → Enable

# Test camera
libcamera-hello --display 0
```

**IR sensors not responding:**
- Check wiring connections to GPIO pins
- Run calibration routine
- Verify power supply (3.3V/5V depending on sensor)
- Test with multimeter for continuity

**Motors not moving:**
- Verify L298N power connections (12V from batteries)
- Check GPIO pin assignments in config.yaml
- Test motor driver with direct connections
- Ensure battery charge level above 10V

**Poor pillar detection:**
- Adjust HSV color ranges in config.yaml  
- Improve lighting conditions
- Check camera focus and lens cleanliness
- Calibrate for specific track conditions

### Debug Mode

Enable detailed logging:
```python
# Add to top of any script
import logging
logging.basicConfig(level=logging.DEBUG)
```

View live sensor readings:
```bash
python3 line_follower.py
# Choose option 6 for status display
```

Test servo positions:
```bash
python3 -c "
from servo import make_servo_and_limiter
servo, _ = make_servo_and_limiter()
servo.set_deg(-25)  # Test left turn
input('Press Enter for right turn...')
servo.set_deg(25)   # Test right turn
input('Press Enter for center...')
servo.center()
servo.close()
"
```

## 🏆 Competition Strategy

### Pre-Competition Checklist
- [ ] Calibrate IR sensors on actual track surface
- [ ] Test camera detection under competition lighting
- [ ] Verify servo steering alignment and limits
- [ ] Test motor speeds for track surface
- [ ] Verify emergency stop functionality
- [ ] Practice repair procedure
- [ ] Check battery charge levels
- [ ] Backup configuration files

### Competition Day Setup
1. **Sensor Calibration**: Calibrate IR sensors on competition track
2. **Vision Tuning**: Adjust HSV ranges for competition lighting  
3. **Servo Alignment**: Center servo and verify straight-line tracking
4. **Speed Optimization**: Fine-tune motor speeds for track surface
5. **Practice Runs**: Test both challenges before official attempts

### Performance Optimization
- Monitor battery voltage (performance drops below 10V)
- Use practice time for sensor recalibration
- Keep spare batteries charged and ready
- Document successful configuration values
- Test servo response under different battery levels

## 🎯 Competition Rules Compliance

### Steering System (Rule 11.3)
- ✅ Single steering actuator (MG90 servo)
- ✅ Front-wheel steering mechanism
- ✅ No additional steering motors or mechanisms

### Start Procedure (Rules 9.10-9.14)
- ✅ Robot waits in stationary position
- ✅ Single start button implementation available
- ✅ No movement until start command
- ✅ Servo centers automatically on startup

### Communication Restrictions (Rules 11.6-11.12)
- ✅ No wireless communication during run
- ✅ All processing on-board Raspberry Pi
- ✅ No external control signals

## 📚 Additional Resources

### Dependencies
- Python 3.7+
- OpenCV 4.5+
- Picamera2
- NumPy
- PyYAML
- PyZMQ
- RPi.GPIO

### Servo Control Theory
- **PWM Period**: 20ms (50Hz frequency)
- **Pulse Width Range**: 0.6ms to 2.4ms
- **Center Position**: 1.5ms pulse width
- **Angular Resolution**: ~0.1° with proper tuning

---

**Project Status**: Ready for WRO 2025 Competition  
**Last Updated**: January 2025  
**Python Version**: 3.7+  
**Platform**: Raspberry Pi 4 with Raspberry Pi OS
