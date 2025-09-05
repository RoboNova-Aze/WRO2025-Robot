"""
Autonomous Racing Robot - Vision System for Pillar Detection
===========================================================

Real-time detection of red and green pillars using Picamera2 or OpenCV fallback.
Provides steering guidance and communicates via ZMQ.

Press 'q' in the preview window to stop the system safely.
"""

import logging
import time
import threading
import json
import yaml
import numpy as np
import cv2
import queue
import zmq
from enum import Enum
from dataclasses import dataclass, asdict
from typing import Dict, List, Tuple, Optional, NamedTuple

# Picamera2 import
try:
    from picamera2 import Picamera2
    PICAMERA_AVAILABLE = True
except ImportError:
    logging.warning("Picamera2 not available - using OpenCV VideoCapture")
    PICAMERA_AVAILABLE = False

# Logging setup
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# ----------------- ENUMS & DATACLASSES -----------------

class PillarColor(Enum):
    RED = "red"
    GREEN = "green"
    UNKNOWN = "unknown"

class SteeringCommand(Enum):
    STRAIGHT = "straight"
    TURN_LEFT = "turn_left"
    TURN_RIGHT = "turn_right"
    PILLAR_LEFT = "pillar_left"
    PILLAR_RIGHT = "pillar_right"
    NO_PILLAR = "no_pillar"

class DetectionResult(NamedTuple):
    color: PillarColor
    centroid: Tuple[int, int]
    bbox: Tuple[int, int, int, int]
    area: float
    confidence: float
    lateral_offset: float

@dataclass
class CameraConfig:
    width: int = 640
    height: int = 480
    fps: int = 30
    awb_mode: str = "auto"
    preview: bool = False

@dataclass
class ColorConfig:
    red_lower: Tuple[int,int,int] = (0,120,120)
    red_upper: Tuple[int,int,int] = (10,255,255)
    red_lower2: Tuple[int,int,int] = (170,120,120)
    red_upper2: Tuple[int,int,int] = (180,255,255)
    green_lower: Tuple[int,int,int] = (40,120,120)
    green_upper: Tuple[int,int,int] = (80,255,255)

@dataclass
class DetectionConfig:
    gaussian_kernel: int = 5
    gaussian_sigma: float = 1.0
    morph_kernel_size: int = 5
    erosion_iterations: int = 1
    dilation_iterations: int = 2
    min_area: int = 100
    max_area: int = 50000
    min_aspect_ratio: float = 0.3
    max_aspect_ratio: float = 3.0
    min_solidity: float = 0.5
    min_extent: float = 0.3
    expected_pillar_ratio: float = 2.0
    ratio_tolerance: float = 0.5

@dataclass
class IntegrationConfig:
    communication_mode: str = "zmq"
    zmq_port: int = 5555
    tcp_port: int = 8888
    shared_file: str = "/tmp/robot_vision_commands.json"
    update_rate: int = 20

# ----------------- CAMERA INTERFACE -----------------

class CameraInterface:
    def __init__(self, config: CameraConfig):
        self.config = config
        self.logger = logging.getLogger(__name__ + '.CameraInterface')
        self.camera = None
        self.capture = None
        self._setup_camera()

    def _setup_camera(self):
        try:
            if PICAMERA_AVAILABLE:
                self._setup_picamera2()
            else:
                self._setup_opencv_camera()
        except Exception as e:
            self.logger.error(f"Camera setup failed: {e}")
            raise

    def _setup_picamera2(self):
        self.camera = Picamera2()
        video_config = self.camera.create_video_configuration(
            main={"size": (self.config.width, self.config.height), "format": "YUV420"},
            controls={"FrameRate": self.config.fps}
        )
        self.camera.configure(video_config)
        self.camera.start()
        time.sleep(2)
        self.logger.info("Picamera2 initialized")

    def _setup_opencv_camera(self):
        self.capture = cv2.VideoCapture(0)
        if not self.capture.isOpened():
            raise RuntimeError("Failed to open OpenCV camera")
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
        self.capture.set(cv2.CAP_PROP_FPS, self.config.fps)
        self.logger.info("OpenCV camera initialized")

    def capture_frame(self) -> Optional[np.ndarray]:
        try:
            if self.camera:
                frame = self.camera.capture_array("main")
                return cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_I420)
            elif self.capture:
                ret, frame = self.capture.read()
                return frame if ret else None
        except Exception as e:
            self.logger.error(f"Frame capture failed: {e}")
        return None

    def release(self):
        if self.camera:
            self.camera.stop()
            self.camera.close()
        if self.capture:
            self.capture.release()
        self.logger.info("Camera released")

# ----------------- IMAGE PROCESSOR -----------------

class ImageProcessor:
    def __init__(self, color_config: ColorConfig, detection_config: DetectionConfig):
        self.color_config = color_config
        self.detection_config = detection_config
        self.logger = logging.getLogger(__name__ + '.ImageProcessor')
        self.morph_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self.detection_config.morph_kernel_size, self.detection_config.morph_kernel_size)
        )

    def preprocess_frame(self, frame: np.ndarray) -> np.ndarray:
        if self.detection_config.gaussian_kernel > 1:
            frame = cv2.GaussianBlur(frame,
                                     (self.detection_config.gaussian_kernel,
                                      self.detection_config.gaussian_kernel),
                                     self.detection_config.gaussian_sigma)
        return cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    def detect_color_regions(self, hsv_frame: np.ndarray, color: PillarColor) -> np.ndarray:
        if color == PillarColor.RED:
            mask1 = cv2.inRange(hsv_frame, np.array(self.color_config.red_lower, np.uint8),
                                np.array(self.color_config.red_upper, np.uint8))
            mask2 = cv2.inRange(hsv_frame, np.array(self.color_config.red_lower2, np.uint8),
                                np.array(self.color_config.red_upper2, np.uint8))
            mask = cv2.bitwise_or(mask1, mask2)
        elif color == PillarColor.GREEN:
            mask = cv2.inRange(hsv_frame, np.array(self.color_config.green_lower, np.uint8),
                               np.array(self.color_config.green_upper, np.uint8))
        else:
            mask = np.zeros(hsv_frame.shape[:2], np.uint8)
        return mask

    def apply_morphological_operations(self, mask: np.ndarray) -> np.ndarray:
        if self.detection_config.erosion_iterations > 0:
            mask = cv2.erode(mask, self.morph_kernel, iterations=self.detection_config.erosion_iterations)
        if self.detection_config.dilation_iterations > 0:
            mask = cv2.dilate(mask, self.morph_kernel, iterations=self.detection_config.dilation_iterations)
        return mask

    def find_contours_and_filter(self, mask: np.ndarray, color: PillarColor) -> List[DetectionResult]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []
        frame_center_x = mask.shape[1] // 2

        for contour in contours:
            area = cv2.contourArea(contour)
            if not (self.detection_config.min_area <= area <= self.detection_config.max_area):
                continue
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = h / w if w else 0
            if not (self.detection_config.min_aspect_ratio <= aspect_ratio <= self.detection_config.max_aspect_ratio):
                continue
            hull = cv2.convexHull(contour)
            hull_area = cv2.contourArea(hull)
            solidity = area / hull_area if hull_area > 0 else 0
            extent = area / (w*h) if (w*h) > 0 else 0
            if solidity < self.detection_config.min_solidity or extent < self.detection_config.min_extent:
                continue
            # expected pillar ratio
            if not (self.detection_config.expected_pillar_ratio - self.detection_config.ratio_tolerance <= aspect_ratio <= self.detection_config.expected_pillar_ratio + self.detection_config.ratio_tolerance):
                continue
            M = cv2.moments(contour)
            cx = int(M["m10"]/M["m00"]) if M["m00"] else x+w//2
            cy = int(M["m01"]/M["m00"]) if M["m00"] else y+h//2
            lateral_offset = np.clip((cx-frame_center_x)/frame_center_x, -1.0, 1.0)
            confidence = min(area/1000*0.3 + (1-abs(aspect_ratio-self.detection_config.expected_pillar_ratio)/self.detection_config.expected_pillar_ratio)*0.4 + solidity*0.2 + extent*0.1, 1.0)
            detections.append(DetectionResult(color, (cx,cy), (x,y,w,h), area, confidence, lateral_offset))
        detections.sort(key=lambda d: (d.confidence, d.area), reverse=True)
        return detections

# ----------------- STEERING GUIDANCE -----------------

class SteeringGuidance:
    def __init__(self, frame_width:int):
        self.frame_center = frame_width//2
        self.confidence_threshold = 0.5

    def analyze_detections(self, red_dets:List[DetectionResult], green_dets:List[DetectionResult]) -> Dict:
        best_red = red_dets[0] if red_dets else None
        best_green = green_dets[0] if green_dets else None
        primary = None
        if best_red and best_green:
            primary = best_red if best_red.area>best_green.area else best_green
        elif best_red:
            primary = best_red
        elif best_green:
            primary = best_green
        command = SteeringCommand.NO_PILLAR
        if primary and primary.confidence>self.confidence_threshold:
            command = SteeringCommand.PILLAR_RIGHT if primary.color==PillarColor.RED else SteeringCommand.PILLAR_LEFT
        return {
            'steering_command': command,
            'primary_pillar': primary,
            'red_count': len(red_dets),
            'green_count': len(green_dets),
            'confidence': primary.confidence if primary else 0.0
        }

# ----------------- COMMUNICATION -----------------

class CommunicationInterface:
    def __init__(self, config:IntegrationConfig):
        self.config = config
        self.logger = logging.getLogger(__name__+'.CommunicationInterface')
        self.zmq_socket = None
        if config.communication_mode=='zmq':
            context = zmq.Context()
            self.zmq_socket = context.socket(zmq.PUB)
            self.zmq_socket.bind(f"tcp://*:{config.zmq_port}")
            self.logger.info(f"ZMQ publisher bound to port {config.zmq_port}")

    def send_command(self, cmd:Dict):
        if self.zmq_socket:
            try:
                self.zmq_socket.send_string(json.dumps(cmd, default=str))
            except Exception as e:
                self.logger.error(f"ZMQ send failed: {e}")

# ----------------- VISION SYSTEM -----------------

class VisionPillarSystem:
    def __init__(self, config_file:str="config.yaml"):
        self.logger = logging.getLogger(__name__+'.VisionPillarSystem')
        with open(config_file,'r') as f:
            cfg = yaml.safe_load(f)
        self.camera = CameraInterface(CameraConfig(**cfg['camera']))
        self.processor = ImageProcessor(ColorConfig(**cfg['colors']), DetectionConfig(**cfg['detection']))
        self.steering = SteeringGuidance(cfg['camera']['width'])
        self.comm = CommunicationInterface(IntegrationConfig(**cfg['integration']))
        self.running = False
        self.result_queue = queue.Queue(maxsize=1)
        self.lock = threading.Lock()
        self.update_rate = cfg['integration'].get('update_rate',20)

    def process_frame(self, frame):
        hsv = self.processor.preprocess_frame(frame)
        red_mask = self.processor.detect_color_regions(hsv, PillarColor.RED)
        green_mask = self.processor.detect_color_regions(hsv, PillarColor.GREEN)
        red_mask = self.processor.apply_morphological_operations(red_mask)
        green_mask = self.processor.apply_morphological_operations(green_mask)
        red_dets = self.processor.find_contours_and_filter(red_mask, PillarColor.RED)
        green_dets = self.processor.find_contours_and_filter(green_mask, PillarColor.GREEN)
        guidance = self.steering.analyze_detections(red_dets, green_dets)
        return guidance

    def cleanup(self):
        self.camera.release()
        if self.comm.zmq_socket:
            self.comm.zmq_socket.close()
        logger.info("Vision system cleaned up.")

# ----------------- MAIN WITH PREVIEW -----------------

def main():
    system = None
    try:
        system = VisionPillarSystem()
        logger.info("Vision system started with live preview.")

        while True:
            frame = system.camera.capture_frame()
            if frame is None:
                continue
            result = system.process_frame(frame)

            # Draw primary pillar
            pillar = result['primary_pillar']
            if pillar:
                x,y,w,h = pillar.bbox
                color = (0,0,255) if pillar.color==PillarColor.RED else (0,255,0)
                cv2.rectangle(frame, (x,y), (x+w,y+h), color, 2)
                text = f"{pillar.color.value} ({pillar.confidence:.2f})"
                cv2.putText(frame,text,(x,y-10),cv2.FONT_HERSHEY_SIMPLEX,0.6,color,2)

            # Show steering command
            cv2.putText(frame,f"Steering: {result['steering_command'].value}",(10,30),
                        cv2.FONT_HERSHEY_SIMPLEX,0.8,(255,255,255),2)

            cv2.imshow("Pillar Detection Preview", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Send command over ZMQ
            system.comm.send_command(result)

            time.sleep(1.0/system.update_rate)

    except KeyboardInterrupt:
        logger.info("Interrupted by user.")
    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
    finally:
        if system:
            system.cleanup()
        cv2.destroyAllWindows()
        logger.info("Preview window closed.")

if __name__=="__main__":
    main()
