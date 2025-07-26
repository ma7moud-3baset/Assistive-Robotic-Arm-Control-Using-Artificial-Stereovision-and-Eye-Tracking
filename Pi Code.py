import cv2
import mediapipe as mp
import numpy as np
from collections import deque, Counter
import time
from threading import Thread
import signal
import sys

# Hardware imports with graceful fallback
try:
    import board
    import busio
    from adafruit_pca9685 import PCA9685
    from adafruit_motor import servo
    PCA9685_AVAILABLE = True
    print("PCA9685 libraries loaded successfully")
except ImportError:
    PCA9685_AVAILABLE = False
    print("WARNING: PCA9685 libraries not available - running in simulation mode")

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
    print("GPIO libraries loaded successfully")
except ImportError:
    GPIO_AVAILABLE = False
    print("WARNING: GPIO libraries not available")

# Configuration constants
SERVO_CHANNELS = [0, 1, 2, 3, 4, 5]
GPIO_PINS = [18, 19, 13, 6, 12, 16]
SERVO_FREQUENCY = 50

CALCULATION_DELAY = 2
READING_DELAY = 5
TOTAL_DELAY = CALCULATION_DELAY + READING_DELAY

INITIAL_SERVO_ANGLES = [25, 60, 10, 40, 150, 0]
CENTER_ANGLES = [90, 90, 90, 90, 90, 0]

MOVEMENT_SCENARIOS = {
    "UP": {1: -15, 2: -15},
    "DOWN": {1: +15, 2: +15},
    "LEFT": {0: -20},
    "RIGHT": {0: +20},
    "CENTER": {}
}

# Eye landmark indices
LEFT_EYE_OUTLINE = [33, 7, 163, 144, 145, 153, 154, 155, 133, 173, 157, 158, 159, 160, 161, 246]
RIGHT_EYE_OUTLINE = [362, 382, 381, 380, 374, 373, 390, 249, 263, 466, 388, 387, 386, 385, 384, 398]
LEFT_IRIS = [468, 469, 470, 471, 472]
RIGHT_IRIS = [473, 474, 475, 476, 477]

class ServoController:
    def __init__(self):
        self.servos = []
        self.control_type = self._initialize_servos()
        self.current_angles = INITIAL_SERVO_ANGLES.copy()
        self._move_to_initial_position()
    
    def _initialize_servos(self):
        if PCA9685_AVAILABLE and self._try_pca9685():
            return "PCA9685"
        elif GPIO_AVAILABLE and self._try_gpio():
            return "GPIO"
        else:
            print("WARNING: No servo control available - running in simulation mode")
            return "SIMULATION"
    
    def _try_pca9685(self):
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            pca = PCA9685(i2c)
            pca.frequency = SERVO_FREQUENCY
            
            self.servos = [servo.Servo(pca.channels[ch]) for ch in SERVO_CHANNELS]
            print(f"PCA9685 initialized with {len(self.servos)} servos")
            return True
        except Exception as e:
            print(f"ERROR: PCA9685 initialization failed: {e}")
            return False
    
    def _try_gpio(self):
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            self.servos = []
            for pin in GPIO_PINS:
                GPIO.setup(pin, GPIO.OUT)
                servo_pwm = GPIO.PWM(pin, SERVO_FREQUENCY)
                servo_pwm.start(0)
                self.servos.append(servo_pwm)
            
            print(f"GPIO initialized with {len(self.servos)} servos")
            return True
        except Exception as e:
            print(f"ERROR: GPIO initialization failed: {e}")
            return False
    
    def _move_to_initial_position(self):
        print("Initializing servos to starting position...")
        for i, angle in enumerate(self.current_angles):
            self.move_servo(i, angle)
            time.sleep(0.2)
        print("Servos initialized")
    
    def move_servo(self, servo_index, angle):
        if not (0 <= servo_index < len(self.servos) and 0 <= angle <= 180):
            return
        
        try:
            if self.control_type == "PCA9685":
                self.servos[servo_index].angle = angle
            elif self.control_type == "GPIO":
                duty_cycle = 2.5 + (angle / 180.0) * 10.0
                self.servos[servo_index].ChangeDutyCycle(duty_cycle)
                time.sleep(0.1)
                self.servos[servo_index].ChangeDutyCycle(0)
            
            print(f"   {self.control_type} Servo {servo_index}: {angle}°")
        except Exception as e:
            print(f"WARNING: Servo {servo_index} movement error: {e}")
    
    def cleanup(self):
        if self.control_type == "PCA9685":
            try:
                for servo_obj in self.servos:
                    servo_obj.angle = None
                print("PCA9685 servos cleaned up")
            except Exception as e:
                print(f"WARNING: PCA9685 cleanup error: {e}")
        elif self.control_type == "GPIO":
            try:
                for servo_pwm in self.servos:
                    servo_pwm.stop()
                GPIO.cleanup()
                print("GPIO cleaned up")
            except Exception as e:
                print(f"WARNING: GPIO cleanup error: {e}")

class EyeTracker:
    def __init__(self):
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.gaze_history = deque(maxlen=10)
    
    def get_eye_coordinates(self, mesh, w, h):
        def get_coords(index_list):
            return np.array([(mesh[i].x * w, mesh[i].y * h) for i in index_list])
        
        return {
            'left_eye': get_coords(LEFT_EYE_OUTLINE),
            'right_eye': get_coords(RIGHT_EYE_OUTLINE),
            'left_iris': get_coords(LEFT_IRIS),
            'right_iris': get_coords(RIGHT_IRIS)
        }
    
    def is_eye_visible(self, iris_coords, eye_coords):
        if len(iris_coords) == 0 or len(eye_coords) == 0:
            return False
        
        iris_center = np.mean(iris_coords, axis=0)
        eye_bounds = {
            'top': np.min(eye_coords[:, 1]),
            'bottom': np.max(eye_coords[:, 1]),
            'left': np.min(eye_coords[:, 0]),
            'right': np.max(eye_coords[:, 0])
        }
        
        iris_in_bounds = (eye_bounds['left'] <= iris_center[0] <= eye_bounds['right'] and 
                         eye_bounds['top'] <= iris_center[1] <= eye_bounds['bottom'])
        
        eye_height = eye_bounds['bottom'] - eye_bounds['top']
        distance_from_top = iris_center[1] - eye_bounds['top']
        
        too_close_edges = (distance_from_top < eye_height * 0.15 or 
                          (eye_bounds['bottom'] - iris_center[1]) < eye_height * 0.15)
        
        return iris_in_bounds and not too_close_edges
    
    def normalize_gaze_position(self, iris_pos, eye_coords):
        eye_center = np.mean(eye_coords, axis=0)
        eye_width = np.max(eye_coords[:, 0]) - np.min(eye_coords[:, 0])
        eye_height = np.max(eye_coords[:, 1]) - np.min(eye_coords[:, 1])
        
        normalized_x = (iris_pos[0] - eye_center[0]) / (eye_width / 2)
        normalized_y = (iris_pos[1] - eye_center[1]) / (eye_height / 2)
        
        return np.array([normalized_x, normalized_y])
    
    def determine_gaze_direction(self, left_norm, right_norm):
        avg_x = (left_norm[0] + right_norm[0]) / 2
        avg_y = (left_norm[1] + right_norm[1]) / 2
        
        horizontal_threshold = 0.25
        vertical_threshold = 0.35
        
        horizontal_dir = ("LEFT" if avg_x < -horizontal_threshold else
                         "RIGHT" if avg_x > horizontal_threshold else "CENTER_H")
        
        vertical_dir = ("UP" if avg_y < -vertical_threshold else
                       "DOWN" if avg_y > vertical_threshold else "CENTER_V")
        
        if horizontal_dir == "CENTER_H" and vertical_dir == "CENTER_V":
            return "CENTER"
        elif horizontal_dir != "CENTER_H" and vertical_dir == "CENTER_V":
            return horizontal_dir
        elif horizontal_dir == "CENTER_H" and vertical_dir != "CENTER_V":
            return vertical_dir
        else:
            return f"{vertical_dir}_{horizontal_dir}"
    
    def smooth_gaze_direction(self, current_gaze):
        self.gaze_history.append(current_gaze)
        
        if len(self.gaze_history) >= 3:
            recent_gazes = list(self.gaze_history)[-5:]
            gaze_counts = Counter(recent_gazes)
            most_common = gaze_counts.most_common(1)[0][0]
            
            if most_common != current_gaze and gaze_counts[current_gaze] < 2:
                return most_common
        
        return current_gaze

class RobotController:
    def __init__(self, servo_controller):
        self.servo_controller = servo_controller
        self.last_movement_time = 0
        self.calculation_start_time = 0
        self.calculation_phase = False
        self.reading_phase = False
        self.pending_gaze_direction = None
        self.grasp_start_time = None
        self.grasped = False
        self.last_command = "System started"
    
    def execute_movement(self, gaze_direction):
        current_time = time.time()
        
        # Handle delay phases
        if self.calculation_phase or self.reading_phase:
            return self._handle_delay_phases(current_time)
        
        # Check minimum time between commands
        if current_time - self.last_movement_time < 1:
            return False, "Please wait..."
        
        # Start new movement
        return self._start_new_movement(gaze_direction, current_time)
    
    def _handle_delay_phases(self, current_time):
        time_since_start = current_time - self.calculation_start_time
        
        if self.calculation_phase and time_since_start >= CALCULATION_DELAY:
            self.calculation_phase = False
            self.reading_phase = True
            remaining = READING_DELAY - int(time_since_start - CALCULATION_DELAY)
            return False, f"Reading phase: {remaining}s left"
        
        elif self.reading_phase and time_since_start >= TOTAL_DELAY:
            return self._execute_pending_movement(current_time)
        
        elif self.calculation_phase:
            remaining = CALCULATION_DELAY - int(time_since_start)
            return False, f"Calculating: {remaining}s left"
        elif self.reading_phase:
            remaining = READING_DELAY - int(time_since_start - CALCULATION_DELAY)
            return False, f"Reading phase: {remaining}s left"
    
    def _execute_pending_movement(self, current_time):
        self.calculation_phase = False
        self.reading_phase = False
        
        if self.pending_gaze_direction in MOVEMENT_SCENARIOS:
            movements = MOVEMENT_SCENARIOS[self.pending_gaze_direction]
            if movements:
                self._apply_servo_movements(movements)
                self.last_movement_time = current_time
                result = f"Executed {self.pending_gaze_direction}"
                self.pending_gaze_direction = None
                return True, result
        
        self.pending_gaze_direction = None
        return False, "Delay complete - Ready for next command"
    
    def _start_new_movement(self, gaze_direction, current_time):
        if gaze_direction in MOVEMENT_SCENARIOS and gaze_direction != "CENTER":
            movements = MOVEMENT_SCENARIOS[gaze_direction]
            if movements:
                self.calculation_phase = True
                self.calculation_start_time = current_time
                self.pending_gaze_direction = gaze_direction
                print(f"\nNew direction detected: {gaze_direction}")
                return False, f"Calculating: {CALCULATION_DELAY}s"
        
        return False, "Monitoring gaze..."
    
    def _apply_servo_movements(self, movements):
        print(f"\nExecuting movement")
        print(f"   Angles before: {self.servo_controller.current_angles}")
        
        servo_changes = {}
        for servo_id, angle_change in movements.items():
            if 0 <= servo_id < len(self.servo_controller.current_angles):
                new_angle = self.servo_controller.current_angles[servo_id] + angle_change
                new_angle = max(0, min(180, new_angle))
                servo_changes[servo_id] = new_angle
                self.servo_controller.current_angles[servo_id] = new_angle
        
        # Execute in background thread
        movement_thread = Thread(target=self._execute_servo_changes, args=(servo_changes,))
        movement_thread.daemon = True
        movement_thread.start()
        
        print(f"   Angles after: {self.servo_controller.current_angles}")
    
    def _execute_servo_changes(self, servo_changes):
        try:
            for servo_id, new_angle in servo_changes.items():
                self.servo_controller.move_servo(servo_id, new_angle)
                time.sleep(0.1)
        except Exception as e:
            print(f"WARNING: Servo movement error: {e}")
    
    def handle_grasp(self, gaze_direction):
        if gaze_direction == "CENTER":
            if self.grasp_start_time is None:
                self.grasp_start_time = time.time()
            elif time.time() - self.grasp_start_time >= 7:
                self._toggle_grasp()
                self.grasp_start_time = None
        else:
            self.grasp_start_time = None
    
    def _toggle_grasp(self):
        if not self.grasped:
            self.servo_controller.move_servo(5, 180)
            self.servo_controller.current_angles[5] = 180
            self.grasped = True
            self.last_command = "Object grasped!"
            print("Object grasped!")
        else:
            self.servo_controller.move_servo(5, 0)
            self.servo_controller.current_angles[5] = 0
            self.grasped = False
            self.last_command = "Object released!"
            print("Object released!")
    
    def reset_to_center(self):
        self.servo_controller.current_angles = CENTER_ANGLES.copy()
        self.last_command = "Reset completed"
        self.calculation_phase = False
        self.reading_phase = False
        self.pending_gaze_direction = None
        self.grasp_start_time = None
        self.grasped = False
        
        print("\nResetting robot arm to center position")
        for i, angle in enumerate(self.servo_controller.current_angles):
            self.servo_controller.move_servo(i, angle)
            time.sleep(0.1)
        print(f"   New angles: {self.servo_controller.current_angles}")
        print("Gripper reset to open position")

class DisplayManager:
    @staticmethod
    def get_delay_status(robot_controller):
        if not (robot_controller.calculation_phase or robot_controller.reading_phase):
            return "Ready", (0, 255, 0)
        
        current_time = time.time()
        time_since_start = current_time - robot_controller.calculation_start_time
        
        if robot_controller.calculation_phase:
            remaining = max(0, CALCULATION_DELAY - time_since_start)
            return f"Calc: {remaining:.1f}s", (255, 165, 0)
        elif robot_controller.reading_phase:
            remaining = max(0, TOTAL_DELAY - time_since_start)
            return f"Read: {remaining:.1f}s", (0, 165, 255)
        
        return "Ready", (0, 255, 0)
    
    @staticmethod
    def get_grasp_status(robot_controller):
        if robot_controller.grasp_start_time is None:
            return ("Grasped", (0, 255, 0)) if robot_controller.grasped else ("Released", (128, 128, 128))
        else:
            elapsed = time.time() - robot_controller.grasp_start_time
            remaining = max(0, 7 - elapsed)
            status = "Release" if robot_controller.grasped else "Grasp"
            return f"{status}: {remaining:.1f}s", (255, 165, 0)
    
    @staticmethod
    def draw_info_overlay(frame, servo_controller, robot_controller):
        h, w = frame.shape[:2]
        
        # Control type
        cv2.putText(frame, f"Control: {servo_controller.control_type}", (10, 15), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        
        # Status info
        delay_status, delay_color = DisplayManager.get_delay_status(robot_controller)
        cv2.putText(frame, f"Status: {delay_status}", (w-200, 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, delay_color, 1)
        
        grasp_status, grasp_color = DisplayManager.get_grasp_status(robot_controller)
        cv2.putText(frame, f"Grip: {grasp_status}", (w-200, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, grasp_color, 1)
        
        cv2.putText(frame, f"Last: {robot_controller.last_command[:15]}", (w-200, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Servo angles
        cv2.putText(frame, "Servos:", (10, 35), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        
        angles = servo_controller.current_angles
        for i in range(0, len(angles), 2):
            y_pos = 50 + (i // 2) * 15
            if i + 1 < len(angles):
                text = f"{i}:{angles[i]}° {i+1}:{angles[i+1]}°"
            else:
                text = f"{i}:{angles[i]}°"
            cv2.putText(frame, text, (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1)
    
    @staticmethod
    def draw_eye_tracking(frame, coords, gaze):
        # Draw eyes and iris
        cv2.polylines(frame, [coords['left_eye'].astype(int)], True, (0, 255, 0), 1)
        cv2.polylines(frame, [coords['right_eye'].astype(int)], True, (0, 255, 0), 1)
        
        left_iris_pos = np.mean(coords['left_iris'], axis=0)
        right_iris_pos = np.mean(coords['right_iris'], axis=0)
        
        cv2.circle(frame, tuple(left_iris_pos.astype(int)), 2, (0, 255, 255), -1)
        cv2.circle(frame, tuple(right_iris_pos.astype(int)), 2, (0, 255, 255), -1)
        
        # Eye centers
        left_center = np.mean(coords['left_eye'], axis=0)
        right_center = np.mean(coords['right_eye'], axis=0)
        cv2.circle(frame, tuple(left_center.astype(int)), 1, (255, 0, 0), -1)
        cv2.circle(frame, tuple(right_center.astype(int)), 1, (255, 0, 0), -1)
        
        # Gaze direction
        h, w = frame.shape[:2]
        color = (0, 255, 0) if gaze == "CENTER" else (0, 0, 255)
        cv2.putText(frame, f"Gaze: {gaze}", (w//2-60, h-20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

def setup_camera():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 900)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 15)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    if not cap.isOpened():
        raise RuntimeError("ERROR: Could not open camera")
    
    print("Camera initialized")
    return cap

def signal_handler(sig, frame, servo_controller):
    print("\nShutting down...")
    servo_controller.cleanup()
    cv2.destroyAllWindows()
    sys.exit(0)

def main():
    # Initialize components
    servo_controller = ServoController()
    eye_tracker = EyeTracker()
    robot_controller = RobotController(servo_controller)
    
    # Setup signal handler
    signal.signal(signal.SIGINT, lambda s, f: signal_handler(s, f, servo_controller))
    
    # Setup camera
    try:
        cap = setup_camera()
    except RuntimeError as e:
        print(e)
        servo_controller.cleanup()
        return
    
    print("Starting eye tracking system...")
    print(f"Servo control mode: {servo_controller.control_type}")
    print("Controls:")
    print("   - Look UP/DOWN/LEFT/RIGHT to control robot arm")
    print("   - Look at CENTER for 7 seconds to grasp/release objects")
    print("   - Press 'q' to quit")
    print("   - Press 'r' to reset robot to center")
    
    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("WARNING: Failed to read frame")
                break

            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = eye_tracker.face_mesh.process(rgb)

            if results.multi_face_landmarks:
                mesh = results.multi_face_landmarks[0].landmark
                h, w, _ = frame.shape
                coords = eye_tracker.get_eye_coordinates(mesh, w, h)

                try:
                    left_visible = eye_tracker.is_eye_visible(coords['left_iris'], coords['left_eye'])
                    right_visible = eye_tracker.is_eye_visible(coords['right_iris'], coords['right_eye'])

                    if not left_visible or not right_visible:
                        gaze = "DOWN"
                        cv2.putText(frame, "Eye not visible", (w//2-80, h-40), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                    else:
                        left_iris_pos = np.mean(coords['left_iris'], axis=0)
                        right_iris_pos = np.mean(coords['right_iris'], axis=0)
                        
                        left_norm = eye_tracker.normalize_gaze_position(left_iris_pos, coords['left_eye'])
                        right_norm = eye_tracker.normalize_gaze_position(right_iris_pos, coords['right_eye'])
                        
                        current_gaze = eye_tracker.determine_gaze_direction(left_norm, right_norm)
                        gaze = eye_tracker.smooth_gaze_direction(current_gaze)
                        
                        DisplayManager.draw_eye_tracking(frame, coords, gaze)

                    smoothed_gaze = gaze
                    robot_controller.handle_grasp(smoothed_gaze)
                    
                    movement_executed, command_result = robot_controller.execute_movement(smoothed_gaze)
                    if movement_executed:
                        robot_controller.last_command = command_result

                except Exception as e:
                    cv2.putText(frame, "Detection error", (30, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            else:
                cv2.putText(frame, "No face detected", (30, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            DisplayManager.draw_info_overlay(frame, servo_controller, robot_controller)
            cv2.imshow("Pi Eye Tracker", frame)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                robot_controller.reset_to_center()

    except Exception as e:
        print(f"ERROR: Error during execution: {e}")
    finally:
        print(f"\nFinal summary:")
        print(f"   Servo control type: {servo_controller.control_type}")
        print(f"   Last arm angles: {servo_controller.current_angles}")
        print(f"   Last executed command: {robot_controller.last_command}")
        
        cap.release()
        cv2.destroyAllWindows()
        servo_controller.cleanup()
        print("Cleanup completed")

if __name__ == "__main__":
    main()