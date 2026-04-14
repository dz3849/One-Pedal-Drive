import math
import time
import random
import threading
from dataclasses import dataclass
from typing import Optional, List, Tuple, Deque
from collections import deque
from queue import Queue, Empty
#zucheng  shi sb
import numpy as np
import pygame
#woshisb

#Communication
import serial
import threading
import platform

data_queue = Queue()

data_dict = {"drive": '', "delta": '', "pwm": '', "Obstacle_Distance": '', "Mode": '', "Encoder_Count": '', "Direction": '', "RPM": ''}
state = ''
# ----------------------------
# Config
# ----------------------------
WIDTH, HEIGHT = 900, 600
FPS = 60

CAR_POS = (200, HEIGHT // 2)
CAR_SIZE = (80, 40)

# Lane dash settings
LANE_COLOR = (235, 235, 235)
LANE_WIDTH = 4
DASH_LENGTH = 40
GAP_LENGTH = 30
LANE_Y_OFFSET = 70  

# Sensor/model assumptions
MAX_RANGE_M = 70
MIN_RANGE_M = 0.05

# Filtering
EMA_ALPHA = 0.25          # higher = less smoothing
OUTLIER_JUMP_M = 0.6      # reject sudden jump bigger than this (tune)

# World scaling
PIXELS_PER_M = 20

# ----------------------------
# Data types
# ----------------------------
@dataclass
class SensorSample:
    t: float           # seconds
    angle_deg: float   # 0 means straight ahead; can be 0 for ultrasonic
    range_m: float

# ----------------------------
# Simple filters
# ----------------------------
class EMAFilter:
    def __init__(self, alpha: float, init: Optional[float] = None):
        self.alpha = alpha
        self.y = init

    def update(self, x: float) -> float:
        if self.y is None:
            self.y = x
        else:
            self.y = self.alpha * x + (1 - self.alpha) * self.y
        return self.y

class RangeOutlierGate:
    def __init__(self, max_jump_m: float):
        self.max_jump_m = max_jump_m
        self.prev: Optional[float] = None

    def accept(self, x: float) -> bool:
        if self.prev is None:
            self.prev = x
            return True
        if abs(x - self.prev) > self.max_jump_m:
            return False
        self.prev = x
        return True

class Kalman1D_DistVel:
    """
    State: [distance; velocity]
    Measurement: distance
    Constant-velocity model.
    """
    def __init__(self):
        self.x = np.array([[2.0], [0.0]])  # start 2m away, 0 m/s
        self.P = np.diag([0.5, 1.0])

        self.sigma_a = 2.0    # process accel noise (tune)
        self.sigma_z = 0.08   # measurement noise (tune)

        self.last_t: Optional[float] = None

    def update(self, z: float, t: float) -> Tuple[float, float]:
        if self.last_t is None:
            self.last_t = t
            self.x[0, 0] = z
            return float(self.x[0, 0]), float(self.x[1, 0])

        dt = max(1e-3, t - self.last_t)
        self.last_t = t

        # State transition
        F = np.array([[1.0, dt],
                      [0.0, 1.0]])

        # Process noise (from accel variance)
        q = self.sigma_a ** 2
        Q = q * np.array([[dt**4 / 4, dt**3 / 2],
                          [dt**3 / 2, dt**2]])

        # Predict
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

        # Measurement update
        H = np.array([[1.0, 0.0]])
        R = np.array([[self.sigma_z ** 2]])

        y = np.array([[z]]) - (H @ self.x)          # innovation
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(2) - K @ H) @ self.P

        return float(self.x[0, 0]), float(self.x[1, 0])

# ----------------------------
# World model
# ----------------------------
class WorldModel:
    def __init__(self):
        self.points_car_frame: Deque[Tuple[float, float]] = deque(maxlen=250)  # (x_m, y_m) relative to car
        self.closest_raw: Optional[float] = None
        self.closest_smooth: Optional[float] = None
        self.closest_kf: Optional[float] = None
        self.approach_speed: Optional[float] = None
        self.gate = RangeOutlierGate(OUTLIER_JUMP_M)
        self.ema = EMAFilter(EMA_ALPHA)
        self.kf = Kalman1D_DistVel()
        self.last_detection_time = None
        self.detection_timeout = 0.25   # seconds (tune this)

    def ingest(self, samples: List[SensorSample]):
        # Convert polar to Cartesian in car frame (x forward, y left)
        for s in samples:
            r = s.range_m
            if not (MIN_RANGE_M <= r <= MAX_RANGE_M):
                continue
            th = math.radians(s.angle_deg)
            x = r * math.cos(th)
            y = r * math.sin(th)
            self.points_car_frame.append((x, y))

        # For UI: define "closest obstacle" as minimum range in this batch
        if not samples:
            return

        closest = min(s.range_m for s in samples)
        self.closest_raw = closest
        self.last_detection_time = time.time()

        # Outlier gate on the "closest" distance (simple + effective)
        if self.gate.accept(closest):
            self.closest_smooth = self.ema.update(closest)
            d_kf, v_kf = self.kf.update(closest, samples[0].t)
            self.closest_kf = d_kf
            self.approach_speed = -v_kf  # positive means approaching
        # else: ignore update; keep last filtered values
    def mark_no_detection(self):
        """
        Call this when no valid sensor samples are received.
        Clears obstacle if it hasn't been seen recently.
        """
        # First time: nothing ever detected
        if self.last_detection_time is None:
            self.closest_raw = None
            self.closest_smooth = None
            self.closest_kf = None
            self.approach_speed = None
            self.points_car_frame.clear()
            return

        # If too much time has passed → forget obstacle
        if time.time() - self.last_detection_time > self.detection_timeout:
            self.closest_raw = None
            self.closest_smooth = None
            self.closest_kf = None
            self.approach_speed = None
            self.points_car_frame.clear()
# ----------------------------
# Rendering
# ----------------------------
def meters_to_screen(dx_m: float, dy_m: float) -> Tuple[int, int]:
    # Car frame: x forward (right on screen), y left (up on screen)
    x = int(CAR_POS[0] + dx_m * PIXELS_PER_M)
    y = int(CAR_POS[1] - dy_m * PIXELS_PER_M)
    return x, y

def draw_lane_dashes(screen: pygame.Surface, dash_offset: float):
    """
    Draw two horizontal dashed lane-divider lines that scroll left.
    dash_offset should increase with vehicle speed.
    """
    y_top = CAR_POS[1] - LANE_Y_OFFSET
    y_bottom = CAR_POS[1] + LANE_Y_OFFSET

    pattern = DASH_LENGTH + GAP_LENGTH

    # start from off-screen so dashes always cover full width
    start_x = -pattern + int(dash_offset % pattern)

    for y in [y_top, y_bottom]:
        x = start_x
        while x < WIDTH:
            pygame.draw.line(
                screen,
                LANE_COLOR,
                (x, y),
                (x + DASH_LENGTH, y),
                LANE_WIDTH
            )
            x += pattern

def draw(world: WorldModel, screen: pygame.Surface, font: pygame.font.Font, dash_offset: float):    
    global data_dict
    screen.fill((18, 18, 22))

    draw_lane_dashes(screen, dash_offset)

    # Draw car
    car_rect = pygame.Rect(0, 0, CAR_SIZE[0], CAR_SIZE[1])
    car_rect.center = CAR_POS
    pygame.draw.rect(screen, (240, 240, 240), car_rect, border_radius=6)

    # Choose one obstacle distance to display
    obstacle_distance = None
    if world.closest_kf is not None:
        obstacle_distance = world.closest_kf
    elif world.closest_smooth is not None:
        obstacle_distance = world.closest_smooth
    elif world.closest_raw is not None:
        obstacle_distance = world.closest_raw

    # Draw one big obstacle block only if it is in range
    if obstacle_distance is not None and MIN_RANGE_M <= obstacle_distance <= MAX_RANGE_M:
        obstacle_x, obstacle_y = meters_to_screen(obstacle_distance, 0.0)

        obstacle_width_px = 40
        obstacle_height_px = 40

        obstacle_rect = pygame.Rect(
            obstacle_x - obstacle_width_px // 2,
            obstacle_y - obstacle_height_px // 2,
            obstacle_width_px,
            obstacle_height_px
        )
        state = "obstacle detected"
        pygame.draw.rect(screen, (255, 130, 90), obstacle_rect, border_radius=4)

    else:
        state = "Obstacle Distance is None"

    #HUD setup
    drive_indicator = data_dict["drive"]
    delta_indicator = data_dict["delta"]
    pwm_indicator = data_dict["pwm"]
    distance_indicator = data_dict["Obstacle_Distance"]
    Encoder_indicator = data_dict["Encoder_Count"]
    direction_indicator = data_dict["Direction"]
    rpm_indicator = data_dict["RPM"]

    #HUD
    lines = [
        f"approach_speed: {world.approach_speed:.2f} m/s" if world.approach_speed is not None else "approach_speed: -",
        f"DRIVE: {drive_indicator}" if drive_indicator is not None else "DRIVE: -",
        f"DELTA: {delta_indicator}" if delta_indicator is not None else "DELTA: -",
        f"PWM: {pwm_indicator}" if pwm_indicator is not None else "PWM: -",
        f"Obstacle_Distance: {distance_indicator}" if distance_indicator is not None else "Obstacle_Distance: -",
        f"State: {state}" if state is not None else "Obstacle State: -",
        f"Encoder_Count: {Encoder_indicator}" if Encoder_indicator is not None else "ENcoder_Count: -",
        f"Direction: {direction_indicator}" if direction_indicator is not None else "Direction: -",
        f"RPM: {rpm_indicator}" if rpm_indicator is not None else "RPM: -"
    ]

    y0 = 18
    for i, txt in enumerate(lines):
        surf = font.render(txt, True, (235, 235, 235))
        screen.blit(surf, (18, y0 + 22 * i))

#Communication:

def read_from_arduino(com_port, baud_rate):
    global data_dict
    try:
        ser = serial.Serial(com_port, baud_rate, timeout=1)
        time.sleep(2) # Give the connection time to establish
        print(f"Connected to {com_port}")

        while True:
            if ser.in_waiting > 0:
                # Read the line, decode it from bytes to string, and strip whitespace/newline
                data_line = ser.readline().decode('utf-8').strip()
                if data_line:
                    data_array = data_line.split(',')
                    data_dict = {
                        "drive": data_array[0],
                        "delta": data_array[1],
                        "pwm": data_array[2],
                        "Obstacle_Distance": data_array[3],
                        "Mode": data_array[4],
                        "Encoder_Count": data_array[5],
                        "Direction": data_array[6],
                        "RPM": data_array[7]
                    }
                   
                    #print(f"Received: {data_dict}")
                if MIN_RANGE_M <= float(data_dict["Obstacle_Distance"]) <= MAX_RANGE_M:
                    sample = SensorSample(
                        t=time.time(),
                        angle_deg=0.0,
                        range_m=float(data_dict["Obstacle_Distance"])
                    )
                    data_queue.put(sample)
                else:
                    print(f"Distance out of range for sim")

    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
    except KeyboardInterrupt:
        print("Serial communication stopped by user")
    finally:
        if 'ser' in locals() and ser.isOpen():
            ser.close()

# ----------------------------
# Main loop
# ----------------------------
def main():

    if platform.system() == "Darwin":   # Mac
        arduino_port = "/dev/tty.usbmodem1101"
    elif platform.system() == "Windows":
        arduino_port = "COM6"
    else: 
        arduino_port = "/dev/ttyUSB0" # Update this to your Arduino's port 
    baud_rate = 9600
    # start the serial reader on a separate thread so the rest of main can run
    reader_thread = threading.Thread(target=read_from_arduino, args=(arduino_port, baud_rate), daemon=True)
    reader_thread.start()

    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Obstacle Proximity Simulator (Arduino stream ready)")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 18)

    world = WorldModel()
    t0 = time.time()

    running = True
    dash_offset = 0.0
    while running:
        dt = clock.tick(FPS) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Replace this with: samples = serial_reader.read_latest_batch()
        samples = []
        while True:
            try:
                sample = data_queue.get_nowait()
                samples.append(sample)
                speed_value = abs(float(data_dict["RPM"]))   
            except (ValueError, TypeError):
                speed_value = 0.0  
            except Empty:
                break

            dash_speed_px_per_sec = speed_value * 0.5
            dash_offset += dash_speed_px_per_sec * dt
        if samples:
            world.ingest(samples)
        else:
            world.mark_no_detection()
        draw(world, screen, font, dash_offset)
        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()