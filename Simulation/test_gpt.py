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

data_queue = Queue()

data_dict = {"drive": '', "delta": '', "pwm": '', "Obstacle_Distance": ''}
# ----------------------------
# Config
# ----------------------------
WIDTH, HEIGHT = 900, 600
FPS = 60

CAR_POS = (200, HEIGHT // 2)
CAR_SIZE = (80, 40)

# Sensor/model assumptions
MAX_RANGE_M = 4.0
MIN_RANGE_M = 0.05

# Filtering
EMA_ALPHA = 0.25          # higher = less smoothing
OUTLIER_JUMP_M = 0.6      # reject sudden jump bigger than this (tune)

# World scaling
PIXELS_PER_M = 120.0


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
# Fake sensor stream (replace with ultrasonic stream)
# ----------------------------
def fake_sensor_stream(t: float) -> List[SensorSample]:
    """
    Produces either:
    - single forward range (angle=0), like ultrasonic
    - or a small "sweep" of angles like a scanning lidar
    """
    # Obstacle moving toward car
    base_dist = 3.5 - 0.35 * t  # meters
    base_dist = max(0.3, base_dist)

    samples = []
    # pretend we have a sweep -30..+30 degrees
    for ang in [-30, -15, 0, 15, 30]:
        noise = random.gauss(0, 0.07)  # sensor noise
        # make side angles slightly longer (geometry-ish)
        r = base_dist / max(0.7, math.cos(math.radians(ang))) + noise

        # occasional outlier
        if random.random() < 0.02:
            r += random.choice([-1.0, 1.0]) * random.uniform(0.8, 1.5)

        r = float(np.clip(r, MIN_RANGE_M, MAX_RANGE_M))
        samples.append(SensorSample(t=time.time(), angle_deg=float(ang), range_m=r))
    return samples

# ----------------------------
# Ultrasonic sensor stream (yet to be tested)
# ----------------------------


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

        # Outlier gate on the "closest" distance (simple + effective)
        if self.gate.accept(closest):
            self.closest_smooth = self.ema.update(closest)
            d_kf, v_kf = self.kf.update(closest, samples[0].t)
            self.closest_kf = d_kf
            self.approach_speed = -v_kf  # positive means approaching
        # else: ignore update; keep last filtered values

# ----------------------------
# Rendering
# ----------------------------
def meters_to_screen(dx_m: float, dy_m: float) -> Tuple[int, int]:
    # Car frame: x forward (right on screen), y left (up on screen)
    x = int(CAR_POS[0] + dx_m * PIXELS_PER_M)
    y = int(CAR_POS[1] - dy_m * PIXELS_PER_M)
    return x, y

def draw(world: WorldModel, screen: pygame.Surface, font: pygame.font.Font):
    # make sure we refer to the shared dictionary
    global data_dict
    screen.fill((18, 18, 22))

    # Draw car
    car_rect = pygame.Rect(0, 0, CAR_SIZE[0], CAR_SIZE[1])
    car_rect.center = CAR_POS
    pygame.draw.rect(screen, (240, 240, 240), car_rect, border_radius=6)

    # Draw sensor points
    for (x_m, y_m) in list(world.points_car_frame)[-180:]:
        px, py = meters_to_screen(x_m, y_m)
        # fade distant points (manual "brightness" without extra libs)
        if x_m < 0:
            continue
        pygame.draw.circle(screen, (90, 200, 255), (px, py), 3)

    # Draw "closest obstacle" marker straight ahead using filtered distance
    if world.closest_kf is not None:
        px, py = meters_to_screen(world.closest_kf, 0.0)
        pygame.draw.circle(screen, (255, 130, 90), (px, py), 10, width=2)

    drive_indicator = (data_dict["drive"])
    delta_indicator = (data_dict["delta"])
    pwm_indicator = (data_dict["pwm"])
    distance_indicator = (data_dict["Obstacle_Distance"])
    # HUD text
    lines = [
        #f"closest_raw:    {world.closest_raw:.2f} m" if world.closest_raw is not None else "closest_raw:    -",
        #f"closest_ema:    {world.closest_smooth:.2f} m" if world.closest_smooth is not None else "closest_ema:    -",
        #f"closest_kalman: {world.closest_kf:.2f} m" if world.closest_kf is not None else "closest_kalman: -",
        f"approach_speed: {world.approach_speed:.2f} m/s" if world.approach_speed is not None else "approach_speed: -",
        f"DRIVE: {drive_indicator} " if drive_indicator is not None else "Drive Indicator: -",
        f"DELTA: {delta_indicator} " if delta_indicator is not None else "Change in pedal angle: -",
        f"PWM: {pwm_indicator}" if pwm_indicator is not None else "PWM Indicator: -",
        f"Obstacle_Distance: {distance_indicator}" if distance_indicator is not None else "Obstacle_Distance: -",

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
                    data_array = data_line.split(' ')
                    data_dict = {
                        "drive": data_array[0],
                        "delta": data_array[1],
                        "pwm": data_array[2],
                        "Obstacle_Distance": data_array[3]
                    }
                    #print(f"Received: {data_dict}")

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

    arduino_port = 'COM5' # This may change make sure to check
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
    while running:
        dt = clock.tick(FPS) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Replace this with: samples = serial_reader.read_latest_batch()
        t = time.time() - t0
        samples = fake_sensor_stream(t) #replease with ultrasonic data

        world.ingest(samples)
        draw(world, screen, font)
        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()