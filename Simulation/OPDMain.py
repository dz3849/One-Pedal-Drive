import serial
import time
import numpy as np
import pygame
import platform
from scipy.signal import cont2discrete


# Serial
if platform.system() == "Darwin": 
    PORT = "/dev/tty.usbmodem1101"
elif platform.system() == "Windows":
    PORT = "COM6"
else: 
    PORT = "/dev/ttyUSB0"   # CHANGE THIS

BAUD = 115200

# Timing / MPC
TS = 0.10
HORIZON = 15
CONTROL_PERIOD = TS

# UI Setup
WIDTH = 1280
HEIGHT = 420
FPS = 30

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (220, 70, 70)
GREEN = (70, 220, 70)
YELLOW = (240, 220, 80)
GRAY = (120, 120, 120)
CYAN = (80, 180, 255)
BLUE = (80, 120, 255)

ROAD_Y = 270
EGO_X = 140
EGO_Y = 230
CAR_W = 70
CAR_H = 35

HUD_X_START = 30
HUD_Y_START = 20
HUD_LINE_H = 24
HUD_LINES_PER_COLUMN = 8
HUD_COLUMN_W = 360
HUD_COLUMN_GAP = 70
HUD_VEHICLE_CLEARANCE = 24

PIXELS_PER_METER_RENDER = 12.0
MIN_VISUAL_GAP_M = 1.0

# Ultrasonic Parameters
MAX_DISTANCE_CM = 130
NO_OBSTACLE_THRESHOLD_CM = 125  
VIRTUAL_GAP_AT_THRESHOLD_M = 100  
ULTRA_ALPHA = 0.30
OBSTACLE_TIMEOUT_S = 0.35

SEND_REVERSE_BRAKE = False

# Following-distance logic
DISTANCE_ACTIVE_M = 60.0
MIN_GAP_M = 8.0
TIME_GAP_S = 1.2

STOP_LATCH_GAP_M = 9.0
STOP_RELEASE_GAP_M = 11.0
STOP_SPEED_KMH = 2.0

# Graded braking parameters
BRAKE_MIN_PWM_PERCENT = 10.0
BRAKE_MAX_PWM_PERCENT = 25.0
BRAKE_GAIN = 4.0
BRAKE_RELEASE_MARGIN_M = 1.0 

# PWM operating region
PWM_LEVELS = [20, 30, 40, 50, 60, 70, 80, 90]

def requested_pwm_from_pedal(pedal_percent: float) -> float:
    if pedal_percent < 20:
        return 0.0
    if pedal_percent > 90:
        return 90.0
    return pedal_percent

def real_speed_kmh_from_requested_pwm(req_pwm: float) -> float:
    if req_pwm <= 20:
        return 0.0
    if req_pwm >= 90:
        return 120.0
    return 120.0 * (req_pwm - 20.0) / (90.0 - 20.0)

# RPM to speed interpolation
RPM_POINTS = np.array([951.0, 1332.0, 1698.0, 2074.0, 2474.0, 2891.0, 3376.0, 3750.0])
SPD_POINTS = np.array([0.0, 17.14, 34.29, 51.43, 68.57, 85.71, 102.86, 120.0])

def real_speed_kmh_from_rpm(rpm: float) -> float:
    rpm = abs(rpm)
    return float(np.interp(rpm, RPM_POINTS, SPD_POINTS, left=0.0, right=120.0))

# Local second-order models
MODELS = {
    20: {"bias_rpm": 951.0,  "num": 18782.577978, "a1": 60.064827, "a0": 437.408237},
    30: {"bias_rpm": 1332.0, "num": 13512.482244, "a1": 45.100093, "a0": 376.625465},
    40: {"bias_rpm": 1698.0, "num": 13485.701121, "a1": 43.800590, "a0": 371.060145},
    50: {"bias_rpm": 2074.0, "num": 14658.816429, "a1": 47.070057, "a0": 379.628758},
    60: {"bias_rpm": 2474.0, "num": 13793.204325, "a1": 44.908179, "a0": 341.484294},
    70: {"bias_rpm": 2891.0, "num": 14266.543945, "a1": 47.971692, "a0": 341.501125},
    80: {"bias_rpm": 3376.0, "num": 10654.770474, "a1": 37.369263, "a0": 256.133412},
    90: {"bias_rpm": 3750.0, "num": 9974.404622,  "a1": 42.167667, "a0": 265.583477},
}

DISC_MODELS = {}
for pwm in PWM_LEVELS:
    m = MODELS[pwm]
    num = [m["num"]]
    den = [1.0, m["a1"], m["a0"]]

    numd, dend, _ = cont2discrete((num, den), TS, method="zoh")[:3]
    numd = np.squeeze(numd)
    dend = np.squeeze(dend)

    numd = numd / dend[0]
    dend = dend / dend[0]

    DISC_MODELS[pwm] = {
        "bias_rpm": m["bias_rpm"],
        "numd": numd,
        "dend": dend
    }

def nearest_pwm_level(req_pwm: float) -> int:
    if req_pwm <= 20:
        return 20
    return min(PWM_LEVELS, key=lambda p: abs(p - req_pwm))

# Gap logic
def desired_follow_gap_m(v_ego_kmh: float) -> float:
    v_ego_mps = v_ego_kmh / 3.6
    return MIN_GAP_M + TIME_GAP_S * v_ego_mps

def hard_safe_gap_m(v_ego_kmh: float, v_lead_kmh: float) -> float:
    v_ego_mps = v_ego_kmh / 3.6
    v_lead_mps = v_lead_kmh / 3.6
    closing = max(0.0, v_ego_mps - v_lead_mps)
    return MIN_GAP_M + TIME_GAP_S * v_ego_mps + 0.5 * closing

# Graded braking 
def compute_brake_pwm_percent(gap_m, d_safe):
    gap_error = d_safe - gap_m
    if gap_error <= 0:
        return 0.0

    brake_percent = BRAKE_MIN_PWM_PERCENT + BRAKE_GAIN * gap_error
    brake_percent = max(BRAKE_MIN_PWM_PERCENT, min(BRAKE_MAX_PWM_PERCENT, brake_percent))
    return brake_percent

# Predict local rpm response
def predict_rpm_response(level_pwm, rpm_now, rpm_prev, u_cmd, u_prev, horizon):
    model = DISC_MODELS[level_pwm]
    bias_rpm = model["bias_rpm"]
    numd = model["numd"]
    dend = model["dend"]

    b = np.zeros(3)
    a = np.zeros(3)
    b[:min(3, len(numd))] = numd[:min(3, len(numd))]
    a[:min(3, len(dend))] = dend[:min(3, len(dend))]

    yk_1 = rpm_now - bias_rpm
    yk_2 = rpm_prev - bias_rpm

    uk_1 = u_prev - level_pwm
    uk_2 = u_prev - level_pwm

    preds = []

    for _ in range(horizon):
        uk = u_cmd - level_pwm
        yk = -a[1] * yk_1 - a[2] * yk_2 + b[0] * uk + b[1] * uk_1 + b[2] * uk_2
        rpm_pred = max(0.0, bias_rpm + yk)
        preds.append(rpm_pred)

        yk_2 = yk_1
        yk_1 = yk
        uk_2 = uk_1
        uk_1 = uk

    return np.array(preds)

# Controller weights
W_SPEED_TRACK = 0.4
W_PWM = 0.05
W_SMOOTH = 0.4
W_GAP = 12
W_SAFETY = 3000

def choose_command_no_lead(pedal_percent):
    req_pwm = requested_pwm_from_pedal(pedal_percent)
    if req_pwm <= 0:
        return ("COAST", 0, req_pwm)
    return ("DRIVE", int(req_pwm * 255 / 100.0), req_pwm)

def choose_command_follow(pedal_percent, gap_m, lead_speed_kmh, rpm_now, rpm_prev, u_prev_percent, stop_latched):
    req_pwm = requested_pwm_from_pedal(pedal_percent)
    v_ego_kmh = real_speed_kmh_from_rpm(rpm_now)
    v_req_kmh = real_speed_kmh_from_requested_pwm(req_pwm)

    d_safe_now = hard_safe_gap_m(v_ego_kmh, lead_speed_kmh)

    if req_pwm <= 0:
        if gap_m <= d_safe_now:
            brake_percent = compute_brake_pwm_percent(gap_m, d_safe_now)
            return ("BRAKE", int(brake_percent * 255 / 100.0), req_pwm, stop_latched)
        return ("COAST", 0, req_pwm, stop_latched)

    if (gap_m <= STOP_LATCH_GAP_M) and (v_ego_kmh <= STOP_SPEED_KMH):
        stop_latched = True

    if stop_latched and gap_m >= STOP_RELEASE_GAP_M:
        stop_latched = False

    if stop_latched:
        if v_ego_kmh > 1.0:
            brake_percent = compute_brake_pwm_percent(gap_m, max(d_safe_now, STOP_LATCH_GAP_M))
            return ("BRAKE", int(brake_percent * 255 / 100.0), req_pwm, stop_latched)
        return ("COAST", 0, req_pwm, stop_latched)

    if gap_m > DISTANCE_ACTIVE_M:
        return ("DRIVE", int(req_pwm * 255 / 100.0), req_pwm, stop_latched)

    if gap_m <= d_safe_now:
        brake_percent = compute_brake_pwm_percent(gap_m, d_safe_now)
        return ("BRAKE", int(brake_percent * 255 / 100.0), req_pwm, stop_latched)

    level = nearest_pwm_level(req_pwm)
    candidates = list(range(0, 91, 5))

    best_cost = 1e18
    best_u = 0

    for u_cmd in candidates:
        rpm_pred = predict_rpm_response(
            level_pwm=level,
            rpm_now=rpm_now,
            rpm_prev=rpm_prev,
            u_cmd=u_cmd,
            u_prev=u_prev_percent,
            horizon=HORIZON
        )

        v_pred = np.array([real_speed_kmh_from_rpm(r) for r in rpm_pred])

        d = gap_m
        J = 0.0

        for k in range(HORIZON):
            v_ego_mps = v_pred[k] / 3.6
            v_lead_mps = lead_speed_kmh / 3.6

            d = max(0.0, d + TS * (v_lead_mps - v_ego_mps))
            d_safe = hard_safe_gap_m(v_pred[k], lead_speed_kmh)

            J += W_SPEED_TRACK * (v_pred[k] - v_req_kmh) ** 2
            J += W_PWM * (u_cmd - req_pwm) ** 2
            J += W_SMOOTH * (u_cmd - u_prev_percent) ** 2

            gap_shortfall = max(0.0, d_safe - d)
            J += W_GAP * (gap_shortfall ** 2)

            if d < d_safe:
                J += W_SAFETY * (d_safe - d) ** 2

        if J < best_cost:
            best_cost = J
            best_u = u_cmd

    if best_u <= 0:
        if gap_m < d_safe_now - BRAKE_RELEASE_MARGIN_M:
            brake_percent = compute_brake_pwm_percent(gap_m, d_safe_now)
            return ("BRAKE", int(brake_percent * 255 / 100.0), req_pwm, stop_latched)
        return ("COAST", 0, req_pwm, stop_latched)

    return ("DRIVE", int(best_u * 255 / 100.0), req_pwm, stop_latched)

def ultrasonic_cm_to_virtual_gap_m(dist_cm: float):
    if dist_cm <= 0.0 or dist_cm >= NO_OBSTACLE_THRESHOLD_CM:
        return None

    ratio = dist_cm / NO_OBSTACLE_THRESHOLD_CM
    return ratio * VIRTUAL_GAP_AT_THRESHOLD_M


def parse_latest_telemetry(ser, latest_raw_cm, ultra_gap_m, last_ultra_time):
    pedal_percent = None
    rpm_now = None

    latest_line = None
    while True:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            break
        latest_line = line

    if latest_line is None:
        return pedal_percent, rpm_now, latest_raw_cm, ultra_gap_m, last_ultra_time

    if latest_line == "READY":
        return pedal_percent, rpm_now, latest_raw_cm, ultra_gap_m, last_ultra_time

    if latest_line.startswith("TEL,"):
        parts = latest_line.split(",")
        if len(parts) == 5:
            try:
                _, _time_ms_str, pedal_str, dist_str, rpm_str = parts
                pedal_percent = float(pedal_str)
                rpm_now = float(rpm_str)

                dist_cm = float(dist_str)
                latest_raw_cm = dist_cm

                new_gap_m = ultrasonic_cm_to_virtual_gap_m(dist_cm)

                if new_gap_m is None:
                    ultra_gap_m = None
                    last_ultra_time = None
                else:
                    if ultra_gap_m is None:
                        ultra_gap_m = new_gap_m
                    else:
                        ultra_gap_m = ULTRA_ALPHA * new_gap_m + (1.0 - ULTRA_ALPHA) * ultra_gap_m
                    last_ultra_time = time.time()
            except ValueError:
                pass

    if last_ultra_time is not None and (time.time() - last_ultra_time) > OBSTACLE_TIMEOUT_S:
        ultra_gap_m = None

    return pedal_percent, rpm_now, latest_raw_cm, ultra_gap_m, last_ultra_time

# Main Setup
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("One-Pedal Drive Prototype - Ultrasonic Obstacle Following")
clock = pygame.time.Clock()
font = pygame.font.SysFont("consolas", 22)

ser = serial.Serial(PORT, BAUD, timeout=0.001)
time.sleep(2)
ser.reset_input_buffer()

print("Connected. Waiting for Arduino READY...")

pedal_percent = 0.0
rpm_now = 0.0
prev_rpm = 0.0
prev_cmd_pwm_percent = 0.0
stop_latched = False

ego_distance_m = 0.0
lead_active = False
gap_m = None
ultra_gap_m = None
latest_raw_cm = None
last_ultra_time = None

last_control_time = 0.0
mode = "COAST"
sent_mode = "COAST"
sent_pwm255 = 0
pwm255 = 0
req_pwm = 0.0
lead_speed_kmh = 0.0
displayed_gap_m = None
desired_gap_m = None
safe_gap_m = None

running = True

while running:
    dt = clock.tick(FPS) / 1000.0
    if dt <= 0:
        dt = 1.0 / FPS

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    new_pedal, new_rpm, latest_raw_cm, ultra_gap_m, last_ultra_time = parse_latest_telemetry(
        ser, latest_raw_cm, ultra_gap_m, last_ultra_time
    )
    if new_pedal is not None:
        pedal_percent = new_pedal
    if new_rpm is not None:
        rpm_now = new_rpm

    ego_speed_kmh = real_speed_kmh_from_rpm(rpm_now)
    ego_speed_mps = ego_speed_kmh / 3.6
    ego_distance_m += ego_speed_mps * dt

    lead_active = ultra_gap_m is not None
    gap_m = ultra_gap_m if lead_active else None
    if not lead_active:
        stop_latched = False

    now = time.time()
    if now - last_control_time >= CONTROL_PERIOD:
        last_control_time = now

        if not lead_active:
            mode, pwm255, req_pwm = choose_command_no_lead(pedal_percent)
            lead_speed_kmh = 0.0
            displayed_gap_m = None
            desired_gap_m = None
            safe_gap_m = None
        else:
            lead_speed_kmh = 0.0
            displayed_gap_m = gap_m
            desired_gap_m = desired_follow_gap_m(ego_speed_kmh)
            safe_gap_m = hard_safe_gap_m(ego_speed_kmh, lead_speed_kmh)

            mode, pwm255, req_pwm, stop_latched = choose_command_follow(
                pedal_percent=pedal_percent,
                gap_m=gap_m,
                lead_speed_kmh=lead_speed_kmh,
                rpm_now=abs(rpm_now),
                rpm_prev=abs(prev_rpm),
                u_prev_percent=prev_cmd_pwm_percent,
                stop_latched=stop_latched
            )

        if mode == "BRAKE" and not SEND_REVERSE_BRAKE:
            sent_mode = "COAST"
            sent_pwm255 = 0
        else:
            sent_mode = mode
            sent_pwm255 = pwm255

        cmd_str = f"CMD,{sent_mode},{sent_pwm255}\n"
        ser.write(cmd_str.encode("utf-8"))

        prev_rpm = rpm_now
        prev_cmd_pwm_percent = sent_pwm255 * 100.0 / 255.0

    # Draw UI
    screen.fill(BLACK)
    pygame.draw.line(screen, GRAY, (0, ROAD_Y), (WIDTH, ROAD_Y), 2)

    pygame.draw.rect(screen, WHITE, (EGO_X, EGO_Y, CAR_W, CAR_H))

    if lead_active and gap_m is not None:
        draw_gap_m = max(gap_m, MIN_VISUAL_GAP_M)
        lead_x = EGO_X + CAR_W + int(draw_gap_m * PIXELS_PER_METER_RENDER)
        lead_y = EGO_Y
        if lead_x < WIDTH + 100:
            pygame.draw.rect(screen, RED, (lead_x, lead_y, CAR_W, CAR_H))

        if desired_gap_m is not None:
            des_x = EGO_X + CAR_W + int(desired_gap_m * PIXELS_PER_METER_RENDER)
            pygame.draw.line(screen, YELLOW, (des_x, ROAD_Y - 90), (des_x, ROAD_Y + 10), 2)

        if safe_gap_m is not None:
            safe_x = EGO_X + CAR_W + int(min(safe_gap_m, 140.0) * PIXELS_PER_METER_RENDER)
            pygame.draw.line(screen, GREEN, (safe_x, ROAD_Y - 70), (safe_x, ROAD_Y + 10), 2)

        if desired_gap_m is not None:
            pygame.draw.rect(
                screen,
                BLUE,
                (EGO_X + CAR_W, ROAD_Y - 8, int(min(desired_gap_m, 140.0) * PIXELS_PER_METER_RENDER), 8)
            )

    pygame.draw.rect(screen, GRAY, (40, 360, 250, 18), 2)
    pygame.draw.rect(screen, CYAN, (42, 362, int(246 * pedal_percent / 100.0), 14))

    text_lines = [
        f"Pedal:            {pedal_percent:6.1f} %",
        f"Req PWM:          {req_pwm:6.1f} %",
        f"Applied PWM:      {pwm255 * 100.0 / 255.0:6.1f} %",
        f"Ego RPM:          {rpm_now:6.1f}",
        f"Ego speed:        {ego_speed_kmh:6.1f} km/h",
        f"Ego distance:     {ego_distance_m:6.1f} m",
        f"Raw ultrasonic:   {latest_raw_cm:6.1f} cm" if latest_raw_cm is not None else "Raw ultrasonic:   ---",
        f"Obstacle active:  {lead_active}",
    ]

    if lead_active and displayed_gap_m is not None:
        text_lines += [
            f"Lead speed:       {lead_speed_kmh:6.1f} km/h",
            f"Gap:              {displayed_gap_m:6.2f} m",
            f"Desired gap:      {desired_gap_m:6.1f} m" if desired_gap_m is not None else "Desired gap:      ---",
            f"Safe gap:         {safe_gap_m:6.1f} m" if safe_gap_m is not None else "Safe gap:         ---",
        ]
    else:
        text_lines += [
            "Lead speed:       ---",
            "Gap:              ---",
            "Desired gap:      ---",
            "Safe gap:         ---",
        ]

    text_lines += [
        f"Mode:             {mode}",
        f"Stop latched:     {stop_latched}",
    ]


    max_text_bottom = EGO_Y - HUD_VEHICLE_CLEARANCE

    for i, txt in enumerate(text_lines):
        col = i // HUD_LINES_PER_COLUMN
        row = i % HUD_LINES_PER_COLUMN
        x = HUD_X_START + col * (HUD_COLUMN_W + HUD_COLUMN_GAP)
        y = HUD_Y_START + row * HUD_LINE_H

        if y > max_text_bottom:
            extra_rows = max(1, int((max_text_bottom - HUD_Y_START) // HUD_LINE_H) + 1)
            col = i // extra_rows
            row = i % extra_rows
            x = HUD_X_START + col * (HUD_COLUMN_W + HUD_COLUMN_GAP)
            y = HUD_Y_START + row * HUD_LINE_H

        surf = font.render(txt, True, WHITE)
        screen.blit(surf, (x, y))

    pygame.display.flip()

ser.close()
pygame.quit()
