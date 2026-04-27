#include <NewPing.h>

// =====================================
// Arduino sensor/actuator node for Python MPC
// L298N + HC-SR04 + pedal + encoder
// with pedal filtering + state LEDs
// =====================================

// ---------- Pedal ----------
const int SENSOR_PIN = A0;

// Pedal filter parameters
float pedalFiltered = 0.0;
const float PEDAL_ALPHA = 0.15;   // 0.10 = stronger smoothing, 0.25 = lighter smoothing
int lastPedalPercent = 0;
const int PEDAL_DEADBAND = 1;     // suppress tiny 1% jitter

// ---------- Motor pins (L298N) ----------
const int IN3 = 8;
const int IN4 = 9;
const int ENB = 10;

// ---------- State indicator LEDs ----------
const int accelLED = 6;
const int decelLED = 4;
const int neutralLED = 5;
float prevAbsRPM = 0.0;
const float RPM_CHANGE_THRESHOLD = 80.0;  
const float STOP_RPM_THRESHOLD = 6.0;

// ---------- Ultrasonic (HC-SR04) ----------
const int MAX_DISTANCE_CM = 150;
const int TRIGGER_PIN = 12;
const int ECHO_PIN = 13;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE_CM);

// ---------- Encoder ----------
const int ENC_A = 2;
const int ENC_B = 3;
volatile long encoderCount = 0;
const float CPR = 32.0;

// ---------- Timing ----------
const unsigned long TELEMETRY_PERIOD_MS = 100;
unsigned long lastTelemetryMs = 0;

unsigned long lastRPMMs = 0;
long lastCountForRPM = 0;
float rpm = 0.0;

// ---------- Command from Python ----------
String inputLine = "";
String currentMode = "COAST";
int currentPWM255 = 0;

// ---------- Encoder ISR ----------
void encoderISR_A() {
  int a = digitalRead(ENC_A);
  int b = digitalRead(ENC_B);

  if (a == b) encoderCount++;
  else encoderCount--;
}

// ---------- LED helper ----------
void updateStateLEDsFromEncoder(float currentRPM, float previousAbsRPM) {
  float absRPM = abs(currentRPM);
  float rpmChange = absRPM - previousAbsRPM;

  // If motor is basically stopped, show neutral
  if (absRPM < STOP_RPM_THRESHOLD) {
    digitalWrite(accelLED, LOW);
    digitalWrite(decelLED, LOW);
    digitalWrite(neutralLED, HIGH);
  }
  // Speed is increasing
  else if (rpmChange > RPM_CHANGE_THRESHOLD) {
    digitalWrite(accelLED, HIGH);
    digitalWrite(decelLED, LOW);
    digitalWrite(neutralLED, LOW);
  }
  // Speed is decreasing
  else if (rpmChange < -RPM_CHANGE_THRESHOLD) {
    digitalWrite(accelLED, LOW);
    digitalWrite(decelLED, HIGH);
    digitalWrite(neutralLED, LOW);
  }
  // Speed is roughly constant
  else {
    digitalWrite(accelLED, LOW);
    digitalWrite(decelLED, LOW);
    digitalWrite(neutralLED, HIGH);
  }
}

// ---------- Motor helpers ----------
void forwardDrive(int pwm255) {
  pwm255 = constrain(pwm255, 0, 255);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, pwm255);
}

void reverseBrake(int pwm255) {
  pwm255 = constrain(pwm255, 0, 255);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, pwm255);
}

void coastStop() {
  analogWrite(ENB, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void applyCommand() {
  if (currentMode == "DRIVE") {
    forwardDrive(currentPWM255);
  } 
  else if (currentMode == "BRAKE") {
    reverseBrake(currentPWM255);
  } 
  else {
    coastStop();
  }
}

void parseCommand(String line) {
  line.trim();
  if (!line.startsWith("CMD,")) return;

  int p1 = line.indexOf(',');
  int p2 = line.indexOf(',', p1 + 1);
  if (p2 < 0) return;

  String modeStr = line.substring(p1 + 1, p2);
  String pwmStr  = line.substring(p2 + 1);

  currentMode = modeStr;
  currentPWM255 = constrain(pwmStr.toInt(), 0, 255);

  applyCommand();
}

void setup() {
  Serial.begin(115200);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(accelLED, OUTPUT);
  pinMode(decelLED, OUTPUT);
  pinMode(neutralLED, OUTPUT);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR_A, CHANGE);

  // Initialize pedal filter with the first raw reading
  pedalFiltered = analogRead(SENSOR_PIN);

  coastStop();

  lastTelemetryMs = millis();
  lastRPMMs = millis();

  Serial.println("READY");
}

void loop() {
  // ---------- Receive command from Python ----------
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputLine.length() > 0) {
        parseCommand(inputLine);
        inputLine = "";
      }
    } else {
      inputLine += c;
    }
  }

  unsigned long now = millis();

  // ---------- Update RPM ----------
if (now - lastRPMMs >= TELEMETRY_PERIOD_MS) {
  noInterrupts();
  long countNow = encoderCount;
  interrupts();

  long deltaCount = countNow - lastCountForRPM;
  float dt = (now - lastRPMMs) / 1000.0;

  if (dt > 0) {
    rpm = (deltaCount / CPR) / dt * 60.0;
  }

  // Update LEDs based on encoder speed change
  updateStateLEDsFromEncoder(rpm, prevAbsRPM);
  prevAbsRPM = abs(rpm);

  lastCountForRPM = countNow;
  lastRPMMs = now;
}

  // ---------- Send telemetry ----------
  if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    // Raw pedal read
    int rawPedal = analogRead(SENSOR_PIN);

    // Exponential low-pass filter
    pedalFiltered = PEDAL_ALPHA * rawPedal + (1.0 - PEDAL_ALPHA) * pedalFiltered;

    // Map filtered value to percent
    int pedalPercent = map((int)pedalFiltered, 168, 869, 0, 100);
    pedalPercent = constrain(pedalPercent, 0, 100);

    // Small deadband to suppress tiny jitter
    if (abs(pedalPercent - lastPedalPercent) <= PEDAL_DEADBAND) {
      pedalPercent = lastPedalPercent;
    }
    lastPedalPercent = pedalPercent;

    int distanceCm = sonar.ping_cm();
    if (distanceCm == 0) distanceCm = MAX_DISTANCE_CM;

    Serial.print("TEL,");
    Serial.print(now);
    Serial.print(",");
    Serial.print(pedalPercent);
    Serial.print(",");
    Serial.print(distanceCm);
    Serial.print(",");
    Serial.println(rpm, 2);

    lastTelemetryMs = now;
  }
}