// =====================================
// One-Pedal Drive + Reverse Braking + Encoder + Ultrasonic
// Arduino Uno + L298N
// =====================================

#include <NewPing.h>

// ---------- Pedal ----------
const int sensorPin = A0;
const int accelLED = 6;
const int decelLED = 4;
const int neutralLED = 5;

// ---------- Pedal variables ----------
int currentVal = 0;
int previousVal = 0;
const int threshold = 1;

// ---------- Motor pins (L298N) ----------
const int IN3 = 8;
const int IN4 = 9;
const int ENB = 10;   // PWM pin

// ---------- Reverse braking logic ----------
const int BRAKE_DELTA_THRESHOLD = -2;   // if current - previous <= -2 => brake
const int BRAKE_PULSE_MS = 120;         // reverse torque pulse length
const int MIN_BRAKE_PWM = 80;           // minimum braking PWM
const int MAX_BRAKE_PWM = 200;          // max braking PWM

// ---------- Ultrasonic ----------
const int maxDistance = 100;   // cm
const int TriggerPin = 12;
const int EchoPin = 13;
NewPing sonar(TriggerPin, EchoPin, maxDistance);

// ---------- Encoder pins ----------
const int ENC_A = 2;   // Yellow
const int ENC_B = 3;   // White

volatile long encoderCount = 0;

// ---------- RPM calculation ----------
unsigned long lastRPMTime = 0;
long lastCountForRPM = 0;
const float CPR = 32.0;

// ---------- Encoder ISR ----------
void encoderISR_A() {
  int a = digitalRead(ENC_A);
  int b = digitalRead(ENC_B);

  if (a == b) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

// ---------- Motor helpers ----------
void forwardDrive(int speed) {
  speed = constrain(speed, 0, 255);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

void reverseTorque(int speed) {
  speed = constrain(speed, 0, 255);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed);
}

void coastStop() {
  analogWrite(ENB, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void setup() {
  Serial.begin(9600);

  // LED pins
  pinMode(accelLED, OUTPUT);
  pinMode(decelLED, OUTPUT);
  pinMode(neutralLED, OUTPUT);

  // Motor pins
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Encoder pins
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR_A, CHANGE);

  // Safe motor state
  coastStop();

  lastRPMTime = millis();

}

void loop() {
  // ---------- Read pedal ----------
  int sensorValue = analogRead(sensorPin);
  int percent = map(sensorValue, 168, 869, 1, 100);
  percent = constrain(percent, 1, 100);

  currentVal = percent;
  int delta = currentVal - previousVal;

  // ---------- Convert pedal to PWM ----------
  int pwm = map(percent, 1, 100, 0, 255);
  pwm = constrain(pwm, 0, 255);

  // ---------- Ultrasonic ----------
  int distance = sonar.ping_cm();
  if (distance == 0) {
    distance = maxDistance;   // treat no echo as "far"
  }

  // ---------- LED indication ----------
  if (delta > threshold) {
    digitalWrite(accelLED, HIGH);
    digitalWrite(decelLED, LOW);
    digitalWrite(neutralLED, LOW);
  } else if (delta < -threshold) {
    digitalWrite(accelLED, LOW);
    digitalWrite(decelLED, HIGH);
    digitalWrite(neutralLED, LOW);
  } else {
    digitalWrite(accelLED, LOW);
    digitalWrite(decelLED, LOW);
    digitalWrite(neutralLED, HIGH);
  }

  // ---------- Drive / brake logic ----------
  const char* mode = "NEUTRAL";

  if (delta <= BRAKE_DELTA_THRESHOLD) {
    int brakePwm = pwm;

    if (brakePwm < MIN_BRAKE_PWM) brakePwm = MIN_BRAKE_PWM;
    if (brakePwm > MAX_BRAKE_PWM) brakePwm = MAX_BRAKE_PWM;

    reverseTorque(brakePwm);
    mode = "BRAKE";
    delay(BRAKE_PULSE_MS);

    // after brake pulse, resume forward drive based on current pedal
    forwardDrive(pwm);
  } else {
    forwardDrive(pwm);
    mode = "DRIVE";
  }

  // ---------- Compute encoder-based direction and RPM ----------
  unsigned long now = millis();
  float rpm = 0.0;
  const char* directionText = "stopped";

  if (now - lastRPMTime >= 200) {
    noInterrupts();
    long countNow = encoderCount;
    interrupts();

    long deltaCount = countNow - lastCountForRPM;
    float dt = (now - lastRPMTime) / 1000.0;

    if (dt > 0) {
      rpm = (deltaCount / CPR) / dt * 60.0;
    }

    if (deltaCount > 0) directionText = "forward";
    else if (deltaCount < 0) directionText = "reverse";

    // ---------- Serial output ----------
    Serial.print(percent);
    Serial.print(",");
    Serial.print(delta);
    Serial.print(",");
    Serial.print(pwm);
    Serial.print(",");
    Serial.print(distance);
    Serial.print(",");
    Serial.print(mode);
    Serial.print(",");
    Serial.print(countNow);
    Serial.print(",");
    Serial.print(directionText);
    Serial.print(",");
    Serial.println(rpm, 2);

    lastCountForRPM = countNow;
    lastRPMTime = now;
  }

  previousVal = currentVal;

  delay(30);
}