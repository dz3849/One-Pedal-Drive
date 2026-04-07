#include <NewPing.h>
#include <Servo.h>

// Arduino Pins
int sensorPin = A0;
int accelLED = 6;
int decelLED = 4;
int neutralLED = 5;

// Variables
int currentVal = 0;
int previousVal = 0;

// Threshold for accel/decel indicator
int threshold = 1;

// MOTOR PINS (L298N)
int IN3 = 8;
int IN4 = 9;
int ENB = 10; // PWM pin

// --- OPD BRAKING LOGIC ---
const int BRAKE_DELTA_THRESHOLD = -2;   // if current - previous <= -2 => brake
const int BRAKE_PULSE_MS = 120;         // how long to apply reverse torque pulse
const int MIN_BRAKE_PWM = 80;           // minimum PWM during braking so it actually bites
const int MAX_BRAKE_PWM = 200;          // cap braking PWM

// Ultrasonic Parameters, cm
int maxDistance = 100;
int TriggerPin = 12;
int EchoPin = 13;
NewPing sonar(TriggerPin, EchoPin, maxDistance);

// ---------- Encoder pins ----------
const int ENC_A = 2;   // Yellow
const int ENC_B = 3;   // White

volatile long encoderCount = 0;

// RPM calculation
unsigned long lastRPMTime = 0;
long lastCountForRPM = 0;
const float CPR = 32.0;

void encoderISR_A() {
  int a = digitalRead(ENC_A);
  int b = digitalRead(ENC_B);

  if (a == b) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(accelLED, OUTPUT);
  pinMode(decelLED, OUTPUT);
  pinMode(neutralLED, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR_A, CHANGE);

  // Safe state
  analogWrite(ENB, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  lastRPMTime = millis();

  Serial.println("state delta pwm distance encoder_count direction rpm");
}

void loop() {
  // Read and map pedal
  int sensorValue = analogRead(sensorPin);
  int percent = map(sensorValue, 168, 869, 1, 100);
  percent = constrain(percent, 1, 100);

  currentVal = percent;

  // Delta between consecutive readings
  int delta = currentVal - previousVal;

  // Convert pedal to PWM
  int pwm = map(percent, 1, 100, 0, 255);
  pwm = constrain(pwm, 0, 255);

  // Ultrasonic reading
  int distance = sonar.ping_cm();

  // LEDs (based on delta)
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

  int state;

  // --- BRAKE CONDITION: pedal dropped fast enough ---
  if (delta <= BRAKE_DELTA_THRESHOLD) {
    int brakePwm = pwm;

    if (brakePwm < MIN_BRAKE_PWM) brakePwm = MIN_BRAKE_PWM;
    if (brakePwm > MAX_BRAKE_PWM) brakePwm = MAX_BRAKE_PWM;

    reverseTorque(brakePwm);
    delay(BRAKE_PULSE_MS);

    // After brake pulse, go back to forward drive based on current pedal
    forwardDrive(pwm);
    state = "DRIVE"; // braking
  } else {
    forwardDrive(pwm);
    state = "BREAK"; // driving
  }

  // ---------- Compute encoder-based direction and RPM ----------
  unsigned long now = millis();
  long countNow;
  long deltaCount;
  float rpm = 0.0;
  const char* directionText = "stopped";

  noInterrupts();
  countNow = encoderCount;
  interrupts();

  deltaCount = countNow - lastCountForRPM;

  if (now - lastRPMTime >= 200) {
    float dt = (now - lastRPMTime) / 1000.0;
    if (dt > 0) {
      rpm = (deltaCount / CPR) / dt * 60.0;
    }

    if (deltaCount > 0) directionText = "forward";
    else if (deltaCount < 0) directionText = "reverse";

    lastCountForRPM = countNow;
    lastRPMTime = now;
  }

  // Serial output: state delta pwm distance encoder_count direction rpm
  Serial.print(state);
  Serial.print(" ");
  Serial.print(delta);
  Serial.print(" ");
  Serial.print(pwm);
  Serial.print(" ");
  Serial.print(distance);
  Serial.print(" ");
  Serial.print(countNow);
  Serial.print(" ");
  Serial.print(directionText);
  Serial.print(" ");
  Serial.println(rpm, 2);

  previousVal = currentVal;

  // Faster loop is better for control feel
  delay(30);
}

// --- Motor helpers ---
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
