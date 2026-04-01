#include <NewPing.h>
#include <Servo.h>

//Arduino Pins
int sensorPin = A0;
int accelLED = 3;
int decelLED = 4;
int neutralLED = 5;

// variables
int currentVal = 0;
int previousVal = 0;

// threshold for accel/decel indicator
int threshold = 1;

// MOTOR PINS (L298N)
int IN3 = 8;
int IN4 = 9;
int ENB = 10; // PWM pin

// --- OPD BRAKING LOGIC ---
const int BRAKE_DELTA_THRESHOLD = -2;   // if current - previous <= -3 => brake
const int BRAKE_PULSE_MS = 120;         // how long to apply reverse torque pulse
const int MIN_BRAKE_PWM = 80;           // minimum PWM during braking so it actually bites (tune)

// Optional: limit brake strength so you don't slam
const int MAX_BRAKE_PWM = 200;          // cap braking PWM (tune)

//Ultrasonic Parameters, cm
int maxDistance = 100;
int TriggerPin = 12;
int EchoPin = 13;
NewPing sonar(TriggerPin, EchoPin, 100);

void setup() {
  Serial.begin(9600);

  pinMode(accelLED, OUTPUT);
  pinMode(decelLED, OUTPUT);
  pinMode(neutralLED, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // safe state
  analogWrite(ENB, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void loop() {
  // Read and map pedal
  int sensorValue = analogRead(sensorPin);
  int percent = map(sensorValue, 168, 869, 1, 100);
  percent = constrain(percent, 1, 100);

  currentVal = percent;

  // delta between consecutive readings
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

  // --- BRAKE CONDITION: pedal dropped fast enough ---
  if (delta <= BRAKE_DELTA_THRESHOLD) {
    // Apply reverse torque pulse (active braking)
    int brakePwm = pwm;

    // make sure braking has enough PWM to be effective
    if (brakePwm < MIN_BRAKE_PWM) brakePwm = MIN_BRAKE_PWM;
    if (brakePwm > MAX_BRAKE_PWM) brakePwm = MAX_BRAKE_PWM;

    reverseTorque(brakePwm);
    delay(BRAKE_PULSE_MS);

    // After brake pulse, go back to forward drive based on current pedal
    forwardDrive(pwm);

    Serial.print(0); //state Break
    Serial.print(" ");
    Serial.print(delta); //motor delta
    Serial.print(" ");
    Serial.print(pwm); // pedal angle
    Serial.print(" "); // Obstacle Distance
    Serial.println(distance);

  }
  else {
    // Normal forward drive
    forwardDrive(pwm);
    Serial.print(1); //state drive
    Serial.print(" ");
    Serial.print(delta); //motor delta
    Serial.print(" ");
    Serial.print(pwm); // pedal angle
    Serial.print(" "); // Obstacle Distance
    Serial.println(distance);
  }

  previousVal = currentVal;

  // faster loop is better for control feel
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