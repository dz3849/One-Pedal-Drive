//Arduino Pins
int sensorPin = A0;   // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor
int accelLED = 3;
int decelLED = 4;
int neutralLED = 5;

// variables
int currentVal = 0;
int previousVal = 0;
int threshold = 1;

// MOTOR PINS (L298N)
int IN3 = 8;
int IN4 = 9;
int ENB = 10; // PWM pin

void setup() {
  //Serial Begin and declare pins
   Serial.begin(9600);

  pinMode(accelLED, OUTPUT);
  pinMode(decelLED, OUTPUT);
  pinMode(neutralLED, OUTPUT);

  // Motor
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

}

void loop() {
  //Map sensed value to 1-100
  int sensorValue = analogRead(A0);
  int percent = map(sensorValue, 168, 869, 1, 100);
  percent = constrain(percent, 1, 100);

  Serial.println(percent);
  
  int currentVal = percent;

   // Determine movement
  if (currentVal > previousVal + threshold) {
    // accelerating
    digitalWrite(accelLED, HIGH);
    digitalWrite(decelLED, LOW);
    digitalWrite(neutralLED, LOW);

    driveMotor(map(percent, 1, 100, 0, 255));

  } else if (currentVal < previousVal - threshold) {
    // decelerating
    digitalWrite(accelLED, LOW);
    digitalWrite(decelLED, HIGH);
    digitalWrite(neutralLED, LOW);

    driveMotor(map(percent, 1, 100, 0, 255));

  } else {
    // neutral
    digitalWrite(accelLED, LOW);
    digitalWrite(decelLED, LOW);
    digitalWrite(neutralLED, HIGH);

    driveMotor(map(percent, 1, 100, 0, 255));
  }

  // store last reading
  previousVal = currentVal;

  delay(500);
}

// Drive direction will be implemented with shift gears
// MOTOR CONTROL FUNCTION
void driveMotor(int speed) {
  // direction: 1 = forward, -1 = backward, 0 = stop
  // speed: 0–255
  
  speed = constrain(speed, 0, 255);
  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);


  analogWrite(ENB, speed);
  Serial.println("Driving");
}
