/* 
 * ------------------------------------------------------------------------------
 * Reads from an analog channel with time stamping
 * This code is used with Analog_PyArduino_DAQ_Driver.py, which will
 * read serial data and create a data file of (time,voltage)
 * 
 * Version 1.0 (Spring 2022)
 * Comments: To use serial monitor, use the send box and input "f,50"
 * This will output data in serial monitor (free run)
 * To log data, use the Python code above.
 * ------------------------------------------------------------------------------
 */

const int PIN = A0;          // Pin use to collect analog data from potenitometer
unsigned long timer = 0;    // used to check current time [microseconds]
long loopTime = 0;       // default time between updates, but will be set in python Code [microseconds]
bool initLooptime = false;  // boolean (T/F) to check if loop time has already been set
bool stopProgram = false;

int analogVal = 0;          // variable to store potentiometer data [ints]
float voltage = 0;          // variable to store potentiometer voltage [V] 

float int2volt = 5.0/1023.0; // Conversion constant from ints to volts [V/int]

void setup() {
  Serial.begin(19200);         // Being serial comms and set Baud rate
  timer = micros();             // start timer
}
 
void loop() {

  if (Serial.available() > 0) {       // if data is available
    String str = Serial.readStringUntil('\n');
    readFromPC(str); 
  }
  
  if (initLooptime && !stopProgram) // once loop time has been initialized
  {
    // initLooptime 
  
    timeSync(loopTime);   // sync up time to mach data rate
  
    analogVal = analogRead(PIN); // get analog data from pin
  
    voltage = (float)analogVal*int2volt; // convert to volts
  
    unsigned long currT = micros();  // get current time
  
   
    // Send data over serial line to computer
    sendToPC(&currT);
    sendToPC(&voltage);

  
  }
  else if (initLooptime && stopProgram)
  {
    // also free run
    analogVal = analogRead(PIN); // get analog data from pin
    voltage = (float)analogVal*int2volt; // convert to volts
    Serial.print(voltage);
    Serial.print('\n');
  }


}

/*
 * Timesync calculates the time the arduino needs to wait so it 
 * outputs data at the specified rate
 * Input: deltaT - the data transfer period in microseconds
 */
void timeSync(unsigned long deltaT)
{
  unsigned long currTime = micros();  // get current time
  long timeToDelay = deltaT - (currTime - timer); // calculate how much time to delay for [us]
  
  if (timeToDelay > 5000) // if time to delay is large 
  {
    // Split up delay commands into delay(milliseconds)
    delay(timeToDelay / 1000);

    // and delayMicroseconds(microseconds)
    delayMicroseconds(timeToDelay % 1000);
  }
  else if (timeToDelay > 0) // If time to delay is positive and small
  {
    // Use delayMicroseconds command
    delayMicroseconds(timeToDelay);
  }
  else
  {
      // timeToDelay is negative or zero so don't delay at all
  }
  timer = currTime + timeToDelay;
}


void readFromPC(const String input)
{
  int commaIndex = input.indexOf(',');
  char command = input.charAt(commaIndex - 1);
  String data = input.substring(commaIndex + 1);    
  int rate = 0;
  switch(command)
  {
    case 'r':
      // rate command
      rate = data.toInt();
      loopTime = 1000000/rate;         // set loop time in microseconds to 1/frequency sent
      initLooptime = true;             // no longer check for data
      timer = micros();
      break;
    case 's':
      // stop command
      stopProgram = true;
      break;
    case 'f':
      // free run
      initLooptime = true;
      stopProgram = true;
      break;
    default:
    // Otherwise, do nothing
      break;
  
  }

}

// ------------------------------------------------------------------------------------------------------------
// Send Data to PC: Methods to send different types of data to PC
// ------------------------------------------------------------------------------------------------------------

void sendToPC(int* data)
{
  byte* byteData = (byte*)(data);
  Serial.write(byteData, 2);
}

void sendToPC(float* data)
{
  byte* byteData = (byte*)(data);
  Serial.write(byteData, 4);
}
 
void sendToPC(double* data)
{
  byte* byteData = (byte*)(data);
  Serial.write(byteData, 4);
}

void sendToPC(unsigned long* data)
{
  byte* byteData = (byte*)(data);
  Serial.write(byteData, 4);
}
