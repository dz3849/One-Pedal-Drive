from ArduinoDAQ import SerialConnect
import numpy as np
import time

# Read a single analog voltage channel using ArduinoDAQ
# Spring 2022
# Can be used with: Arduino_AnalogDAQ_to_Python.ino
# Should also have ArduinoDAQ.py in working directory

# Make sure portName matches your specific port and set in Arduino IDE
portName = 'COM5' # typical Mac portName
# e.g., portName = 'COM4' (typical Win portName)
baudRate   = 19200                     # Baud Rate
dataRate   = 50                        # Acquisition data rate (Hz), do not exceed 500
recordTime = 5                        # Number of secornds to record data
numDataPoints = recordTime * dataRate  # Total number of data points to be saved

#%% Data lists and Arduino commands
#----------------------------------------------------------------------
# Data to read from Arduino file
#----------------------------------------------------------------------
dataNames = ['Time', 'voltage']
dataTypes = [  '=L',      '=f']

#---------------------------------------------------------------------- 
# Command strings that can be sent to Arduino
#----------------------------------------------------------------------
rate_c     = 'r' # Data rate command
stop_c     = 's' # Data rate command


#%% Command data structures 
# Set recordTime variable to 10 seconds
#----------------------------------------------------------------------
commandTimes = [recordTime] # Time to send command
commandData  = [0] # Value to send over
commandTypes = ['s'] # Type of command

# The following creates a unique filename based on universal time
# (so you don't overwrite a previous file)
now = int( time.time() )
snow = str(now)
fileName     = 'test'+snow+'.csv'

#%% Communication with Arduino
#----------------------------------------------------------------------
# Do not edit code below
#----------------------------------------------------------------------
# initializes all required variables
s = SerialConnect(portName, fileName, baudRate, dataRate, \
                  dataNames, dataTypes, commandTimes, commandData, commandTypes)

# Connect to Arduino and send over rate
s.connectToArduino()

# Start Recording Data
print("Recording...")

# Collect data
while len(s.dataStore[0]) < numDataPoints:
    s.getSerialData()
    
    s.sendCommand() # send command to arduino if ready
    
    # Print number of seconds that have passed
    if len(s.dataStore[0]) % dataRate == 0:
        print(len(s.dataStore[0]) /dataRate)   

# Close Arduino connection and save data
s.close()
