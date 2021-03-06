import serial
import math
import time
import numpy as np

#sets up serial connection to arduino atMega
ser = serial.Serial('/dev/ttyACM0',115200, timeout=.2);

#wait for serial connection to be established
time.sleep(1)

#clear buffers just incase garbage inside
ser.reset_input_buffer()
ser.reset_output_buffer()

pid = 0;


# This functions sends  pwm signals to the motor and reverses the direction if given negative
# Example motor(255,0,0) would turn motor 0 on all the away and 1,2 off
# motor(125,-200,-100) motor 0 would have a half duty cycle, motor 1 would move backwards at a pwm of 200 etc...
def motors(m1,m2,m3):
	motorValues = [m1,m2,m3]
	for x in range(3):
		ser.write(("m %d %d %d\r" % (x, abs(motorValues[x]), int(motorValues[x]>=0))).encode())



#read encoder value from motor number given
def encoder(encoderNum):
	ser.reset_input_buffer()
	ser.write(("e %d \r" % (encoderNum)).encode())
	encoderValue = (ser.readline().decode())
	return encoderValue.rstrip()

def ultrasound(ultraSoundNum):
	#ser.reset_input_buffer()
	ser.write(("u %d \r" % (ultraSoundNum)).encode())
	ultraSoundValue = (ser.readline().decode())
	return ultraSoundValue.rstrip()

def infrared(infraredNum):
	ser.reset_input_buffer()
	ser.write(("i %d \r" % (infraredNum)).encode())
	infraredValue = (ser.readline().decode())
	return infraredValue.rstrip()

def rpm(rpmNum):
	ser.reset_input_buffer()
	ser.write(("r %f \r" % (rpmNum)).encode())
	rpmValue = (ser.readline().decode())
	return rpmValue.rstrip()

def enablePID(pidValue):
	pid = pidValue
	ser.write(("p %d \r" % (pid)).encode())






# assume PID is always off

enablePID(0)

