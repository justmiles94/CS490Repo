import init_library as robot  #Library tha handles all the serial commands to arduino AtMega
import time

while True:
	control = input("Enter mode: Motor Command = m, ultrasonic = u, infrared = i, encoder = e >")

	if control == 'm':
		motorNum = int(input("Enter motor number (0-2): "))
		motorPWM = int(input("Enter PWM value (0-255): "))
		motorDir = (input("Enter motor dir (c or ccw): "))
		if(motorDir == 'ccw'):
			motorPWM= motorPWM*-1
		if(motorNum == 0):
			robot.motors(motorPWM,0,0)
		if(motorNum == 1):
			robot.motors(0,motorPWM,0)
		if(motorNum == 2):
			robot.motors(0,0,motorPWM)
		time.sleep(2)
		rpm = motorNum
		nu = robot.rpm(rpm)
		print(nu)

	elif control == 'u':
		ultraNum = int(input("Enter ultrasonic sensor number (0-5) >"))
		while True:
			print("Distance from sensor (cm): "+robot.ultrasound(ultraNum))

	elif control == 'i':
		infraredNum = int(input("Enter infrared sensor number (0-3) >"))
		while True:
			print("Distance from sensor (cm): "+robot.infrared(infraredNum))

	elif control == 'e':
		encoderNum = int(input("Enter encoder motor number (0-2) >"))
		while True:
			print("Encoder ticks: "+robot.encoder(encoderNum))
