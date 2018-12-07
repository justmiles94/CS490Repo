import serial
import math
import time
import numpy as np

#sets up serial connection to arduino atMega
ser = serial.Serial('/dev/ttyACM0',115200, timeout=.4);

#clear buffers just incase garbage inside
ser.reset_input_buffer()
ser.reset_output_buffer()

pid = 1;

oldEncoder0 = 0
oldEncoder1 = 0
oldEncoder2 = 0

newEncoder0 = 0
newEncoder1 = 0
newEncoder2 = 0

#odemetry position
current_x = 0
current_y = 0
current_theta = 0

#PID goToGoal
kp = 1.2
ki = 0
kd = 0

minDistU = 35
minDistI = 50

# This functions sends  pwm signals to the motor and reverses the direction if given negative
# Example motor(255,0,0) would turn motor 0 on all the away and 1,2 off
# motor(125,-200,-100) motor 0 would have a half duty cycle, motor 1 would move backwards at a pwm of 200 etc...
def motors(m1,m2,m3):
	motorValues = [m1,m2,m3]
	for x in range(3):
		ser.write(("m %d %d %d\r" % (x, abs(motorValues[x]), int(motorValues[x]>=0))).encode())

def motorVelocity(m1,m2,m3):
	motorV= [m1*10,m2*10,m3*10]
	ser.write(("v %d %d %d \r" %(motorV[0],motorV[1],motorV[2])).encode())

#read encoder value from motor number given
def encoder(encoderNum):
	ser.reset_input_buffer()
	ser.write(("e %d \r" % (encoderNum)).encode())

	encoderValue = (ser.readline().decode())
	if encoderValue.rstrip() is '':
		return 0
	return int(encoderValue.rstrip())

def ultrasound(ultraSoundNum):
	index = 10
	while index > 0:
		#print("scan ultra")
		ser.reset_input_buffer()
		ser.write(("u %d \r" % (ultraSoundNum)).encode())
		ultraSoundValue = (ser.readline().decode())
		try:
			ret = int(ultraSoundValue.rstrip())
			if ret > 3:
				break
		except ValueError:
			ret = 0
		time.sleep(.1)
		index = index - 1
	return ret

def infrared(infraredNum):
	index = 10
	while index > 0:
		#print("scan ir")
		ser.reset_input_buffer()
		ser.write(("i %d \r" % (infraredNum)).encode())
		infraredValue = (ser.readline().decode())
		try:
			val =  float(infraredValue.rstrip())
			if val > 3:
				return val
		except:
			pass
		time.sleep(.1)
		index = index - 1
		
def rpm(rpmNum):
	ser.reset_input_buffer()
	ser.write(("r %f \r" % (rpmNum)).encode())
	rpmValue = (ser.readline().decode())
	return rpmValue.rstrip()

def enablePID(pidValue):
	pid = pidValue
	ser.write(("p %d \r" % (pid)).encode())

####################################################################################################################
#This specific version of move will not allow RPM to go over a certain value, if you give it a command that causes
#RPM to go over it will decrease all motors by a ratio so that it fits in the bounds of motor RPM
####################################################################################################################
def move(xd,yd,thetad):

	r = 0.03 # radius of each wheel [m]
	l = 0.19 # distance from each wheel to the point of reference [m]

	xd_des = xd # velocity in the x-direction in the local frame [m/s]
	yd_des = yd # velocity in the y-direction in the local frame [m/s]
	thd_des = thetad # velocity in the x-direction in the local frame [rad/sa]

	vel_des = np.array([xd_des,yd_des,thd_des]).reshape(3,1)

	FK_M = (2*np.pi*r/60)*np.array([1/np.sqrt(3),0,-1/np.sqrt(3),-1/3,2/3,-1/3,-1/(3*l),-1/(3*l),-1/(3*l)]).reshape(3,3) # Forward kinematics matrix

	IK_M = np.linalg.inv(FK_M) # Inverse kinematics matrix

	motor_spd_vec = np.dot(IK_M,vel_des, out=None)

	wheel1RPM = motor_spd_vec[0] # motor 2 speed [rpm]
	wheel0RPM = motor_spd_vec[1] # motor 1 speed [rpm]
	wheel2RPM = motor_spd_vec[2] # motor 3 speed [rpm]

	maxAllowedSpeed = 150
	if (abs(wheel1RPM) > maxAllowedSpeed or abs(wheel0RPM) > maxAllowedSpeed or abs(wheel2RPM) > maxAllowedSpeed):
		maxRPM = max(abs(motor_spd_vec))
		ratio = abs(maxRPM)/maxAllowedSpeed
		wheel0RPM = wheel0RPM/ratio
		wheel1RPM = wheel1RPM/ratio
		wheel2RPM = wheel2RPM/ratio

	#print("Wheel RPM: " +str(wheel0RPM[0])+", "+str(wheel1RPM[0])+ ", " +str(wheel2RPM[0]))

	motorVelocity(int(wheel0RPM),int(wheel1RPM),int(wheel2RPM))

def odemetryCalc(xk,yk,thetak,l=0.19, N=2249, r=0.03):
	global oldEncoder0
	global oldEncoder1
	global oldEncoder2

	newEncoder0 = encoder(0)
	newEncoder1 = encoder(1)
	newEncoder2 = encoder(2)

	deltaEncoder0 = newEncoder0 - oldEncoder0
	deltaEncoder1 = newEncoder1 - oldEncoder1
	deltaEncoder2 = newEncoder2 - oldEncoder2

	D0=(deltaEncoder0/N)*((2*np.pi*r))
	D1=(deltaEncoder1/N)*((2*np.pi*r))
	D2=(deltaEncoder2/N)*((2*np.pi*r))

	kinematic_mat = np.array([1/np.sqrt(3),0,-1/np.sqrt(3),-1/3,2/3,-1/3,-1/(3*l),-1/(3*l),-1/(3*l)]).reshape(3,3)

	rotation_mat= np.array([np.cos(thetak),-np.sin(thetak),0,np.sin(thetak),np.cos(thetak),0,0,0,1]).reshape(3,3)

	#   diffrence in ticks (rpm1)
	distance_mat = np.array([D1,D0,D2])[:,None]

	oldPos_mat = np.array([xk,yk,thetak])[:,None]

	# np.dot explanation https://stackoverflow.com/questions/21562986/numpy-matrix-vector-multiplication
	kinxrot = np.dot(rotation_mat,kinematic_mat)
	newPos_mat = oldPos_mat + np.dot(kinxrot,distance_mat)

	oldEncoder0 = newEncoder0
	oldEncoder1 = newEncoder1
	oldEncoder2 = newEncoder2

	return  newPos_mat

#Sets up odemetry for when the program starts, we need this because arduino stores the encoders
#and this will "Zero" it.
def initOdometry():
	global oldEncoder0
	global oldEncoder1
	global oldEncoder2
	oldEncoder0 = encoder(0)
	oldEncoder1 = encoder(1)
	oldEncoder2 = encoder(2)

ultra = {}
ultra['center'] = 3
ultra['right'] = 5
ultra['right1']= 4
ultra['left'] = 1
ultra['left1']= 2
ultra['back'] = 0
def isTooClose(v1_x, v1_y):
        #is anything too close
	#get array of ultra SENSORS
	#if middle is less than destination
	center = ultrasound(ultra['center'])
	right = ultrasound(ultra['right'])
	right1 = ultrasound(ultra['right1'])
	left = ultrasound(ultra['left'])
	left1 = ultrasound(ultra['left1'])
	back = ultrasound(ultra['back'])
	print("Center: " +  str(center) + " Left: " + str(left) + " Left1: " + str(left1) + " Right: " + str(right) + " right1: " + str(right1) + " Back: " + str(back))
	ir0 = infrared(0)
	ir1 = infrared(1)
	ir2 = infrared(2)
	ir3 = infrared(3)

	

	if ((minDistI > ir1) and (minDistI > ir2)) or (minDistU > center) or (minDistU > left1) or (minDistU > right1) or (minDistU > left) or (minDistU > right) or (minDistU > back):
		print("can't move forward")
		if ((minDistI < ir0) and (minDistI < ir1)) or ((minDistU < left) and (minDistU < left1)):
			print("is moving left")
			v1_x=-v1_y
			v1_y=v1_x
			v1_theta=0
			avoid(v1_x,v1_y,v1_theta)
			return True
		elif ((minDistI < ir3) and (minDistI < ir2)) or ((minDistU < right) and (minDistU < right1)):
			print("is moving right")
			v1_x=v1_y
			v1_y=-v1_x
			v1_theta=0
			avoid(v1_x,v1_y,v1_theta)
			return True
		elif minDistU < back:
			print("is moving back")
			v1_x=-v1_x
			v1_y=-v1_y
			v1_theta=0
			avoid(v1_x,v1_y,v1_theta)
			return True
		else:
			print("I am stuck and don't know how to go on so I'll stop...")
			print("v1_x: " + str(v1_x) + " v1_y: " + str(v1_y))
			exit()
	return False
	#maintained weighted alt solutions to randomly pick by how likely they have been to resolve

def avoid(vl_x,vl_y,vl_theta):
	print("avoid some shit")
	global current_x
	global current_y
	global current_theta
	move(vl_x,vl_y,vl_theta)
	pose = odemetryCalc(current_x,current_y,current_theta)
	current_x = pose.item(0)
	current_y = pose.item(1)
	current_theta = pose.item(2)
	time.sleep(1)
	data_write = "x: "+str(pose[0][0])+" y: "+str(pose[1][0])+" theta: "+str(pose[2][0])
	print(data_write)
	move(0, 0, 0)
	return True

def isAtGoal(xd, yd):
	val = False
	error = 0.1
	return ((xd-error) <= current_x <= (xd+error)) and ((yd-error) <= current_y <= (yd+error))

def speed(duration, xd, xc, yd, yc):
	dist = np.sqrt(np.power((xd-xc), 2) + np.power(yd-yc, 2))
	timeleft = dist*2
	return timeleft

def goToGoalTimed(xd,yd,dtheta,duration):
	global current_x
	global current_y
	global current_theta
	dt = 0.1
	start = time.time()
	rate = speed(duration, xd, current_x, yd, current_y)
	while time.time()-float(start) <= float(rate):
		print("XD: " + str(xd) + " YD: " + str(yd))
		xc = current_x
		yc = current_y
		thetac = current_theta

		inv_rotation_mat = np.array([np.cos(thetac), np.sin(thetac), 0, -np.sin(thetac), np.cos(thetac), 0, 0, 0, 1]).reshape(3,3)

		d = np.sqrt(np.power((yd-yc), 2) + np.power((xd-xc), 2))# calculate the distance from the goal

		phi = math.atan2(yd-yc, xd-xc)#calculate the required angle to go to goal

		vel_global = np.array([d*np.cos(phi),d*np.sin(phi),0])[:,None] #calculate the required global velocity to go to goal

		vel_local = np.dot(inv_rotation_mat, vel_global)#calculate the local velocity as input to the inverse kinematics algorithm

		time_left = rate - (time.time() - start) #duration - time elapsed = time left
		vl_x = vel_local[0] / time_left
		vl_y = vel_local[1] / time_left
		vl_theta = vel_local[2] / time_left
		if isTooClose(vl_x, vl_y):
			break
		move(vl_x,vl_y,vl_theta)
		pose = odemetryCalc(current_x,current_y,current_theta)
		current_x = pose.item(0)
		current_y = pose.item(1)
		current_theta = pose.item(2)
		time.sleep(dt)
		data_write = "x: "+str(pose[0][0])+"  y: "+str(pose[1][0])+"  theta: "+str(pose[2][0])
		print(data_write)
	move(0,0,0)

initOdometry()
while True:
	print("######### Enter your goal (x,y) :) ########## ")
	xd = float(input("enter x desired: "))
	yd = float(input("enter y desired: "))
	thetad = float(input("enter theta desired: "))
	duration = int(input("duration (s): "))
	while (not isAtGoal(xd,yd)):
		goToGoalTimed(xd,yd,thetad,duration)





