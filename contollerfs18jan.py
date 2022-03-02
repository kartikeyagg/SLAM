#!/usr/bin/env python3
#importing libraries
import dronekit
from pymavlink import mavutil
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import numpy as np
from math import *
import threading
import random
import roslib
import rospy
import logging
import os
import filters

from nav_msgs.msg import Odometry
#from shapely.geometry import Point
#from shapely.geometry.polygon import Polygon
#from shapely.geometry import LineString

import time
import sys
import socket
import json
import threading
import time
import dronekit as dk
import logging
import sys
import simple_pid
import math
req_up_v = 0

#print("Python version")
#print (sys.version)

kp = 1.10 # Kp for roll, pitch PID controllers
ki = 0.01 # Ki for roll, pitch PID controllers
kd = 0.012
 # Kd for roll, pitch PID controllers

tkp = 0.02 # Kp for thrust PID controller
tki = 0.0 # Ki for thrust PID controller
tkd = 0.005 #Kd for thrust PID controller
rnorth = 0
rdown = 0
reast  = 0
#orient = 0
global f
desired_accel =0
try:
	flight_aim = sys.argv[1]
except Exception as err:
	print("Please Specify Flight number or aim as argument in terminal while running code for log file name")
	exit()
file_name = "slam_logs/slam_log_"+str(flight_aim)+".txt"
print("Creating log "+file_name)

class swarmbot:
	def __init__(self,s):
		logging.basicConfig(filename="./controller_logs" ,format=' %(module)s %(lineno)d %(message)s',filemode='w')
		self.logger = logging.getLogger()
		self.logger.setLevel(logging.INFO)
		self.velocity=[0,0]
		print("In Program")
		f.write("In Program"+"\n")
		#self.vehicle = dronekit.connect(s,wait_ready=False)
		self.vehicle = connect('udp:127.0.0.1:14551')
		print("connecting")
		f.write("connecting"+"\n")
		#self.id=int(self.vehicle.parameters['SYSID_THISMAV'])
		self.id=1
		print("Connected to vehicle id : " + str(self.id))
		f.write("Connected to vehicle id : " + str(self.id)+"\n")
		self.alt = 0
		@self.vehicle.on_message('RANGEFINDER')
		def listener(slf, name, message):
			self.alt = float(message.distance)

		self.wplist=[]

	def get_pos(self):
		self.pos = [self.vehicle.location.global_frame.lat,self.vehicle.location.global_frame.lon]
		return self.pos

	def update_pos(self,pos,v):
		self.position=pos
		pos_x = pos[0]
		pos_y = pos[1]
		pos_z = 25

		a_location = LocationGlobalRelative(pos_x, pos_y, pos_z)
		self.vehicle.simple_goto(a_location,v,v)

	def update_vel(self,v):
		self.velocity=v
		velocity_x=v[0]
		velocity_y=v[1]
		velocity_z=0
		msg = self.vehicle.message_factory.set_position_target_global_int_encode(
			0,       # time_boot_ms (not used)
			0, 0,    # target system, target component
			mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
			0b0000111111000111, # type_mask (only speeds enabled)
			0, # lat_int - X Position in WGS84 frame in 1e7 * meters
			0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
			0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
			# altitude above terrain if GLOBAL_TERRAIN_ALT_INT
			velocity_x, # X velocity in NED frame in m/s
			velocity_y, # Y velocity in NED frame in m/s
			velocity_z, # Z velocity in NED frame in m/s
			0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
			0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

		self.vehicle.send_mavlink(msg)

	def heading(self):
		self.head=self.vehicle.heading
		return self.head

	def altitude(self):
		return self.alt

	def arm_and_takeoff(self, aTargetAltitude):##################
		while not self.vehicle.is_armable:
			print(" Waiting for vehicle to initialise...",self.id)
			f.write(" Waiting for vehicle to initialise..." + str(self.id)+"\n")
			time.sleep(1)


		print ("Arming motors",self.id)
		f.write("Arming motors " + str(self.id)+"\n")
		# Copter should arm in GUIDED mode
		self.vehicle.mode = dronekit.VehicleMode("GUIDED")
		self.vehicle.armed = True

		while not self.vehicle.armed:
			print (" Waiting for arming...",self.id)
			time.sleep(1)

		print ("Taking off!",self.id)
		self.vehicle.simple_takeoff(aTargetAltitude)

		while True:
			print (" Altitude: ", self.vehicle.location.global_relative_frame.alt)
			if self.vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.90:
				print ("Reached target altitude",self.id)
				break
			time.sleep(1)

	def heightu(self):
		return self.vehicle.location.global_relative_frame.alt
	def arm_fun(self):
		self.vehicle.mode = dronekit.VehicleMode("GUIDED_NOGPS")
		self.vehicle.armed = True
		while not self.vehicle.armed:
			print (" Waiting for arming...",self.id)
			f.write(" Waiting for arming..."+str(self.id)+"\n")
			time.sleep(1)

	def land(self):
		self.vehicle.mode = dronekit.VehicleMode("LAND")
		f.write("VehicleMode is "+str(self.vehicle.mode)+"\n")
	def brk(self):
		self.vehicle.mode = dronekit.VehicleMode("BRAKE")
		f.write("VehicleMode is "+str(self.vehicle.mode)+"\n")
	def guid(self):
		self.vehicle.mode = dronekit.VehicleMode("GUIDED")
		f.write("VehicleMode is "+str(self.vehicle.mode)+"\n")
	def guidednogps(self):
		self.vehicle.mode = dronekit.VehicleMode("GUIDED_NOGPS")
		f.write("VehicleMode is "+str(self.vehicle.mode)+"\n")
	def wpoints(self,p):
		self.wplist=[]
		for i in range(len(p)):
		   self.wplist.append(p[i])
		return self.wplist



	def waypoints(self):
		return self.wplist
	def get_home_location(self):
		self.vehicle.commands.download()
		self.vehicle.commands.wait_ready()

		return self.vehicle.home_location

	def get_pos_local(self):
		self.pos1=[self.vehicle.location.local_frame.north,self.vehicle.location.local_frame.east,self.vehicle.location.local_frame.down]
		f.write("Pos_Local "+str(self.pos1)+"\n")
		return self.pos1
	def update_pos_local_frame(self,pos):
		vehicle.Capabilities.set_attitude_target_local_ned(pos)
	def get_attitude(self):
		return [((self.vehicle.attitude.roll)*180/pi),((self.vehicle.attitude.pitch)*180/pi)]
	def get_mode(self):
		return self.vehicle.mode

'''
	def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0,yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,thrust = 0.5):
		"""
		use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
					  When one is used, the other is ignored by Ardupilot.
		thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
				Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
				the code for maintaining current altitude.
		"""
		if yaw_angle is None:
			# this value may be unused by the vehicle, depending on use_yaw_rate
			yaw_angle = self.vehicle.attitude.yaw
		# Thrust >  0.5: Ascend
		# Thrust == 0.5: Hold the altitude
		# Thrust <  0.5: Descend
		msg = self.vehicle.message_factory.set_attitude_target_encode(
			0, # time_boot_ms
			1, # Target system
			1, # Target component
			0b00000000 if use_yaw_rate else 0b00000100,
			to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
			0, # Body roll rate in radian
			0, # Body pitch rate in radian
			radians(yaw_rate), # Body yaw rate in radian/second
			thrust  # Thrust
		)
		self.vehicle.send_mavlink(msg)


	def set_attitude(self,roll_angle = 0.0, pitch_angle = 0.0,
					 yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
					 thrust = 0.5, duration = 0):
		"""
		Note that from AC3.3 the message should be re-sent more often than every
		second, as an ATTITUDE_TARGET order has a timeout of 1s.
		In AC3.2.1 and earlier the specified attitude persists until it is canceled.
		The code below should work on either version.
		Sending the message multiple times is the recommended way.
		"""
		self.send_attitude_target(roll_angle, pitch_angle,yaw_angle, yaw_rate, False,thrust)
		start = time.time()
		while time.time() - start < duration:
			send_attitude_target(roll_angle, pitch_angle,
								 yaw_angle, yaw_rate, False,
								 thrust)
			time.sleep(0.1)
		# Reset attitude, or it will persist for 1s more due to the timeout
		self.send_attitude_target(0, 0,0, 0, True,thrust)

'''
'''def euler_from_quaternion(x, y, z, w):
		"""
		Convert a quaternion into euler angles (roll, pitch, yaw)
		roll is rotation around x in radians (counterclockwise)
		pitch is rotation around y in radians (counterclockwise)
		yaw is rotation around z in radians (counterclockwise)
		"""
		t0 = +2.0 * (w * x + y * z)
		t1 = +1.0 - 2.0 * (x * x + y * y)
		roll_x = math.atan2(t0, t1)

		t2 = +2.0 * (w * y - z * x)
		t2 = +1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		pitch_y = math.asin(t2)

		t3 = +2.0 * (w * z + x * y)
		t4 = +1.0 - 2.0 * (y * y + z * z)
		yaw_z = math.atan2(t3, t4)
		return [roll_x,pitch_y,yaw_z] # in radians'''
#a callback function to recieve odom data from rtabmap (after ekf)
def get_vo_pos_local(odom):
	global rnorth
	global rdown
	global reast
	global orient



	#print("North " + str(odom.pose.pose.position.z ))
	#print("East " + str(odom.pose.pose.position.x ))
	#print( "Down " + str(odom.pose.pose.position.y ))
	#print("--------------------------------------- \n =====================================")
	rnorth = odom.pose.pose.position.x
	reast = odom.pose.pose.position.y
	rdown = odom.pose.pose.position.z
	'''orient_x = odom.pose.pose.orientation.x
	orient_y = odom.pose.pose.orientation.y
	orient_z = odom.pose.pose.orientation.z
	orient_w = odom.pose.pose.orientation.w
	orientation_all  = euler_from_quaternion(orient_x, orient_y, orient_z, orient_w)
	orient = orientation_all[2]'''

def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
	"""
	Convert degrees to quaternions
	"""
	t0 = math.cos(math.radians(yaw * 0.5))
	t1 = math.sin(math.radians(yaw * 0.5))
	t2 = math.cos(math.radians(roll * 0.5))
	t3 = math.sin(math.radians(roll * 0.5))
	t4 = math.cos(math.radians(pitch * 0.5))
	t5 = math.sin(math.radians(pitch * 0.5))

	w = t0 * t2 * t4 + t1 * t3 * t5
	x = t0 * t3 * t4 - t1 * t2 * t5
	y = t0 * t2 * t5 + t1 * t3 * t4
	z = t1 * t2 * t4 - t0 * t3 * t5
	return [w, x, y, z]


def set_attitude(vehicle,roll_angle=0.0, pitch_angle=0.0, yaw_angle=0, thrust=0.5, duration=0, yaw_rate = 0): #control
	#print(yaw_angle,yaw_rate)
	# Duration is seconds to do this for
	#print ("r:",round(roll_angle,2),"\tp:",round(pitch_angle,2))
	msg = vehicle.message_factory.set_attitude_target_encode(
		0,
		0,  # target system
		0,  # target component
		0b00000100,  # type mask: bit 1 is LSB
		to_quaternion(roll_angle, pitch_angle, yaw_angle),  # q
		0,  # body roll rate in radian
		0,  # body pitch rate in radian
		math.radians(yaw_rate),  # body yaw rate in radian
		thrust)  # thrust

	vehicle.send_mavlink(msg)
	if duration != 0:
		# Divide the duration into the frational and integer parts
		modf = math.modf(duration)

		# Sleep for the fractional part
		time.sleep(modf[0])

		# Send command to vehicle on 1 Hz cycle
		for x in range(0, int(modf[1])):
			time.sleep(0.05)
			vehicle.send_mavlink(msg)
			# print vehicle.velocity
	#print(vehicle.attitude)
def limit(value, limits):
	"""ssssss
	Limits the value to limits(a tuple of lower and upper limit)
	"""
	lower, upper = limits
	if value is None:
		return None
	elif upper is not None and value > upper:
		return upper
	elif lower is not None and value < lower:
		return lower
	return value


def distance(lat1,lon1,lat2,lon2):
	lon1=radians(lon1)
	lon2=radians(lon2)
	lat1=radians(lat1)
	lat2=radians(lat2)
	dlon=lon2-lon1
	dlat=lat2-lat1
	a=sin(dlat/2)**2+cos(lat1)*cos(lat2)*sin(dlon/2)**2
	c=2*asin(sqrt(a))
	R=6371
	return(c*R)
def distance_metres(x1,y1,z1,x2,y2,z2):
	p1 = np.array([x1, y1, z1])
	p2 = np.array([x2, y2, z2])
	squared_dist = np.sum((p1-p2)**2, axis=0)
	dist = np.sqrt(squared_dist)
	#dis= ((x1 - x2)**2 + (y1-y2)**2)**1/2
	#if dis <0:
	#	dis=dis * (-1)
	return dist

class thrust(object):
	"""
	Generates thrust values using a PID controller of controlling the velocity z
	"""
	def __init__(self, setpoint):
		"""
		:param setpoint: setpoint in z
		"""
		self.initial_value = 0.5 # value of thrust for 0 velocity in z direction
		self.dT = 0 # time elapsed since the last update of the value
		self.h_new = 0 # current height
		self.h_last = 0 # height when the value of thrust was updated last time
		self.t = simple_pid.PID(tkp, tki, tkd, 0.0, 0.01, (-0.1, 0.1)) # PID for thrust to maintain a certain z velocity, initially setpoint of velocity is 0 and the limits are -0.1 to 0.1
																	   # (0.5 is added to the value generated by PID)
		self.setpoint = setpoint


	def calc_speed(self, error):
		global req_up_v
		"""
		calculates required velocity using using the equation
		f(error) = a * error * |error| /((b ^ 2) + (error ^ 2))
		a = maximum output value of f(error)
		b = smoothness of f(error) as error tends to 0. Higher value of b causes slow convergence whereas lower values can result in over shooting.
		PLOT f(error) FOR BETTER UNDERSTANDING AND VARY THE VALUE OF a AND b.
		:param error: error in position(along z)
		returns: required velocity for the UAV
		"""
		req_up_v = 0.5 * error * abs(error) / ((0.5 ** 2) + (error ** 2))

		return 0.5 * error * abs(error) / ((0.5 ** 2) + (error ** 2))

	def __call__(self, h, delta_time):
		"""
		The class object is callable.
		:param h: current height
		:param delta_time: time difference between the last the current call of the function
		"""
		# print(self.h_new, self.h_last)
		if self.dT > 0.25: # generate new values of thrust after every 0.25 seconds
			error = self.setpoint - h # error in position
			self.t.setpoint = self.calc_speed(error) # setting the setpoint of the PID controller to the desired z velocity
		   # print(self.t.setpoint)
			e = self.t((self.h_new - self.h_last) / self.dT) # calling the PID thrust object to generate required thrust
			self.initial_value = 0.5 + e  # adding 0.5 to the thrust
			self.dT = 0 # setting it to zero as an output is generated
			self.h_last = h # setting last height to current height
		self.t._integral = limit(self.t._integral, (-0.2, 0.2)) # limiting the PID thrust integral to 0.2 to prevent over shooting
		self.h_new = h # updatig current height
		self.dT += delta_time # summing up dT
		#print(self.initial_value)
		current_up_vel = (self.h_last - self.h_new)/delta_time
		f.write("current upward_vel is : "+str(current_up_vel)+"\n")
		f.write("Thrust Attitude "+str(self.initial_value)+"\n")
		return self.initial_value # returning the required thrust
class control_rp(object):
	"""
	Generates roll, pitch values using 2 PID controllers for controlling velocity in x and y direction
	"""
	def __init__(self,
				 setpoint,
				 output_limits=(None, None)):
		"""
		:param setpoint: setpoint in (x, y) (a numpy array)
		:param output_limits: To limit the output (in degrees)
		"""
		#self.setpoint = setpoint
		#self.logger = logging.getLogger()
		#self.logger.setLevel(logging.INFO)
		self._min_output, self._max_output = output_limits
		self._last_output = None # last output of the function
		self.scaling_factor = 0.6# constant s in the desired velocity equation given below in function calc_required_vel. The maximum desired velocity of the UAV
		self.smoothing_factor = 0.70# constant b in the desired velocity equation given below in function calc_required_vel
		self.g = 9.81 # acceleration due to gravity
		self.pid_x = simple_pid.PID(kp, ki, kd) # PID for x acceleration to maintain a certain x velocity.
		self.pid_y = simple_pid.PID(kp, ki, kd) # PID for y acceleration to maintain a certain y velocity.
		self.filter = filters.Filters() # initializing the filters class object for low pass filter

	def calc_required_vel(self, error): # NOT USED NOW
		return (self.scaling_factor * error * abs(error)) / ((self.smoothing_factor ** 2) + (error ** 2))

	def calc_required_vel2(self, error):
		"""
		calculates required velocity using using the function
		f(error) = u * a * error ^ 2 /((b ^ 2) + (error ^ 2))
		u = unit vector in the direction of positional error
		error = magnitude of positional error
		a = maximum output value of f(error)
		b = smoothness of the function f(error) as error tends to 0. Higher value of b causes slow convergence whereas lower values can result in over shooting.
		PLOT f(error) FOR BETTER UNDERSTANDING AND VARY THE VALUE OF a AND b, KEEPING u AS CONSTANT.
		:param error: error in position(along z)
		returns: required velocity for the UAV
		"""
		r = np.sum(error ** 2) ** 0.5 # magnitude of positional error
		if r == 0: # set r to 1 if it is 0 to avoid ZeroDivisionError
			r = 1
		unit_vector = error / r # unit vector in the direction of error
		v = (self.scaling_factor * (r ** 2)) / ((self.smoothing_factor ** 2) + (r ** 2)) # magnitude of required velocity
		return v * unit_vector # desired velocity vector

	def __call__(self, input_, current_vel, h, dt):
		"""
		The class object is callable.
		:param input_: current position (x, y) of the uav as a numpy error
		:param current_vel: current velocity of the UAV
		:param h: current altitude of the UAV
		:param dt: time difference between the last the current call of the function
		"""
		vel_error = np.array([0.0, 0.0])
		global desired_accel




		f.write("current velocity is "+str(current_vel[0])+" "+str(current_vel[1])+"\n")
		#f.write("current velocity is "+str(current_vel[0])+" "+str(current_vel[1])+"\n")
		#print(self.setpoint,input_)
		f.write("RP_setpoint "+str(self.setpoint)+"\n")
		error = (self.setpoint - input_)
		#print(error,"error") # positional error
		f.write("Positional Error " + str(str(error[0])+" "+str(error[1])) +"\n")
		required_vel = self.calc_required_vel2(error) # desired velocity
		#print(required_vel,"required vel")
		f.write("Required Velocity"+str(str(required_vel[0])+" "+ str(required_vel[1]) )+"\n")
		f.write(str(current_vel[0]) + str(current_vel[1])+"\n") # writing current velocity to file
		current_vel = self.filter.lowpass(current_vel, dt, 0.15 ) # filtering the current velocity using low pass filter
		f.write("filtered velocity is\t" + str(current_vel[0]) + "\t" + str(current_vel[1]) +"\n") # writing to file

		self.pid_x.setpoint = required_vel[0] # setting PID x setpoint as desired velocity x
		self.pid_y.setpoint = required_vel[1] # setting PID y setpoint as desired velocity y

		desired_accel = np.array([self.pid_x(current_vel[0]), self.pid_y(current_vel[1])]) # Calling PID x and PID y object to calculate desired acceleration
		f.write("Desired Acceleration "+str(desired_accel[0]) + " "+str(desired_accel[1] )+ "\n")
		f.write("Pitch then roll direction \n")
		# print(desired_accel)
		f.write(str(error[0]) + "\t" + str(required_vel[0]) + "\t" + str(error[1]) + "\t" + str(required_vel[1]) + "\t" + str(desired_accel[0]) + "\t " + str(desired_accel[1]) + "\n") #writing to file
		#print(error, required_vel, desired_accel,"error, required_vel, desired_accel")
		self._last_vel = current_vel # updating last velocity
		self._last_output = desired_accel # updating last acceleration output
		#self._last_des_vel = required_vel
		return self.accel_to_angles(desired_accel) # converting desired acceleration to angles

	def accel_to_angles(self, accel):
		"""
		Converts desired acceleration to angles
		"""
		pitch_angle = atan(-accel[0] / self.g)
		roll_angle = atan(accel[1] * cos(pitch_angle) / self.g)

		return roll_angle * 180 / pi, pitch_angle * 180 / pi

	def vel_from_smc(u,m1,m2,R,l,lem1,lem2):
		t1 = lem2*(2.303**(lem1-l))
		t2 = lem1*(2.303**(lem2-1))
		t3= (sqrt(m2**2 + 4m1)*R)
		t4 = (sqrt(m2**2 + 4m1)*R)
		velf =( t1-t2+t3)*u/t4
		return velf1;
'''
foi1 = open("/home/uas-dtu/coordinate_list.txt") #opening waypoints txt file from server
fi1 = foi1.readlines()
fi2  =   fi1[0]    #the string of waypoints
#extracting waypoints
fi2 = fi2.split(" ")
print(fi2)
mainarr = []
for i in fi2:
    wparr  =  []
    tfi = i.split("#")
    print(tfi)

    for j in range(1,len(tfi)):
        wparr.append(int(tfi[j]))
    wparr.append(int(fi1[1])) #fi1[1] is height
    mainarr.append(wparr)
mainarr.pop()
print(mainarr)
#waypoints extracted
exarr = [] #an array to copy
exarr.extend(mainarr)
print(int(fi1[2]))
#extending list for no. of loops
for ty in range(int(fi1[2])):
    mainarr.pop()
    mainarr.extend(exarr)
mainarr.append([0,0,int(fi1[1])]) #adding pseudo(dummy) waypoints at the end
mainarr.append([0,0,int(fi1[1])])
print("yop")
print(mainarr)#the final waypoints array
'''

f = open(file_name,"w+") #file for logs
print("SLAM started")
last_h =0
lflag = 0 # flag indicating uav has reached last waypoint

f.write("SLAM started\n"+"\n")
#f.write(mainarr+"\n")
#f.write(("pid params kp:- %f kd:- %f ki:- %f ",kp,kd,ki))
uav=swarmbot("127.0.0.1:" + str(14551)) #connecting to pixhawk
f.write("Starting SLAM"+"\n")
uav.arm_fun() #arming the uav
dflag = 0 # flag for mode land after reaching a minimal height (20cm)
zflag = 0 #flag for odom lost failsafe
current_heading = uav.vehicle.heading #for yaw corrections
print("connected")
f.write("Connected"+"\n")
dhei = 0.8#desired height for the navigation
tr = thrust(dhei) #initializing class thrust object
#uav.arm_and_takeoff(0.5)
uav.get_home_location()
con_rp = control_rp(np.array([0,0], dtype=np.float32))
posL = [[0,0,dhei],[1.5,0,dhei],[1.5,0,dhei],[0,0,dhei],[0,0,dhei]]

#posL = [[0,0,dhei],[1.5,0,dhei],[1.5,1.5,dhei],[0,1.5,dhei],[0,0,dhei],[0,0,0],[0,0,0]]#[1.5,1.5,dhei],[0,1.5,dhei],[0,0,dhei],[0,0,0],[0,0,0]]#[1.5,0,dhei],[1.5,1.5,dhei],[0,1.5,dhei],[0,0,dhei],[1.5,0,dhei],[1.5,1.5,dhei],[0,1.5,dhei],[0,0,dhei],[100,100,dhei]] #,[0,0,dhei],[100,100,dhei]]#
# posL = mainarr
print(posL)
i=0
intial_pose=[0,0]
uav.guidednogps() #changing mode to guided no gps
delta_time=0.1
timer = 0.05
rospy.init_node('odometry_node', anonymous=True) #make node
f.write("Node Created"+"\n")
rospy.Subscriber('/rtabmap/odom',Odometry,get_vo_pos_local)
time.sleep(1)
print("Reached loop")
f.write("Subscribing Odom Topic"+"\n")
#thrus_val = 0.54
flag = 0
'''while True:
	set_attitude(uav.vehicle, roll_angle=0,pitch_angle=0,yaw_angle=current_heading, thrust = thrust_val)

	if  abs(uav.altitude()-dhei) <0.1` and flag==0:
		set_attitude(uav.vehicle, roll_angle=0,pitch_angle=0,yaw_angle=current_heading, thrust = thrust_val)
		f.write("Changed Thrust to 0.5"+"\n")
		flag = 1
		break
f.close()'''
#ensuring odometery available
while [rnorth,reast,rdown] == [0,0,0] :
	print("Waiting for r-odom \n")
	time.sleep(0.2)
	if [rnorth,reast,rdown] != [0,0,0]:

		break

starting_time = time.time()

while True:
	#uav.guidednogps()
	starttime= time.time()#############
	f = open(file_name,"a+")
	pos=posL[i]######################
	#z=uav.get_pos_local()
	'''print("North " + str(odom.pose.pose.position.z ))
	print("East " + str(odom.pose.pose.position.x ))
	print( "Down " + str(odom.pose.pose.position.y ))'''
	#print("--------------------------------------- \n =====================================")
	'''if pos==posL[-1]:#####uncomment this code if you dont want to hover###################
				print("reached my point")##################
				break######################'''
	f.write("-------------------------------------"+"\n")
	#a failsafe for rtabmap lost odom
	z = [rnorth,reast,rdown] #position acc to rtabmap#####################
	if z == [0,0,0]:
		zflag =zflag+1
	else :
		zflag = 0
	if zflag == 10:
		print("odometry'map lost \n odometry'map lost \n odometry'map lost \n odometry'map lost ")
		f.write("odometry'map lost \n odometry'map lost \n odometry'map lost \n odometry'map lost ")
		if uav.altitude() > 0.1:
			uav.land()
		else :
			uav.vehicle.armed = False
	print("printing rtabmap pos :- "+ str(str(rnorth)+ " "+str(reast) +" "+str(rdown)+ " "))
	f.write("printing rtabmap pos :- "+ str(str(rnorth)+ " "+str(reast) +" "+str(rdown)+ " "))
	''''if z[1] == 0 and z[0] == 0:

		uav.land()'''

	mav_pos=uav.get_pos_local() #position acc to mavproxy#####################
	#print("printing mav pos " + str(mav_pos))
	f.write("printing mav pos " + str(mav_pos)+"\n")

	#print("############################################################")
	f.write("############################################################")
	#print("####################first element ########################")
	f.write("####################first element ########################")
	#print("############################################################")
	f.write("############################################################"+"\n")
	con_rp.setpoint=np.array([pos[0],pos[1]])##########################################
	cur_height = uav.altitude()############
	'''if abs(pos[2]-uav.altitude())<0.15: ###########
		att_height=0.5#############
	elif uav.altitude()<pos[2]:#########
		att_height=0.515#############
	elif uav.altitude()>pos[2]+0.25:#########
		att_height=0.490#############'''
	'''elif pos[2]<cur_height:##############
		att_height=0.48###############'''



	#print(z,pos," current position and target pos")
	f.write("Current position "+str(z)+"\n")
	f.write("Target position "+str(pos)+"\n")
	print("current target waypoint ", pos)

	distance = distance_metres(z[0],z[1],cur_height,pos[0],pos[1],pos[2])
	distance2= distance_metres(z[0],z[1],0,pos[0],pos[1],0)
	print(distance2,"circle distance")
	print("Distance is ",distance) #distance is spherical distance while distance2 is circular
	f.write("Distance is "+str(distance)+"\n")
	#for first waypoint only (means 0,0,dhei)
	if distance <= 0.30 and i==0  :                 ############remove this if you want to navigate
		i+=0 ## '1' for mission and '0' to hover
		print("#################################changed Waypoint #################################################################### \n "  ) ############
	#for waypoint afterward the first waypint
	elif distance2 <=0.30 and i!=0 and lflag ==0 :
		i+=0
		print("##########################################waypoint changed \n #########################################################")
	if i>=len(posL)-1:
		#uav.land()
		print(" landing\n")
		f.write("landing \n")




	#print(np.array([((z[0]-intial_pose[0])/delta_time),(z[1]-intial_pose[1])/delta_time]),"current calculated velocity")
	roll,pitch=con_rp(np.array([z[0],z[1]]),np.array([(z[0]-intial_pose[0])/delta_time,(z[1]-intial_pose[1])/delta_time]),cur_height,delta_time)#########################




	#automatic land and flick to mode land after certain height to disarm accordingly
	att_height =float(tr(cur_height,delta_time))
	if att_height>0.56:
		att_height = 0.56            #limiting thrust value to avoid overshoot
	if i>=len(posL)-2:
		att_height  = 0.48
		lflag =1
		print("\nland-land\n")
		if cur_height<0.20:
			dflag += 1
		if dflag > 12:
			uav.land()
			print("*************ending code****************")
			break



	print("attitude for thrust is  ",att_height)


	if (desired_accel[1] * roll) < 0:
		print("OMG!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! \n")
		f.write("OMG!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n")
	print("Height", cur_height)
	f.write("Height "+str(cur_height)+"\n")
	roll = limit(roll,(-5,5))##################
	pitch = limit(pitch,(-5,5))##################liminting roll and pitch
	print(roll, " ", pitch,"roll and pitch")
	f.write("Roll is "+str(roll)+"\n")
	f.write("Altitude is "+str(att_height)+"\n")
	f.write("Pitch is "+str(pitch)+"\n")
	#print("current attitude",uav.get_attitude())
	f.write("current attitude "+str(uav.get_attitude())+"\n")
	f.write("current MODE "+str(uav.get_mode())+"\n")
	'''print("rtab orintation "+str(orient *180/pi)+"\n" )
	f.write("rtab orintation "+str(orient *180/pi)+"\n")
	print("home heading "+str(current_heading - (orient*180/pi))+"\n")
	f.write("home heading " +str(current_heading - (orient*180/pi))+"\n")'''

	set_attitude(uav.vehicle, roll_angle=roll,pitch_angle=pitch,yaw_angle=current_heading, thrust = att_height)#Giving uav the desired attitude
	#print("updated attitude",uav.get_attitude())
	f.write("updated attitude "+str(uav.get_attitude())+"\n")
	#print(z)
	f.write("North-East Array "+str(z)+"\n")
	intial_pose=z.copy()#####################################################
	#print(intial_pose,"last known position ")
	f.write("Last known Position "+str(intial_pose)+"\n")
	#print(delta_time,"initial delta time ")
	f.write("Initial Delta Time "+str(delta_time)+"\n")
	f.write("reqiured up_velocity:- "+str(req_up_v)+"\n")
	current_up_vel = (cur_height- last_h)/delta_time
	f.write("Current up_vel: "+ str(current_up_vel)+"\n")
	#f.write("Thrust Attitude "+str(self.initial_value)+"\n")


	#print(delta_time,"delta_time after update")

	f.write("Current_time "+str(time.time()-starting_time )+"\n")
	f.close()
	##############################################
	time.sleep(0.1)
	#if uav.get_mode() != "GUIDED_NOGPS":
		#break
	if uav.get_mode() != "GUIDED_NOGPS": # Breaking out if mode is not guidednogps
		break

	delta_time = time.time()-starttime    #time between one loop

#uav.land()
f.close()
rospy.spin()
