#!/usr/bin/env python
import rospy
import pid
import numpy as np
import matplotlib.pyplot as plt
import time
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from mav_msgs.msg import Actuators
import time

def publish_waypoint(x,y,z,yaw):
	"""
	Publish a waypoint to 
	"""
	print("here")

	plotz = []
	plot1 = []
	velo = []
	timer = []

	pub = rospy.Publisher('/hummingbird/command/motor_speed', Actuators, queue_size = 1)
	acc = Actuators()

	pi = pid.PIDController(1.477, 0, 0.0008679)
	start_time = time.time()

	while True:
		#read position
		data = None
		data = rospy.wait_for_message('/hummingbird/odometry_sensor1/position', PointStamped, timeout=1)
		    
		plotz.append(data.point.z)
		plot1.append(10)
		
		plt.plot(plotz, color='blue')
		plt.plot(plot1, color='red')
		plt.pause(0.000001)

		u_x, u_y, u_z, u_rot_z = pi.run(data.point.x, x, data.point.y, y, data.point.z, z, 0, 0)

		throttle = 457.72396556425923 + u_z
		velo.append(throttle)
		acc.angular_velocities = [throttle, throttle, throttle, throttle]
		pub.publish(acc)
	
		#if(int(round(data.point.z)) == 30):
			#acc.angular_velocities = [0, 0, 0, 0]
			#pub.publish(acc)
			#break
		timer.append(time.time())





if __name__ == '__main__':
	
	rospy.init_node("rotors_waypoint_publisher", anonymous = True)

	# get command line params
	x_des = 0
	y_des = 0
	z_des = 10.0
	yaw_des = 0

	publish_waypoint(x_des, y_des, z_des, yaw_des)

	
