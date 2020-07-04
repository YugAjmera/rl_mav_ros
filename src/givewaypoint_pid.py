#!/usr/bin/env python
import rospy
import pid
import sys
import tf
import numpy as np

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped

def publish_waypoint(x,y,z,yaw):
	"""
	Publish a waypoint to 
	"""
	print("here")

	pi = pid.PIDController(0.5, 0.00416, 0.001)

	#read position
        data = None
	
        while data is None:
            try:
                data = rospy.wait_for_message('/hummingbird/odometry_sensor1/position', PointStamped, timeout=1)
            except:
		print("waiting")
                pass

	full_state = []
	
	full_state.append(int(round(data.point.x)))
	full_state.append(int(round(data.point.y)))
	full_state.append(int(round(data.point.z)))
	print(full_state)

	u_x, u_y, u_z, u_rot_z = pi.run(full_state[0], x, full_state[1], y, full_state[2], z, 0, 0)
	print(u_x)

	


if __name__ == '__main__':
	
	rospy.init_node("rotors_waypoint_publisher", anonymous = True)

	# get command line params
	x_des = float(sys.argv[1])
	y_des = float(sys.argv[2])
	z_des = float(sys.argv[3])
	yaw_des = float(sys.argv[4])

	publish_waypoint(x_des, y_des, z_des, yaw_des)

	#rospy.spinOnce()
	rospy.loginfo(" >> Published waypoint: x: {}, y: {}, z: {}, yaw: {}".format(x_des, y_des, z_des, yaw_des))

