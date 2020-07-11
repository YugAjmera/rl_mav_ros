import pid
from geometry_msgs.msg import Twist
import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

def publish(des_x, des_y, des_z):

	pid_x = pid.PID(3, 0.001, 0.5)
	pid_y = pid.PID(3, 0.001, 0.5)
	pid_z = pid.PID(3, 0.001, 0.5)
	
	
	plot1 = []
	plotz = []
	vel = rospy.Publisher('/quadrotor/cmd_vel', Twist, queue_size = 1)

	while True:

		data = rospy.wait_for_message('/quadrotor/ground_truth/state', Odometry, timeout=1)
	   	error_x = des_x - data.pose.pose.position.x
		error_y = des_y - data.pose.pose.position.y
		error_z = des_z - data.pose.pose.position.z

		com = Twist()
	
		com.linear.x = pid_x.Update(error_x)
		com.linear.y = pid_y.Update(error_y)
		com.linear.z = pid_z.Update(error_z)
		com.angular.z = 0.0

		vel.publish(com)
		

if __name__ == '__main__':
	
	rospy.init_node("rotors_waypoint_publisher", anonymous = True)

	# get command line params
	x_des = 3.0
	y_des = 2.0
	z_des = 2.0
	
	publish(x_des, y_des, z_des)
