import pid
from geometry_msgs.msg import Twist
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
	
	
def callback(msg):

	des_x = msg.x
	des_y = msg.y
	des_z = msg.z	

	pid_x = pid.PID(1, 0.001, 0.1)
	pid_y = pid.PID(1, 0.001, 0.1)
	pid_z = pid.PID(3, 0.001, 0.5)

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
	
	rospy.init_node("waypoint_publisher", anonymous = True)

	sub = rospy.Subscriber('waypoint', Point, callback)
	rospy.spin()
					
