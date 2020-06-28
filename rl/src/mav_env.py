import gym
import rospy
import roslaunch
import time
import numpy as np

from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from gym.utils import seeding
from gym.envs.registration import register
from std_srvs.srv import Empty

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped

import time


reg = register(
	id='MavEnv-v0',
	entry_point='mav_env:MavEnv',
	max_episode_steps = 300,
)

class MavEnv(gazebo_env.GazeboEnv):

    def __init__(self):
        
        
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

	self.waypoint_pub = rospy.Publisher('/hummingbird/command/pose', PoseStamped, queue_size = 1)


        self.action_space = spaces.Discrete(3) #F,L,R
        self.reward_range = (-np.inf, np.inf)

        self._seed()
	self.flag = False
	self.current_state = []
	

    def return_state(self,data):
	
	full_state = []
	
	full_state.append(int(round(data.point.x)))
	full_state.append(int(round(data.point.y)))
	full_state.append(int(round(data.point.z)))

        min_range = 5

	goal_x = 4
	goal_y = 5

        done = False
	self.flag = False
	
	if (full_state[0] >  min_range or full_state[0] < 0 or full_state[1] < 0 or full_state[1] > min_range):
		done = True
		print "Out of Limits"

	if(full_state[0] == goal_x and full_state[1] == goal_y):
		done = True
		self.flag = True
		print "Goal Reached"
	#print(full_state)
	self.current_state = full_state
	return full_state, done


    def step(self, action):

        if action == 0: #FORWARD
	    waypose = PoseStamped()
	    waypose.pose.position.x = self.current_state[0] + 1
	    waypose.pose.position.y = self.current_state[1]
	    waypose.pose.position.z = self.current_state[2]
	    self.waypoint_pub.publish(waypose)
	    print("Action: Forward")
	    time.sleep(2)

        elif action == 1: #LEFT
            waypose = PoseStamped()
	    waypose.pose.position.x = self.current_state[0]
	    waypose.pose.position.y = self.current_state[1] + 1
	    waypose.pose.position.z = self.current_state[2]
	    self.waypoint_pub.publish(waypose)
	    print("Action: Left")
	    time.sleep(2)
            
        elif action == 2: #RIGHT
            waypose = PoseStamped()
	    waypose.pose.position.x = self.current_state[0]
	    waypose.pose.position.y = self.current_state[1] - 1
	    waypose.pose.position.z = self.current_state[2]
	    self.waypoint_pub.publish(waypose)
	    print("Action: Right")
            time.sleep(2)

	data = None
	
        while data is None:
            try:
                data = rospy.wait_for_message('/hummingbird/odometry_sensor1/position', PointStamped, timeout=1)
            except:
                pass


        state,done = self.return_state(data)
	
        if not done:
            reward = -1
        else:
	    if self.flag:
		reward = 100
	    else:
            	reward = -10

        return state, reward, done, {}

    def reset(self):

	time.sleep(2)
        #Reset
	waypose = PoseStamped()
    	waypose.pose.position.x = 0.0
    	waypose.pose.position.y = 0.0
    	waypose.pose.position.z = 1.0
    	self.waypoint_pub.publish(waypose)
    	print("Reset.......")
    	time.sleep(5)
	

	#read position
        data = None
	
        while data is None:
            try:
                data = rospy.wait_for_message('/hummingbird/odometry_sensor1/position', PointStamped, timeout=1)
            except:
                pass


        state, done = self.return_state(data)

        return state, done

