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
import waypoint_pub

import time


reg = register(
	id='MavEnv-v0',
	entry_point='mav_env:MavEnv',
)

class MavEnv(gazebo_env.GazeboEnv):

    def __init__(self):
        
        
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

	self.takeoff = rospy.Publisher('/quadrotor/ardrone/takeoff', Empty, queue_size = 1)


        self.action_space = spaces.Discrete(4) #F,L,R,B
        self.reward_range = (-np.inf, np.inf)

        self._seed()
	self.flag = False
	self.current_state = []

	self.waypoint = waypoint_pub.Waypoint()
	

    def return_state(self,data):
	
	full_state = []
	
	full_state.append(int(round(data.pose.pose.position.x)))
	full_state.append(int(round(data.pose.pose.position.y)))
	full_state.append(int(round(data.pose.pose.position.z)))

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
	    waypoint.publish(self.current_state[0] + 1, self.current_state[1], self.current_state[2])

        elif action == 1: #LEFT
            waypoint.publish(self.current_state[0], self.current_state[1] + 1, self.current_state[2])
            
        elif action == 2: #RIGHT
            waypoint.publish(self.current_state[0], self.current_state[1] - 1, self.current_state[2])

	elif action == 3: #Back
            waypoint.publish(self.current_state[0] - 1, self.current_state[1], self.current_state[2])

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

	# Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            #reset_proxy.call()
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_simulation service call failed")

        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            #resp_pause = pause.call()
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

	
	#takeoff
	self.takeoff.publish(Empty())
	time.sleep(1)
	

	#read position
        data = None
	
        while data is None:
            try:
                data = rospy.wait_for_message('/quadrotor/ground_truth/state', Odometry, timeout=1)
            except:
                pass


        state, done = self.return_state(data)

        return state, done

