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

import waypub
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


        self.action_space = spaces.Discrete(3) #F,L,R
        self.reward_range = (-np.inf, np.inf)

        self._seed()
	self.flag = False

    def return_state(self,data):
	
	full_state = []
	
	full_state.append(round(data.point.x))
	full_state.append(round(data.point.y))
	full_state.append(round(data.point.z))

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
		print "Goal Reached"

	print(full_state)
	return full_state, done


    def stepup(self, prevstate, action):

        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        if action == 0: #FORWARD
            waypub.publish_waypoint(prevstate[0] + 1,prevstate[1],prevstate[2],0)
	    print("Action: Forward")
	    time.sleep(1)
        elif action == 1: #LEFT
            waypub.publish_waypoint(prevstate[0],prevstate[1] +1,prevstate[2],0)
	    print("Action: Left")
            time.sleep(1)
        elif action == 2: #RIGHT
            waypub.publish_waypoint(prevstate[0],prevstate[1] - 1,prevstate[2],0)
	    print("Action: Right")
            time.sleep(1)

	data = None
	
        while data is None:
            try:
                data = rospy.wait_for_message('/hummingbird/odometry_sensor1/position', PointStamped, timeout=1)
            except:
                pass

	rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

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

        #Take off
	waypub.publish_waypoint(0,0,1,0)
	print("Taking off")
	time.sleep(1)
	#read position
        data = None
	
        while data is None:
            try:
                data = rospy.wait_for_message('/hummingbird/odometry_sensor1/position', PointStamped, timeout=1)
            except:
                pass

	rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        state, done = self.return_state(data)

        return state, done
