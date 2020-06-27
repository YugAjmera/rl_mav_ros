import gym
import rospy
import roslaunch
import time
import numpy as np

from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from gym.utils import seeding
from gym.envs.registration import register

from geometry_msgs.msg import PointStamped


reg = register(
	id='Mav-v0',
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

        min_range = 5
	min_range = min_range + 1

	goal_x = 4
	goal_y = 5

        done = False
	self.flag = False
	
	if(-1 < full_state[0] < min_range or -1 < full_state[1] < min_range):
		done = True

	if(full_state[0] == goal_x and full_state[1] == goal_y):
		done = True
		print "Goal Reached"
	
	return full_state,done

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):

        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        if action == 0: #FORWARD
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.3
            vel_cmd.angular.z = 0.0
            self.vel_pub.publish(vel_cmd)
        elif action == 1: #LEFT
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.08
            vel_cmd.angular.z = 0.8
            self.vel_pub.publish(vel_cmd)
        elif action == 2: #RIGHT
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.08
            vel_cmd.angular.z = -0.8
            self.vel_pub.publish(vel_cmd)

	data = None
	
        while data is None:
            try:
                data = rospy.wait_for_message'/hummingbird/odometry_sensor1/position', PointStamped, timeout=1)
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

        #Take off
	

	#read position
        data = None
	
        while data is None:
            try:
                data = rospy.wait_for_message'/hummingbird/odometry_sensor1/position', PointStamped, timeout=1)
            except:
                pass
	
	#Pause the simulation	
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        state = self.return_state(data)

        return state
