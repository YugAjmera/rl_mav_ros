#!/usr/bin/env python
import gym
import rospy
import mav_env
from gym import wrappers
import gym_gazebo
import time
import numpy
import random
import time
import liveplot
import expectedSarsa as es


if __name__ == '__main__':

    rospy.init_node('expected_sarsa', anonymous=True)
    env = gym.make('MavEnv-v0')
    print "Gym Make Done"


    outdir = '/tmp/gazebo_gym_experiments'
    env = gym.wrappers.Monitor(env, outdir, force=True)
    print "Monitor Wrapper Started"

    plotter = liveplot.LivePlot(outdir)

    es = es.ExpectedSarsa(actions=range(env.action_space.n),
                    alpha=0.8, gamma=0.9, epsilon=0.1)
	

    start_time = time.time()
    total_episodes = 10
 
    for x in range(total_episodes):
        done = False

        cumulated_reward = 0 
        print("Episode = " +str(x)+ " started")
        observation = env.reset()

        state = ''.join(map(str, observation))

	i = 0
        #for i in range(300):
	while(True):

            # Pick an action based on the current state
            action = es.chooseAction(state)

            # Execute the action and get feedback
            observation, reward, done, info = env.step(action)
            cumulated_reward += reward


            nextState = ''.join(map(str, observation))

            es.learn(state, action, reward, nextState)

            env._flush(force=True)

            if not(done):
                state = nextState
            else:
                break

	    i = i+1

	
	plotter.plot(env)


        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        print ("EP: "+str(x+1)+"| Reward: "+str(cumulated_reward)+" | Steps: "+str(i)+"  | Time: %d:%02d:%02d" % (h, m, s))
	

    #Github table content
    print ("\n|"+str(total_episodes)+"|"+str(es.alpha)+"|"+str(es.gamma)+"|"+str(es.epsilon)+"")

    
    print ("Q table: ")
    print(es.q)
    env.close()
    plotter.show()
