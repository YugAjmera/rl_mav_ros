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
import qlearn


if __name__ == '__main__':

    rospy.init_node('qlearning', anonymous=True)
    env = gym.make('MavEnv-v0')
    print "Gym Make Done"


    outdir = '/tmp/gazebo_gym_experiments'
    env = gym.wrappers.Monitor(env, outdir, force=True)
    print "Monitor Wrapper Started"

    plotter = liveplot.LivePlot(outdir)

    last_time_steps = numpy.ndarray(0)

    qlearn = qlearn.QLearn(actions=range(env.action_space.n),
                    alpha=0.8, gamma=0.9, epsilon=0.1)
	

    start_time = time.time()
    total_episodes = 5
 
    for x in range(total_episodes):
        done = False

        cumulated_reward = 0 

        print("Episode = " +str(x)+ " started")

        observation, done = env.reset()

        state = ''.join(map(str, observation))

	i = 0
	while(True):

            # Pick an action based on the current state
            action = qlearn.chooseAction(state)
            
            # Execute the action and get feedback
            observation, reward, done, info = env.stepup(observation, action)
            cumulated_reward += reward


            nextState = ''.join(map(str, observation))

            qlearn.learn(state, action, reward, nextState)

            env._flush(force=True)

            if not(done):
                state = nextState
            else:
            	last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
                break

	    i = i+1

	if x%1==0:
		plotter.plot(env)


        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        print ("EP: "+str(x+1)+"| Reward: "+str(cumulated_reward)+"  | Time: %d:%02d:%02d" % (h, m, s))
	print ("Q table: ")
	print(qlearn.q)

    #Github table content
    print ("\n|"+str(total_episodes)+"|"+str(qlearn.alpha)+"|"+str(qlearn.gamma)+"|"+str(qlearn.epsilon)+"")

    l = last_time_steps.tolist()
    l.sort()

    #print("Parameters: a="+str)
    print("Overall score: {:0.2f}".format(last_time_steps.mean()))
    print("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-100:]) / len(l[-100:])))
    env.close()
    plotter.show()
