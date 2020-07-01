#!/usr/bin/env python
import gym
import rospy
import mav_env
from gym import wrappers
import gym_gazebo
import time
import random
import time
import liveplot
import sarsa


if __name__ == '__main__':	

    rospy.init_node('sarsa', anonymous=True)
    env = gym.make('MavEnv-v0')
    print "Gym Make Done"

    outdir = '/tmp/gazebo_gym_experiments'
    env = gym.wrappers.Monitor(env, outdir, force=True)
    print "Monitor Wrapper Started"

    plotter = liveplot.LivePlot(outdir)


    sarsa = sarsa.Sarsa(actions=range(env.action_space.n),
                    epsilon=0.1, alpha=0.8, gamma=0.9)

    start_time = time.time()

    total_episodes = 10
    

    for x in range(total_episodes):
        done = False

        cumulated_reward = 0 
	print("Episode = " +str(x)+ " started")
        observation = env.reset()

        #render() #defined above, not env.render()

        state = ''.join(map(str, observation))

        i = 0
        #for i in range(300):
	while(True):

            # Pick an action based on the current state
            action = sarsa.chooseAction(state)

            # Execute the action and get feedback
            observation, reward, done, info = env.step(action)
            cumulated_reward += reward


            nextState = ''.join(map(str, observation))
            nextAction = sarsa.chooseAction(nextState)

            #sarsa.learn(state, action, reward, nextState)
            sarsa.learn(state, action, reward, nextState, nextAction)

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
    print ("\n|"+str(total_episodes)+"|"+str(sarsa.alpha)+"|"+str(sarsa.gamma)+"|"+str(sarsa.epsilon)+"")

    
    print ("Q table: ")
    print(sarsa.q)
    env.close()
    plotter.show()
