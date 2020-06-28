# Autonomous Navigation of MAVs using Reinforcement Learning algorithms

This repository contains the simulation source code for implementing reinforcement learning aglorithms for autonomous navigation of MAVs in indoor environments. Used RotorS, OpenAI gym and Gazebo for simulation.

### Dependencies
- ROS Melodic
- Gazebo 9
- Install OpenAI gym and gym_gazebo :
```
sudo pip install gym
sudo apt-get install python-skimage
sudo pip install h5py
pip install tensorflow-gpu (if you have a gpu if not then just pip install tensorflow)
sudo pip install keras

cd ~
git clone https://github.com/erlerobot/gym-gazebo
cd gym-gazebo
sudo pip install -e .
```

### Q-Learning
- State: Discrete(X,Y Coordinate obtained from generic odometry sensor).
- Action: Forward, Back, Left, Right.
- Space: 5x5 grid space.


```
roslaunch rl world.launch
roslaunch rl start_qlearning.launch
```

