# Autonomous Navigation of MAVs using Reinforcement Learning algorithms

This ROS Package contains the simulation source code for implementing reinforcement learning aglorithms for autonomous navigation of MAVs in indoor environments.

### Dependencies
- Install OpenAI gym and <a href="https://github.com/erlerobot/gym-gazebo">gym_gazebo</a> package:
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
- Install <a href="https://github.com/ethz-asl/rotors_simulator">RotorS simulator</a> package: <br />

```
cd ~/catkin_ws/src
git clone https://github.com/ethz-asl/rotors_simulator.git
```
- Clone this package:
```
git clone https://github.com/YugAjmera/rl_mav_ros
cd ..
catkin_make
```

### Environment
- State: Discrete(X,Y Coordinate obtained from generic odometry sensor).
- Action: Forward, Back, Left, Right.
- Space: 5x5 grid space.
- Goal: [4,5]


### Q-learning

```
roslaunch rl_mav_ros world.launch
roslaunch rl_mav_ros start_qlearning.launch
```
<p align= "center">
<img src="greedy-policy.gif">
</p>


- Results:
   * alpha = 0.8 (learning rate) 
   * gamma = 0.9 (discount factor) 
   * epsilon = 0.1 (𝜖 -greedy action selection)  <br />
   * Results after learning 100 episodes: The agent learns after about 50 episodes and then completes each episode in 8 steps with the optimal reward of 92 always. <br />
   ![](graphs.png) <br />


### Sarsa
```
roslaunch rl_mav_ros world.launch
roslaunch rl_mav_ros start_sarsa.launch
```

### Expected-Sarsa
```
roslaunch rl_mav_ros world.launch
roslaunch rl_mav_ros start_expected_sarsa.launch
```


### References
- <a href="http://wiki.ros.org/openai_ros">openai_ros</a> package
- Pham, Huy X., et al. <b><a href="https://arxiv.org/abs/1801.05086">Autonomous uav navigation using reinforcement learning.</a></b> arXiv preprint arXiv:1801.05086 (2018)

