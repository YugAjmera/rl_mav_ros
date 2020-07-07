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
git checkout pid
cd ..
catkin_make
```

### PID

```
roslaunch rl_mav_ros world.launch
```
- go to src/pid folder
- run the givewaypoint_pid.py

