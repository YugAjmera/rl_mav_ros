# Autonomous Navigation of MAVs using Reinforcement Learning algorithms

This repository contains the simulation source code for implementing reinforcement learning aglorithms for autonomous navigation of MAVs in indoor environments. Used RotorS, OpenAI gym and Gazebo for simulation.

### Q-Learning
- State: Discrete(X,Y Coordinate obtained from generic odometry sensor).
- Action: Forward, Back, Left, Right.
- Space: 5x5 grid space.


```
roslaunch rl world.launch
roslaunch rl start_qlearning.launch
```

