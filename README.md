# ROS2 Reinforcement Learning Drive
[![Video Link](https://i.postimg.cc/N09kH4H1/RL-multi-example-2024-11-10-01-05-03-online-video-cutter-com.gif)](https://youtu.be/7srWDqT24MA)

This repository provides a customizable ROS2 environment for training multiple 2D drive cars using Deep Q-Network (DQN). Currently, agents learn collision-free navigation by avoiding obstacles, but the environment is designed to allow easy modification to train for various objectives in the future.

# Occupancy Map launch
```
ros2 launch reinforcement_learing_drive occupancy.launch.py
```

# DQN model run
```
ros2 run reinforcement_learning_drive_model dqn_node
```

# Demo
- [Single Actor Youtube Video](https://youtu.be/kQk4VJepGPk)
- [Multi Actor Youtube Video](https://youtu.be/7srWDqT24MA)
