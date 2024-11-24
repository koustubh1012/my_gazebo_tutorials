# My_Gazebo_Tutorials

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

## Overview

This repository contains a simple turtorial of how to implement a walker algorithm on a turtlebot3 in ROS2.

The walker algorirth is somewhat similar to the algorithm used in the evry first Roomba robot. The turtlebot will keep moving forward until it encounters an obstacle. It alternates between clockwise and counter clockwise turns untill the obstacle is clear and then it starts moving again.

## Design Pattern

This implemetation has been created using the state pattern design. The state machine dynamically changes the behaviour based on the state of the robot

## Dependencies

1. Ubuntu 22.04
2. ROS2 Humble

## Cloning and Building the package

1. Source ROS2 underlay

   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Create new workspace

   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

3. Clone the repository

   ```bash
   git clone https://github.com/koustubh1012/my_gazebo_tutorials.git
   ```

4. Resolve missing dependencies

   ```bash
   cd ..
   rosdep install -i --from-path src --rosdistro humble -y

   ```

5. Build the workspace with colcon build

   ```bash
   colcon build
   ```

6. Source Overlay

   ```bash
   source install/setup.bash
   ```

## Launching the nodes

1. Launch turtlebot3 gazebo world
  
   ```bash
   echo "export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models/" >> ~/.bashrc
   echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
   source ~/.bashrc
   cd ~/ros2_ws
   source install/setup.bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. Launch the walker node

   In another terminal, run the following command after sourcing.

    ```bash
   ros2 launch walker walker.launch.py
    ```

3. Recording bag file

   If you want to record a bag file recording all the topics, add the record_bag argument as true.

   ```bash
   ros2 launch walker walker.launch.py record_bag:=true
   ```

## Replaying bag file

To replay the bag file, you can use the following command.

```bash
    cd ~/ros2_ws/src/walker/bag_files
    # Usage ros2 bag play <bag_file>
    ros2 bag play rosbag2_2024_11_22-20_53_49/
```

## Output Video

![Demo_Video](/results/demo.gif)
