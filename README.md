# simbot

ROS2 (galactic) Example of robot with gazebo simulation

An example ROS2 galactic robot with a gazebo simulation

![Image of simbot](https://github.com/gnaur/simbot/blob/main/simbot.png)
 
## Launching

To launch an actual robot:


    ros2 launch simbot_node simbot.launch.py map:=<your map yaml file> joy_config:=<your joystick type>


To launch the  robot in gazebo simulation: 


    ros2 launch simbot_node simbot.launch.py use_sim:=true map:=gazebo_house5.yaml joy_config:=<your joystick type>

 
The joystick type can be xbox or xbox360  - ps2 may also work but not tested
 
## Supported Features

It has the gazebo house as an example world for simulation 

It references a roboclaw2 driver - you will need to change this for your robot

The model simulates 2 intel realsense D415 cameras , a SICK TIM 240 lidar and an RGB camera on a differnetial base robot

## Dependencies
For ROS2 Galactic you will need these packages built from src as they are not released in galactic yet

* laser_filters
* pointcloud_to_laserscan
* twist_mux

You also need the realsense  sdk and realsense-ros packages

The launch  file (if not simulating) tries to launch a roboclaw2 driver, change this to the real base driver you have

Use rosdep to install other dependent packages

