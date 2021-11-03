# simbot

ROS2 (galactic) Example of robot with gazebo simulation

An example ROS2 galactic robot with a gazebo simulation

![Image of simbot in gazebo](https://github.com/gnaur/simbot/blob/main/simbot_gazebo.png)

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

The model simulates:
 * 2 intel realsense D415 cameras
 * SICK TIM 240 lidar
 * RGB camera
 * Differnetial drive base robot

The two depth images from the realsense are converted to laser scans and can be used for obstical avoidance along with the lidar.

The package supports the Navigation2 framework for SLAM mapping and Navigation

When not simulationg in gazebo the package can be used to run a real robot.

The packages assumes you have a roboclaw2 motor driver and a SICK TIM lidar.

If you are using different devices on your real robot you will need to modify the launch files to include drivers for the devices you are using.

## Dependencies
For ROS2 Galactic you will need these packages built from src as they are not released in galactic yet

* laser_filters
* pointcloud_to_laserscan
* twist_mux

You also need the realsense sdk and realsense-ros packages

Use rosdep to install other dependent packages





