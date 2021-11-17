# simbot

ROS2 (galactic) Example of robot with gazebo simulation

An example ROS2 galactic robot with a gazebo simulation

![Image of simbot in gazebo](https://github.com/gnaur/simbot/blob/main/simbot_gazebo.png)

![Image of simbot](https://github.com/gnaur/simbot/blob/main/simbot.png)
 


## Feedback

Please feel free to open discussions in the discussions tab.

Contributions via pull request are welcome.

Repot bugs in the Issues tab.


## Launching


### Navigation Mode

To run and navigate the robot in a pre mapped space use these commands:


To launch an actual robot:


    ros2 launch simbot_node simbot.launch.py map:=<your map yaml file> joy_config:=<your joystick type>


To launch the  robot in gazebo simulation: 


    ros2 launch simbot_node simbot.launch.py use_sim:=true map:=<your map yaml file>  joy_config:=<your joystick type> [world:=<gazebo world file>]

An example map for the default simulated environment is = gazebo_house5.yaml

You can override the default simulation model by providing the name of an alternative world file by using the optional world:=<gazebo world file> parameter
 
World files should be saved in simbot_description/worlds/simbot_worlds/
 
### Mapping Mode

To create a map of an environment use these commands:


To launch an actual robot:


    ros2 launch simbot_node simbot.launch.py mappping:=True joy_config:=<your joystick type>


To launch the  robot in gazebo simulation: 


    ros2 launch simbot_node simbot.launch.py use_sim:=true mappping:=True joy_config:=<your joystick type> [world:=<gazebo world file>]
    
    
Done forget to save the map with the standard map_server tools after you are done.

Maps should be saved to the  'simbot_navigation/maps' folder.


As above the joystick type can be specified and also the optional simulation world file can be used.

 
## Supported Features

It has the gazebo house as an example world for simulation 

It references a roboclaw2 driver - you will need to change this for your robot

The model simulates:
 * Intel realsense D415 camera
 * RGB camera
 * Lidar
 * Differential drive robot base

The depth images from the realsense are converted to laser scan and can be used for obstical avoidance along with the lidar.

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





