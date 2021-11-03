# simbot

ROS2 (galactic) Example of robot with gazebo simulation

An example ROS2 galactic robot with a gazebo simulation
 
## Launching

To launch an actual robot:

...
ros2 launch simbot_node simbot.launch.py map:=<your map yaml file> joy_config:=<your joystick type>
...

to launch the  robot in gazebo simulation: 

...
ros2 launch simbot_node simbot.launch.py use_sim:=true map:=gazebo_house5.yaml joy_config:=<your joystick type>
...
 
The joystick type can be xbox or xbox360  - ps2 may also work but not tested
 

It has the gazebo house as an example world for simulation 

It references a roboclaw2 driver - you will need to change this for your robot

The model simulates 2 intel realsense D415 cameras , a SICK TIM 240 lidar and an RGB camera on a differnetial base robot
