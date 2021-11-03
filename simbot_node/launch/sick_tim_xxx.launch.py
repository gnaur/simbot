import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution

config_arg=DeclareLaunchArgument(
        name= 'config_file', default_value='sick_tim_240.yaml',
        description='Select lidar config file to use')

use_sim_arg = DeclareLaunchArgument(
            name='use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true')

use_sim = LaunchConfiguration('use_sim_time')

def generate_launch_description():

    ld = LaunchDescription()

    config_file=[TextSubstitution(text=os.path.join(get_package_share_directory('simbot_node'),'config','')),LaunchConfiguration('config_file')]

    node=Node(
        package='sick_scan2',
        name = 'sick_scan2', # 'sick_scan2_tim_240', # For compatibility with ros versions previous to foxy, node name changed to sick_scan2 for all supported scanner. The type of scanner is configured by scanner_name in the yaml config file.
        #node_executable='sick_generic_caller',       # ROS versions eloquent and earlier require "node_executable", ROS foxy and later use "executable".
        executable='sick_generic_caller',          # Please use executable='sick_generic_caller', if ROS2 can't launch sick_generic_caller.
        output='screen',
        parameters = [config_file],
        remappings=[('scan','scan_raw')],
        condition=launch.conditions.UnlessCondition(use_sim)
    )


    laser_filter=Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        output='screen',
        name='lidar_filters',
        parameters=[
           config_file,
           {'use_sim_time' : use_sim }
        ],
        remappings=[('scan','scan_raw'),
                   ('scan_filtered','scan')
        ]
    )

    ld.add_action(use_sim_arg)
    ld.add_action(config_arg)
    ld.add_action(node)
    ld.add_action(laser_filter)
    return ld
