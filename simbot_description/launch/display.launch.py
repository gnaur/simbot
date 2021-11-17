import launch
import launch_ros
import os

from ament_index_python.packages import get_package_share_directory

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration,TextSubstitution,PathJoinSubstitution
from launch.actions import IncludeLaunchDescription

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_share = get_package_share_directory('simbot_description')

   
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    # run rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{
            'use_sim_time' : use_sim_time
        }]
    )

    # if simulationg launch gazebo
    gazebo_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_share + '/launch/gazebo.launch.py'),
        launch_arguments = {
            'use_sim_time': use_sim_time,
            'world' : PathJoinSubstitution([TextSubstitution(text=os.path.join(pkg_share,'worlds','simbot_worlds','')),LaunchConfiguration('world')])
        }.items(),
        condition=launch.conditions.IfCondition(use_sim_time)
    )

    return LaunchDescription([

        DeclareLaunchArgument(name= 'use_sim_time', default_value='false',
                                            description='Use simulation (Gazebo) clock if true'),      
        DeclareLaunchArgument(name= 'world', default_value='gazebo_ros_ray_sensor_demo.world',
                                            description='If using simulation - gazebo world to launch'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),

        
        rviz_node,
        gazebo_include
    ])
