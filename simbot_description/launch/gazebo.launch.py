import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
            
from launch.actions import DeclareLaunchArgument, ExecuteProcess,IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time')
  world = LaunchConfiguration('world')

  #simbot_pkg_path = get_package_share_directory('simbot_node')

  pkg_gazebo_ros = get_package_share_directory('gazebo_ros')


  return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'world',
            default_value='/opt/ros/galactic/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world',
            description='specify full path and file name of world file to launch'),

        #ExecuteProcess(
        #    cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so', world, '__params:=' + os.path.join(simbot_pkg_path,'config','gazebo.config.yaml')],
        #    shell=True,
        #    output='screen'),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        #ExecuteProcess(
        #    cmd=['ros2', 'param', 'set', '/gazebo', 'config',os.path.join(simbot_pkg_path,'config','gazebo.config.yaml')],
        #    output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
            output='screen'),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", "robot"])
  ])