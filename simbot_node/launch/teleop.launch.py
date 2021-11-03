
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration,TextSubstitution


def generate_launch_description():

    simbot_node_path = get_package_share_directory('simbot_node')

    joy_config = LaunchConfiguration('joy_config')

    config_locks = os.path.join(simbot_node_path, 'config', 'twist_mux_locks.yaml')
    config_topics = os.path.join(simbot_node_path, 'config', 'twist_mux_topics.yaml')

    joy_pkg="joy"

    try:
        get_package_share_directory('joy_linux')
        joy_pkg = "joy_linux"
    except:
        pass

    teleop_config_file=[TextSubstitution(text=os.path.join(simbot_node_path,'config','')),joy_config,TextSubstitution(text='.config.yaml')]

    joy_node=Node(
        package=joy_pkg,
        executable=joy_pkg + '_node',
        output='screen',
        name='joystick',
        parameters=[teleop_config_file]
    )

    teleop_twist=Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        output='screen',
        name='teleop',
        parameters=[teleop_config_file],
        remappings=[('/cmd_vel','cmd_vel_joy')]
    )

    mux_node=Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            remappings={('/cmd_vel_out', LaunchConfiguration('cmd_vel_topic'))},
            parameters=[config_locks,config_topics]
    )

    return LaunchDescription([
        DeclareLaunchArgument(name= 'joy_config', default_value='xbox360',
                              description='Select joystick type to use'),
        DeclareLaunchArgument(name= 'cmd_vel_topic', default_value='/cmd_vel',
                              description='Set output cmd_vel topic name'),
        teleop_twist,
        joy_node,
        mux_node
    ])


