import launch
import launch_ros
import os

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration,PythonExpression,TextSubstitution
from launch.actions import IncludeLaunchDescription

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gui = LaunchConfiguration('use_state_pub_gui')

    pkg_share = launch_ros.substitutions.FindPackageShare(package='simbot_description').find('simbot_description')

    default_model_path = os.path.join(pkg_share, 'src/description/simbot_description.urdf')

    # launch robot_state_publisher to publish URDF and static transforms
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', LaunchConfiguration('model'), TextSubstitution(text=" use_sim:="), use_sim_time ],on_stderr='warn'),
            'use_sim_time': use_sim_time
        }]
    )

    # launch joint_state publisher to handle dynamic joints
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(use_gui),
        parameters=[{
            'source_list': ['base_wheel_joints'],
            'use_sim_time': use_sim_time
        }]
    )

    # optionaly use joint state gui for debugging (if arg use_state_pub_gui = true)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(use_gui),
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    return launch.LaunchDescription([

        DeclareLaunchArgument(name= 'use_sim_time', default_value='false',
                                            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(name='use_state_pub_gui', default_value='False',
                                            description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),

        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node

    ])
