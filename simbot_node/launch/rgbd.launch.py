import os
import launch

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, GroupAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node,PushRosNamespace,SetRemap
from launch.substitutions import LaunchConfiguration,PythonExpression,TextSubstitution,PathJoinSubstitution


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('simbot_node'),
        'config',
        'rgbd.yaml'
    )

    ld = LaunchDescription([
        DeclareLaunchArgument(name='rgbd_config_file', default_value=config, description="Camera config parameters file"),
        DeclareLaunchArgument(name='use_sim_time', default_value="False", description="True if doing simulation"),
        DeclareLaunchArgument(name='camera_name', default_value="rgbd_camera", description="camera name") 
    ])

    realsense_pkg_path=get_package_share_directory('realsense2_camera')

    camera_name = LaunchConfiguration('camera_name')

    print(config)

    
    use_sim = LaunchConfiguration('use_sim_time')

    config_file = LaunchConfiguration('rgbd_config_file')

    # launch node to convert depth image to a lidar scan as well
    pc_to_ls = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        output='screen',
        namespace=camera_name,
        name='pc_to_ls',
        parameters=[
            {'use_sim_time' : use_sim },
            [config_file]
        ],
        remappings=[('cloud_in','depth/color/points'),
                   ('scan','depth_scan')
        ]
    )
    
    li=LogInfo(msg=use_sim)

    # luanch the realsense driver
    realsense_include=GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(realsense_pkg_path,  'launch', 'rs_launch.py')),
            launch_arguments = {
                'camera_name': camera_name,
                'config_file': [ TextSubstitution(text= '"'), config_file , TextSubstitution(text= '"') ]
            }.items(),
            condition=launch.conditions.UnlessCondition([use_sim])

        )
    ])

    ld.add_action(realsense_include)
    ld.add_action(pc_to_ls)
    ld.add_action(li)


    return ld
    
