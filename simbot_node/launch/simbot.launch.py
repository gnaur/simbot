import launch
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node,PushRosNamespace,SetRemap
from launch.substitutions import LaunchConfiguration,TextSubstitution,PathJoinSubstitution

from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    use_sim = LaunchConfiguration('use_sim')
    do_mapping = LaunchConfiguration('mapping')
    map = LaunchConfiguration('map')

    lidar_launch_path = get_package_share_directory('simbot_node')
    simbot_description_pkg_path = get_package_share_directory('simbot_description')
    simbot_navigation_pkg_path = get_package_share_directory('simbot_navigation')

    simbot_node_pkg_path = get_package_share_directory('simbot_node')


    print(simbot_node_pkg_path)
    print(simbot_navigation_pkg_path)

    nav_pkg_path = get_package_share_directory('nav2_bringup')

    map_path = os.path.join(simbot_navigation_pkg_path,'maps','')

    #print(map_path)
    map_file_name=PathJoinSubstitution([TextSubstitution(text=map_path),map])

    default_model_path = os.path.join(simbot_description_pkg_path, 'src/description/simbot_description.urdf')



    rbcl_path=os.path.join(get_package_share_directory('roboclaw2'),'launch','roboclaw2.launch.py')
    rbcl_driver_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rbcl_path),
        condition=launch.conditions.UnlessCondition(use_sim)
    )

   

    lidar_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path + '/launch/sick_tim_xxx.launch.py'),
        launch_arguments = {
            #'config_file' : 'sick_tim_571.yaml',
            'config_file' : 'sick_tim_240.yaml',
            'use_sim_time' : use_sim
        }.items()
    )


    description_include= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simbot_description_pkg_path + '/launch/description.launch.py'),
        launch_arguments = {
            'use_sim_time': use_sim,
            'model': LaunchConfiguration('model'),
            'use_state_pub_gui' : 'false'
        }.items()
    )

    display_include= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simbot_description_pkg_path + '/launch/display.launch.py'),
        launch_arguments = {
            'use_sim_time': use_sim,
            'world': LaunchConfiguration('world'),
        }.items(),
        condition=launch.conditions.IfCondition(use_sim)
    )



    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'default_nav_through_poses_bt_xml': os.path.join(simbot_navigation_pkg_path,"behavior_trees","navigate_through_poses_w_replanning_and_recovery.xml"),
        'default_nav_to_pose_bt_xml': os.path.join(simbot_navigation_pkg_path,"behavior_trees","navigate_to_pose_w_replanning_and_recovery.xml")
    }

    configured_params = RewrittenYaml(
            source_file=os.path.join(simbot_navigation_pkg_path,'params','nav2_params.yaml'),
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True)

    nav_include = GroupAction(
        actions=[
            #PushRosNamespace('nav'),
            #SetRemap(src='/tf',dst='/tf'),
            #SetRemap(src='/tf_static',dst='/tf_static'),
            SetRemap(src='/cmd_vel',dst='/cmd_vel_nav'),

            IncludeLaunchDescription(
                 PythonLaunchDescriptionSource(nav_pkg_path + '/launch/bringup_launch.py'),
                 launch_arguments = {
                       'use_sim_time' : use_sim,
                       'map' : map_file_name,
                       'autostart' : 'true',
                       'slam' : do_mapping,
                       #'namespace' : 'nav',
                       'params_file' : [configured_params]
                       # os.path.join(simbot_navigation_pkg_path,'params','nav2_params.yaml')
                       
                 }.items(),

            )
        ]
    )

    bottom_rgbd_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simbot_node_pkg_path + '/launch/rgbd.launch.py'),
        launch_arguments = {
            'camera_name': "cam_bottom",
            'use_sim_time' : LaunchConfiguration('use_sim'),
        }.items()
    )

    top_rgbd_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simbot_node_pkg_path + '/launch/rgbd.launch.py'),
        launch_arguments = {
            'camera_name': "cam_top",
            'use_sim_time' : LaunchConfiguration('use_sim'),
        }.items()
    )

    teleop_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simbot_node_pkg_path + '/launch/teleop.launch.py'),
        launch_arguments = {
            'joy_config' : LaunchConfiguration('joy_config')   
        }.items()
    )

    simbot_node=Node(
        package='simbot_node',
        executable='simbot_node',
        output='screen',
        name='simbot_node'
    )

    ld= LaunchDescription([

        DeclareLaunchArgument(
            name= 'joy_config', default_value='xbox',
            description='Select joystick type to use'),

        DeclareLaunchArgument(
            name='use_sim', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            name='model', default_value=default_model_path,
            description='Absolute path to robot urdf file'),

        DeclareLaunchArgument(name= 'world', default_value='simbot2.world',
            description='If using simulation - gazebo world to launch'),

        DeclareLaunchArgument(name= 'mapping', default_value='False',
            description='set to true to run mapping mode'),

        DeclareLaunchArgument(name= 'map', default_value='map.yaml',
            description='map file to use for navigation'),

        lidar_include,

        simbot_node,

        teleop_include,
        description_include,
        display_include,
        nav_include,
        bottom_rgbd_include,
        top_rgbd_include,
        rbcl_driver_include,

        ExecuteProcess(
            cmd= [ 'ros2', 'topic', 'pub','-1','/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped', '{header: {frame_id: \'map\'} }'],
            output='screen'),


    ])

    return ld




