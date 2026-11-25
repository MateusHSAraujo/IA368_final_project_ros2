"""Bringup for the Pioneer P3-DX digital twin: CoppeliaSim bridge, mission supervisor
(missions, keepout filter mask, saved maps), twist_mux (Unity teleop overrides Nav2),
slam_toolbox, Nav2 and ROS-TCP-Endpoint (Unity).

Examples:
    ros2 launch vr_robot_pkg vr_robot_bringup.launch.py
    ros2 launch vr_robot_pkg vr_robot_bringup.launch.py map:=scene_house
    ros2 launch vr_robot_pkg vr_robot_bringup.launch.py mock_mode:=true use_rviz:=true
    ros2 launch vr_robot_pkg vr_robot_bringup.launch.py coppelia_host:=172.23.224.1 use_nav2:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('vr_robot_pkg')
    nav2_params = PathJoinSubstitution([pkg_share, 'config', 'nav2_params_jazzy.yaml'])
    slam_params = PathJoinSubstitution([pkg_share, 'config', 'mapper_params_online_async.yaml'])
    twist_mux_params = PathJoinSubstitution([pkg_share, 'config', 'twist_mux.yaml'])
    rviz_config = PathJoinSubstitution([pkg_share, 'config', 'vr_robot.rviz'])

    mock_mode = LaunchConfiguration('mock_mode')
    coppelia_host = LaunchConfiguration('coppelia_host')
    camera_enabled = LaunchConfiguration('camera_enabled')
    coppelia_display = LaunchConfiguration('coppelia_display')
    use_slam = LaunchConfiguration('use_slam')
    use_nav2 = LaunchConfiguration('use_nav2')
    nav2_delay = LaunchConfiguration('nav2_delay')
    use_tcp_endpoint = LaunchConfiguration('use_tcp_endpoint')
    tcp_port = LaunchConfiguration('tcp_port')
    use_rviz = LaunchConfiguration('use_rviz')
    # Parameter substitutions are parsed as YAML: force strings, so that e.g. map:=2026 or
    # map:=on does not reach the node as an int/bool (InvalidParameterTypeException).
    map_name = ParameterValue(LaunchConfiguration('map'), value_type=str)
    maps_dir = ParameterValue(LaunchConfiguration('maps_dir'), value_type=str)

    args = [
        DeclareLaunchArgument('mock_mode', default_value='false',
                              description='Simulate the robot in-process instead of CoppeliaSim'),
        DeclareLaunchArgument('coppelia_host', default_value='',
                              description='CoppeliaSim host; empty = detect it (Windows host from WSL, 127.0.0.1 on native Linux)'),
        DeclareLaunchArgument('camera_enabled', default_value='false',
                              description='Stream the front camera from the start (/camera/enable toggles it)'),
        DeclareLaunchArgument('coppelia_display', default_value='false',
                              description="Keep CoppeliaSim's window rendering on (halves the sim speed in scene_house)"),
        DeclareLaunchArgument('use_slam', default_value='true'),
        DeclareLaunchArgument('use_nav2', default_value='true'),
        DeclareLaunchArgument('nav2_delay', default_value='8.0',
                              description='Seconds to wait for SLAM before starting Nav2'),
        DeclareLaunchArgument('use_tcp_endpoint', default_value='true',
                              description='Start ROS-TCP-Endpoint for Unity'),
        DeclareLaunchArgument('tcp_port', default_value='10000'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('map', default_value='',
                              description='Saved map to load at startup (e.g. scene_house); '
                                          'empty = map from scratch'),
        DeclareLaunchArgument('maps_dir', default_value='',
                              description='Saved maps folder; empty = <repo>/maps'),
    ]

    bridge = Node(
        package='vr_robot_pkg',
        executable='coppelia_bridge_node',
        name='coppelia_bridge',
        output='screen',
        parameters=[{
            'mock_mode': mock_mode,
            'coppelia_host': ParameterValue(coppelia_host, value_type=str),
            'camera_enabled': camera_enabled,
            'coppelia_display': coppelia_display,
        }],
    )

    # Mission supervisor: Nav2 action client for Unity, keepout mask publisher, saved maps.
    supervisor = Node(
        package='vr_robot_pkg',
        executable='mission_supervisor_node',
        name='mission_supervisor',
        output='screen',
        parameters=[{'initial_map': map_name, 'maps_dir': maps_dir}],
    )
    # Keepout filter plumbing (the mask itself comes from the supervisor).
    filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        parameters=[nav2_params],
    )
    # Started a few seconds late: at t=0 its first 'configure' request was sometimes lost
    # while the nodes were still discovering each other, leaving the server unconfigured.
    filter_lifecycle = TimerAction(period=3.0, actions=[Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap_filters',
        output='screen',
        parameters=[nav2_params],
    )])

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])),
        launch_arguments={
            'slam_params_file': slam_params,
            # The bridge stamps scans and TF with wall time.
            'use_sim_time': 'false',
        }.items(),
        condition=IfCondition(use_slam),
    )

    nav2 = TimerAction(
        period=nav2_delay,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])),
            launch_arguments={
                'params_file': nav2_params,
                'use_sim_time': 'false',
                'autostart': 'true',
            }.items(),
        )],
        condition=IfCondition(use_nav2),
    )

    # Muxes Nav2 (cmd_vel_nav_safe, priority 10) and Unity teleop (cmd_vel_teleop, priority 100)
    # into /cmd_vel. Always started: teleop must work with use_nav2:=false.
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_params],
        remappings=[('cmd_vel_out', '/cmd_vel')],
    )

    # ROS-TCP-Endpoint sometimes exits ([Errno 9] Bad file descriptor) when Unity leaves Play
    # mode; respawn keeps teleop sessions alive.
    tcp_endpoint = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='ros_tcp_endpoint',
        output='screen',
        parameters=[{'ROS_IP': '0.0.0.0', 'ROS_TCP_PORT': tcp_port}],
        condition=IfCondition(use_tcp_endpoint),
        respawn=True,
        respawn_delay=2.0,
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='log',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(args + [
        bridge, supervisor, filter_info_server, filter_lifecycle,
        twist_mux, slam, nav2, tcp_endpoint, rviz,
    ])
