"""Bringup for the Pioneer P3-DX digital twin: CoppeliaSim bridge, keepout filter,
slam_toolbox, Nav2 and ROS-TCP-Endpoint (Unity).

Examples:
    ros2 launch vr_robot_pkg vr_robot_bringup.launch.py
    ros2 launch vr_robot_pkg vr_robot_bringup.launch.py mock_mode:=true use_rviz:=true
    ros2 launch vr_robot_pkg vr_robot_bringup.launch.py coppelia_host:=172.23.224.1 use_nav2:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('vr_robot_pkg')
    nav2_params = PathJoinSubstitution([pkg_share, 'config', 'nav2_params_jazzy.yaml'])
    slam_params = PathJoinSubstitution([pkg_share, 'config', 'mapper_params_online_async.yaml'])
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

    args = [
        DeclareLaunchArgument('mock_mode', default_value='false',
                              description='Simulate the robot in-process instead of CoppeliaSim'),
        DeclareLaunchArgument('coppelia_host', default_value='',
                              description='CoppeliaSim host; empty = detect the Windows host'),
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
    ]

    bridge = Node(
        package='vr_robot_pkg',
        executable='coppelia_bridge_node',
        name='coppelia_bridge',
        output='screen',
        parameters=[{
            'mock_mode': mock_mode,
            'coppelia_host': coppelia_host,
            'camera_enabled': camera_enabled,
            'coppelia_display': coppelia_display,
        }],
    )

    # Keepout filter: mask publisher (Stage 5 supervisor replaces it) + filter info server.
    keepout_mask = Node(
        package='vr_robot_pkg',
        executable='keepout_mask_node',
        name='keepout_mask_node',
        output='screen',
    )
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

    tcp_endpoint = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='ros_tcp_endpoint',
        output='screen',
        parameters=[{'ROS_IP': '0.0.0.0', 'ROS_TCP_PORT': tcp_port}],
        condition=IfCondition(use_tcp_endpoint),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='log',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(args + [
        bridge, keepout_mask, filter_info_server, filter_lifecycle,
        slam, nav2, tcp_endpoint, rviz,
    ])
