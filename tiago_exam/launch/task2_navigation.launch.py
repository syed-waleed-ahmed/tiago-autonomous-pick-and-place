import os
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    map_path = os.path.expanduser('~/tiago_ws/my_map')

    # Step 1 (t=0s): Gazebo with MoveIt
    gazebo = ExecuteProcess(
        cmd=['ros2', 'launch', 'tiago_exam', 'tiago_exam.launch.py',
             'world_name:=group26', 'moveit:=true'],
        output='screen'
    )

    # Step 2 (t=5s): Odom relay
    odom_relay = TimerAction(period=5.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'run', 'topic_tools', 'relay',
                 '/mobile_base_controller/odom', '/odom',
                 '--ros-args', '-p', 'use_sim_time:=true'],
            output='screen'
        )
    ])

    # Step 3 (t=5s): Scan relay
    scan_relay = TimerAction(period=5.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'run', 'topic_tools', 'relay',
                 '/scan_raw', '/scan',
                 '--ros-args', '-p', 'use_sim_time:=true'],
            output='screen'
        )
    ])

    # Step 4 (t=5s): cmd_vel relay
    cmd_vel_relay = TimerAction(period=5.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'run', 'topic_tools', 'relay',
                 '/nav_vel', '/mobile_base_controller/cmd_vel_unstamped',
                 '--ros-args', '-p', 'use_sim_time:=true'],
            output='screen'
        )
    ])

    # Step 5 (t=15s): Nav2 with map
    nav2 = TimerAction(period=15.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('tiago_2dnav'),
                '/launch/tiago_nav_bringup.launch.py'
            ]),
            launch_arguments={
                'is_public_sim': 'false',
                'rviz': 'True',
                'slam': 'false',
                'map_path': map_path,
                'use_sim_time': 'true',
            }.items()
        )
    ])

    # Step 6 (t=25s): ArUco station markers (ID 26, 238)
    aruco_station = TimerAction(period=25.0, actions=[
        Node(
            package='aruco_ros',
            executable='marker_publisher',
            name='marker_publisher',
            output='screen',
            parameters=[
                {'image_is_rectified': True},
                {'marker_size': 0.25},
                {'reference_frame': 'base_footprint'},
                {'camera_frame': 'head_front_camera_rgb_optical_frame'},
                {'use_sim_time': True},
            ],
            remappings=[
                ('/image', '/head_front_camera/rgb/image_raw'),
                ('/camera_info', '/head_front_camera/rgb/camera_info'),
                ('/marker_publisher/markers', '/aruco_station/markers'),
            ]
        )
    ])

    # Step 7 (t=35s): Task 2 navigator (with teleport)
    explorer = TimerAction(period=35.0, actions=[
        Node(
            package='tiago_exam',
            executable='task2_navigator.py',
            name='task2_navigator',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'test_teleport': True},
                {'test_teleport_x': -1.7},
                {'test_teleport_y': -3.5},
                {'test_teleport_yaw': 0.0},
            ]
        )
    ])

    # Step 8 (t=35s): rqt_image_view (optional, for camera monitoring)
    rqt_image = TimerAction(period=35.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view'],
            output='screen'
        )
    ])

    return LaunchDescription([
        gazebo,
        odom_relay,
        scan_relay,
        cmd_vel_relay,
        nav2,
        aruco_station,
        explorer,
        rqt_image,
    ])
