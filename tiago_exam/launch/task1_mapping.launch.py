import os
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    map_save_path = os.path.expanduser('~/tiago_ws/map')
    os.makedirs(os.path.dirname(map_save_path), exist_ok=True)

    gazebo = TimerAction(
        period=0.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'launch', 'tiago_exam', 'tiago_exam.launch.py',
                     'world_name:=group26'],
                output='screen'
            )
        ]
    )

    fold_arm = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'action', 'send_goal', '/arm_controller/follow_joint_trajectory',
                    'control_msgs/action/FollowJointTrajectory',
                    '{"trajectory":{"joint_names":["arm_1_joint","arm_2_joint","arm_3_joint","arm_4_joint","arm_5_joint","arm_6_joint","arm_7_joint"],"points":[{"positions":[2.6,-1.5,0.6,2.0,1.2,-1.39,2.0],"time_from_start":{"sec":5}}]}}'
                ],
                output='screen'
            )
        ]
    )

    odom_relay = TimerAction(
        period=25.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'topic_tools', 'relay',
                     '/mobile_base_controller/odom', '/odom',
                     '--ros-args', '-p', 'use_sim_time:=true'],
                output='screen'
            )
        ]
    )

    cmd_vel_relay = TimerAction(
        period=30.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'topic_tools', 'relay',
                     '/nav_vel', '/mobile_base_controller/cmd_vel_unstamped',
                     '--ros-args', '-p', 'use_sim_time:=true'],
                output='screen'
            )
        ]
    )

    nav2_slam = TimerAction(
        period=40.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('tiago_2dnav'),
                    '/launch/tiago_nav_bringup.launch.py'
                ]),
                launch_arguments={
                    'is_public_sim': 'false',
                    'rviz': 'True',
                    'slam': 'True',
                    'use_sim_time': 'true',
                }.items()
            )
        ]
    )

    explore_lite = TimerAction(
        period=55.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('explore_lite'),
                    '/launch/explore.launch.py'
                ]),
                launch_arguments={
                    'use_sim_time': 'true',
                }.items()
            )
        ]
    )

    map_saver = TimerAction(
        period=475.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                    '-f', map_save_path,
                    '--ros-args', '-p', 'save_map_timeout:=30.0',
                    '-p', 'use_sim_time:=true'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        fold_arm,
        odom_relay,
        cmd_vel_relay,
        nav2_slam,
        explore_lite,
        map_saver,
    ])
