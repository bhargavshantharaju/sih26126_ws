"""Test launch for minimal stable world - robot dynamics verification."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for minimal world test."""
    pkg_sim = get_package_share_directory('sih26126_sim')
    pkg_bringup = get_package_share_directory('sih26126_bringup')

    # World file
    world_file = os.path.join(pkg_sim, 'worlds', 'minimal_stable.sdf')

    # Gazebo launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_bringup, 'launch', 'simulation.launch.py'),
        ]),
        launch_arguments={
            'world': world_file,
            'gui': 'true',
        }.items(),
    )

    # ROS-Gazebo bridge for essential topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/sih_ugv/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start Gazebo GUI',
        ),
        gz_sim,
        bridge,
    ])
