"""Minimal world test - standalone launch."""

import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    """Launch minimal world for testing."""
    sim_share = get_package_share_directory('sih26126_sim')
    world_file = f'{sim_share}/worlds/minimal_stable.sdf'

    gz = shutil.which('gz')
    if gz is None:
        raise RuntimeError('Gazebo `gz` not found on PATH')

    # Gazebo sim
    gazebo = ExecuteProcess(
        cmd=[gz, 'sim', '-r', '-v', '4', world_file],
        cwd=sim_share,
        additional_env={'GZ_SIM_RESOURCE_PATH': f'{sim_share}/models'},
        output='screen',
    )

    # Minimal bridge for testing
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='test_bridge',
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
        gazebo,
        TimerAction(period=3.0, actions=[bridge]),
    ])
