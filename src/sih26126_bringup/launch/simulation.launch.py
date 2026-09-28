import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('sih26126_sim')
    world = f'{sim_share}/worlds/ugv_test.sdf'
    gz = shutil.which('gz')
    if gz is None:
        raise RuntimeError('Gazebo executable `gz` is not available on PATH.')

    gazebo = ExecuteProcess(
        cmd=[
            gz,
            'sim',
            '-r',
            world,
        ],
        cwd=sim_share,
        additional_env={
            'GZ_SIM_RESOURCE_PATH': sim_share,
        },
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='sih26126_bridge',
        output='screen',
        arguments=[
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/sih_ugv/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
    )

    return LaunchDescription([
        gazebo,
        TimerAction(
            period=5.0,
            actions=[bridge],
        ),
    ])
