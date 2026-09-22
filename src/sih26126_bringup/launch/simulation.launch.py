from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    workspace_sim = '/home/venom/sih26126_ws/src/sih26126_sim'
    world = f'{workspace_sim}/worlds/ugv_test.sdf'
    gz = '/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz'

    gazebo = ExecuteProcess(
        cmd=[
            gz,
            'sim',
            '-r',
            world,
        ],
        cwd=workspace_sim,
        additional_env={
            'GZ_SIM_RESOURCE_PATH': workspace_sim,
        },
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='sih26126_bridge',
        output='screen',
        arguments=[
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        ],
    )

    return LaunchDescription([
        gazebo,
        TimerAction(
            period=5.0,
            actions=[bridge],
        ),
    ])
