from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    workspace_sim = '/home/venom/sih26126_ws/src/sih26126_sim'
    default_world = f'{workspace_sim}/worlds/terrain_test.sdf'
    world = LaunchConfiguration('world')
    gz = '/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz'

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Absolute path to the Gazebo world SDF.',
    )

    gazebo = ExecuteProcess(
        cmd=[
            gz,
            'sim',
            '-s',
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
            '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/sih_ugv/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
    )

    perception = Node(
        package='sih26126_perception',
        executable='perception_baseline',
        name='perception_baseline',
        output='screen',
    )

    traversability = Node(
        package='sih26126_traversability',
        executable='traversability_baseline',
        name='traversability_baseline',
        output='screen',
    )

    sensor_contract = Node(
        package='sih26126_sensor_contract',
        executable='sensor_contract',
        name='sensor_contract',
        output='screen',
    )

    terrain_geometry = Node(
        package='sih26126_geometry',
        executable='geometry_terrain',
        name='terrain_geometry',
        output='screen',
    )

    terrain_classifier = Node(
        package='sih26126_geometry',
        executable='terrain_classifier',
        name='terrain_classifier',
        output='screen',
    )

    terrain_aware_traversability = Node(
        package='sih26126_traversability',
        executable='terrain_aware_traversability',
        name='terrain_aware_traversability',
        output='screen',
    )

    planner = Node(
        package='sih26126_planner',
        executable='goal_path_planner',
        name='goal_path_planner',
        output='screen',
    )

    local_path_selector = Node(
        package='sih26126_planner',
        executable='local_path_selector',
        name='local_path_selector',
        output='screen',
    )

    safety_supervisor = Node(
        package='sih26126_safety',
        executable='safety_supervisor',
        name='safety_supervisor',
        output='screen',
    )

    dashboard = Node(
        package='sih26126_dashboard',
        executable='dashboard',
        name='sih26126_dashboard',
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        gazebo,

        TimerAction(
            period=5.0,
            actions=[bridge],
        ),

        TimerAction(
            period=6.0,
            actions=[
                perception,
                traversability,
                terrain_aware_traversability,
                planner,
            ],
        ),

        TimerAction(
            period=7.0,
            actions=[dashboard, sensor_contract, terrain_geometry, terrain_classifier],
        ),

        TimerAction(
            period=8.0,
            actions=[local_path_selector, safety_supervisor],
        ),
    ])
