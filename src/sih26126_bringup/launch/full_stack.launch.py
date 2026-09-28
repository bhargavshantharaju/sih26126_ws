import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('sih26126_sim')
    default_world = f'{sim_share}/worlds/terrain_test.sdf'
    world = LaunchConfiguration('world')
    dashboard_port = LaunchConfiguration('dashboard_port')
    gui = LaunchConfiguration('gui')
    start_dashboard = LaunchConfiguration('start_dashboard')
    gz = shutil.which('gz')
    if gz is None:
        raise RuntimeError('Gazebo executable `gz` is not available on PATH.')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Absolute path to the Gazebo world SDF.',
    )

    dashboard_port_arg = DeclareLaunchArgument(
        'dashboard_port',
        default_value='8080',
        description='Preferred HTTP port for the read-only dashboard.',
    )

    start_dashboard_arg = DeclareLaunchArgument(
        'start_dashboard',
        default_value='true',
        description='Start the dashboard with the full stack.',
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Show the Gazebo GUI when true; otherwise run server-only.',
    )

    gazebo_gui = ExecuteProcess(
        cmd=[
            gz,
            'sim',
            '-r',
            world,
        ],
        condition=IfCondition(gui),
        cwd=sim_share,
        additional_env={
            'GZ_SIM_RESOURCE_PATH': sim_share,
        },
        output='screen',
    )

    gazebo_headless = ExecuteProcess(
        cmd=[
            gz,
            'sim',
            '-s',
            '-r',
            world,
        ],
        condition=UnlessCondition(gui),
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
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/sih_ugv/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
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

    pose_adapter = Node(
        package='sih26126_autonomy',
        executable='odometry_pose_adapter',
        name='localization_pose_adapter',
        output='screen',
    )

    online_mapper = Node(
        package='sih26126_autonomy',
        executable='online_mapper',
        name='online_mapper',
        output='screen',
    )

    frontier_explorer = Node(
        package='sih26126_autonomy',
        executable='frontier_explorer',
        name='frontier_explorer',
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
        remappings=[('/planner/path', '/planner/global_path')],
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
        condition=IfCondition(start_dashboard),
        additional_env={
            'SIH26126_DASHBOARD_PORT': dashboard_port,
        },
    )

    return LaunchDescription([
        world_arg,
        dashboard_port_arg,
        start_dashboard_arg,
        gui_arg,
        gazebo_gui,
        gazebo_headless,

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
                pose_adapter,
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

        TimerAction(
            period=7.0,
            actions=[online_mapper],
        ),

        TimerAction(
            period=9.0,
            actions=[frontier_explorer],
        ),
    ])
