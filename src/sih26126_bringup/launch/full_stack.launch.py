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
    exploration_radius = LaunchConfiguration('exploration_radius_m')
    frontier_threshold = LaunchConfiguration('frontier_threshold')
    minimum_completed_frontiers = LaunchConfiguration('minimum_completed_frontiers')
    minimum_explored_area = LaunchConfiguration('minimum_explored_area_m2')
    mission_name = LaunchConfiguration('mission_name')
    mission_timeout = LaunchConfiguration('mission_timeout_sec')
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

    exploration_radius_arg = DeclareLaunchArgument(
        'exploration_radius_m',
        default_value='1000.0',
        description='Bounded exploration radius from the captured home pose.',
    )
    frontier_threshold_arg = DeclareLaunchArgument(
        'frontier_threshold', default_value='0',
        description='Raw frontier cell count treated as exhausted below this value.',
    )
    minimum_completed_frontiers_arg = DeclareLaunchArgument(
        'minimum_completed_frontiers', default_value='3',
        description='Minimum count of reached frontier targets before returning.',
    )
    minimum_explored_area_arg = DeclareLaunchArgument(
        'minimum_explored_area_m2', default_value='24.0',
        description='Minimum observed map area before the return-home phase.',
    )
    mission_name_arg = DeclareLaunchArgument(
        'mission_name', default_value='autonomous_outdoor_mission',
        description='Name recorded in the mission result artifact.',
    )
    mission_timeout_arg = DeclareLaunchArgument(
        'mission_timeout_sec', default_value='600.0',
        description='Failure cutoff in seconds; never used as exploration completion.',
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
            '/ugv_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
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
        parameters=[{'exploration_radius_m': exploration_radius}],
    )

    mission_manager = Node(
        package='sih26126_autonomy',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[{
            'frontier_threshold': frontier_threshold,
            'minimum_completed_frontiers': minimum_completed_frontiers,
            'minimum_explored_area': minimum_explored_area,
            'mission_name': mission_name,
            'mission_timeout_sec': mission_timeout,
        }],
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
        exploration_radius_arg,
        frontier_threshold_arg,
        minimum_completed_frontiers_arg,
        minimum_explored_area_arg,
        mission_name_arg,
        mission_timeout_arg,
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
            actions=[frontier_explorer, mission_manager],
        ),
    ])
