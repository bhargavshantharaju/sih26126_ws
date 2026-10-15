import fcntl
import os
import shutil
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    AndSubstitution, LaunchConfiguration, NotSubstitution,
    PathJoinSubstitution, PythonExpression,
)
from launch_ros.actions import Node


_STACK_LOCK_FD = None


def acquire_stack_lock():
    """Prevent duplicate full stacks from sharing Gazebo and ROS topics."""
    global _STACK_LOCK_FD
    if getattr(sys, '_dristhi_stack_lock_fd', None) is not None:
        return  # held by this process; launch re-imports this module per include
    lock_path = '/tmp/dristhi_full_stack.lock'
    descriptor = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o600)
    try:
        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        os.close(descriptor)
        raise RuntimeError(
            'Another DRISTHI full_stack launch is already running; stop it '
            'before starting a second Gazebo/autonomy stack.'
        ) from exc
    _STACK_LOCK_FD = descriptor
    sys._dristhi_stack_lock_fd = descriptor


def generate_launch_description():
    """
    Generate the full-stack launch description.

    Launch modes
    ------------
    slam:=false  fusion:=false  — baseline Gazebo odometry only
    slam:=true   fusion:=false  — SLAM + plain odometry TF  (Objective 2)
    slam:=true   fusion:=true   — SLAM + EKF sensor fusion  (Objective 3)

    TF authority when fusion:=true
    --------------------------------
    map  -> odom       : slam_toolbox
    odom -> base_link  : robot_localization EKF  (odom_tf_publisher disabled)
    base_link -> lidar : lidar_static_tf_publisher (static)
    base_link -> imu   : imu_static_tf_publisher   (static)
    """
    acquire_stack_lock()
    loc_share = get_package_share_directory('sih26126_localization')
    sim_share = get_package_share_directory('sih26126_sim')
    bringup_share = get_package_share_directory('sih26126_bringup')
    default_world = f'{sim_share}/worlds/showcase_terrain.sdf'

    world = LaunchConfiguration('world')
    dashboard_port = LaunchConfiguration('dashboard_port')
    gui = LaunchConfiguration('gui')
    slam = LaunchConfiguration('slam')
    fusion = LaunchConfiguration('fusion')
    start_dashboard = LaunchConfiguration('start_dashboard')
    exploration_radius = LaunchConfiguration('exploration_radius_m')
    frontier_threshold = LaunchConfiguration('frontier_threshold')
    minimum_completed_frontiers = LaunchConfiguration('minimum_completed_frontiers')
    minimum_explored_area = LaunchConfiguration('minimum_explored_area_m2')
    mission_name = LaunchConfiguration('mission_name')
    mission_timeout = LaunchConfiguration('mission_timeout_sec')
    scenario = LaunchConfiguration('scenario')
    # Backward compatibility: dynamic_obstacle_demo is deprecated but still supported
    dynamic_obstacle_demo = LaunchConfiguration('dynamic_obstacle_demo')
    gui_stream = LaunchConfiguration('gui_stream')
    # Determine if we're running the dynamic obstacle scenario
    # Support both new scenario:= and legacy dynamic_obstacle_demo:= parameters
    is_dynamic_obstacle = PythonExpression([
        '"', scenario, '" == "dynamic_obstacle" or "',
        dynamic_obstacle_demo, '" == "true"'
    ])
    is_showcase = NotSubstitution(is_dynamic_obstacle)

    gz = shutil.which('gz')
    if gz is None:
        raise RuntimeError('Gazebo executable `gz` is not available on PATH.')

    # ------------------------------------------------------------------ args
    world_arg = DeclareLaunchArgument(
        'world', default_value=default_world,
        description='Absolute path to the Gazebo world SDF.',
    )
    dashboard_port_arg = DeclareLaunchArgument(
        'dashboard_port', default_value='8080',
        description='Preferred HTTP port for the read-only dashboard.',
    )
    start_dashboard_arg = DeclareLaunchArgument(
        'start_dashboard', default_value='true',
        description='Start the dashboard with the full stack.',
    )
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='false',
        description='Show the Gazebo GUI when true; otherwise run server-only.',
    )
    slam_arg = DeclareLaunchArgument(
        'slam', default_value='false',
        description='Use SLAM for localization instead of Gazebo odometry.',
    )
    fusion_arg = DeclareLaunchArgument(
        'fusion', default_value='false',
        description=(
            'Enable robot_localization EKF to fuse wheel odometry + IMU '
            'for odom->base_link. Requires slam:=true. '
            'When true the plain odom_tf_publisher is disabled.'
        ),
    )
    exploration_radius_arg = DeclareLaunchArgument(
        'exploration_radius_m', default_value='1000.0',
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
        'mission_timeout_sec', default_value='1800.0',
        description='Failure cutoff in seconds; never used as exploration completion.',
    )
    scenario_arg = DeclareLaunchArgument(
        'scenario', default_value='showcase',
        description=(
            'Demonstration scenario: "showcase" (default integrated demo) or '
            '"dynamic_obstacle" (experimental Objective 4 scenario).'
        ),
    )
    # Backward compatibility
    dynamic_obstacle_demo_arg = DeclareLaunchArgument(
        'dynamic_obstacle_demo', default_value='false',
        description='[Deprecated] Use scenario:= instead. Enable Objective 4 scenario.',
    )
    gui_stream_arg = DeclareLaunchArgument(
        'gui_stream', default_value='false',
        description=(
            'Embed the Gazebo GUI inside the DRISTHI browser dashboard. '
            'Requires: xvfb, x11vnc, websockify, novnc.  '
            'Starts gz sim -g on a virtual display and streams it via noVNC.  '
            'All traffic is localhost-only; the Gazebo simulation server must '
            'already be running (gazebo_headless or gui).  '
            'View at http://localhost:<dashboard_port> in the Gazebo Simulator panel.'
        ),
    )

    # --------------------------------------------------------------- Gazebo
    gazebo_gui = ExecuteProcess(
        cmd=[gz, 'sim', '-r', world],
        condition=IfCondition(gui),
        cwd=sim_share,
        additional_env={'GZ_SIM_RESOURCE_PATH': f'{sim_share}/models'},
        output='screen',
    )
    gazebo_headless = ExecuteProcess(
        cmd=[gz, 'sim', '-s', '-r', world],
        condition=UnlessCondition(gui),
        cwd=sim_share,
        additional_env={'GZ_SIM_RESOURCE_PATH': f'{sim_share}/models'},
        output='screen',
    )

    # ---------------------------------------------------------------- bridge
    # /clock MUST be explicitly bridged. ros_gz_bridge does NOT automatically
    # bridge /clock. Without this entry all nodes with use_sim_time=True block
    # indefinitely waiting for ROS time and no sensors, SLAM, EKF, or planner
    # output will flow.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='sih26126_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/sih_ugv/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/ugv_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        ],
    )
    # Dedicated bridge for LiDAR point cloud: GZ /lidar/points/points -> ROS /lidar/points.
    # Uses config_file alone (no arguments) because parameter_bridge ignores config_file
    # when CLI topic arguments are also present.
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='sih26126_lidar_bridge',
        output='screen',
        parameters=[
            {'config_file': PathJoinSubstitution([
                bringup_share,
                'config',
                'bridge.yaml',
            ])},
        ],
    )
    dynamic_obstacle_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='objective4_pose_service_bridge',
        output='screen',
        arguments=[
            '/world/dynamic_obstacle_demo/set_pose@'
            'ros_gz_interfaces/srv/SetEntityPose',
        ],
        condition=IfCondition(is_dynamic_obstacle),
    )

    # Gazebo's Pose_V bridge carries simulator poses (ground truth) into ROS
    # TF. Keep it only in baseline mode; SLAM mode uses its odometry relay,
    # while fusion mode must have exactly one odom->base_link authority.
    gazebo_tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='sih26126_gazebo_tf_bridge',
        output='screen',
        arguments=['/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        condition=IfCondition(AndSubstitution(
            NotSubstitution(slam), NotSubstitution(fusion)
        )),
    )

    # ------------------------------------------------ perception/traversability
    perception = Node(
        package='sih26126_perception',
        executable='perception_baseline',
        name='perception_baseline',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    traversability = Node(
        package='sih26126_traversability',
        executable='traversability_baseline',
        name='traversability_baseline',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    sensor_contract = Node(
        package='sih26126_sensor_contract',
        executable='sensor_contract',
        name='sensor_contract',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    lidar_monitor = Node(
        package='sih26126_sensor_contract',
        executable='lidar_monitor',
        name='lidar_monitor',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    scan_relay = Node(
        package='sih26126_sensor_contract',
        executable='scan_relay',
        name='scan_relay',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    lidar_processor = Node(
        package='sih26126_sensor_contract',
        executable='lidar_processor',
        name='lidar_processor',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    terrain_geometry = Node(
        package='sih26126_geometry',
        executable='geometry_terrain',
        name='terrain_geometry',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    terrain_classifier = Node(
        package='sih26126_geometry',
        executable='terrain_classifier',
        name='terrain_classifier',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    terrain_aware_traversability = Node(
        package='sih26126_traversability',
        executable='terrain_aware_traversability',
        name='terrain_aware_traversability',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ------------------------------------------ baseline pose adapter (no SLAM)
    pose_adapter = Node(
        package='sih26126_autonomy',
        executable='odometry_pose_adapter',
        name='localization_pose_adapter',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=UnlessCondition(slam),
    )

    # ------------------------------------------------------ SLAM nodes
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            f'{loc_share}/config/slam_params.yaml',
            {'use_sim_time': True},
        ],
        condition=IfCondition(slam),
    )
    slam_pose_adapter = Node(
        package='sih26126_localization',
        executable='slam_pose_adapter',
        name='slam_pose_adapter',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(slam),
    )
    slam_activator = Node(
        package='sih26126_localization',
        executable='slam_activator',
        name='slam_activator',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(slam),
    )

    # -------------------------------------------- static TF publishers
    # LiDAR: always needed when SLAM is active.
    lidar_static_tf_publisher = Node(
        package='sih26126_localization',
        executable='lidar_static_tf_publisher',
        name='lidar_static_tf_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(slam),
    )

    # IMU: only needed when fusion is active so the EKF can resolve the IMU
    # sensor frame relative to base_link.
    imu_static_tf_publisher = Node(
        package='sih26126_localization',
        executable='imu_static_tf_publisher',
        name='imu_static_tf_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(fusion),
    )

    # -------------------------------- odom->base_link TF authority
    #
    # fusion=false → odom_tf_publisher (plain Gazebo odometry relay)
    #                Active only when slam=true AND fusion=false.
    #                When slam=false the Gazebo bridge TF covers this edge.
    #
    # fusion=true  → EKF node publishes odom->base_link (publish_tf: true
    #                in ekf_params.yaml).  odom_tf_publisher MUST NOT run
    #                simultaneously — two publishers on the same TF edge
    #                cause TF tree inconsistency and extrapolation warnings.
    odom_tf_publisher = Node(
        package='sih26126_localization',
        executable='odom_tf_publisher',
        name='odom_tf_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(AndSubstitution(slam, NotSubstitution(fusion))),
    )

    # ---------------------------------------- robot_localization EKF
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            f'{loc_share}/config/ekf_params.yaml',
            {'use_sim_time': True},
        ],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
        ],
        condition=IfCondition(fusion),
    )

    ekf_sensor_adapter = Node(
        package='sih26126_localization',
        executable='ekf_sensor_adapter',
        name='ekf_sensor_adapter',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(fusion),
    )

    # -------------------------------------- fusion health monitor
    fusion_health_monitor = Node(
        package='sih26126_localization',
        executable='fusion_health_monitor',
        name='fusion_health_monitor',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(fusion),
    )

    # -------------------------------------------- occupancy mapper
    # slam_toolbox owns /map when SLAM is active; redirect depth mapper.
    online_mapper_slam = Node(
        package='sih26126_autonomy',
        executable='online_mapper',
        name='online_mapper',
        output='screen',
        parameters=[{'map_topic': '/slam_map', 'use_sim_time': True}],
        condition=IfCondition(slam),
    )
    online_mapper_default = Node(
        package='sih26126_autonomy',
        executable='online_mapper',
        name='online_mapper',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=UnlessCondition(slam),
    )

    # ------------------------------------------------ planning / safety
    planner = Node(
        package='sih26126_planner',
        executable='goal_path_planner',
        name='goal_path_planner',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    local_path_selector = Node(
        package='sih26126_planner',
        executable='local_path_selector',
        name='local_path_selector',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    safety_supervisor = Node(
        package='sih26126_safety',
        executable='safety_supervisor',
        name='safety_supervisor',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    stuck_recovery = Node(
        package='sih26126_safety',
        executable='stuck_recovery',
        name='stuck_recovery_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    frontier_explorer = Node(
        package='sih26126_autonomy',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[{'exploration_radius_m': exploration_radius, 'use_sim_time': True}],
        condition=IfCondition(is_showcase),
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
            'use_sim_time': True,
        }],
        condition=IfCondition(is_showcase),
    )
    dynamic_obstacle_controller = Node(
        package='sih26126_autonomy',
        executable='objective4_obstacle_controller',
        name='objective4_obstacle_controller',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(is_dynamic_obstacle),
    )

    # ---------------------------------------- obstacle perception pipeline
    # obstacle_extractor: /lidar/points -> /obstacles/raw  (detection clusters)
    # obstacle_tracker:   /obstacles/raw -> /obstacles/tracked + /obstacles/dynamic_risk
    obstacle_extractor = Node(
        package='sih26126_sensor_contract',
        executable='obstacle_extractor',
        name='obstacle_extractor',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    obstacle_tracker = Node(
        package='sih26126_sensor_contract',
        executable='obstacle_tracker',
        name='obstacle_tracker',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ---------------------------------------- degraded autonomy manager
    # Monitors sensor health, localization, recovery state ->
    # publishes /autonomy/mode (AutonomyMode)
    degraded_autonomy = Node(
        package='sih26126_safety',
        executable='degraded_autonomy',
        name='degraded_autonomy_manager',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ---------------------------------------- mission intelligence
    # Turns real runtime state changes into structured MissionEvent messages ->
    # /autonomy/events (MissionEvent) and /autonomy/summary (String)
    mission_intelligence = Node(
        package='sih26126_autonomy',
        executable='mission_intelligence',
        name='mission_intelligence',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    dashboard = Node(
        package='sih26126_dashboard',
        executable='dashboard',
        name='sih26126_dashboard',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(start_dashboard),
        additional_env={'DRISTHI_DASHBOARD_PORT': dashboard_port},
    )

    # Gazebo GUI streaming: virtual display + x11vnc + websockify + noVNC
    # Connects gz sim -g to the already-running Gazebo server (headless or GUI).
    # Does NOT create a second world or robot.  All traffic stays on localhost.
    gz_gui_stream_node = Node(
        package='sih26126_dashboard',
        executable='gz_gui_stream',
        name='gz_gui_stream',
        output='screen',
        parameters=[{
            'vnc_port': 5900,
            'websocket_port': 6080,
            'virtual_display': ':99',
            'resolution': '1280x800x24',
        }],
        condition=IfCondition(gui_stream),
    )

    return LaunchDescription([
        world_arg, dashboard_port_arg, start_dashboard_arg,
        gui_arg, slam_arg, fusion_arg, scenario_arg,
        gui_stream_arg,
        exploration_radius_arg, frontier_threshold_arg,
        minimum_completed_frontiers_arg, minimum_explored_area_arg,
        mission_name_arg, mission_timeout_arg, dynamic_obstacle_demo_arg,

        gazebo_gui,
        gazebo_headless,

        # T+5s — bridge
        TimerAction(period=5.0, actions=[
            bridge, lidar_bridge, gazebo_tf_bridge, dynamic_obstacle_pose_bridge]),

        # T+6s — core localization + perception
        TimerAction(period=6.0, actions=[
            perception,
            traversability,
            terrain_aware_traversability,
            pose_adapter,
            slam_toolbox_node,
            slam_pose_adapter,
            odom_tf_publisher,        # slam=true, fusion=false
            lidar_static_tf_publisher,
            imu_static_tf_publisher,  # fusion=true
            ekf_sensor_adapter,       # fusion=true
            ekf_node,                 # fusion=true
            planner,
        ]),

        # T+7s — sensors, mapping, activators
        TimerAction(period=7.0, actions=[
            dashboard,
            gz_gui_stream_node,       # gui_stream=true: Gazebo GUI → noVNC → dashboard
            sensor_contract,
            lidar_monitor,
            scan_relay,
            lidar_processor,
            terrain_geometry,
            terrain_classifier,
            online_mapper_slam,
            online_mapper_default,
            slam_activator,
            fusion_health_monitor,    # fusion=true
            dynamic_obstacle_controller,
            obstacle_extractor,       # /lidar/points -> /obstacles/raw
            obstacle_tracker,         # /obstacles/raw -> /obstacles/tracked
        ]),

        # T+8s — safety + degraded autonomy
        TimerAction(period=8.0, actions=[
            local_path_selector,
            safety_supervisor,
            stuck_recovery,
            degraded_autonomy,        # /autonomy/mode
        ]),

        # T+9s — mission + intelligence
        TimerAction(period=9.0, actions=[
            frontier_explorer,
            mission_manager,
            mission_intelligence,     # /autonomy/events, /autonomy/summary
        ]),
    ])
