"""Launch the closed-loop outdoor exploration mission benchmark."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_share = get_package_share_directory('sih26126_sim')
    bringup_share = get_package_share_directory('sih26126_bringup')
    default_world = f'{sim_share}/worlds/autonomous_forest.sdf'
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    dashboard = LaunchConfiguration('start_dashboard')
    frontier_threshold = LaunchConfiguration('frontier_threshold')
    completed_frontiers = LaunchConfiguration('minimum_completed_frontiers')
    minimum_area = LaunchConfiguration('minimum_explored_area_m2')
    exploration_radius = LaunchConfiguration('exploration_radius_m')
    mission_timeout = LaunchConfiguration('mission_timeout_sec')
    mission_name = LaunchConfiguration('mission_name')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Outdoor benchmark world SDF.'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Show Gazebo GUI.'),
        DeclareLaunchArgument('start_dashboard', default_value='true',
                              description='Start the live mission dashboard.'),
        DeclareLaunchArgument('frontier_threshold', default_value='0'),
        DeclareLaunchArgument('minimum_completed_frontiers', default_value='3'),
        DeclareLaunchArgument('minimum_explored_area_m2', default_value='24.0'),
        DeclareLaunchArgument(
            'exploration_radius_m', default_value='8.0',
            description='Maximum map-derived frontier distance from captured home.'),
        DeclareLaunchArgument(
            'mission_timeout_sec', default_value='600.0',
            description='Failure cutoff only; exploration completion stays map-driven.'),
        DeclareLaunchArgument('mission_name', default_value='autonomous_forest'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f'{bringup_share}/launch/full_stack.launch.py'),
            launch_arguments={
                'world': world,
                'gui': gui,
                'start_dashboard': dashboard,
                'exploration_radius_m': exploration_radius,
                'mission_timeout_sec': mission_timeout,
                'frontier_threshold': frontier_threshold,
                'minimum_completed_frontiers': completed_frontiers,
                'minimum_explored_area_m2': minimum_area,
                'mission_name': mission_name,
            }.items()),
    ])
