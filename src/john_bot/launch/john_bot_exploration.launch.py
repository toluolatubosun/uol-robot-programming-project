import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('john_bot')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    explore_params_file = LaunchConfiguration('explore_params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Nodes managed by lifecycle manager
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower'
    ]
    
    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true',
        description='Use simulation clock')
    
    declare_autostart = DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='Automatically startup nav2')
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
        description='Nav2 parameters file')
    
    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml'),
        description='SLAM parameters file')
    
    declare_explore_params_file = DeclareLaunchArgument(
        'explore_params_file',
        default_value=os.path.join(pkg_dir, 'config', 'explore_params.yaml'),
        description='Explore parameters file')
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization')
    
    # SLAM Toolbox
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 
                        'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file
        }.items())
    
    # Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}])
    
    # Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}])
    
    # Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}])
    
    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}])
    
    # Waypoint Follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}])
    
    # Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': lifecycle_nodes}])
    
    # Explore Lite
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[
            explore_params_file, 
            {'use_sim_time': use_sim_time}, 
            {'return_to_init': True}
        ])
    
    # RViz
    rviz_config_file = os.path.join(pkg_dir, 'config', 'navigation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz))
    
    # Exploration Monitor
    exploration_monitor = Node(
        package='john_bot',
        executable='exploration_monitor',
        name='exploration_monitor',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'periodic_save_interval': 30.0}
        ])
    
    # Verification Controller
    verification_controller = Node(
        package='john_bot',
        executable='verification_controller',
        name='verification_controller',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'standoff_distance': 1.5},
            {'object_verification_duration': 5.0},
            {'object_match_distance_tolerance': 1.0}
        ])
    
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(declare_params_file)
    ld.add_action(declare_slam_params_file)
    ld.add_action(declare_explore_params_file)
    ld.add_action(declare_use_rviz)
    ld.add_action(slam_toolbox_launch)
    ld.add_action(controller_server)
    ld.add_action(planner_server)
    ld.add_action(behavior_server)
    ld.add_action(bt_navigator)
    ld.add_action(waypoint_follower)
    ld.add_action(lifecycle_manager)
    ld.add_action(explore_node)
    ld.add_action(exploration_monitor)
    ld.add_action(verification_controller)
    ld.add_action(rviz_node)
    
    return ld