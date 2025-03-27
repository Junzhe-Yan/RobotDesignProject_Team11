import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    ld = LaunchDescription()
    
    # -----------------------------------------------------------
    # Package and configuration setup
    # -----------------------------------------------------------
    # 'SLAM' package holds the configuration files for mapping and navigation.
    pkg_nav = get_package_share_directory('SLAM')
    
    # Remapping for TF topics to maintain consistency across nodes.
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # List of lifecycle nodes to be autostarted by the lifecycle manager.
    lifecycle_nodes = [
        'amcl',
        'controller_server',
        'planner_server',
        'behaviour_server',
        'bt_navigator',
    ]

    # -----------------------------------------------------------
    # Load configuration parameters from YAML files
    # -----------------------------------------------------------
    config_bt_nav     = PathJoinSubstitution([pkg_nav, 'config', 'bt_nav.yaml'])
    config_planner    = PathJoinSubstitution([pkg_nav, 'config', 'planner.yaml'])
    config_controller = PathJoinSubstitution([pkg_nav, 'config', 'controller.yaml'])
    config_amcl       = PathJoinSubstitution([pkg_nav, 'config', 'amcl.yaml'])
    config_costmap    = PathJoinSubstitution([pkg_nav, 'config', 'costmap.yaml'])
    config_twist_mux  = PathJoinSubstitution([pkg_nav, 'config', 'twist_mux.yaml'])

    # -----------------------------------------------------------
    # Include the Gazebo simulation environment
    # This launch file is from the 'leo_simulation' package and sets up the simulation world.
    # -----------------------------------------------------------
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('leo_simulation'),
            '/launch',
            '/leo_sim_bringup.launch.py'
        ]),
        launch_arguments={}.items(),
    )

    # -----------------------------------------------------------
    # Launch the SLAM Toolbox for mapping and localization
    # -----------------------------------------------------------
    launch_slamtoolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('slam_toolbox'),
            '/launch',
            '/online_async_launch.py'
        ]),
        launch_arguments={}.items(),
    )
    
    # -----------------------------------------------------------
    # Launch the twist multiplexer to combine velocity commands
    # -----------------------------------------------------------
    node_twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[config_twist_mux],
        remappings=[('/cmd_vel_out', '/leo/cmd_vel')],
    )

    # -----------------------------------------------------------
    # Launch Nav2 Navigation Nodes
    # -----------------------------------------------------------
    # Behavior Tree Navigator node: orchestrates high-level navigation actions.
    node_bt_nav = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[config_bt_nav],
        remappings=remappings,
    )

    # Behavior Server node: handles recovery behaviors and decision-making.
    node_behaviour = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behaviour_server',
        output='screen',
        parameters=[config_bt_nav],
        remappings=[('/cmd_vel', '/nav_cmd_vel')],
    )

    # Planner Server node: computes a global path from the current position to the target.
    node_planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[config_planner, config_costmap],
        remappings=remappings,
    )

    # Controller Server node: calculates velocity commands to follow the computed path.
    node_controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[config_controller, config_costmap],
        remappings=[('/cmd_vel', '/nav_cmd_vel')],
    )

    # AMCL node: provides probabilistic localization using sensor and odometry data.
    node_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[config_amcl],
        remappings=remappings,
    )
    
    # -----------------------------------------------------------
    # Launch the Lifecycle Manager to auto-start Nav2 nodes
    # -----------------------------------------------------------
    node_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'autostart': True}, {'node_names': lifecycle_nodes}],
    )

    # -----------------------------------------------------------
    # Add all nodes and launch actions to the Launch Description
    # -----------------------------------------------------------
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(launch_gazebo)
    ld.add_action(launch_slamtoolbox)
    ld.add_action(node_twist_mux)
    ld.add_action(node_bt_nav)
    ld.add_action(node_behaviour)
    ld.add_action(node_planner)
    ld.add_action(node_controller)
    ld.add_action(node_amcl)
    ld.add_action(node_lifecycle_manager)

    return ld
