import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    # Specify the name of the package and path to xacro file in external package
    pkg_name = 'example_gz_robot'

    # Start simulation
    launch_sim_world = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory(pkg_name), 'launch'),
         '/simulation_bringup.launch.py'])
      )

    # map server node
    map_subpath = 'maps/turtlebot3_world.yaml'
    map_yaml_filepath = os.path.join(get_package_share_directory(pkg_name), map_subpath)

    # Publishes a 2D occupancy grid based on a .pgm (and accompanying .yaml) file
    node_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='example_map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_filepath}] # add other parameters here if required
    )

    # https://docs.nav2.org/configuration/packages/configuring-amcl.html
    # AMCL (Adaptive Monte Carlo Localisation)
    node_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl'
    )


    # Lifecycle Manager - Ensures localisation waits for map to be published etc
    lifecycle_nodes = ['example_map_server', 'amcl']
    node_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        output='screen',
        parameters=[{'node_names':lifecycle_nodes, 'autostart': True}] # add other parameters here if required
    )


    # Rviz node
    rviz_config_file = os.path.join(get_package_share_directory(pkg_name), 'rviz', 'localisation.rviz')
    node_rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [rviz_config_file]]
    )


    # Add actions to LaunchDescription
    # Simulation
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(launch_sim_world)
    # Localisation
    ld.add_action(node_lifecycle_manager)
    ld.add_action(node_map_server)
    ld.add_action(node_amcl)
    # Visualisation
    ld.add_action(node_rviz)
    return ld
