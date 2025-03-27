import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
import xacro

def generate_launch_description():
    # Create the launch description object
    ld = LaunchDescription()
    
    # Define package and file paths for the robot description
    pkg_name = 'custom_leo_description'
    file_subpath = 'urdf/leo.urdf.xacro'
    
    # Set Ignition Gazebo resource path so that world files can be located
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[os.path.join(get_package_share_directory(pkg_name), 'worlds')]
    )
    
    # Process the Xacro file to generate the robot description in URDF format
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Update the Ignition Gazebo resource path to include any additional world paths
    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        gz_world_path = os.environ['IGN_GAZEBO_RESOURCE_PATH'] + os.pathsep + os.path.join(get_package_share_directory('custom_leo_description'), "worlds")
    else:
        gz_world_path = os.path.join(get_package_share_directory(pkg_name), "worlds")

    ign_resource_path_update = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[gz_world_path]
    )
    
    # Include the Gazebo simulation launch file from the ros_gz_sim package
    gz_start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('ros_gz_sim'),
            '/launch',
            '/gz_sim.launch.py'
        ]),
        launch_arguments={'gz_args' : '-r ' + 'empty.sdf'}.items(),
    )

    # Spawn additional objects into the simulation world using an SDF file
    sdf_path = os.path.join(get_package_share_directory(pkg_name), 'worlds', 'tb3.sdf')
    gz_spawn_objects = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', sdf_path, '-x', '2.0', '-y', '0.5', '-z', '0.0'],
        output='screen'
    )
    
    # Spawn the robot entity using the robot description topic
    node_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description', '-z', '0.5'],
        output='screen'
    )

    # Launch robot state publisher to broadcast the robot's TF tree from the URDF description
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )
    
    # Optionally, launch joint state publisher GUI for debugging joint movements
    node_joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    # Add all the actions to the launch description
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(ign_resource_path)
    ld.add_action(ign_resource_path_update)
    ld.add_action(gz_start_world)
    ld.add_action(gz_spawn_objects)
    ld.add_action(node_spawn_entity)
    ld.add_action(node_robot_state_publisher)
    # Optionally add the joint state publisher if needed
    # ld.add_action(node_joint_state_publisher)

    return ld
