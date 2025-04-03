import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Create the launch description object
    ld = LaunchDescription()

    # Package name for simulation initialization
    pkg_name = 'leo_simulation'

    # ------------------------------------------------------------
    # Include Gazebo Simulation Environment Launch File
    # This file (leo_gz_bringup.launch.py) from the custom_leo_description package 
    # sets up the Gazebo Ignition world and spawns the robot model.
    # ------------------------------------------------------------
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('custom_leo_description'),
            '/launch',
            '/leo_gz_bringup.launch.py'
        ]),
        launch_arguments={}.items(),
    )

    # ------------------------------------------------------------
    # Bridge Node: ROS2 - Ignition Gazebo Bridge
    # The ros_gz_bridge node bridges topics between ROS2 and Gazebo Ignition.
    # This enables the simulation data (e.g., /clock, /scan, /odom, etc.) to be
    # communicated between the two systems.
    # ------------------------------------------------------------
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/clock'                           + '@rosgraph_msgs/msg/Clock'   + '[' + 'ignition.msgs.Clock',
            '/model/leo/cmd_vel'               + '@geometry_msgs/msg/Twist'   + '@' + 'ignition.msgs.Twist',
            '/model/leo/odometry'              + '@nav_msgs/msg/Odometry'     + '[' + 'ignition.msgs.Odometry',
            '/model/leo/scan'                  + '@sensor_msgs/msg/LaserScan' + '[' + 'ignition.msgs.LaserScan',
            '/model/leo/tf'                    + '@tf2_msgs/msg/TFMessage'    + '[' + 'ignition.msgs.Pose_V',
            '/model/leo/imu'                   + '@sensor_msgs/msg/Imu'       + '[' + 'ignition.msgs.IMU',
            '/world/empty/model/leo/joint_state'+ '@sensor_msgs/msg/JointState' + '[' + 'ignition.msgs.Model',
            'model/leo/image'                  + '@sensor_msgs/msg/Image'     + '[' + 'ignition.msgs.Image',
            '/world/empty/model/leo/link/px150/gripper_link/sensor/depth/depth_image' 
                                               + '@sensor_msgs/msg/Image'     + '[' + 'ignition.msgs.Image',
            '/world/empty/model/leo/link/px150/gripper_link/sensor/color/image'
                                               + '@sensor_msgs/msg/Image'     + '[' + 'ignition.msgs.Image',
            '/world/empty/model/leo/link/px150/gripper_link/sensor/depth/depth_image/points'
                                               + '@sensor_msgs/msg/PointCloud2' + '[' + 'ignition.msgs.PointCloudPacked',
            '/model/leo/battery/linear_battery/state' + '@sensor_msgs/msg/BatteryState' + '[' + 'ignition.msgs.Battery',
        ],
        parameters=[
            {'qos_overrides./leo.subscriber.reliability': 'reliable'},
            {'qos_overrides./leo.subscriber.durability': 'transient_local'}
        ],
        remappings=[
            ('/model/leo/cmd_vel',  '/leo/cmd_vel'),
            ('/model/leo/odometry', '/odom'),
            ('/model/leo/scan',     '/scan'),
            ('/model/leo/tf',       '/tf'),
            ('/model/leo/imu',      '/imu_raw'),
            ('/world/empty/model/leo/joint_state', 'joint_states'),
            ('/model/leo/image', '/image_raw'),
            ('/world/empty/model/leo/link/px150/gripper_link/sensor/depth/depth_image', '/depth'),
            ('/world/empty/model/leo/link/px150/gripper_link/sensor/depth/depth_image/points', '/depth/points'),
            ('/world/empty/model/leo/link/px150/gripper_link/sensor/color/image', '/depth_image_raw'),
            ('/model/leo/battery/linear_battery/state', '/battery_state')
        ],
        output='screen'
    )
    
    # ------------------------------------------------------------
    # Teleoperation Node: Keyboard Teleop for Manual Control
    # This node enables manual control of the robot via keyboard commands.
    # ------------------------------------------------------------
    node_teleop_twist_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard_node',
        remappings={('/cmd_vel', '/teleop_cmd_vel')},
        output='screen',
        prefix='xterm -e'  # Opens the node in a separate terminal window for visibility
    )
    
    # ------------------------------------------------------------
    # RViz Visualization Node
    # This node launches RViz2 with a preconfigured layout for the Leo Rover simulation.
    # ------------------------------------------------------------
    node_rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d' + os.path.join(get_package_share_directory(pkg_name), 'rviz', 'nav2.rviz')
        ],
        output='screen'
    )

    # Add necessary parameters and nodes to the launch description
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(launch_gazebo)
    ld.add_action(node_ros_gz_bridge)
    ld.add_action(node_teleop_twist_keyboard)
    ld.add_action(node_rviz)
    
    return ld
