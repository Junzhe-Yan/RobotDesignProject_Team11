# Leo Rover Navigation Simulation – Simulation Bridge

This repository is part of the Leo Rover navigation simulation project and is responsible for initializing the simulation bridge between Gazebo Ignition and ROS2. This package, **leo_simulation**, forms a critical component of our overall simulation environment by integrating sensor data and command topics between Gazebo and ROS2.

## Overview

The simulation bridge launch file (`leo_sim_bringup.launch.py`) performs the following functions:

1. **Gazebo Environment Initialization:**

   - It includes the `leo_gz_bringup.launch.py` file from the `custom_leo_description` package, which sets up the Gazebo Ignition simulation world and spawns the robot model.
2. **ROS2–Ignition Bridge:**

   - The file launches the `ros_gz_bridge` node from the `ros_gz_bridge` package. This node bridges topics between the Gazebo simulation and ROS2. A variety of topics are bridged, including `/clock`, `/scan`, `/odom`, `/imu`, and others, ensuring that sensor data and control commands are properly synchronized between the systems.
3. **Teleoperation:**

   - A keyboard teleoperation node (`teleop_twist_keyboard`) is launched to allow manual control of the robot during simulation. This node remaps the command velocity topic to facilitate testing and debugging.
4. **Visualization:**

   - An RViz2 instance is started using a preconfigured RViz file (`leo_sim.rviz`), providing a visual interface to monitor the robot, sensor data, and navigation status.
5. **Simulation Time:**

   - The launch file sets the `use_sim_time` parameter to `True`, ensuring that all nodes use the simulation clock provided by Gazebo.

## Repository Structure

```
leo_rover_nav_sim/
├── custom_leo_description         # Contains resources for describing the Leo Rover and its environment
│   ├── urdf/                 
│   │   ├── leo.urdf.xacro        # Main robot model description for ROS (URDF generated from Xacro)
│   │   └── leo.gazebo.xacro      # Gazebo-specific robot model description (may include simulation-specific elements)
│   ├── worlds/                   # Contains world files and associated assets for Gazebo simulation
│   │   ├── tb3.sdf           
│   │   └── meshes/           
│   │       ├── hexagon.dae   
│   │       └── wall.dae     
│   ├── models/                   # Additional model files and assets for the robot
│   │   ├── Chassis.dae      
│   │   ├── Chassis_outline.stl   
│   │   └── ...                   # Other models as required for the simulation
│   ├── rviz/                
│   │   ├── leo_view.rviz         # RViz configuration for visualizing the Leo Rover model
│   │   └── nav2.rviz             # RViz configuration for monitoring the Nav2 navigation stack
│   └── launch/               
│       └── leo_gz_bringup.launch.py  # Launch file to set up Gazebo simulation environment and spawn the robot model
├── leo_simulation                 # Package that manages the simulation bridge between Gazebo Ignition and ROS2
│   └── launch/
│       └── leo_sim_bringup.launch.py  # Launch file to initialize the ROS2–Gazebo bridge and synchronize simulation time
└── SLAM                           # Package containing mapping and navigation controls (SLAM and Nav2 stack)
    ├── launch/
    │   └── leo_nav_sim.launch.py      # Launch file to run mapping and navigation (SLAM Toolbox, Nav2 nodes, etc.)
    └── config/                    # Configuration files for the navigation system
        ├── bt_nav.yaml            # Parameters for the Behavior Tree Navigator (bt_navigator)
        ├── planner.yaml           # Parameters for the Planner Server (global path planning)
        ├── controller.yaml        # Parameters for the Controller Server (local trajectory control)
        ├── amcl.yaml              # Parameters for the AMCL node (localization)
        ├── costmap.yaml           # Costmap configuration parameters (global and local costmaps)
        └── twist_mux.yaml         # Parameters for the Twist Mux node (combining velocity commands from multiple sources)
```

## How to Launch

To start the simulation bridge and the associated nodes, run:

```bash
ros2 launch leo_simulation leo_sim_bringup.launch.py
```

This command will:

* Launch the Gazebo simulation environment with your robot model.
* Start the ROS2–Ignition bridge to translate topics between systems.
* Open the teleoperation interface in a separate terminal window.
* Start RViz2 with the preconfigured layout for monitoring the simulation.

## Troubleshooting

* **Missing Models or World Files:**
  Ensure that the `IGN_GAZEBO_RESOURCE_PATH` environment variable is set correctly so that Gazebo Ignition can locate the necessary model and world files.
* **Topic Remappings:**
  Verify that the topic remappings in the launch file align with your simulation setup. Adjust them as necessary to ensure that all nodes can communicate correctly.
* **Simulation Time:**
  Confirm that `use_sim_time` is set to `True` for all nodes, so they properly synchronize with the Gazebo simulation clock.
