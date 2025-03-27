# Leo Rover Navigation Simulation

This repository is an early version of the Leo Rover navigation simulation project. It integrates multiple ROS2 packages to enable a complete simulation environment for mapping, localization, and autonomous navigation using Gazebo Ignition and the Nav2 stack.

## Overview

The project is organized into three primary components:

1. **Simulation Environment Setup (`custom_leo_description`):**

   - Contains the robot’s URDF/Xacro model and world files.
   - Launch file `leo_gz_bringup.launch.py` initializes Gazebo Ignition and spawns the robot model.
2. **Simulation Bridge (`leo_simulation`):**

   - Manages the integration between Gazebo Ignition and ROS2.
   - Launch file `leo_sim_bringup.launch.py` (not shown here) sets up the necessary bridges and synchronizes simulation time.
3. **Mapping and Navigation Controls (SLAM):**

   - Contains configuration and launch files for SLAM and autonomous navigation.
   - Launch file `leo_nav_sim.launch.py` (this file) launches the SLAM Toolbox and the ROS2 Navigation (Nav2) nodes including planner, controller, AMCL, and the behavior tree navigator.

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

## How to Launch the Simulation

To start the full navigation simulation, execute the following command from your workspace:

```bash
ros2 launch SLAM leo_nav_sim.launch.py
```

This command will:

- Launch the Gazebo simulation environment and load the Leo Rover model.
- Start the ROS2–Ignition bridge via `ros_gz_bridge`.
- Launch the SLAM Toolbox for mapping and localization.
- Initialize the Nav2 navigation stack (including planner, controller, AMCL, and behavior nodes).
- Open RViz2 with a preconfigured view to visualize the simulation.

## Troubleshooting

- **Missing Parameters or Topics:**
  Verify that all configuration files in the `config/` directory are correctly referenced. Ensure that the workspace is built and sourced properly.
- **TF Remappings:**
  Confirm that the topic remappings (e.g., `/tf` and `/tf_static`) match your system's configuration.
- **Simulation Time:**
  Ensure that `use_sim_time` is set to `True` for all nodes to synchronize with the Gazebo simulation clock.
