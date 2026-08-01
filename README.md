# Aerial Manipulator for Precision Agriculture

This repository contains the ROS 2 packages and PX4 customizations for the **EFT E610P** agricultural multirotor simulation.

## Repository Structure
* **`eft_ws/`**: ROS 2 Workspace containing:
  * **`eft_control`**: ROS 2 package with MAVLink teleop bridges, URDF definitions, and RViz2 launch configurations.
  * **`px4_msgs`**: ROS 2 PX4 message interfaces.
* **`px4_customizations/`**: Customized PX4 SITL files:
  * **`airframes/`**: Multi-rotor control allocator configurations.
  * **`models/`**: Visual models and dynamics parameters.

## Setup Instructions

### 1. Copy PX4 Customizations
To load the custom EFT E610P airframe and physical visuals into your local `PX4-Autopilot` installation:
```bash
# Copy airframe posix setup
cp px4_customizations/airframes/22000_gz_eft_e610p ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/

# Copy visual model, meshes, and SDF parameters
cp -r px4_customizations/models/eft_e610p ~/PX4-Autopilot/Tools/simulation/gz/models/
```

### 2. Build ROS 2 Workspace
```bash
cd eft_ws
colcon build --symlink-install
source install/setup.bash
```
