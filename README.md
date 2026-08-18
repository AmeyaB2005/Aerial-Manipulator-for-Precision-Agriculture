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

## Launching Simulations

Before launching any configuration, make sure you have built and sourced the workspace as described above.

### 1. Waypoint Flight (Gazebo Simulation only)
Starts the Gazebo simulation and the autonomous waypoint navigation controller.
```bash
ros2 launch eft_control waypoint_flight.launch.py
```

### 2. Waypoint Flight with MuJoCo (Co-simulation)
Starts the drone waypoint navigation in Gazebo and triggers the MuJoCo-simulated bending arm at targets.
* **Terminal 1: Start simulation and controllers**
  ```bash
  ros2 launch eft_control waypoint_flight_mujoco.launch.py
  ```
* **Terminal 2: Run the MuJoCo simulation script**
  ```bash
  cd ~/mujoco_tdcr
  python3 simulate_ros.py
  ```

### 3. QGroundControl (QGC) Teleoperation
Launches the full simulation and boots up the QGroundControl application.
```bash
ros2 launch eft_control qgc_teleop.launch.py
```
*(Optional: If your AppImage is located elsewhere, run: `ros2 launch eft_control qgc_teleop.launch.py qgc_path:=/path/to/your/QGroundControl.AppImage`)*

### 4. Keyboard Teleoperation
Launches the simulation with offboard teleoperation bridges, allowing you to fly the drone using your keyboard.
* **Terminal 1: Start simulation and teleop bridge**
  ```bash
  ros2 launch eft_control keyboard_teleop.launch.py
  ```
* **Terminal 2: Launch the keyboard listener node**
  ```bash
  source ~/Aerial-Manipulator-for-Precision-Agriculture/eft_ws/install/setup.bash
  python3 ~/Aerial-Manipulator-for-Precision-Agriculture/eft_ws/src/eft_control/eft_control/keyboard_teleop.py
  ```

