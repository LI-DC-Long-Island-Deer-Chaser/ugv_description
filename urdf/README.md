# UGV URDF Description

This directory contains the complete robot description files for the UGV (Unmanned Ground Vehicle) using ROS 2 URDF/Xacro format.

## File Structure

### Core Files

#### `ugv.xacro` - Main Robot Description
The master file that defines the robot's complete structure.

**Contains:**
- All link definitions (physical components with inertia, visual meshes, collision geometry)
- All joint definitions (connections between links with position/orientation)
- Robot kinematic tree structure (base_link → wheels, sensors, peripherals)
- Includes for other modular files (materials, transmissions, Gazebo properties)

**Key components:**
- `base_link` - Main chassis
- Wheel links: `FrontLeft_1`, `FrontRight_1`, `BackLeft_1`, `BackRight_1`
- Steering hubs: `LeftHub_1`, `RightHub__1`
- Sensor links: `imu_link`, `360lidar_link`, `ToFsensor_link`, `GPSsensor_link`, `frontcam_link`, `backcam_link`
- Peripheral links: Electronics, power systems, antennas, etc.

**Usage:** This is the entry point for robot visualization and simulation.

---

#### `materials.xacro` - RViz Visual Materials
Defines visual appearance for RViz visualization.

**Contains:**
- RGBA color definitions (e.g., `silver` material)
- Material properties for RViz rendering

**Note:** These materials are **only for RViz**. Gazebo uses its own material system defined in `ugv.gazebo`.

---

#### `ugv.trans` - Transmission Definitions
Defines the actuator-to-joint mappings for `ros_control`.

**Contains:**
- Transmission interfaces for each actuated joint
- Hardware interface types (`EffortJointInterface`)
- Mechanical reduction ratios (currently all set to 1:1)

**Joints with transmissions:**
- `left_steering` / `right_steering` - Front wheel steering actuators
- `front_left_wheel` / `front_right_wheel` - Front drive motors
- `back_left_wheel` / `back_right_wheel` - Rear drive motors

**Purpose:** Enables `gazebo_ros_control` to command wheel velocities and steering angles through ROS topics/services.

---

#### `ugv.gazebo` - Gazebo Simulation Properties
Configures Gazebo-specific physics, sensors, and visual properties.

**Contains:**

**1. Physics Properties** (for all links):
- Friction coefficients (`mu1`, `mu2`)
- Self-collision flags
- Gravity settings
- Gazebo visual materials (`Gazebo/Silver`)

**2. Sensor Definitions:**
- **IMU Sensor** (`imu_link`):
  - Type: `imu`
  - Topic: `/imu/data`
  - Update rate: 100 Hz
  - Plugin: `gz-sim-imu-system`

- **360° Lidar** (`360lidar_link`):
  - Type: `gpu_lidar`
  - Topic: `/lidar`
  - Horizontal: 640 samples, 160° FOV
  - Range: 0.08 - 40m
  - Update rate: 30 Hz

- **ToF Range Sensor** (`ToFsensor_link`):
  - Type: `gpu_lidar` (1D mode)
  - Topic: `/tof/range`
  - Single beam (VL53L0X emulation)
  - Range: 0.03 - 1.0m
  - Gaussian noise: 3mm stddev
  - Update rate: 50 Hz

- **GPS/NavSat** (`GPSsensor_link`):
  - Type: `navsat`
  - Topic: `/gps/fix`
  - Position noise: 1.0m horizontal, 1.5m vertical
  - Velocity noise: 0.05 m/s horizontal, 0.1 m/s vertical
  - Update rate: 10 Hz

- **Wide-Angle Cameras** (`frontcam_link`, `backcam_link`):
  - Type: `wideanglecamera`
  - Topics: `/camera/front`, `/camera/back`
  - Resolution: 640x360 @ 30 fps
  - FOV: 165° (stereographic projection)
  - Insta360 One X2 emulation

**3. Control Plugin:**
- `gazebo_ros_control` - Connects transmissions to ROS control interfaces

---

## How They Work Together

```
ugv.xacro (Master)
├── includes → materials.xacro (RViz colors)
├── includes → ugv.trans (actuator mappings)
└── includes → ugv.gazebo (simulation properties + sensors)
```

### Workflow:

1. **URDF Processing:**
   - ROS reads `ugv.xacro`
   - Xacro processor expands includes
   - Generates complete URDF XML

2. **RViz Visualization:**
   - Uses link geometry from `ugv.xacro`
   - Applies materials from `materials.xacro`
   - Displays robot with silver coloring

3. **Gazebo Simulation:**
   - Loads link geometry and inertia from `ugv.xacro`
   - Applies physics properties from `ugv.gazebo`
   - Spawns sensors attached to sensor links
   - Loads control plugin for wheel/steering actuation
   - Uses transmissions from `ugv.trans` for motor control

### Sensor Link Pattern:

For each sensor, the pattern is:

**In `ugv.xacro`:**
```xml
<link name="sensor_link">
  <inertial>...</inertial>
</link>

<joint name="sensor_joint" type="fixed">
  <parent link="physical_component"/>
  <child link="sensor_link"/>
  <origin xyz="..." rpy="..."/>
</joint>
```

**In `ugv.gazebo`:**
```xml
<gazebo reference="sensor_link">
  <sensor name="..." type="...">
    <!-- sensor configuration -->
  </sensor>
</gazebo>
```

The joint's `origin` defines sensor position/orientation. The `<gazebo reference>` tag attaches the sensor to that link's frame.

---

## Usage Examples

### View in RViz:
```bash
ros2 launch ugv_description display.launch.py
```

### Spawn in Gazebo:
```bash
ros2 launch ugv_description gazebo.launch.py
```

### Check processed URDF:
```bash
xacro /path/to/ugv.xacro > output.urdf
check_urdf output.urdf
```

---

## Modification Guide

### Adding a New Sensor:

1. **Create sensor link** in `ugv.xacro`:
   ```xml
   <link name="new_sensor_link">
     <inertial>
       <mass value="0.001"/>
       ...
     </inertial>
   </link>
   
   <joint name="new_sensor_joint" type="fixed">
     <parent link="mounting_location"/>
     <child link="new_sensor_link"/>
     <origin xyz="x y z" rpy="r p y"/>
   </joint>
   ```

2. **Configure sensor** in `ugv.gazebo`:
   ```xml
   <gazebo reference="new_sensor_link">
     <sensor name="..." type="...">
       <!-- sensor parameters -->
     </sensor>
   </gazebo>
   ```

### Changing Colors:
- **RViz:** Edit `materials.xacro`
- **Gazebo:** Edit `body_color` property in `ugv.gazebo`

### Adding Actuators:
1. Add joint to `ugv.xacro`
2. Add transmission to `ugv.trans`
3. Joint will be controllable via ROS topics

---

## Dependencies

- **ROS 2** (Humble or later)
- **Gazebo** (Ignition/GZ recommended)
- **Xacro** - XML macro processor
- **ros_gz_bridge** - ROS/Gazebo communication
- **gazebo_ros_control** - Control plugin

---

## Notes

- All positions in **meters**
- All angles in **radians**
- Coordinate system: **ROS REP-103** (X forward, Y left, Z up)
- Mesh files located in `../meshes/` directory
- Sensor frame IDs match link names for TF tree consistency
