# Cartographer SLAM Integration for UGV Rover

This document explains how Cartographer SLAM is integrated with your ArduPilot rover and how odometry feedback works.

## Overview

Your rover uses Cartographer for SLAM (Simultaneous Localization and Mapping), which provides corrected odometry to ArduPilot's Extended Kalman Filter (EKF3) for improved position estimation.

## Architecture

### Data Flow
```
┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│   Gazebo    │────>│  ros_gz_     │────>│ Cartographer│
│  Simulator  │     │   bridge     │     │    SLAM     │
└─────────────┘     └──────────────┘     └─────────────┘
      │                    │                      │
      │ (sensors)          │ (ROS topics)         │ (optimized pose)
      v                    v                      v
┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│ ArduPilot   │<────│ micro_ros_   │<────│  /ap/tf     │
│   SITL      │     │    agent     │     │  (relay)    │
│   (EKF3)    │     │    (DDS)     │     └─────────────┘
└─────────────┘     └──────────────┘
```

### Topics and Data Flow

1. **Sensor Topics** (from Gazebo via ros_gz_bridge):
   - `/ugv/lidar` - LaserScan data
   - `/ugv/imu/data` - IMU data
   - `/ugv/odom` - Raw odometry from Gazebo

2. **Cartographer Processing**:
   - Subscribes to: `/ugv/lidar`, `/ugv/imu/data`, `/ugv/odom`
   - Performs SLAM to create a map
   - Publishes corrected transforms on `/tf` topic:
     - `map` -> `odom`
     - `odom` -> `base_link`

3. **TF Relay to ArduPilot**:
   - `topic_tools relay` node copies `/tf` to `/ap/tf`
   - ArduPilot subscribes to `/ap/tf` via DDS (micro_ros_agent)

4. **ArduPilot Processing**:
   - ArduPilot receives `tf2_msgs/TFMessage` on `/ap/tf`
   - `AP_DDS_External_Odom::handle_external_odom()` extracts `odom -> base_link` transform
   - Converts from ROS ENU frame to ArduPilot NED frame
   - Feeds position/orientation to `AP_VisualOdom` library
   - EKF3 fuses this data with other sensors (GPS, IMU)

## Configuration Files

### 1. Launch File: `launch/slam/cartographer.launch.py`
- Starts Cartographer node
- Creates occupancy grid
- Relays TF to ArduPilot
- Opens RViz for visualization

### 2. Cartographer Config: `config/cartographer/cartographer_rover.lua`
- Configured for 2D SLAM (ground rover)
- Tuned for RPLidar (0.15m - 12m range)
- Uses IMU and wheel odometry
- Loop closure enabled

### 3. RViz Config: `config/cartographer/cartographer_rover.rviz`
- Displays map, laser scans, robot model, TF tree

## ArduPilot Parameters

Your `mav.parm` is already configured for external odometry:

```
EK3_SRC1_POSXY   3    # Use Visual Odometry for XY position
EK3_SRC1_VELXY   3    # Use Visual Odometry for XY velocity  
EK3_SRC1_VELZ    3    # Use Visual Odometry for Z velocity
VISO_TYPE        0    # Visual Odom type (0 = use DDS/TF input)
```

The value `3` for EK3_SRC parameters means "Visual Odometry" source, which in this case comes from DDS (the `/ap/tf` topic).

## How External Odometry Works in ArduPilot

### DDS Topic Subscription
ArduPilot subscribes to `/ap/tf` (topic name: `rt/ap/tf`) expecting `tf2_msgs/TFMessage` messages.

### Frame Validation
The code in `AP_DDS_External_Odom::is_odometry_frame()` checks for:
- Parent frame: `"odom"`
- Child frame: `"base_link"`

Only transforms matching these frames are used as external odometry.

### Coordinate Frame Conversion
ArduPilot converts from ROS ENU (East-North-Up) to NED (North-East-Down):
```cpp
translation = {
    x,      // East -> North
    -y,     // North -> -East  
    -z      // Up -> -Down
}
```

### Integration with EKF
The odometry data is passed to `AP_VisualOdom::handle_pose_estimate()`, which:
1. Applies position/rotation offsets (VISO_POS parameters)
2. Validates quality threshold (VISO_QUAL_MIN)
3. Feeds data to EKF3 for sensor fusion

## Usage

### Terminal 1: Launch Main Simulation
```bash
cd ~/ugv_ws
source install/setup.bash
ros2 launch ugv_description ugv_gz_sitl.launch.py
```

This starts:
- Gazebo with your rover
- ArduPilot SITL
- micro_ros_agent (DDS bridge)
- ros_gz_bridge (Gazebo->ROS bridge)
- RViz

### Terminal 2: Launch MAVProxy
```bash
mavproxy.py --master=tcp:127.0.0.1:5760 --sitl=127.0.0.1:5501 --out=127.0.0.1:14550
```

### Terminal 3: Launch Cartographer SLAM
```bash
cd ~/ugv_ws
source install/setup.bash
ros2 launch ugv_description cartographer.launch.py
```

This starts:
- Cartographer node (SLAM)
- Occupancy grid generator
- TF relay to ArduPilot
- RViz with SLAM visualization

## Verification

### Check Topics
```bash
# List all topics
ros2 topic list

# Echo TF (should show map->odom->base_link)
ros2 topic echo /tf --no-arr

# Echo AP TF (should show same as /tf)
ros2 topic echo /ap/tf --no-arr
```

### Check Transforms
```bash
# View TF tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo odom base_link
```

### MAVProxy Monitoring
In MAVProxy, check if ArduPilot is receiving visual odometry:
```
# Check EKF source status
param show EK3_SRC1_*

# Monitor visual odometry health
status
```

Look for messages like:
- "EKF3 IMU0 is using external nav data"
- Visual odometry quality indicators

## Troubleshooting

### No External Odometry Received
1. **Check TF is publishing:**
   ```bash
   ros2 topic hz /tf
   ros2 topic hz /ap/tf
   ```

2. **Check frame IDs are correct:**
   ```bash
   ros2 topic echo /tf --no-arr | grep frame_id
   # Should show "odom" and "base_link"
   ```

3. **Check ArduPilot DDS connection:**
   - Look for micro_ros_agent output showing topic subscriptions
   - Should see "rt/ap/tf" subscriber

4. **Check VISO parameters:**
   ```
   param show VISO_*
   param show EK3_SRC1_*
   ```

### Poor SLAM Performance
1. Check sensor data rates:
   ```bash
   ros2 topic hz /ugv/lidar
   ros2 topic hz /ugv/imu/data
   ros2 topic hz /ugv/odom
   ```

2. Adjust Cartographer parameters in `cartographer_rover.lua`:
   - `TRAJECTORY_BUILDER_2D.min_range` / `max_range`
   - `POSE_GRAPH.optimize_every_n_nodes`
   - `constraint_builder.min_score`

3. Check for TF warnings:
   ```bash
   ros2 run tf2_ros tf2_monitor
   ```

### Map Drift
If the map drifts over time:
1. Increase loop closure frequency:
   - Lower `POSE_GRAPH.optimize_every_n_nodes` (try 30-60)
2. Improve loop closure detection:
   - Increase `constraint_builder.max_constraint_distance`
   - Adjust `constraint_builder.min_score` (lower = more closures, higher = more reliable)

## Advanced Configuration

### Tuning EKF3 for Visual Odometry
In `mav.parm`, you can adjust:
```
VISO_POS_X       0.0    # Camera X offset from center of rotation
VISO_POS_Y       0.0    # Camera Y offset
VISO_POS_Z       0.0    # Camera Z offset
VISO_POS_NOISE   0.1    # Position measurement noise (m)
VISO_VEL_NOISE   0.1    # Velocity measurement noise (m/s)
VISO_YAW_NOISE   0.1    # Yaw measurement noise (rad)
VISO_DELAY_MS    50     # Sensor delay relative to IMU
```

### Switching Between GPS and Visual Odom
To use GPS instead of visual odometry:
```
param set EK3_SRC1_POSXY 3    # 3=GPS, 6=ExternalNav
param set EK3_SRC1_VELXY 3    # 3=GPS, 6=ExternalNav
```

To use both (GPS as backup):
```
param set EK3_SRC1_POSXY 6    # Primary: ExternalNav
param set EK3_SRC2_POSXY 3    # Backup: GPS
```

## Files Modified/Created

- ✅ `launch/slam/cartographer.launch.py` - Cartographer launch file
- ✅ `config/cartographer/cartographer_rover.lua` - Cartographer config
- ✅ `config/cartographer/cartographer_rover.rviz` - RViz config
- ✅ `setup.py` - Updated to install new files
- ⚙️ `mav.parm` - Already configured (EK3_SRC1_* = 3)

## References

- ArduPilot DDS Documentation: `/home/odinroast/ugv_ws/src/ardupilot/libraries/AP_DDS/README.md`
- Cartographer Documentation: https://google-cartographer-ros.readthedocs.io/
- ROS REP-105 (Coordinate Frames): https://www.ros.org/reps/rep-0105.html
- ArduPilot External Position: https://ardupilot.org/copter/docs/common-external-position-estimation.html
