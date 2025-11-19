# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This repository builds a ROS1-ROS2 bridge for Ubuntu 22.04 (Jammy) with ROS2 Humble, enabling communication between ROS1 Noetic and ROS2 Humble systems. The project is a WDS-specific fork that bridges MAVROS and custom topics for drone control systems.

**Architecture**: Multi-stage Docker build creates a pre-compiled bridge package that can be extracted and used on host systems. Supports both amd64 and arm64 architectures.

## Build Commands

### Build the Docker builder image:
```bash
docker build . -t ros-humble-ros1-bridge-builder

# Optional build arguments:
docker build . --build-arg ADD_ros_tutorials=0 -t ros-humble-ros1-bridge-builder  # disable ros-tutorials
docker build . --build-arg ADD_grid_map=1 -t ros-humble-ros1-bridge-builder       # add grid-map support
docker build . --build-arg ADD_example_custom_msgs=1 -t ros-humble-ros1-bridge-builder  # add custom messages
```

### Extract the pre-compiled bridge package:
```bash
docker run --rm ros-humble-ros1-bridge-builder | tar xvzf -
```

This creates `~/ros-humble-ros1-bridge/` containing the compiled bridge.

## Running the Bridge

### WDS Deployment (via Docker Compose):
```bash
# Start the bridge service
docker compose up

# Check logs
docker compose logs -f ros1_bridge
```

### Manual Bridge Execution:
```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash
source ~/ros-humble-ros1-bridge/install/local_setup.bash

# Run dynamic bridge (auto-discovers topics)
ros2 run ros1_bridge dynamic_bridge

# Or bridge all topics (useful for introspection with rostopic/ros2 topic)
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics

# Or use parameter bridge (requires config file)
rosparam load config/bridge_topic.yaml
ros2 run ros1_bridge parameter_bridge
```

**Important**: Always use `local_setup.bash` (not `setup.bash`) when sourcing the bridge overlay, since it was compiled in a Docker container with different underlay paths.

### Bridge Options:
```bash
ros2 run ros1_bridge dynamic_bridge --help
# Key options:
#   --show-introspection: Print introspection of both sides
#   --print-pairs: List supported ROS2 ↔ ROS1 conversion pairs
#   --bridge-all-topics: Bridge all topics bidirectionally
#   --bridge-all-1to2-topics: Bridge all ROS1 → ROS2
#   --bridge-all-2to1-topics: Bridge all ROS2 → ROS1
```

## SystemD Service Installation

```bash
cd scripts
./install_service.sh
```

This installs `wds_ros1_bridge.service` which:
- Depends on `docker.service` and `corvus_roscore.service`
- Runs `docker compose up` in foreground (Type=simple)
- Auto-restarts on failure (RestartSec=5s)

## Architecture & Key Concepts

### Docker Build Pipeline (Dockerfile):

1. **Base Setup** (steps 1-3): Install ROS2 Humble Desktop, temporarily remove ROS2 repos, resolve catkin conflicts
2. **ROS1 Installation** (steps 4-5): Install ROS1 Noetic Desktop Dev (includes roscore, tf, tf2)
3. **Message Package Compilation** (steps 6.1-6.3):
   - ros_tutorials: Bridges `example_interfaces` (e.g., AddTwoInts service)
   - grid_map: Bridges grid-map messages (requires navigation stack dependencies)
   - custom_msgs: Example of adding custom message types (must end with `_msgs` and use same package name in ROS1/ROS2)
4. **Bridge Compilation** (step 7): Sources all message overlays then builds `ros1_bridge` with memory-aware parallelism (`MAKEFLAGS="-j $MIN"`)
5. **RMW Setup** (step 8): Installs CycloneDDS and sets as default RMW implementation
6. **Packaging** (steps 9-10): Bundles ROS1 dependency libraries and creates tarball

**Critical Build Detail**: The bridge must be compiled with both ROS1 and ROS2 message packages sourced in the environment. This is why custom messages require both ROS1 and ROS2 versions compiled before building the bridge.

### Topic Configuration (config/bridge_topic.yaml):

Defines topics for `parameter_bridge` mode with QoS policies:
- MAVROS topics: `/mavros/local_position/pose`
- VOXL vision topics: `/voxl/pose/local`, `/voxl/qvio/error_code`
- Corvus drone topics: `/corvus/state`, `/corvus/is_armed`, `/corvus/navcontrol/*`
- WDS landing system: `/wds/start_landing`, `/wds/set_land`

Each topic specifies: `topic`, `type`, `queue_size`, and `qos` (reliability/durability)

### RMW Implementation:

Uses **CycloneDDS** (not FastDDS) as the RMW implementation. Set via:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

This is critical for cross-vendor DDS interoperability and is pre-configured in the Docker image.

## Adding Custom Messages

1. Create ROS1 and ROS2 packages with **identical names ending in `_msgs`**
2. Add compilation steps to Dockerfile (see steps 6.3 and 6.4 as examples):
   ```dockerfile
   # Compile ROS1 version
   cd /path/to/custom_msgs_ros1
   unset ROS_DISTRO
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

   # Compile ROS2 version
   cd /path/to/custom_msgs_ros2
   source /opt/ros/humble/setup.bash
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
3. Source both overlays before building the bridge in step 7
4. Verify with: `ros2 run ros1_bridge dynamic_bridge --print-pairs | grep YourMsg`

**Reference**: https://github.com/ros2/ros1_bridge/blob/master/doc/index.rst

### WDS Battery Message (Custom)

The repository includes a custom `wds_battery_msgs/WdsBattery` message for bridging battery status:

```bash
# Build with WDS battery message support
docker build . --build-arg ADD_wds_battery_msgs=1 -t ros-humble-ros1-bridge-builder
```

**Message fields** (simplified from sensor_msgs/BatteryState):
- `std_msgs/Header header`
- `float32 voltage` - Battery voltage in Volts
- `float32 current` - Current in Amperes
- `float32 charge` - Remaining charge in Ah
- `float32 capacity` - Current capacity in Ah
- `float32 design_capacity` - Design capacity in Ah
- `float32 percentage` - Charge percentage (0-100)
- `uint8 power_supply_status` - Status constants (charging, discharging, etc.)

**ROS1 Relay Node**: A relay node (`battery_relay_ros1`) converts `sensor_msgs/BatteryState` → `wds_battery_msgs/WdsBattery`. This runs on the ROS1 host (NOT in the bridge container) to avoid environment conflicts. See `wds_battery_msgs/README_WDS_BATTERY.md` for setup details.

**Topic**: `/wds/battery` is configured in `config/bridge_topic.yaml` with QoS parameters.

## Testing & Verification

### Check supported message pairs:
```bash
ros2 run ros1_bridge dynamic_bridge --print-pairs | grep -i <message_type>
```

### Test with ROS1 Noetic container:
```bash
# Start ROS1 container with GUI
rocker --x11 --user --privileged \
  --volume /dev/shm /dev/shm --network=host -- ros:noetic-ros-base-focal \
  'bash -c "sudo apt update; sudo apt install -y ros-noetic-rospy-tutorials tilix; tilix"'

# In container: start roscore
source /opt/ros/noetic/setup.bash
roscore

# In container: run talker
source /opt/ros/noetic/setup.bash
rosrun rospy_tutorials talker

# On host: run listener
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp listener
```

### Remote ROS1 master:
```bash
# If ROS1 master is on different machine (e.g., 192.168.1.208)
ROS_MASTER_URI='http://192.168.1.208:11311' ros2 run ros1_bridge dynamic_bridge

# Verify connectivity
nc -v -z 192.168.1.208 11311
```

## Important Notes

- **Memory Requirements**: Bridge compilation uses ~1GB per CPU core. Build parallelism is automatically limited by available memory.
- **Host System Requirements**: Install `ros-humble-desktop` on host to match builder image dependencies and avoid missing library errors.
- **Network Sharing**: ROS1 containers MUST use `--network=host` and share `/dev/shm` for bridge communication.
- **ARM64 Support**: Automatic pkgconfig path fixes applied for ARM64/aarch64 architectures.
- **Dynamic Bridge Limitation**: `ros2 topic echo <topic>` won't work without type specified unless using `--bridge-all-2to1-topics`, because topics are only bridged when subscribers exist on both sides.
