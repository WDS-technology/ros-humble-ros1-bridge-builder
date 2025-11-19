# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This repository builds a ROS1-ROS2 bridge for Ubuntu 22.04 (Jammy) with ROS2 Humble, enabling communication between ROS1 Noetic and ROS2 Humble systems. The project is a WDS-specific fork that bridges MAVROS and custom drone control topics.

**Current Branch**: `stable_w_landing` - Production-ready branch with landing system integration and optimized minimal Docker image.

**Architecture**: Multi-stage Docker build creates a pre-compiled bridge package. Supports both amd64 and arm64 architectures. Two build variants available: minimal (~1-1.5GB) for production drones, and full (~4.3GB) with all optional packages.

## Quick Start - Deployment Script

The `scripts/deploy_bridge.sh` script is the primary tool for all installation and management tasks:

```bash
# Full deployment (pull pre-built image + install)
./scripts/deploy_bridge.sh deploy

# Build locally with minimal Dockerfile and install
./scripts/deploy_bridge.sh deploy-local --minimal

# Build locally with full Dockerfile
./scripts/deploy_bridge.sh deploy-local --full

# Check service status
./scripts/deploy_bridge.sh status

# Uninstall everything
./scripts/deploy_bridge.sh uninstall -y
```

**Script Features**:
- Prerequisite checking (Docker, Docker Compose, directories)
- Image pulling from Docker Hub OR local building
- Configuration file installation to `/etc/wds/bridge/`
- SystemD service installation with proper dependencies
- Status checking and service management
- Colored output with progress indicators

## Build Commands

### Build Options:

**Minimal Build** (Recommended for production):
```bash
docker build -f Dockerfile_minimal -t ros-humble-ros1-bridge-builder .
```
- Size: ~1-1.5GB (vs 4.31GB full)
- Only includes essential ROS2 packages (ros-core, geometry-msgs, std-msgs, sensor-msgs)
- No desktop dependencies
- Limits build parallelism to max 2 jobs
- Sets `ROS_PACKAGE_PATH=/opt/ros/noetic/share` with symlinks to message packages
- Removes build artifacts and src directories from tarball

**Full Build** (Development/Extended features):
```bash
docker build -f Dockerfile_full -t ros-humble-ros1-bridge-builder .
```
- Size: ~4.31GB
- Includes ROS2 Desktop
- Supports build args for optional packages:
  - `--build-arg ADD_ros_tutorials=1` (default: enabled)
  - `--build-arg ADD_grid_map=1`
  - `--build-arg ADD_example_custom_msgs=1`

### Extract the Bridge Package:
```bash
docker run --rm ros-humble-ros1-bridge-builder | tar xvzf -
```

This creates `~/ros-humble-ros1-bridge/` containing the compiled bridge.

## Running the Bridge

### WDS Production Deployment (Docker Compose):
```bash
# Start the bridge service
docker compose up

# Check logs
docker compose logs -f ros1_bridge
```

**Docker Compose Configuration**:
- Image: `wdsdrones/wds_ros1_to_ros2_humble_bridge:latest`
- RMW: CycloneDDS (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`)
- ROS Domain: 23 (changed from 22 in earlier versions)
- Network: Host mode (required for ROS1 communication)
- Volumes: Mounts `config/` to `/home/topics` for topic configuration
- Command: Uses `parameter_bridge` (not `dynamic_bridge`) with YAML config

### Manual Bridge Execution:
```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash
source ~/ros-humble-ros1-bridge/install/local_setup.bash

# Run dynamic bridge (auto-discovers topics)
ros2 run ros1_bridge dynamic_bridge

# Or bridge all topics (useful for introspection with rostopic/ros2 topic)
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics

# Or use parameter bridge (requires config file - WDS production method)
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

### Using Deploy Script (Recommended):
```bash
./scripts/deploy_bridge.sh install
```

### Manual Installation:
```bash
cd scripts
./install_service.sh
```

**Service Details** (`wds_ros1_bridge.service`):
- **Type**: `simple` (runs docker compose in foreground)
- **Dependencies**: `docker.service` and `corvus_roscore.service`
- **ExecStart**: `/usr/bin/docker compose up` (no `-d` flag)
- **ExecStop**: `/usr/bin/docker compose down --remove-orphans`
- **Restart Policy**: `on-failure` with 5s delay
- **Working Directory**: Points to WDS_generalTools path

**Enable and Start**:
```bash
sudo systemctl enable wds_ros1_bridge.service
sudo systemctl start wds_ros1_bridge.service
sudo systemctl status wds_ros1_bridge.service
```

## Architecture & Key Concepts

### Dockerfile Variants:

**Dockerfile_minimal** (Current production standard):
1. **Minimal ROS2 Base**: Only `ros-humble-ros-core` + essential message packages (geometry-msgs, std-msgs, sensor-msgs, nav-msgs, actionlib-msgs, diagnostic-msgs, shape-msgs, stereo-msgs, trajectory-msgs, visualization-msgs)
2. **ROS1 Installation**: Full `ros-desktop-dev` for ROS1 Noetic
3. **Critical: ROS2 Message Package Reinstallation**: After installing ROS1, reinstalls all ROS2 message packages with `--reinstall` flag to ensure they're available for bridge compilation
4. **ROS_PACKAGE_PATH Setup**: Creates `/opt/ros/noetic/share` with symlinks to message packages to satisfy bridge requirements
5. **Bridge Compilation**: Uses smith-doug/ros1_bridge `action_bridge_humble` branch with limited parallelism (max 2 jobs)
6. **Verification**: Checks that message packages are available and verifies bridge has message mappings with `--print-pairs`
7. **Cleanup**: Removes build/ and src/ directories, cleans apt cache
8. **Size Optimization**: Result is ~1.5-2.0GB vs 4.31GB full version

**Dockerfile_full** (Original, for extended features):
- Full ROS2 Desktop installation (includes ALL message packages by default)
- Supports ros_tutorials, grid_map, and custom message packages
- Compiles additional ROS1 dependencies (navigation stack, filters, etc.)
- Memory-aware parallelism calculation

**CRITICAL: Why Message Package Reinstallation is Required**:

The ros1_bridge is a **compile-time code generator**, not just a runtime bridge. It generates C++ template specializations for message type conversions by introspecting both ROS1 and ROS2 message packages present during compilation.

The build sequence that causes problems:
1. Install ROS2 message packages → apt clean (removes package metadata)
2. Remove ROS2 apt repos
3. Install ROS1 (which has its own versions of these messages)
4. Restore ROS2 apt repos
5. Build bridge → **FAILS to generate message mappings** because ROS2 packages aren't properly available

The fix in Dockerfile_minimal:
- After restoring ROS2 apt repos (step 6), **reinstall all ROS2 message packages with --reinstall**
- This ensures ROS2 message package metadata and files are present for bridge compilation
- Result: Bridge generates proper message type mappings and topics work at runtime

**Common Error Without This Fix**:
```
failed to create bidirectional bridge for topic '/mavros/local_position/pose'
with ROS 2 type 'geometry_msgs/msg/PoseStamped': No template specialization for the pair
```

See `DOCKERFILE_MINIMAL_FIX.md` for detailed explanation.

### Topic Configuration (config/bridge_topic.yaml):

WDS uses **parameter_bridge** mode with explicit topic configuration (not dynamic_bridge). Topics include:

**MAVROS Integration**:
- `/mavros/local_position/pose` - Position data from flight controller

**VOXL Vision System**:
- `/voxl/pose/local` - Local position from VOXL
- `/voxl/qvio/error_code` - QVIO status codes

**Corvus Drone Control**:
- `/corvus/state` - Drone state machine status
- `/corvus/is_armed` - Arming status
- `/corvus/pose/fixed` - Fixed frame pose
- `/corvus/powermodule/battery` - Battery telemetry
- `/corvus/payload/set_recording` - Camera recording control
- `/corvus/qvio/soft_reset` - Vision system reset
- `/corvus/navcontrol/*` - Navigation control commands
  - `flight_started`, `flight_finished`
  - `flight/resume`, `flight/pause`
  - `command/emergency_land`, `command/external_command`
  - `task/next_command`

**WDS Landing System** (new in stable_w_landing):
- `/wds/start_landing` - Initiate landing sequence
- `/wds/set_land` - Landing command

**Camera Topics**:
- `/corvus/cam/hires/CameraInfo`
- `/corvus/cam/hires2/CameraInfo`
- `/corvus/cam/tracking/CameraInfo`

**CPU Monitoring**:
- `/corvus/cpu/temperature`

Each topic specifies: `topic`, `type`, `queue_size`, and `qos` (reliability/durability).

### RMW Implementation:

Uses **CycloneDDS** (not FastDDS) as the RMW implementation. Set via:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

**ROS Domain ID**: 23 (changed from 22 in main branch)

This is critical for cross-vendor DDS interoperability and is pre-configured in the Docker image.

## Branch Structure

- **main**: Original upstream-aligned branch with full build
- **stable**: WDS stable production branch (merged to main periodically)
- **stable_w_landing**: Current active branch with landing system integration and minimal Docker build
- **docker_pull**: Branch for Docker Hub integration

**When working on this repository**: Most development happens on `stable_w_landing`, then merges to `stable`, then PRs to `main`.

## Adding Custom Messages

1. Create ROS1 and ROS2 packages with **identical names ending in `_msgs`**
2. Add compilation steps to Dockerfile_full (see steps 6.3 as example):
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

**Note**: The minimal build does NOT support custom messages beyond standard ROS message packages. Use Dockerfile_full for custom message support.

**Reference**: https://github.com/ros2/ros1_bridge/blob/master/doc/index.rst

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

## Log Files

The repository contains build logs for reference:
- `build.log` - Standard minimal build log
- `build_with_ros_package_path.log` - Build log with ROS_PACKAGE_PATH configuration

These are useful for debugging build issues or understanding the compilation process.

## Important Notes

- **Minimal vs Full**: Use `Dockerfile_minimal` for production deployments. Only use `Dockerfile_full` if you need ros_tutorials, grid_map, or custom messages.
- **Memory Requirements**: Full build uses ~1GB per CPU core. Minimal build limits parallelism to 2 jobs to reduce memory pressure.
- **Host System Requirements**: For full build, install `ros-humble-desktop` on host to match builder image dependencies.
- **Network Sharing**: ROS1 containers MUST use `--network=host` and share `/dev/shm` for bridge communication.
- **ARM64 Support**: Automatic pkgconfig path fixes applied for ARM64/aarch64 architectures.
- **Dynamic vs Parameter Bridge**: WDS production uses `parameter_bridge` with YAML config (not `dynamic_bridge`), providing explicit control over bridged topics and QoS policies.
- **Service Dependencies**: The bridge service requires `corvus_roscore.service` to be running first (ROS1 master on the drone).
- **Docker Compose Mode**: Service runs docker compose in foreground (Type=simple), not detached mode. This allows systemd to properly track and restart the service.
