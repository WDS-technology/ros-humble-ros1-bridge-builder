# WDS Battery Message Setup

This directory contains the custom WdsBattery message implementation for bridging battery data between ROS1 Noetic and ROS2 Humble.

## Overview

The WDS Battery system provides a simplified battery status message that bridges between ROS1 and ROS2. It includes:
- Custom message definitions for both ROS1 and ROS2
- A ROS1 relay node to convert standard `sensor_msgs/BatteryState` to `wds_battery_msgs/WdsBattery`

## Directory Structure

```
wds_battery_msgs/
├── wds_battery_msgs_ros1/          # ROS1 Noetic message package
│   ├── msg/
│   │   └── WdsBattery.msg          # Message definition
│   ├── CMakeLists.txt
│   └── package.xml
│
├── wds_battery_msgs_ros2/          # ROS2 Humble message package
│   ├── msg/
│   │   └── WdsBattery.msg          # Message definition (identical)
│   ├── CMakeLists.txt
│   └── package.xml
│
└── battery_relay_ros1/             # ROS1 relay node
    ├── src/
    │   └── battery_relay_node.cpp  # Conversion node
    ├── CMakeLists.txt
    └── package.xml
```

## Message Definition

**wds_battery_msgs/WdsBattery** contains only essential battery metrics:

```
std_msgs/Header header
float32 voltage              # Voltage in Volts (V)
float32 current              # Current in Amperes (A)
float32 charge               # Remaining charge in Ah
float32 capacity             # Current capacity in Ah
float32 design_capacity      # Design capacity in Ah
float32 percentage           # Charge percentage (0-100)
uint8 power_supply_status    # Status (charging, discharging, etc.)
```

## Building the Docker Image

Add the WDS battery messages to your bridge by building with the custom build argument:

```bash
cd /home/jacob/ros-humble-ros1-bridge-builder

# Build with WDS battery message support
docker build . --build-arg ADD_wds_battery_msgs=1 -t ros-humble-ros1-bridge-builder
```

## Deploying the Bridge

After building, extract and use the bridge:

```bash
# Extract the bridge package
docker run --rm ros-humble-ros1-bridge-builder | tar xvzf -

# Or use Docker Compose (recommended for WDS deployment)
docker compose up
```

## Running the ROS1 Relay Node

The relay node converts `sensor_msgs/BatteryState` to `wds_battery_msgs/WdsBattery`.

**IMPORTANT:** This node must run on the HOST system in a pure ROS1 environment, NOT inside the bridge container.

### Build the Relay Node (on ROS1 host)

```bash
# 1. Build the message package first
cd /path/to/wds_battery_msgs/wds_battery_msgs_ros1
source /opt/ros/noetic/setup.bash
colcon build

# 2. Build the relay node
cd /path/to/wds_battery_msgs/battery_relay_ros1
source /opt/ros/noetic/setup.bash
source ../wds_battery_msgs_ros1/install/setup.bash
colcon build
```

### Run the Relay Node

```bash
# Source ROS1 and the packages
source /opt/ros/noetic/setup.bash
source /path/to/wds_battery_msgs/wds_battery_msgs_ros1/install/setup.bash
source /path/to/wds_battery_msgs/battery_relay_ros1/install/setup.bash

# Run with default topics
rosrun battery_relay_ros1 battery_relay_node

# Or specify custom topics
rosrun battery_relay_ros1 battery_relay_node \
  _input_topic:=/corvus/powermodule/battery \
  _output_topic:=/wds/battery
```

### Relay Node Parameters

- `~input_topic` (default: `/corvus/powermodule/battery`)
  - Subscribe to this topic for sensor_msgs/BatteryState messages

- `~output_topic` (default: `/wds/battery`)
  - Publish converted wds_battery_msgs/WdsBattery messages here

## Verification

### 1. Verify Message Type is Registered

```bash
source /opt/ros/humble/setup.bash
source ~/ros-humble-ros1-bridge/install/local_setup.bash

# Check that WdsBattery is in the bridge's supported pairs
ros2 run ros1_bridge dynamic_bridge --print-pairs | grep -i wdsbattery

# Expected output:
#   - 'wds_battery_msgs/msg/WdsBattery' (ROS 2) <=> 'wds_battery_msgs/WdsBattery' (ROS 1)
```

### 2. Test End-to-End Flow

**Terminal 1 (ROS1 host):** Run relay node
```bash
source /opt/ros/noetic/setup.bash
source /path/to/wds_battery_msgs/wds_battery_msgs_ros1/install/setup.bash
source /path/to/wds_battery_msgs/battery_relay_ros1/install/setup.bash
rosrun battery_relay_ros1 battery_relay_node
```

**Terminal 2 (ROS1 host):** Check relay output
```bash
source /opt/ros/noetic/setup.bash
source /path/to/wds_battery_msgs/wds_battery_msgs_ros1/install/setup.bash
rostopic echo /wds/battery
```

**Terminal 3 (ROS2 host):** Check bridged topic
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /wds/battery wds_battery_msgs/msg/WdsBattery
```

## Troubleshooting

### Issue: Bridge doesn't recognize WdsBattery message

**Solution:** Make sure you built the Docker image with `--build-arg ADD_wds_battery_msgs=1`

```bash
docker build . --build-arg ADD_wds_battery_msgs=1 -t ros-humble-ros1-bridge-builder
docker run --rm ros-humble-ros1-bridge-builder | tar xvzf -
```

### Issue: Relay node can't find wds_battery_msgs

**Solution:** Build the message package first, then source it before building the relay node

```bash
cd wds_battery_msgs/wds_battery_msgs_ros1
source /opt/ros/noetic/setup.bash
colcon build
source install/setup.bash

cd ../battery_relay_ros1
colcon build
```

### Issue: "ImportError: cannot import name 'Log'" when running rostopic

**Solution:** This error occurs when ROS1 tools try to use ROS2 packages. The relay node must run in a PURE ROS1 environment on the host, not inside the bridge container which has both ROS1 and ROS2.

### Issue: Battery topic not bridging from ROS1 to ROS2

**Checklist:**
1. ✅ Relay node is running on ROS1 host
2. ✅ Bridge container is running (`docker compose up`)
3. ✅ `/wds/battery` topic is configured in `config/bridge_topic.yaml`
4. ✅ Message type appears in `--print-pairs` output

## Integration with WDS System

The battery relay and bridge are configured to work with the WDS drone control stack:

- **Input:** `/corvus/powermodule/battery` (sensor_msgs/BatteryState from Corvus power module)
- **Relay:** Converts to `/wds/battery` (wds_battery_msgs/WdsBattery)
- **Bridge:** Bridges `/wds/battery` to ROS2 Humble for WDS Drone Manager

## Configuration Files

The `/wds/battery` topic is configured in `config/bridge_topic.yaml`:

```yaml
- topic: /wds/battery
  type: wds_battery_msgs/msg/WdsBattery
  queue_size: 10
  qos:
    reliability: reliable
    durability: volatile
```

## Alternative: Try QoS Fix First

Before using the custom message, you can try fixing the existing `sensor_msgs/BatteryState` bridge configuration by ensuring it has proper QoS parameters in `config/bridge_topic.yaml`:

```yaml
- topic: /corvus/powermodule/battery
  type: sensor_msgs/msg/BatteryState
  queue_size: 10
  qos:
    reliability: reliable
    durability: volatile
```

If this works, you may not need the custom WdsBattery message at all.
