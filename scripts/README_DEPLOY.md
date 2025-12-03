# Bridge Deployment Script

Automated deployment script for WDS ROS1-ROS2 Bridge with Battery Relay.

## Quick Start

```bash
# Clone or pull latest
cd ~/wds
git clone <repo> ros-humble-ros1-bridge-builder
# OR
cd ~/wds/ros-humble-ros1-bridge-builder
git pull

# Run deployment
cd ~/wds/ros-humble-ros1-bridge-builder
./scripts/deploy_bridge.sh
```

## Deployment Modes

### 1. Full Deployment
- Pulls/builds Docker image
- Builds battery relay node
- Installs systemd services
- Copies configuration files
- Removes source code (optional)

### 2. Configuration Update Only
- Updates `bridge_topic.yaml`
- Updates `docker-compose.yml`
- Updates ROS_DOMAIN_ID

### 3. Service Reinstall Only
- Reinstalls systemd service files
- Updates bash aliases

## What Gets Deployed

### File Locations

```
/etc/wds/ros_bridge/              # Bridge deployment directory
├── docker-compose.yml            # Bridge container config
└── config/
    └── bridge_topic.yaml         # Topic mappings

/home/root/wds_battery_deployment/  # Relay node binaries (no source)
├── wds_battery_msgs_ros1_install/
└── battery_relay_ros1_install/

/etc/systemd/system/
├── wds_ros1_bridge.service       # Bridge service
└── wds_battery_relay.service     # Relay service
```

### Bash Aliases (Added to ~/.bashrc)

**Bridge Management:**
- `bridge-start` - Start bridge service
- `bridge-stop` - Stop bridge service
- `bridge-restart` - Restart bridge service
- `bridge-status` - Show bridge status
- `bridge-log` - Follow bridge logs

**Relay Management:**
- `relay-start` - Start battery relay
- `relay-stop` - Stop battery relay
- `relay-restart` - Restart battery relay
- `relay-status` - Show relay status
- `relay-log` - Follow relay logs

## Usage Examples

### First Time Deployment

```bash
cd ~/wds/ros-humble-ros1-bridge-builder
./scripts/deploy_bridge.sh

# Select: 1) Full deployment
# Select: 1) Pull from Docker Hub (or 2 to build)
# Enter image: wdsdrones/wds_ros1_to_ros2_humble_bridge
# Enter tag: latest
# Enter ROS_DOMAIN_ID: 26
# Remove source code: y

# Start services
source ~/.bashrc
relay-start
bridge-start

# Check status
relay-status
bridge-status

# View logs
relay-log     # Ctrl+C to exit
bridge-log    # Ctrl+C to exit
```

### Update Configuration Only

```bash
cd ~/wds/ros-humble-ros1-bridge-builder
git pull
./scripts/deploy_bridge.sh

# Select: 2) Update bridge configuration only
# Enter ROS_DOMAIN_ID: 27

# Restart bridge to apply changes
bridge-restart
```

### Reinstall Services After Modification

```bash
# After editing service files
cd ~/wds/ros-humble-ros1-bridge-builder
./scripts/deploy_bridge.sh

# Select: 3) Reinstall services only

source ~/.bashrc
bridge-restart
relay-restart
```

## Verification

### Check Services Are Running

```bash
relay-status
bridge-status
```

### Verify Battery Message Bridging

**On ROS1 side:**
```bash
# Check relay is publishing
rostopic echo /wds/battery
```

**On ROS2 side:**
```bash
# Check bridge is forwarding
ros2 topic list | grep battery
ros2 topic echo /wds/battery wds_battery_msgs/msg/WdsBattery
```

### Check Bridge Recognizes Custom Message

```bash
docker exec -it <bridge-container-id> bash
source /opt/ros/humble/setup.bash
source /ros-humble-ros1-bridge/install/setup.bash
ros2 run ros1_bridge dynamic_bridge --print-pairs | grep -i wdsbattery

# Expected output:
#   - 'wds_battery_msgs/msg/WdsBattery' (ROS 2) <=> 'wds_battery_msgs/WdsBattery' (ROS 1)
```

## Troubleshooting

### Bridge Says "No template specialization" for WdsBattery

**Problem:** Docker image wasn't built with custom message support.

**Solution:**
```bash
cd ~/wds/ros-humble-ros1-bridge-builder
./scripts/deploy_bridge.sh
# Select: 1) Full deployment
# Select: 2) Build from source
```

### Relay Node Not Starting

**Check service status:**
```bash
relay-status
relay-log
```

**Common issues:**
- ROS1 Noetic not installed: `sudo apt install ros-noetic-desktop`
- roscore not running: Check `corvus_roscore.service`

### Bridge Not Starting

**Check service status:**
```bash
bridge-status
bridge-log
```

**Common issues:**
- Docker not running: `sudo systemctl start docker`
- Wrong working directory: Check `/etc/wds/ros_bridge` exists

### Configuration Changes Not Applied

Restart services after config changes:
```bash
bridge-restart
relay-restart
```

## ROS_DOMAIN_ID Guidelines

Each drone must have a **unique** ROS_DOMAIN_ID:

- Valid range: 20-100 (for CycloneDDS)
- Recommended assignments:
  - First drone: 26
  - Second drone: 27
  - Third drone: 28
  - etc.

Change via deployment script or manually edit:
```bash
sudo nano /etc/wds/ros_bridge/docker-compose.yml
# Update: ROS_DOMAIN_ID=XX
bridge-restart
```

## Manual Commands

If you need to manage services without aliases:

```bash
# Bridge
sudo systemctl start wds_ros1_bridge.service
sudo systemctl stop wds_ros1_bridge.service
sudo systemctl status wds_ros1_bridge.service
sudo journalctl -u wds_ros1_bridge.service -f

# Relay
sudo systemctl start wds_battery_relay.service
sudo systemctl stop wds_battery_relay.service
sudo systemctl status wds_battery_relay.service
sudo journalctl -u wds_battery_relay.service -f
```

## Notes

- Source code is removed after deployment (binaries remain)
- Services auto-start on boot (enabled by default)
- Bridge depends on Docker and roscore services
- Relay depends on roscore service
- All configuration lives in `/etc/wds/ros_bridge/`
