# Bridge Deployment Script

Automated deployment script for the WDS ROS1-ROS2 Bridge.

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
- Installs systemd service
- Copies configuration files

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

/etc/systemd/system/
└── wds_ros1_bridge.service       # Bridge service
```

### Bash Aliases (Added to ~/.bashrc)

**Bridge Management:**
- `bridge-start` - Start bridge service
- `bridge-stop` - Stop bridge service
- `bridge-restart` - Restart bridge service
- `bridge-status` - Show bridge status
- `bridge-log` - Follow bridge logs

## Usage Examples

### First Time Deployment

```bash
cd ~/wds/ros-humble-ros1-bridge-builder
./scripts/deploy_bridge.sh

# Select: 1) Full deployment
# Select: 1) Pull from Docker Hub (or 2 to build)
# Enter image: wdsdrones/wds_ros1_to_ros2_humble_bridge
# Enter tag: deploy    - ***** PLEASE FOR DEPLOYMENT MAKE SURE U CALL THE TAG: DEPLOY!!! ********
# Enter ROS_DOMAIN_ID: 26

# Start service
source ~/.bashrc
bridge-start

# Check status
bridge-status

# View logs
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
```

## Verification

### Check Service Is Running

```bash
bridge-status
```

### Verify Topic Bridging

**On ROS2 side:**
```bash
# Check bridge is forwarding
ros2 topic list
```

**On ROS1 side:**
```bash
# Check topics are visible
rostopic list
```

## Troubleshooting

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

Restart the service after config changes:
```bash
bridge-restart
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

If you need to manage the service without aliases:

```bash
# Bridge
sudo systemctl start wds_ros1_bridge.service
sudo systemctl stop wds_ros1_bridge.service
sudo systemctl status wds_ros1_bridge.service
sudo journalctl -u wds_ros1_bridge.service -f
```

## Notes

- Service auto-starts on boot (enabled by default)
- Bridge depends on Docker and roscore services
- All configuration lives in `/etc/wds/ros_bridge/`