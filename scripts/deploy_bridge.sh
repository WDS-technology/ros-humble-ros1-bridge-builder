#!/bin/bash

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Resolve script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

# Deployment paths
BRIDGE_DEPLOY_DIR="/etc/wds/ros_bridge"
BATTERY_DEPLOY_DIR="${BRIDGE_DEPLOY_DIR}/battery_relay"
SERVICE_INSTALL_SCRIPT="${SCRIPT_DIR}/install_service.sh"

# Default Docker image
DEFAULT_IMAGE="wdsdrones/wds_ros1_to_ros2_humble_bridge"
DEFAULT_TAG="latest"

echo "========================================================================="
echo "   WDS ROS1-ROS2 Bridge Deployment Script"
echo "========================================================================="
echo ""

# ============================================================================
# FUNCTION: Print section header
# ============================================================================
print_header() {
    echo ""
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}  $1${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
}

# ============================================================================
# FUNCTION: Print success message
# ============================================================================
print_success() {
    echo -e "${GREEN}[✓]${NC} $1"
}

# ============================================================================
# FUNCTION: Print error message
# ============================================================================
print_error() {
    echo -e "${RED}[✗]${NC} $1"
}

# ============================================================================
# FUNCTION: Print warning message
# ============================================================================
print_warning() {
    echo -e "${YELLOW}[!]${NC} $1"
}

# ============================================================================
# FUNCTION: Print info message
# ============================================================================
print_info() {
    echo -e "${BLUE}[i]${NC} $1"
}

# ============================================================================
# STEP 0: Deployment Mode Selection
# ============================================================================
print_header "Deployment Mode Selection"

echo "What would you like to do?"
echo ""
echo "  1) Full deployment (Docker image + relay node + services)"
echo "  2) Update bridge configuration only (topics.yaml, docker-compose.yml)"
echo "  3) Reinstall services only"
echo "  4) Build and install relay node only (no Docker image)"
echo ""
read -p "Select [1-4]: " DEPLOY_MODE

case $DEPLOY_MODE in
    1)
        FULL_DEPLOY=true
        CONFIG_ONLY=false
        SERVICE_ONLY=false
        RELAY_ONLY=false
        ;;
    2)
        FULL_DEPLOY=false
        CONFIG_ONLY=true
        SERVICE_ONLY=false
        RELAY_ONLY=false
        ;;
    3)
        FULL_DEPLOY=false
        CONFIG_ONLY=false
        SERVICE_ONLY=true
        RELAY_ONLY=false
        ;;
    4)
        FULL_DEPLOY=false
        CONFIG_ONLY=false
        SERVICE_ONLY=false
        RELAY_ONLY=true
        ;;
    *)
        print_error "Invalid selection"
        exit 1
        ;;
esac

# ============================================================================
# STEP 1: Docker Image Management (Full Deploy Only)
# ============================================================================
if [ "$FULL_DEPLOY" = true ]; then
    print_header "Docker Image Management"

    echo "How do you want to get the Docker image?"
    echo ""
    echo "  1) Pull from Docker Hub (recommended)"
    echo "  2) Build from source (requires ~10 minutes)"
    echo ""
    read -p "Select [1-2]: " IMAGE_ACTION

    case $IMAGE_ACTION in
        1)
            # Pull from registry
            read -p "Enter Docker image name [${DEFAULT_IMAGE}]: " IMAGE_NAME
            IMAGE_NAME=${IMAGE_NAME:-$DEFAULT_IMAGE}

            read -p "Enter image tag [${DEFAULT_TAG}]: " IMAGE_TAG
            IMAGE_TAG=${IMAGE_TAG:-$DEFAULT_TAG}

            DOCKER_IMAGE="${IMAGE_NAME}:${IMAGE_TAG}"

            print_info "Pulling image: ${DOCKER_IMAGE}"
            if docker pull "${DOCKER_IMAGE}"; then
                print_success "Image pulled successfully"
            else
                print_error "Failed to pull image"
                exit 1
            fi
            ;;
        2)
            # Build from source
            read -p "Enter Docker image name to create [${DEFAULT_IMAGE}]: " IMAGE_NAME
            IMAGE_NAME=${IMAGE_NAME:-$DEFAULT_IMAGE}

            read -p "Enter image tag to create [${DEFAULT_TAG}]: " IMAGE_TAG
            IMAGE_TAG=${IMAGE_TAG:-$DEFAULT_TAG}

            DOCKER_IMAGE="${IMAGE_NAME}:${IMAGE_TAG}"

            print_info "Building image with WDS battery message support..."
            print_info "This will take approximately 10 minutes..."

            cd "${REPO_ROOT}"
            if docker build . --build-arg ADD_wds_battery_msgs=1 -t "${DOCKER_IMAGE}"; then
                print_success "Image built successfully: ${DOCKER_IMAGE}"
            else
                print_error "Failed to build image"
                exit 1
            fi
            ;;
        *)
            print_error "Invalid selection"
            exit 1
            ;;
    esac
fi

# ============================================================================
# STEP 2: ROS Domain ID Configuration
# ============================================================================
if [ "$FULL_DEPLOY" = true ] || [ "$CONFIG_ONLY" = true ]; then
    print_header "ROS Domain ID Configuration"

    echo "ROS_DOMAIN_ID isolates ROS 2 network traffic between drones."
    echo "Each drone must have a UNIQUE ID (20-100 for CycloneDDS)."
    echo ""

    while true; do
        read -p "Enter ROS_DOMAIN_ID [20-100, default: 26]: " ROS_DOMAIN_ID
        ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-26}

        if [[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] && [ "$ROS_DOMAIN_ID" -ge 20 ] && [ "$ROS_DOMAIN_ID" -le 100 ]; then
            print_success "Selected ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
            break
        else
            print_error "Invalid input. Please enter a number between 20 and 100."
        fi
    done
fi

# ============================================================================
# STEP 3: Create Deployment Directory Structure
# ============================================================================
if [ "$FULL_DEPLOY" = true ] || [ "$CONFIG_ONLY" = true ]; then
    print_header "Creating Deployment Directories"

    print_info "Creating ${BRIDGE_DEPLOY_DIR}"
    sudo mkdir -p "${BRIDGE_DEPLOY_DIR}"
    sudo chown -R $(whoami):$(whoami) "${BRIDGE_DEPLOY_DIR}"
    print_success "Created ${BRIDGE_DEPLOY_DIR}"
fi

# ============================================================================
# STEP 4: Copy Bridge Configuration Files
# ============================================================================
if [ "$FULL_DEPLOY" = true ] || [ "$CONFIG_ONLY" = true ]; then
    print_header "Copying Bridge Configuration"

    # Copy docker-compose.yml and update it
    print_info "Copying docker-compose.yml"
    cp "${REPO_ROOT}/docker-compose.yml" "${BRIDGE_DEPLOY_DIR}/docker-compose.yml"

    # Update ROS_DOMAIN_ID in docker-compose.yml
    sed -i "s/ROS_DOMAIN_ID=[0-9]*/ROS_DOMAIN_ID=${ROS_DOMAIN_ID}/" "${BRIDGE_DEPLOY_DIR}/docker-compose.yml"

    # Update image name if doing full deploy
    if [ "$FULL_DEPLOY" = true ]; then
        sed -i "s|image:.*|image: ${DOCKER_IMAGE}|" "${BRIDGE_DEPLOY_DIR}/docker-compose.yml"
    fi

    print_success "Updated docker-compose.yml with ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

    # Copy config directory
    print_info "Copying config directory (topics.yaml)"
    cp -r "${REPO_ROOT}/config" "${BRIDGE_DEPLOY_DIR}/"
    print_success "Copied bridge configuration files"

    # Update volume paths in docker-compose.yml to point to deployment location
    sed -i "s|/home/root/wds/ros-humble-ros1-bridge-builder/config/|${BRIDGE_DEPLOY_DIR}/config/|g" \
        "${BRIDGE_DEPLOY_DIR}/docker-compose.yml"
    print_success "Updated volume paths in docker-compose.yml"
fi

# ============================================================================
# STEP 5: Build Battery Relay Node (Full Deploy or Relay Only)
# ============================================================================
if [ "$FULL_DEPLOY" = true ] || [ "$RELAY_ONLY" = true ]; then
    print_header "Building Battery Relay Node"

    # Detect ROS1 distribution (Noetic or Melodic)
    ROS1_DISTRO=""
    if [ -f "/opt/ros/noetic/setup.bash" ]; then
        ROS1_DISTRO="noetic"
    elif [ -f "/opt/ros/melodic/setup.bash" ]; then
        ROS1_DISTRO="melodic"
    else
        print_error "ROS1 not found. Please install ros-noetic-desktop or ros-melodic-desktop"
        exit 1
    fi

    print_info "Detected ROS1 ${ROS1_DISTRO}"
    ROS1_SETUP="/opt/ros/${ROS1_DISTRO}/setup.bash"

    # Check for required build tools
    print_info "Checking for required build tools..."
    if ! command -v catkin_make &> /dev/null; then
        print_error "catkin_make not found. Installing ros-${ROS1_DISTRO}-catkin..."
        sudo apt-get update
        sudo apt-get install -y ros-${ROS1_DISTRO}-catkin python3-catkin-tools
    fi

    if ! command -v cmake &> /dev/null; then
        print_error "cmake not found. Installing cmake..."
        sudo apt-get update
        sudo apt-get install -y cmake build-essential
    fi

    # Create a catkin workspace for ROS1 packages
    print_info "Creating catkin workspace for ROS1 packages..."
    CATKIN_WS="${REPO_ROOT}/wds_battery_msgs/catkin_ws"

    # Clean old workspace if it exists
    if [ -d "${CATKIN_WS}/build" ] || [ -d "${CATKIN_WS}/devel" ]; then
        print_info "Cleaning old catkin workspace..."
        rm -rf "${CATKIN_WS}/build" "${CATKIN_WS}/devel"
    fi

    mkdir -p "${CATKIN_WS}/src"

    # Link packages into workspace
    ln -sf "${REPO_ROOT}/wds_battery_msgs/wds_battery_msgs_ros1" "${CATKIN_WS}/src/"
    ln -sf "${REPO_ROOT}/wds_battery_msgs/battery_relay_ros1" "${CATKIN_WS}/src/"

    print_info "Building ROS1 packages with catkin_make..."
    cd "${CATKIN_WS}"
    source ${ROS1_SETUP}
    catkin_make -DCMAKE_BUILD_TYPE=Release

    if [ -d "devel" ]; then
        print_success "ROS1 packages built successfully"
    else
        print_error "Failed to build ROS1 packages"
        exit 1
    fi

    # ============================================================================
    # STEP 6: Deploy Battery Relay Binaries
    # ============================================================================
    print_header "Deploying Battery Relay Binaries"

    print_info "Creating deployment directory: ${BATTERY_DEPLOY_DIR}"
    mkdir -p "${BATTERY_DEPLOY_DIR}"

    print_info "Copying compiled binaries from catkin workspace (no source code)..."
    # Copy the entire devel space which contains all compiled libraries and executables
    cp -r "${CATKIN_WS}/devel" "${BATTERY_DEPLOY_DIR}/devel"

    print_success "Battery relay binaries deployed to ${BATTERY_DEPLOY_DIR}"
    print_info "Deployed devel space: ${BATTERY_DEPLOY_DIR}/devel"

    # No need to create service file here - will use install_service.sh
fi

# ============================================================================
# STEP 7: Update Service Files with Deployment Paths
# ============================================================================
if [ "$FULL_DEPLOY" = true ] || [ "$SERVICE_ONLY" = true ] || [ "$RELAY_ONLY" = true ]; then
    print_header "Updating Service Files"

    # Detect ROS1 distro for service file
    if [ -z "${ROS1_DISTRO}" ]; then
        if [ -f "/opt/ros/noetic/setup.bash" ]; then
            ROS1_DISTRO="noetic"
        elif [ -f "/opt/ros/melodic/setup.bash" ]; then
            ROS1_DISTRO="melodic"
        else
            ROS1_DISTRO="noetic"  # Default
        fi
    fi

    # Update bridge service working directory
    BRIDGE_SERVICE_SRC="${REPO_ROOT}/service/wds_ros1_bridge.service"
    BRIDGE_SERVICE_TEMP="/tmp/wds_ros1_bridge.service.tmp"

    sed "s|WorkingDirectory=.*|WorkingDirectory=${BRIDGE_DEPLOY_DIR}|" \
        "${BRIDGE_SERVICE_SRC}" > "${BRIDGE_SERVICE_TEMP}"

    sudo cp "${BRIDGE_SERVICE_TEMP}" "${REPO_ROOT}/service/wds_ros1_bridge.service.updated"
    rm "${BRIDGE_SERVICE_TEMP}"

    print_info "Bridge service file updated"

    # Update battery relay service with correct ROS1 distro
    RELAY_SERVICE_SRC="${REPO_ROOT}/service/wds_battery_relay.service"
    RELAY_SERVICE_TEMP="/tmp/wds_battery_relay.service.tmp"

    sed "s|/opt/ros/noetic/|/opt/ros/${ROS1_DISTRO}/|g" \
        "${RELAY_SERVICE_SRC}" > "${RELAY_SERVICE_TEMP}"

    sudo cp "${RELAY_SERVICE_TEMP}" "${REPO_ROOT}/service/wds_battery_relay.service.updated"
    rm "${RELAY_SERVICE_TEMP}"

    print_info "Relay service file updated for ROS1 ${ROS1_DISTRO}"
fi

# ============================================================================
# STEP 8: Install All Services Using install_service.sh
# ============================================================================
if [ "$FULL_DEPLOY" = true ] || [ "$SERVICE_ONLY" = true ] || [ "$RELAY_ONLY" = true ]; then
    print_header "Installing Systemd Services"

    # Use the updated service files if they exist
    if [ -f "${REPO_ROOT}/service/wds_ros1_bridge.service.updated" ]; then
        mv "${REPO_ROOT}/service/wds_ros1_bridge.service.updated" "${REPO_ROOT}/service/wds_ros1_bridge.service"
    fi

    if [ -f "${REPO_ROOT}/service/wds_battery_relay.service.updated" ]; then
        mv "${REPO_ROOT}/service/wds_battery_relay.service.updated" "${REPO_ROOT}/service/wds_battery_relay.service"
    fi

    # Run install_service.sh to install all .service files
    print_info "Running install_service.sh..."
    bash "${SERVICE_INSTALL_SCRIPT}"

    print_success "All services installed"
fi

# ============================================================================
# STEP 9: Reload Systemd and Enable Services
# ============================================================================
if [ "$FULL_DEPLOY" = true ] || [ "$SERVICE_ONLY" = true ] || [ "$RELAY_ONLY" = true ]; then
    print_header "Enabling Systemd Services"

    print_info "Reloading systemd daemon..."
    sudo systemctl daemon-reload

    if [ "$FULL_DEPLOY" = true ] || [ "$RELAY_ONLY" = true ]; then
        print_info "Enabling wds_battery_relay.service"
        sudo systemctl enable wds_battery_relay.service
        print_success "Battery relay service enabled"
    fi

    if [ "$FULL_DEPLOY" = true ] || [ "$SERVICE_ONLY" = true ]; then
        print_info "Enabling wds_ros1_bridge.service"
        sudo systemctl enable wds_ros1_bridge.service
        print_success "Bridge service enabled"
    fi
fi

# ============================================================================
# STEP 10: Install Bash Aliases
# ============================================================================
if [ "$FULL_DEPLOY" = true ] || [ "$SERVICE_ONLY" = true ] || [ "$RELAY_ONLY" = true ]; then
    print_header "Installing Bash Aliases"

    BASHRC="/home/root/.bashrc"
    ALIAS_MARKER="# WDS ROS1 Bridge Aliases"

    # Remove old aliases if they exist
    if grep -q "$ALIAS_MARKER" "$BASHRC"; then
        print_info "Removing old bridge aliases..."
        sed -i "/$ALIAS_MARKER/,/# End WDS ROS1 Bridge Aliases/d" "$BASHRC"
    fi

    # Add new aliases
    print_info "Adding bridge management aliases and ROS setup to ~/.bashrc"
    cat >> "$BASHRC" <<'EOF'

# WDS ROS1 Bridge Aliases
alias bridge-start='sudo systemctl start wds_ros1_bridge.service'
alias bridge-stop='sudo systemctl stop wds_ros1_bridge.service'
alias bridge-restart='sudo systemctl restart wds_ros1_bridge.service'
alias bridge-status='sudo systemctl status wds_ros1_bridge.service'
alias bridge-log='sudo journalctl -u wds_ros1_bridge.service -f'
alias bridge-enable='sudo systemctl enable wds_ros1_bridge.service'
alias bridge-disable='sudo systemctl disable wds_ros1_bridge.service'

alias relay-start='sudo systemctl start wds_battery_relay.service'
alias relay-stop='sudo systemctl stop wds_battery_relay.service'
alias relay-restart='sudo systemctl restart wds_battery_relay.service'
alias relay-status='sudo systemctl status wds_battery_relay.service'
alias relay-log='sudo journalctl -u wds_battery_relay.service -f'
alias relay-enable='sudo systemctl enable wds_battery_relay.service'
alias relay-disable='sudo systemctl disable wds_battery_relay.service'
# Source WDS battery messages for ROS1 tools (rostopic, rosmsg, etc.)
if [ -f /etc/wds/ros_bridge/battery_relay/devel/setup.bash ]; then
    source /etc/wds/ros_bridge/battery_relay/devel/setup.bash
fi
# End WDS ROS1 Bridge Aliases
EOF

    print_success "Bash aliases installed"
    print_info "Available commands:"
    echo "    bridge-start, bridge-stop, bridge-restart, bridge-status, bridge-log"
    echo "    relay-start, relay-stop, relay-restart, relay-status, relay-log"
fi

# ============================================================================
# STEP 11: Cleanup Source Code (Full Deploy or Relay Only)
# ============================================================================
if [ "$FULL_DEPLOY" = true ] || [ "$RELAY_ONLY" = true ]; then
    print_header "Cleanup"

    read -p "Remove source code and catkin workspace from ${REPO_ROOT}/wds_battery_msgs? [y/N]: " REMOVE_SOURCE

    if [[ "$REMOVE_SOURCE" =~ ^[Yy]$ ]]; then
        print_info "Removing source code and catkin workspace (keeping deployed binaries)..."
        rm -rf "${REPO_ROOT}/wds_battery_msgs"
        print_success "Source code and catkin workspace removed"
        print_info "Deployed binaries are safe at: ${BATTERY_DEPLOY_DIR}/devel"
    else
        print_info "Source code kept at: ${REPO_ROOT}/wds_battery_msgs"
    fi
fi

# ============================================================================
# STEP 12: Summary and Next Steps
# ============================================================================
print_header "Deployment Complete!"

echo ""
echo -e "${GREEN}✓ Deployment Summary:${NC}"
echo ""

if [ "$FULL_DEPLOY" = true ]; then
    echo "  Docker Image:       ${DOCKER_IMAGE}"
    echo "  ROS Domain ID:      ${ROS_DOMAIN_ID}"
    echo "  Bridge Config:      ${BRIDGE_DEPLOY_DIR}"
    echo "  Relay Binaries:     ${BATTERY_DEPLOY_DIR}"
    echo "  Bridge Service:     /etc/systemd/system/wds_ros1_bridge.service"
    echo "  Relay Service:      /etc/systemd/system/wds_battery_relay.service"
elif [ "$CONFIG_ONLY" = true ]; then
    echo "  Bridge Config:      ${BRIDGE_DEPLOY_DIR}"
    echo "  ROS Domain ID:      ${ROS_DOMAIN_ID}"
elif [ "$SERVICE_ONLY" = true ]; then
    echo "  Services reinstalled and enabled"
elif [ "$RELAY_ONLY" = true ]; then
    echo "  Relay Binaries:     ${BATTERY_DEPLOY_DIR}"
    echo "  Relay Service:      /etc/systemd/system/wds_battery_relay.service"
fi

echo ""
echo -e "${YELLOW}Next Steps:${NC}"
echo ""

if [ "$FULL_DEPLOY" = true ] || [ "$SERVICE_ONLY" = true ]; then
    echo "  1. Start the battery relay:"
    echo "     ${BLUE}sudo systemctl start wds_battery_relay.service${NC}"
    echo ""
    echo "  2. Start the ROS1-ROS2 bridge:"
    echo "     ${BLUE}sudo systemctl start wds_ros1_bridge.service${NC}"
    echo ""
    echo "  3. Check status:"
    echo "     ${BLUE}relay-status${NC}"
    echo "     ${BLUE}bridge-status${NC}"
    echo ""
    echo "  4. View logs:"
    echo "     ${BLUE}relay-log${NC}"
    echo "     ${BLUE}bridge-log${NC}"
    echo ""
    echo "  5. Source new aliases:"
    echo "     ${BLUE}source ~/.bashrc${NC}"
fi

if [ "$CONFIG_ONLY" = true ]; then
    echo "  Configuration updated. Restart the bridge service:"
    echo "     ${BLUE}sudo systemctl restart wds_ros1_bridge.service${NC}"
fi

if [ "$RELAY_ONLY" = true ]; then
    echo "  1. Start the battery relay:"
    echo "     ${BLUE}sudo systemctl start wds_battery_relay.service${NC}"
    echo "     ${BLUE}# or use: relay-start${NC}"
    echo ""
    echo "  2. Check status:"
    echo "     ${BLUE}relay-status${NC}"
    echo ""
    echo "  3. View logs:"
    echo "     ${BLUE}relay-log${NC}"
    echo ""
    echo "  4. Source new aliases:"
    echo "     ${BLUE}source ~/.bashrc${NC}"
fi

echo ""
echo -e "${GREEN}Deployment script completed successfully!${NC}"
echo ""
