#!/bin/bash
#
# deploy_bridge.sh - One-stop deployment script for WDS ROS1↔ROS2 Bridge
#
# This script handles installation, configuration, and service setup for the
# ROS1 to ROS2 Humble bridge running in Docker.
#

set -e  # Exit on error

# =============================================================================
# CONFIGURATION
# =============================================================================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"
SERVICE_DIR="${PACKAGE_DIR}/service"
CONFIG_DIR="${PACKAGE_DIR}/config"
DOCKER_COMPOSE_FILE="${PACKAGE_DIR}/docker-compose.yml"

# Installation paths
SYSTEMD_DIR="/etc/systemd/system"
WDS_CONFIG_DIR="/etc/wds/bridge"
BRIDGE_CONFIG_FILE="bridge_topic.yaml"

# Docker image details
DOCKER_IMAGE="wdsdrones/wds_ros1_to_ros2_humble_bridge:latest"
LOCAL_IMAGE_TAG="wds_ros1_bridge:local"

# Colors for output (using tput for better compatibility)
if command -v tput &> /dev/null && [ -t 1 ]; then
    RED=$(tput setaf 1)
    GREEN=$(tput setaf 2)
    YELLOW=$(tput setaf 3)
    BLUE=$(tput setaf 4)
    BOLD=$(tput bold)
    NC=$(tput sgr0) # No Color
else
    RED=''
    GREEN=''
    YELLOW=''
    BLUE=''
    BOLD=''
    NC=''
fi

# =============================================================================
# HELPER FUNCTIONS
# =============================================================================

print_header() {
    echo "${BLUE}========================================"
    echo "$1"
    echo "========================================${NC}"
}

print_info() {
    echo "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo "${RED}[ERROR]${NC} $1"
}

print_success() {
    echo "${GREEN}[SUCCESS]${NC} $1"
}

show_help() {
    cat << EOF
${GREEN}${BOLD}WDS ROS1↔ROS2 Bridge Deployment Script${NC}

${BLUE}${BOLD}USAGE:${NC}
    $0 [COMMAND] [OPTIONS]

${BLUE}${BOLD}COMMANDS:${NC}
    ${BOLD}pull${NC}            Pull the pre-built Docker image from Docker Hub
    ${BOLD}build${NC}           Build the Docker image locally from Dockerfile
    ${BOLD}install${NC}         Install configuration files and systemd service
    ${BOLD}deploy${NC}          Full deployment (pull + install)
    ${BOLD}deploy-local${NC}    Full deployment using locally built image (build + install)
    ${BOLD}uninstall${NC}       Remove service and configuration files
    ${BOLD}status${NC}          Check the status of the bridge service
    ${BOLD}help${NC}            Show this help message

${BLUE}${BOLD}OPTIONS:${NC}
    -f, --force     Force overwrite of existing configuration files
    -y, --yes       Skip confirmation prompts
    -v, --verbose   Enable verbose output
    --minimal       Build using minimal Dockerfile (Dockerfile_minimal)
    --full          Build using full Dockerfile (Dockerfile_full)

${BLUE}${BOLD}EXAMPLES:${NC}
    ${GREEN}# Pull pre-built image and install everything${NC}
    $0 deploy

    ${GREEN}# Build locally with minimal Dockerfile and install${NC}
    $0 deploy-local --minimal

    ${GREEN}# Just pull the latest image${NC}
    $0 pull

    ${GREEN}# Just install configuration and service (assumes image exists)${NC}
    $0 install

    ${GREEN}# Build the image locally (full version)${NC}
    $0 build --full

    ${GREEN}# Check service status${NC}
    $0 status

    ${GREEN}# Uninstall everything${NC}
    $0 uninstall -y

${BLUE}${BOLD}INSTALLATION DETAILS:${NC}
    - Docker Image: ${DOCKER_IMAGE}
    - Service File: ${SYSTEMD_DIR}/wds_ros1_bridge.service
    - Config File:  ${WDS_CONFIG_DIR}/${BRIDGE_CONFIG_FILE}
    - Working Dir:  ${PACKAGE_DIR}

${BLUE}${BOLD}PREREQUISITES:${NC}
    - Docker and Docker Compose installed
    - Root/sudo access for service installation
    - corvus_roscore.service running (ROS1 core)

${BLUE}${BOLD}SERVICE DEPENDENCIES:${NC}
    The bridge service requires:
    - docker.service (Docker daemon)
    - corvus_roscore.service (ROS1 core on the drone)

${BLUE}${BOLD}MORE INFO:${NC}
    GitHub: https://github.com/wdsdrones
    Docs:   Check README.md in ${PACKAGE_DIR}

EOF
}

check_prerequisites() {
    print_info "Checking prerequisites..."
    
    # Check if running as root or with sudo
    if [ "$EUID" -ne 0 ] && ! sudo -n true 2>/dev/null; then
        print_warning "This script requires sudo privileges for some operations."
        print_info "You may be prompted for your password."
    fi
    
    # Check Docker
    if ! command -v docker &> /dev/null; then
        print_error "Docker is not installed. Please install Docker first."
        exit 1
    fi
    print_info "✓ Docker found: $(docker --version)"
    
    # Check Docker Compose
    if ! docker compose version &> /dev/null; then
        print_error "Docker Compose is not installed or not available."
        exit 1
    fi
    print_info "✓ Docker Compose found: $(docker compose version)"
    
    # Check if Docker daemon is running
    if ! docker info &> /dev/null; then
        print_error "Docker daemon is not running. Please start Docker first."
        exit 1
    fi
    print_info "✓ Docker daemon is running"
    
    # Check for required directories
    if [ ! -d "$SERVICE_DIR" ]; then
        print_error "Service directory not found: $SERVICE_DIR"
        exit 1
    fi
    
    if [ ! -d "$CONFIG_DIR" ]; then
        print_error "Config directory not found: $CONFIG_DIR"
        exit 1
    fi
    
    if [ ! -f "${CONFIG_DIR}/${BRIDGE_CONFIG_FILE}" ]; then
        print_error "Bridge config file not found: ${CONFIG_DIR}/${BRIDGE_CONFIG_FILE}"
        exit 1
    fi
    
    print_success "All prerequisites met!"
}

pull_image() {
    print_header "Pulling Docker Image"
    print_info "Pulling image: ${DOCKER_IMAGE}"
    
    if docker pull ${DOCKER_IMAGE}; then
        print_success "Image pulled successfully!"
        docker images | grep wds_ros1_to_ros2_humble_bridge || true
    else
        print_error "Failed to pull Docker image"
        exit 1
    fi
}

build_image() {
    local dockerfile="$1"
    
    print_header "Building Docker Image Locally"
    
    if [ -z "$dockerfile" ]; then
        # Default to minimal if exists, otherwise full
        if [ -f "${PACKAGE_DIR}/Dockerfile_minimal" ]; then
            dockerfile="Dockerfile_minimal"
            print_info "Using minimal Dockerfile (default)"
        elif [ -f "${PACKAGE_DIR}/Dockerfile_full" ]; then
            dockerfile="Dockerfile_full"
            print_info "Using full Dockerfile"
        else
            print_error "No Dockerfile found in ${PACKAGE_DIR}"
            exit 1
        fi
    fi
    
    local dockerfile_path="${PACKAGE_DIR}/${dockerfile}"
    
    if [ ! -f "$dockerfile_path" ]; then
        print_error "Dockerfile not found: $dockerfile_path"
        exit 1
    fi
    
    print_info "Building from: ${dockerfile}"
    print_info "This may take 15-30 minutes depending on your system..."
    
    cd "${PACKAGE_DIR}"
    
    # Detect platform
    local platform="linux/arm64"
    if [ "$(uname -m)" != "aarch64" ] && [ "$(uname -m)" != "arm64" ]; then
        print_warning "Building for ARM64 on non-ARM64 host. This may be slow."
        print_info "Consider pulling the pre-built image instead: $0 pull"
    fi
    
    if docker build --platform ${platform} \
        -f "${dockerfile}" \
        -t "${LOCAL_IMAGE_TAG}" \
        .; then
        print_success "Image built successfully!"
        
        # Tag it with the expected name for docker-compose
        docker tag "${LOCAL_IMAGE_TAG}" "${DOCKER_IMAGE}" || true
        
        docker images | grep -E "(wds_ros1|wds_bridge)" || true
    else
        print_error "Failed to build Docker image"
        exit 1
    fi
}

install_config() {
    print_header "Installing Configuration Files"
    
    # Create WDS config directory if it doesn't exist
    if [ ! -d "$WDS_CONFIG_DIR" ]; then
        print_info "Creating config directory: ${WDS_CONFIG_DIR}"
        sudo mkdir -p "${WDS_CONFIG_DIR}"
    fi
    
    local config_source="${CONFIG_DIR}/${BRIDGE_CONFIG_FILE}"
    local config_dest="${WDS_CONFIG_DIR}/${BRIDGE_CONFIG_FILE}"
    
    # Check if config already exists
    if [ -f "$config_dest" ] && [ "$FORCE_OVERWRITE" != "true" ]; then
        print_warning "Configuration file already exists: ${config_dest}"
        if [ "$AUTO_YES" != "true" ]; then
            read -p "Overwrite existing configuration? [y/N] " -n 1 -r
            echo
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                print_info "Skipping configuration installation"
                return
            fi
        else
            print_info "Auto-yes enabled, overwriting configuration"
        fi
    fi
    
    print_info "Installing bridge configuration to: ${config_dest}"
    sudo cp "${config_source}" "${config_dest}"
    sudo chmod 644 "${config_dest}"
    
    print_success "Configuration installed successfully!"
    print_info "Config location: ${config_dest}"
}

install_service() {
    print_header "Installing Systemd Service"
    
    # Find all .service files
    shopt -s nullglob
    SERVICE_FILES=("${SERVICE_DIR}"/*.service)
    
    if [ ${#SERVICE_FILES[@]} -eq 0 ]; then
        print_error "No .service files found in ${SERVICE_DIR}"
        exit 1
    fi
    
    for SERVICE_PATH in "${SERVICE_FILES[@]}"; do
        SERVICE_NAME=$(basename "$SERVICE_PATH")
        DEST_PATH="${SYSTEMD_DIR}/${SERVICE_NAME}"
        
        # Check if service already exists
        if [ -f "$DEST_PATH" ] && [ "$FORCE_OVERWRITE" != "true" ]; then
            print_warning "Service file already exists: ${DEST_PATH}"
            if [ "$AUTO_YES" != "true" ]; then
                read -p "Overwrite existing service? [y/N] " -n 1 -r
                echo
                if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                    print_info "Skipping service installation"
                    continue
                fi
            fi
        fi
        
        print_info "Installing ${SERVICE_NAME} to ${DEST_PATH}"
        sudo cp "$SERVICE_PATH" "$DEST_PATH"
        sudo chmod 644 "$DEST_PATH"
        
        print_success "Service ${SERVICE_NAME} installed!"
    done
    
    # Reload systemd
    print_info "Reloading systemd daemon..."
    sudo systemctl daemon-reexec
    sudo systemctl daemon-reload
    
    print_success "All services installed successfully!"
    
    # Show service status
    for SERVICE_PATH in "${SERVICE_FILES[@]}"; do
        SERVICE_NAME=$(basename "$SERVICE_PATH")
        print_info "Service status: ${SERVICE_NAME}"
        sudo systemctl status "${SERVICE_NAME}" --no-pager || true
    done
    
    echo ""
    print_info "To enable service at boot: ${BOLD}sudo systemctl enable wds_ros1_bridge.service${NC}"
    print_info "To start service now:      ${BOLD}sudo systemctl start wds_ros1_bridge.service${NC}"
}

uninstall_all() {
    print_header "Uninstalling WDS ROS1 Bridge"
    
    if [ "$AUTO_YES" != "true" ]; then
        print_warning "This will remove the service and configuration files."
        read -p "Are you sure you want to uninstall? [y/N] " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            print_info "Uninstall cancelled"
            exit 0
        fi
    fi
    
    # Stop and disable service
    print_info "Stopping and disabling service..."
    sudo systemctl stop wds_ros1_bridge.service 2>/dev/null || true
    sudo systemctl disable wds_ros1_bridge.service 2>/dev/null || true
    
    # Remove service file
    if [ -f "${SYSTEMD_DIR}/wds_ros1_bridge.service" ]; then
        print_info "Removing service file..."
        sudo rm -f "${SYSTEMD_DIR}/wds_ros1_bridge.service"
    fi
    
    # Remove configuration
    if [ -f "${WDS_CONFIG_DIR}/${BRIDGE_CONFIG_FILE}" ]; then
        print_info "Removing configuration file..."
        sudo rm -f "${WDS_CONFIG_DIR}/${BRIDGE_CONFIG_FILE}"
    fi
    
    # Reload systemd
    print_info "Reloading systemd..."
    sudo systemctl daemon-reload
    
    print_success "Uninstall complete!"
    print_info "Docker images were NOT removed. To remove manually:"
    print_info "  ${BOLD}docker rmi ${DOCKER_IMAGE}${NC}"
}

check_status() {
    print_header "Bridge Service Status"
    
    # Check if service file exists
    if [ ! -f "${SYSTEMD_DIR}/wds_ros1_bridge.service" ]; then
        print_error "Service not installed"
        exit 1
    fi
    
    # Show service status
    sudo systemctl status wds_ros1_bridge.service --no-pager || true
    
    echo ""
    print_info "Configuration file: ${WDS_CONFIG_DIR}/${BRIDGE_CONFIG_FILE}"
    if [ -f "${WDS_CONFIG_DIR}/${BRIDGE_CONFIG_FILE}" ]; then
        print_success "✓ Configuration file exists"
    else
        print_error "✗ Configuration file missing"
    fi
    
    echo ""
    print_info "Docker image status:"
    docker images | grep -E "(wds_ros1|REPOSITORY)" || print_warning "Image not found locally"
    
    echo ""
    print_info "Running containers:"
    docker ps | grep -E "(ros.*bridge|CONTAINER)" || print_info "No bridge containers running"
}

# =============================================================================
# MAIN SCRIPT
# =============================================================================

# Parse command line arguments
COMMAND=""
FORCE_OVERWRITE="false"
AUTO_YES="false"
VERBOSE="false"
DOCKERFILE=""

while [[ $# -gt 0 ]]; do
    case $1 in
        pull|build|install|deploy|deploy-local|uninstall|status|help)
            COMMAND="$1"
            shift
            ;;
        -f|--force)
            FORCE_OVERWRITE="true"
            shift
            ;;
        -y|--yes)
            AUTO_YES="true"
            shift
            ;;
        -v|--verbose)
            VERBOSE="true"
            set -x
            shift
            ;;
        --minimal)
            DOCKERFILE="Dockerfile_minimal"
            shift
            ;;
        --full)
            DOCKERFILE="Dockerfile_full"
            shift
            ;;
        *)
            print_error "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# Show help if no command provided
if [ -z "$COMMAND" ]; then
    show_help
    exit 0
fi

# Execute command
case $COMMAND in
    help)
        show_help
        ;;
    
    pull)
        check_prerequisites
        pull_image
        ;;
    
    build)
        check_prerequisites
        build_image "$DOCKERFILE"
        ;;
    
    install)
        check_prerequisites
        install_config
        install_service
        print_success "Installation complete!"
        ;;
    
    deploy)
        check_prerequisites
        pull_image
        install_config
        install_service
        print_header "Deployment Complete!"
        print_success "WDS ROS1↔ROS2 Bridge is now installed"
        echo ""
        print_info "Next steps:"
        print_info "  1. Review configuration: ${WDS_CONFIG_DIR}/${BRIDGE_CONFIG_FILE}"
        print_info "  2. Enable at boot: ${BOLD}sudo systemctl enable wds_ros1_bridge.service${NC}"
        print_info "  3. Start service: ${BOLD}sudo systemctl start wds_ros1_bridge.service${NC}"
        print_info "  4. Check status: ${BOLD}$0 status${NC}"
        ;;
    
    deploy-local)
        check_prerequisites
        build_image "$DOCKERFILE"
        install_config
        install_service
        print_header "Local Deployment Complete!"
        print_success "WDS ROS1↔ROS2 Bridge built and installed"
        echo ""
        print_info "Next steps:"
        print_info "  1. Review configuration: ${WDS_CONFIG_DIR}/${BRIDGE_CONFIG_FILE}"
        print_info "  2. Enable at boot: ${BOLD}sudo systemctl enable wds_ros1_bridge.service${NC}"
        print_info "  3. Start service: ${BOLD}sudo systemctl start wds_ros1_bridge.service${NC}"
        print_info "  4. Check status: ${BOLD}$0 status${NC}"
        ;;
    
    uninstall)
        uninstall_all
        ;;
    
    status)
        check_status
        ;;
    
    *)
        print_error "Unknown command: $COMMAND"
        show_help
        exit 1
        ;;
esac

exit 0