#!/bin/bash
#
# test_bridge_mappings.sh - Verify bridge has message type mappings
#
# This script tests that the ros1_bridge was compiled with proper message
# type mappings. Run this against a built Docker image to verify.
#

set -e

DOCKER_IMAGE="${1:-wdsdrones/wds_ros1_to_ros2_humble_bridge:latest}"

echo "=========================================="
echo "Testing Bridge Message Mappings"
echo "Image: ${DOCKER_IMAGE}"
echo "=========================================="
echo ""

# Check if image exists
if ! docker image inspect "${DOCKER_IMAGE}" >/dev/null 2>&1; then
    echo "ERROR: Docker image not found: ${DOCKER_IMAGE}"
    echo "Build the image first with:"
    echo "  docker build -f Dockerfile_minimal -t ${DOCKER_IMAGE} ."
    exit 1
fi

echo "[1/4] Checking ROS2 packages are installed..."
docker run --rm "${DOCKER_IMAGE}" bash -c "
    source /opt/ros/humble/setup.bash && \
    ros2 pkg list | grep -E '(geometry_msgs|std_msgs|sensor_msgs)' | head -10
"
echo "✓ ROS2 packages found"
echo ""

echo "[2/4] Checking bridge is installed..."
docker run --rm "${DOCKER_IMAGE}" bash -c "
    source /opt/ros/humble/setup.bash && \
    source /ros-humble-ros1-bridge/install/setup.bash && \
    which ros2 && \
    ros2 pkg list | grep ros1_bridge
"
echo "✓ Bridge is installed"
echo ""

echo "[3/4] Testing message pairs (checking for template specializations)..."
docker run --rm "${DOCKER_IMAGE}" bash -c "
    source /opt/ros/humble/setup.bash && \
    source /ros-humble-ros1-bridge/install/setup.bash && \
    ros2 run ros1_bridge dynamic_bridge --print-pairs 2>/dev/null | head -30
" | tee /tmp/bridge_pairs.txt

echo ""
echo "[4/4] Verifying critical message types for WDS topics..."
echo ""

REQUIRED_TYPES=(
    "geometry_msgs/msg/PoseStamped"
    "std_msgs/msg/String"
    "std_msgs/msg/Bool"
    "std_msgs/msg/Empty"
    "sensor_msgs/msg/BatteryState"
    "sensor_msgs/msg/CameraInfo"
    "std_msgs/msg/Float32MultiArray"
    "std_msgs/msg/UInt32"
)

ALL_FOUND=true
for msg_type in "${REQUIRED_TYPES[@]}"; do
    if grep -q "$msg_type" /tmp/bridge_pairs.txt; then
        echo "✓ $msg_type"
    else
        echo "✗ MISSING: $msg_type"
        ALL_FOUND=false
    fi
done

echo ""
echo "=========================================="
if [ "$ALL_FOUND" = true ]; then
    echo "✓ SUCCESS: All required message types found!"
    echo "The bridge is ready to use."
    exit 0
else
    echo "✗ FAILURE: Some message types are missing!"
    echo ""
    echo "This means the bridge was compiled without proper message packages."
    echo "The bridge will fail at runtime with 'No template specialization' errors."
    echo ""
    echo "FIX: Rebuild the image ensuring ROS2 message packages are"
    echo "     reinstalled after ROS1 installation (see Dockerfile_minimal step 6)"
    exit 1
fi
