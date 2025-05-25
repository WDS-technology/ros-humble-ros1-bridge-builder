#!/bin/bash

set -e

# Paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"
SERVICE_DIR="${PACKAGE_DIR}/service"
SYSTEMD_DIR="/etc/systemd/system"

echo "[INFO] Installing all .service files from: $SERVICE_DIR"

# Find all .service files in the service directory
shopt -s nullglob
SERVICE_FILES=("${SERVICE_DIR}"/*.service)

if [ ${#SERVICE_FILES[@]} -eq 0 ]; then
    echo "[ERROR] No .service files found in $SERVICE_DIR"
    exit 1
fi

for SERVICE_PATH in "${SERVICE_FILES[@]}"; do
    SERVICE_NAME=$(basename "$SERVICE_PATH")
    DEST_PATH="${SYSTEMD_DIR}/${SERVICE_NAME}"

    echo "[INFO] Installing $SERVICE_NAME to $DEST_PATH"
    sudo cp "$SERVICE_PATH" "$DEST_PATH"
    sudo chmod 644 "$DEST_PATH"
done

# Reload systemd after all installations
echo "[INFO] Reloading systemd"
sudo systemctl daemon-reexec
sudo systemctl daemon-reload

echo "[SUCCESS] All service files installed successfully."
