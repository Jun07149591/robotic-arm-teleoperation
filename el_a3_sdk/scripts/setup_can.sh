#!/bin/bash
# CAN interface setup script for EL-A3 robot arm with candlight/gs_usb adapter.
#
# Usage: ./setup_can.sh [can_interface] [bitrate]
#   can_interface: CAN interface name (default: can0)
#   bitrate: CAN bitrate in bps (default: 1000000 for 1 Mbps)

CAN_INTERFACE=${1:-can0}
BITRATE=${2:-1000000}

echo "Setting up CAN interface: ${CAN_INTERFACE} at ${BITRATE} bps"

# Check if running as root.
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root (use sudo)"
    exit 1
fi

# Check if can-utils is installed.
if ! command -v candump &> /dev/null; then
    echo "Warning: can-utils not installed. Installing..."
    apt-get update && apt-get install -y can-utils
fi

# Load required kernel modules.
modprobe can
modprobe can_raw
modprobe gs_usb

# Reset and configure CAN interface.
ip link set "${CAN_INTERFACE}" down 2>/dev/null || true
ip link set "${CAN_INTERFACE}" type can bitrate "${BITRATE}"
ip link set "${CAN_INTERFACE}" txqueuelen 1000
ip link set "${CAN_INTERFACE}" up

# Verify interface is up.
if ip link show "${CAN_INTERFACE}" | grep -q "UP"; then
    echo "CAN interface ${CAN_INTERFACE} is now UP at ${BITRATE} bps"
    echo ""
    echo "Interface details:"
    ip -details link show "${CAN_INTERFACE}"
    echo ""
    echo "Test commands:"
    echo "  candump ${CAN_INTERFACE}    # Monitor CAN traffic"
    echo "  cansend ${CAN_INTERFACE} 123#1122334455667788  # Send test frame"
else
    echo "Error: Failed to bring up CAN interface"
    exit 1
fi
