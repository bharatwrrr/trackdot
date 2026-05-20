#!/bin/bash
# refresh_network.sh - flush and reconfigure eno1 and enxc8a362f63b58

set -e

# Require sudo
if [ "$EUID" -ne 0 ]; then
    echo "Run as root (use sudo)."
    exit 1
fi

USB_IF="enxc8a362f63b58"
NIC_IF="eno1"

echo "Bringing interfaces down..."
ip link set "$USB_IF" down 2>/dev/null || true
ip link set "$NIC_IF" down 2>/dev/null || true

echo "Flushing addresses and routes on $USB_IF..."
ip addr flush dev "$USB_IF" 2>/dev/null || true
ip route flush dev "$USB_IF" 2>/dev/null || true

echo "Flushing addresses and routes on $NIC_IF..."
ip addr flush dev "$NIC_IF" 2>/dev/null || true
ip route flush dev "$NIC_IF" 2>/dev/null || true

echo "Configuring $USB_IF (LiDAR)..."
ip addr add 10.42.0.101/24 dev "$USB_IF"
ip addr add 192.168.1.101/22 dev "$USB_IF"
ip link set "$USB_IF" up
ip route add 10.42.0.45 dev "$USB_IF"

echo "Configuring $NIC_IF (cameras)..."
ip link set mtu 9000 dev "$NIC_IF"
ip addr add 10.42.0.200/24 dev "$NIC_IF"
ip link set "$NIC_IF" up
ip route add 10.42.0.201 dev "$NIC_IF"
ip route add 10.42.0.202 dev "$NIC_IF"

echo "Configuring orin -> laptop comms..."
ip route add 10.42.0.100 dev "$NIC_IF"

echo "Done."
echo
echo "--- $USB_IF ---"
ip addr show "$USB_IF"
ip route show dev "$USB_IF"
echo
echo "--- $NIC_IF ---"
ip addr show "$NIC_IF"
ip route show dev "$NIC_IF"
