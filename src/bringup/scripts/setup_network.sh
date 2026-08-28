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
AEVACLI=/home/bharatwrrr/atlas_api/aeva_fix/opt/aeva/aevacli/aeva/tools/cli/aevacli
export ARENA_SDK_UTILS_PATH=/home/bharatwrrr/arena_sdk/ArenaSDK_Linux_ARM64/Utilities

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

sleep 2
echo "Configuring LiDAR setting.."
$AEVACLI configuration set --sensor-url 10.42.0.45 --group scanner --setting scan_pattern=Automotive-Wide1 -t 5
$AEVACLI configuration set --sensor-url 10.42.0.45 --group udp_unicast_config --setting host_ip=10.42.0.101 -t 3

echo "Configuring $NIC_IF (cameras)..."
ip link set mtu 9000 dev "$NIC_IF"
ip addr add 10.42.0.200/24 dev "$NIC_IF"
ip link set "$NIC_IF" up
ip route add 10.42.0.201 dev "$NIC_IF"
ip route add 10.42.0.202 dev "$NIC_IF"

sleep 2
echo "Configuring camera IPs..."
python /home/bharatwrrr/trackdot/src/triton_cam_driver/scripts/configure_ips.py --mode dual_cam || \
    echo "  IP config returned non-zero (cameras may already have persistent IPs — continuing)"

echo "Configuring orin -> laptop comms..."
ip route add 10.42.0.100 dev "$NIC_IF"


# ── PTP (IEEE 1588): Jetson as grandmaster on camera network ─────────────────
# Cameras slave their hardware clocks to this, putting both in the same clock
# domain as CLOCK_REALTIME (and therefore ROS time).
echo "Setting up PTP on $NIC_IF..."
pkill -f "ptp4l.*-i $NIC_IF" 2>/dev/null || true
sudo pkill -f phc2sys 2>/dev/null || true
sleep 0.3

if ! command -v ptp4l &>/dev/null; then
    echo "  WARNING: linuxptp not installed. Run: sudo apt install linuxptp"
    echo "  Skipping PTP — software clock offset correction still active in driver."
else
    # Write a config file — masterOnly is a config option, not a valid CLI flag.
    # priority1=100 ensures Jetson wins BMCA over cameras (which default to 128/129).
    cat > /tmp/ptp4l_trackdot.conf << 'PTPCFG'
[global]
masterOnly        1
priority1         100
priority2         128
logAnnounceInterval  1
logSyncInterval      0
tx_timestamp_timeout 100
PTPCFG

    ptp4l  -i "$NIC_IF" -f /tmp/ptp4l_trackdot.conf >> /tmp/ptp4l_cameras.log 2>&1 &

    # phc2sys is intentionally NOT used here.
    # phc2sys syncs the NIC PHC to CLOCK_REALTIME, but on this system its servo
    # oscillates (±150µs corrections) due to the NIC's large natural frequency offset
    # (~225 ppm). Those PHC jumps propagate through ptp4l's Sync messages to the
    # cameras, causing the cameras' PTP stacks to repeatedly correct their clocks
    # during streaming → incomplete frames. Without phc2sys, the PHC drifts very
    # slowly and ptp4l sends stable timestamps.

    echo "  ptp4l running (log: /tmp/ptp4l_cameras.log)"
    echo "  Allow ~10 s after driver start for cameras to fully sync."
fi

echo "Done."
echo
echo "--- $USB_IF ---"
ip addr show "$USB_IF"
ip route show dev "$USB_IF"
echo
echo "--- $NIC_IF ---"
ip addr show "$NIC_IF"
ip route show dev "$NIC_IF"
