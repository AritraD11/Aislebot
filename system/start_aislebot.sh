#!/bin/bash
# ================================================================
#  AisleBot Headless Autostart — May 2026
#  Aritra Das | IIT Bombay | 25D0074
#
#  Handles:
#    - CycloneDDS loopback fix (no multicast on headless Pi)
#    - udev symlink waits (/dev/esp32, /dev/mega)
#    - Xbox controller wait
#    - Clean logging
#    - Graceful degradation (arm unavailable ≠ abort)
# ================================================================

LOG="$HOME/aislebot_boot.log"
exec >> "$LOG" 2>&1

echo ""
echo "=================================================="
echo "  AisleBot Boot — $(date)"
echo "=================================================="

# ── ROS2 environment ─────────────────────────────────────────
source /opt/ros/jazzy/setup.bash
source /home/aritra/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Fix: CycloneDDS fails to find multicast interface on headless
# Pi with no LAN connected. Force loopback so all nodes on this
# machine can talk to each other.
export CYCLONEDDS_URI='<CycloneDDS><Domain><General>
  <NetworkInterfaceAddress>lo</NetworkInterfaceAddress>
  <AllowMulticast>false</AllowMulticast>
</General></Domain></CycloneDDS>'

echo "[$(date +%H:%M:%S)] ROS2 environment ready"
echo "  Domain ID : $ROS_DOMAIN_ID"
echo "  RMW       : $RMW_IMPLEMENTATION"

# ── Wait for ESP32 (/dev/esp32) ──────────────────────────────
echo "[$(date +%H:%M:%S)] Waiting for ESP32 on /dev/esp32..."
for i in $(seq 1 30); do
    [ -e /dev/esp32 ] && break
    echo "  ($i/30) not yet..."
    sleep 1
done

if [ ! -e /dev/esp32 ]; then
    echo "[ERROR] ESP32 not found after 30s — aborting"
    exit 1
fi
echo "[$(date +%H:%M:%S)] ESP32 found ✓"

# ── Wait for Arduino Mega (/dev/mega) ────────────────────────
echo "[$(date +%H:%M:%S)] Waiting for Mega on /dev/mega..."
for i in $(seq 1 15); do
    [ -e /dev/mega ] && break
    sleep 1
done

if [ ! -e /dev/mega ]; then
    echo "[WARN] Mega not found — arm will be unavailable"
else
    echo "[$(date +%H:%M:%S)] Mega found ✓"
fi

# ── Wait for Xbox controller (/dev/input/js0) ────────────────
echo "[$(date +%H:%M:%S)] Waiting for Xbox controller..."
for i in $(seq 1 10); do
    [ -e /dev/input/js0 ] && break
    sleep 1
done

if [ -e /dev/input/js0 ]; then
    echo "[$(date +%H:%M:%S)] Xbox controller found ✓"
else
    echo "[WARN] No joystick — phone dashboard still works"
fi

# ── Launch ───────────────────────────────────────────────────
echo "[$(date +%H:%M:%S)] Launching AisleBot full stack..."
echo "=================================================="

exec ros2 launch mecanum_robot aislebot_full.launch.py
