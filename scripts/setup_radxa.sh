#!/usr/bin/env bash
# ==============================================================================
# Radxa ZERO 3 (RK3566, 2GB RAM) Autonomous Interceptor System Setup Script
# ==============================================================================

set -e

echo "======================================================================"
echo " Starting Radxa ZERO 3 System Setup & Dependencies Installation"
echo "======================================================================"

# 1. Update package lists
echo "[1/6] Updating system packages..."
sudo apt-get update -y

# 2. Install required system dependencies
echo "[2/6] Installing OpenCV, V4L2, GStreamer, and Python build tools..."
sudo apt-get install -y \
    python3-pip \
    python3-dev \
    python3-opencv \
    python3-numpy \
    v4l-utils \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    libgstreamer1.0-dev

# 3. Install Python dependencies
echo "[3/6] Installing minimal Python requirements (PySerial)..."
pip3 install pyserial>=3.5

# 4. Set User Group Permissions for Serial UART (/dev/ttyS2) and Video (/dev/video0)
echo "[4/6] Configuring user permissions for UART (/dev/ttyS2) and Camera (/dev/video*)..."
CURRENT_USER=$(whoami)
sudo usermod -aG dialout,video "$CURRENT_USER" || true

# 5. Set CPU Performance Governor for RK3566 Cortex-A55 cores
echo "[5/6] Optimizing RK3566 CPU governor for max throughput..."
if [ -d "/sys/devices/system/cpu/cpu0/cpufreq" ]; then
    echo "performance" | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor > /dev/null || true
fi

# 6. Disable X11 GUI to free ~500MB RAM for AI tracking
echo "[6/6] Setting headless multi-user boot mode to conserve RAM..."
sudo systemctl set-default multi-user.target || true

echo "======================================================================"
echo " Radxa ZERO 3 Setup Complete!"
echo ""
echo " To run system diagnostics:"
echo "   python3 scripts/diagnose_system.py"
echo ""
echo " To start the onboard daemon manually:"
echo "   python3 main.py --config config.json"
echo ""
echo " To enable auto-start on boot:"
echo "   sudo cp interceptor.service /etc/systemd/system/"
echo "   sudo systemctl daemon-reload"
echo "   sudo systemctl enable interceptor"
echo "   sudo systemctl start interceptor"
echo "======================================================================"
