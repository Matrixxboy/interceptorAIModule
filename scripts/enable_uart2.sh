#!/usr/bin/env bash
# ==============================================================================
# Radxa ZERO 3W — Enable UART2 on GPIO Pin 8 (TX) / Pin 10 (RX)
#
# Run: sudo bash scripts/enable_uart2.sh
#
# The Radxa ZERO 3W uses RK3566. UART2 (Pin 8 TX / Pin 10 RX) is NOT enabled
# by default. This script enables it via device tree overlay and sets permissions.
# ==============================================================================

set -e

echo "======================================================================"
echo " Radxa ZERO 3W — UART2 Enable Script (Pin 8 TX / Pin 10 RX)"
echo "======================================================================"

# Step 1: Check which serial devices currently exist
echo ""
echo "[1/5] Current serial devices:"
ls -la /dev/ttyS* /dev/ttyAMA* /dev/ttyFIQ* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  No serial devices found."
echo ""

# Step 2: Check if UART2 overlay is already loaded
echo "[2/5] Checking device tree overlays..."
if [ -f /boot/config.txt ]; then
    echo "  Found /boot/config.txt (Radxa Debian/Ubuntu)"
    if grep -q "uart2" /boot/config.txt 2>/dev/null; then
        echo "  UART2 overlay already configured in /boot/config.txt"
    else
        echo "  Adding UART2 overlay to /boot/config.txt..."
        # Radxa OS uses rk3568/rk3566 overlays
        echo "" >> /boot/config.txt
        echo "# Enable UART2 on GPIO Pin 8 (TX) / Pin 10 (RX) for MSP Flight Controller" >> /boot/config.txt
        echo "overlays=uart2" >> /boot/config.txt
        echo "  DONE: Added 'overlays=uart2' to /boot/config.txt"
    fi
elif [ -f /boot/uEnv.txt ]; then
    echo "  Found /boot/uEnv.txt"
    if grep -q "uart2" /boot/uEnv.txt 2>/dev/null; then
        echo "  UART2 overlay already configured"
    else
        echo "  Adding UART2 overlay to /boot/uEnv.txt..."
        echo "overlays=uart2" >> /boot/uEnv.txt
        echo "  DONE: Added 'overlays=uart2'"
    fi
elif [ -f /boot/armbianEnv.txt ]; then
    echo "  Found /boot/armbianEnv.txt (Armbian)"
    if grep -q "uart2" /boot/armbianEnv.txt 2>/dev/null; then
        echo "  UART2 overlay already configured"
    else
        # Armbian uses param_uart overlay
        CURRENT_OVERLAYS=$(grep "^overlays=" /boot/armbianEnv.txt 2>/dev/null | cut -d= -f2-)
        if [ -n "$CURRENT_OVERLAYS" ]; then
            sed -i "s/^overlays=.*/overlays=${CURRENT_OVERLAYS} rk3568-uart2/" /boot/armbianEnv.txt
        else
            echo "overlays=rk3568-uart2" >> /boot/armbianEnv.txt
        fi
        echo "  DONE: Added UART2 overlay"
    fi
else
    echo "  WARNING: No boot config found. Try using 'rsetup' to enable UART2:"
    echo "    sudo rsetup -> Hardware -> Manage overlays -> Enable UART2"
fi

# Step 3: Try using rsetup if available (Radxa's official tool)
echo ""
echo "[3/5] Checking for rsetup (Radxa's hardware config tool)..."
if command -v rsetup &>/dev/null; then
    echo "  rsetup is available. If UART2 is still not working after reboot, run:"
    echo "    sudo rsetup"
    echo "  Then navigate to: Hardware -> Manage overlays -> Enable UART2"
else
    echo "  rsetup not found (optional - overlay was configured via boot config)"
fi

# Step 4: Set permissions
echo ""
echo "[4/5] Setting serial port permissions..."
CURRENT_USER=${SUDO_USER:-$(whoami)}
usermod -aG dialout "$CURRENT_USER" 2>/dev/null && echo "  Added $CURRENT_USER to dialout group" || true

# Create udev rule for persistent UART2 permissions
cat > /etc/udev/rules.d/99-uart2-interceptor.rules << 'EOF'
# Radxa ZERO 3W UART2 — Allow non-root access for Interceptor AI Module
KERNEL=="ttyS2", MODE="0666", GROUP="dialout"
KERNEL=="ttyS0", MODE="0666", GROUP="dialout"
KERNEL=="ttyFIQ0", MODE="0666", GROUP="dialout"
KERNEL=="ttyUSB*", MODE="0666", GROUP="dialout"
KERNEL=="ttyACM*", MODE="0666", GROUP="dialout"
EOF
echo "  Created /etc/udev/rules.d/99-uart2-interceptor.rules"
udevadm control --reload-rules 2>/dev/null || true
udevadm trigger 2>/dev/null || true

# Step 5: Quick test if /dev/ttyS2 exists now
echo ""
echo "[5/5] Post-setup device check..."
if [ -c /dev/ttyS2 ]; then
    echo "  /dev/ttyS2 EXISTS — UART2 is active!"
    ls -la /dev/ttyS2
    echo ""
    echo "  Wiring (Radxa ZERO 3W 40-pin header -> Flight Controller):"
    echo "    Pin 8  (UART2_TX) -----> FC UART RX"
    echo "    Pin 10 (UART2_RX) <----- FC UART TX"
    echo "    Pin 6  (GND)      <----> FC GND"
    echo ""
    echo "  INAV Configurator -> Ports -> UART2 -> Protocol: MSP, Baud: 115200"
else
    echo "  /dev/ttyS2 NOT found yet. A REBOOT is required to load the device tree overlay."
    echo ""
    echo "  Run: sudo reboot"
    echo "  After reboot, verify: ls -la /dev/ttyS2"
fi

echo ""
echo "======================================================================"
echo " UART2 Setup Complete!"
echo ""
echo " If /dev/ttyS2 doesn't exist yet, REBOOT the Radxa:"
echo "   sudo reboot"
echo ""
echo " Then test: python3 scripts/test_hardware.py"
echo "======================================================================"
