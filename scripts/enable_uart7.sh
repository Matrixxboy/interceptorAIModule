#!/usr/bin/env bash
# ==============================================================================
# Radxa ZERO 3W — Enable UART7 on GPIO Pin 11 (TX) / Pin 13 (RX)
#
# Pins 8/10 are the system debug console and are locked by the kernel.
# We will use Pins 11/13 (UART7) for the Flight Controller instead.
# ==============================================================================

set -e

echo "======================================================================"
echo " Radxa ZERO 3W — UART7 Enable Script (Pin 11 TX / Pin 13 RX)"
echo "======================================================================"

echo "[1/3] Adding UART7 overlay to boot config..."
if [ -f /boot/uEnv.txt ]; then
    if grep -q "uart7" /boot/uEnv.txt 2>/dev/null; then
        echo "  UART7 overlay already configured in /boot/uEnv.txt"
    else
        echo "overlays=uart7-m1" >> /boot/uEnv.txt
        echo "  DONE: Added 'overlays=uart7-m1'"
    fi
fi
if [ -f /boot/config.txt ]; then
    if ! grep -q "uart7" /boot/config.txt 2>/dev/null; then
        echo "overlays=uart7-m1" >> /boot/config.txt
        echo "  DONE: Added 'overlays=uart7-m1'"
    fi
fi

echo ""
echo "[2/3] Setting serial port permissions..."
CURRENT_USER=${SUDO_USER:-$(whoami)}
usermod -aG dialout "$CURRENT_USER" 2>/dev/null || true

cat > /etc/udev/rules.d/99-uart7-interceptor.rules << 'EOF'
KERNEL=="ttyS7", MODE="0666", GROUP="dialout"
EOF
udevadm control --reload-rules 2>/dev/null || true

echo ""
echo "[3/3] Done!"
echo "======================================================================"
echo " IMPORTANT: Hardware Wiring Change Required!"
echo " Because Pins 8 and 10 are locked as the system debug console,"
echo " you must move your flight controller wires to Pins 11 and 13."
echo ""
echo " NEW WIRING:"
echo "   Pin 11 (UART7_TX) -----> FC UART RX"
echo "   Pin 13 (UART7_RX) <----- FC UART TX"
echo "   Pin 6  (GND)      <----> FC GND"
echo ""
echo " Please REBOOT the Radxa now: sudo reboot"
echo "======================================================================"
