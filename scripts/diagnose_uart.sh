#!/usr/bin/env bash
# ==============================================================================
# Radxa ZERO 3W — UART Diagnostic Script
# Run: bash scripts/diagnose_uart.sh
# ==============================================================================

echo "======================================================================"
echo " Radxa ZERO 3W UART Diagnostic"
echo "======================================================================"

echo ""
echo "[1/4] Checking dmesg for UART initialization..."
dmesg | grep -i ttyS || echo "No ttyS messages found in dmesg."

echo ""
echo "[2/4] Checking available ttyS devices..."
ls -la /dev/ttyS* /dev/ttyFIQ* 2>/dev/null || echo "No ttyS devices found."

echo ""
echo "[3/4] Checking device tree for UARTs..."
ls -l /sys/class/tty/ttyS* 2>/dev/null || echo "No ttyS in sysfs."

echo ""
echo "[4/4] Checking rsetup overlays..."
if command -v rsetup &>/dev/null; then
    echo "rsetup is installed. Please run:"
    echo "  sudo rsetup"
    echo "Then go to 'Hardware' -> 'Manage overlays' and ensure 'rk3568-uart2' or 'uart2' is enabled."
else
    echo "rsetup not found."
    echo "Check your boot config:"
    cat /boot/uEnv.txt /boot/armbianEnv.txt /boot/config.txt 2>/dev/null | grep -i overlay || echo "No overlays found in boot configs."
fi

echo ""
echo "======================================================================"
echo "If /dev/ttyS2 is still missing, the Radxa OS image might require using 'rsetup'"
echo "to officially enable the device tree overlay."
echo ""
echo "Run: sudo rsetup"
echo "======================================================================"
