#!/usr/bin/env bash
# ==============================================================================
# Radxa ZERO 3 System Cleanup & Performance Optimization Script
# ==============================================================================

echo "======================================================"
echo " Starting Radxa ZERO 3 Cleanup & Optimization..."
echo "======================================================"

# 1. Clean Python Bytecode & Pip Caches
echo "[1/6] Cleaning Python __pycache__, .pyc, and pip caches..."
find ~ -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null
find ~ -name "*.pyc" -delete 2>/dev/null
rm -rf ~/.cache/pip ~/.cache/ultralytics ~/.cache/torch 2>/dev/null

# 2. Vacuum Systemd Logs (Limit to 50MB)
echo "[2/6] Trimming systemd journal logs to 50MB..."
sudo journalctl --vacuum-size=50M 2>/dev/null

# 3. Clean APT Package Cache & Unused Packages
echo "[3/6] Cleaning APT package archives and orphan packages..."
sudo apt-get clean -y
sudo apt-get autoremove -y

# 4. Clean System Temp Folders (/tmp & /var/tmp)
echo "[4/6] Clearing /tmp and /var/tmp..."
sudo rm -rf /tmp/* /var/tmp/* 2>/dev/null

# 5. Flush Linux Kernel PageCache, Dentries, and Inodes RAM
echo "[5/6] Flushing RAM PageCache, Dentries & Inodes..."
sudo sync
echo 3 | sudo tee /proc/sys/vm/drop_caches > /dev/null

# 6. Lock CPU Governor to Maximum Performance (1.8GHz)
echo "[6/6] Setting CPU Governor to MAX PERFORMANCE..."
for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    if [ -f "$cpu" ]; then
        echo performance | sudo tee "$cpu" > /dev/null
    fi
done

echo "======================================================"
echo " SUCCESS: Radxa ZERO 3 Cleaned & Performance Maxed!"
echo " Free RAM Status:"
free -h
echo "======================================================"
