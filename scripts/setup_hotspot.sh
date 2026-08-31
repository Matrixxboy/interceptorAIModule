#!/bin/bash

# ======================================================================
# T.R.I.V.E.N.I AI Module - Standalone Wi-Fi Hotspot Setup
# ======================================================================
# This script configures the Radxa Zero 3W to broadcast its own Wi-Fi
# network instead of connecting to a home router.
# IP Address is permanently bound to 192.168.4.1.

if [ "$EUID" -ne 0 ]; then
  echo "Please run as root (sudo ./scripts/setup_hotspot.sh)"
  exit 1
fi

SSID="TRIVENI-AI"
PASSWORD="triveni123"
IP_ADDRESS="192.168.4.1/24"
WIFI_INTERFACE="wlan0"

echo "[1/4] Checking NetworkManager..."
if ! command -v nmcli &> /dev/null; then
    echo "ERROR: nmcli not found. This OS does not use NetworkManager."
    exit 1
fi

echo "[2/4] Removing existing conflicting connections..."
# Remove any existing connection with this name to ensure a clean slate
nmcli con delete "$SSID" 2>/dev/null || true

echo "[3/4] Creating new Wi-Fi Access Point..."
nmcli con add type wifi ifname $WIFI_INTERFACE con-name "$SSID" autoconnect yes ssid "$SSID"
nmcli con modify "$SSID" 802-11-wireless.mode ap 802-11-wireless.band bg ipv4.method shared ipv4.address "$IP_ADDRESS"
nmcli con modify "$SSID" wifi-sec.key-mgmt wpa-psk
nmcli con modify "$SSID" wifi-sec.psk "$PASSWORD"

echo "[4/4] Activating Hotspot..."
nmcli con up "$SSID"

echo "======================================================================"
echo "✅ SUCCESS! The Radxa is now broadcasting a Wi-Fi network."
echo ""
echo "  Network Name (SSID):  $SSID"
echo "  Password:             $PASSWORD"
echo "  Dashboard URL:        http://192.168.4.1:5000"
echo ""
echo "Note: If you ever want to revert back to your home Wi-Fi,"
echo "run: sudo nmcli con down \"$SSID\" && sudo nmtui"
echo "======================================================================"
