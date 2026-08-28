#!/bin/bash

# ======================================================================
# T.R.I.V.E.N.I AI Module - Systemd Autostart Service Installer
# ======================================================================

if [ "$EUID" -ne 0 ]; then
  echo "Please run as root (sudo ./scripts/install_service.sh)"
  exit 1
fi

PROJECT_DIR="/home/radxa/interceptorAIModule"
VENV_PYTHON="$PROJECT_DIR/.venv/bin/python"
SERVICE_FILE="/etc/systemd/system/triveni-ai.service"

echo "[1/4] Generating systemd service file..."

cat <<EOF > $SERVICE_FILE
[Unit]
Description=T.R.I.V.E.N.I AI Flight Controller Override Daemon
After=network.target
Wants=systemd-udev-settle.service

[Service]
Type=simple
User=radxa
Group=dialout
WorkingDirectory=$PROJECT_DIR
Environment="PATH=$PROJECT_DIR/.venv/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
ExecStart=$VENV_PYTHON main.py
Restart=always
RestartSec=5
# Allow it to access hardware directly without hanging up
StandardOutput=syslog
StandardError=syslog
SyslogIdentifier=triveni-ai

[Install]
WantedBy=multi-user.target
EOF

echo "[2/4] Reloading systemd daemon..."
systemctl daemon-reload

echo "[3/4] Enabling service to start on boot..."
systemctl enable triveni-ai.service

echo "[4/4] Starting service now..."
systemctl start triveni-ai.service

echo "======================================================================"
echo "✅ SUCCESS! T.R.I.V.E.N.I AI will now start automatically on boot."
echo ""
echo "To check the live status and output in the field, run:"
echo "  journalctl -u triveni-ai.service -f"
echo ""
echo "To stop the service manually, run:"
echo "  sudo systemctl stop triveni-ai.service"
echo "======================================================================"
