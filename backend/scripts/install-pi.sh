#!/bin/sh
# One-time setup on Raspberry Pi OS Lite 64-bit (Bookworm). Run from backend/: sh scripts/install-pi.sh
set -eu
sudo apt-get update
sudo apt-get install -y ffmpeg gcc-avr avr-libc avrdude make v4l-utils
if ! command -v node >/dev/null || [ "$(node -p 'process.versions.node.split(".")[0]')" -lt 22 ]; then
  curl -fsSL https://deb.nodesource.com/setup_22.x | sudo -E bash -
  sudo apt-get install -y nodejs
fi
npm ci --omit=dev 2>/dev/null || npm install --omit=dev

# Robot USB port -> /dev/firebird, FTDI latency 1 ms (otherwise every read waits 16 ms)
sudo cp scripts/99-firebird.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout,video "$USER"

# Service: starts on boot, restarts on crash. Runs as this user (no root needed).
sed "s|__USER__|$USER|; s|__DIR__|$(pwd)|" scripts/anc-backend.service | sudo tee /etc/systemd/system/anc-backend.service >/dev/null
sudo systemctl daemon-reload
sudo systemctl enable anc-backend
echo "Done. Copy the YOLO model into models/ (see tools/export_yolo.py), then: sudo systemctl start anc-backend"
echo "Log out and in once so the dialout/video groups apply."
