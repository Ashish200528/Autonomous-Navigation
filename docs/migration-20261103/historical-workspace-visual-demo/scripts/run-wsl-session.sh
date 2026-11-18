#!/usr/bin/env bash
set -Eeuo pipefail

if [[ $# -ne 5 ]]; then
  echo 'Usage: run-wsl-session.sh phase6|lidar S00|S01|S02 SEED true|false LOG_PATH' >&2
  exit 2
fi

mode="$1"
scenario="$2"
seed="$3"
rviz="$4"
linux_log="$5"

exec bash /home/ashish/AutonomousNavigationCar/scripts/windows_demo_session.sh \
  run "$mode" "$scenario" "$seed" "$rviz" direct "$linux_log"
