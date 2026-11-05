#!/usr/bin/env bash
set -Eeuo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "$BASH_SOURCE")" && pwd)
ROOT=$(cd -- "$SCRIPT_DIR/.." && pwd)
STATE_DIR="$HOME/.cache/anc_demo_launcher"
STATE_FILE="$STATE_DIR/current.json"
SCRIPT_PATH="$ROOT/scripts/windows_demo_session.sh"

process_group_alive() {
  kill -0 -- "-$1" 2>/dev/null
}

remove_owned_state() {
  python3 - "$STATE_FILE" "$1" "$2" <<'PY'
import json, pathlib, sys
path = pathlib.Path(sys.argv[1])
try:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("script") == sys.argv[3] and data.get("pid") == int(sys.argv[2]):
        path.unlink()
except (FileNotFoundError, json.JSONDecodeError, OSError):
    pass
PY
}

stop_tracked_session() {
  if [[ ! -f "$STATE_FILE" ]]; then
    echo "No active tracked project session."
    return 0
  fi
  local info pid pgid start_ticks script
  info=$(python3 - "$STATE_FILE" <<'PY'
import json, pathlib, sys
data = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
print(str(data.get("pid", "")) + "\t" + str(data.get("pgid", "")) + "\t" +
      str(data.get("start_ticks", "")) + "\t" + str(data.get("script", "")))
PY
) || { echo "BLOCKED: launcher state file is unreadable; left it untouched."; return 3; }
  IFS=$'\t' read -r pid pgid start_ticks script <<< "$info"
  if [[ "$script" != "$SCRIPT_PATH" || ! "$pid" =~ ^[0-9]+$ || ! "$pgid" =~ ^[0-9]+$ ]]; then
    echo "BLOCKED: session state does not identify this exact launcher; no process was signaled."
    return 3
  fi
  if [[ ! -r "/proc/$pid/stat" ]]; then
    remove_owned_state "$pid" "$SCRIPT_PATH"
    echo "Removed only the stale launcher-owned PID record; no simulator process remained."
    return 0
  fi
  local live_ticks live_pgid cmdline
  live_ticks=$(python3 - "$pid" <<'PY'
from pathlib import Path
import sys
line = Path("/proc", sys.argv[1], "stat").read_text()
print(line[line.rfind(")") + 2:].split()[19])
PY
) || { echo "BLOCKED: could not verify process start time."; return 3; }
  live_pgid=$(ps -o pgid= -p "$pid" | tr -d ' ')
  cmdline=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)
  if [[ "$live_ticks" != "$start_ticks" ]]; then
    remove_owned_state "$pid" "$SCRIPT_PATH"
    echo "The recorded PID was reused; removed the stale launcher-owned PID record and did not signal it."
    return 0
  fi
  if [[ "$live_pgid" != "$pgid" || "$pgid" != "$pid" || "$cmdline" != *"$SCRIPT_PATH"* ]]; then
    echo "BLOCKED: live PID/group/command identity did not match the recorded project session; no process was signaled."
    return 3
  fi
  if ! process_group_alive "$pgid"; then
    remove_owned_state "$pid" "$SCRIPT_PATH"
    echo "No process remains in the tracked group; removed its stale PID record."
    return 0
  fi
  echo "Requesting graceful SIGINT for verified project process group $pgid only."
  kill -INT -- "-$pgid"
  local i
  for i in $(seq 1 300); do
    if ! process_group_alive "$pgid"; then
      echo "Tracked project session stopped cleanly."
      return 0
    fi
    sleep 0.1
  done
  echo "BLOCKED: project process group $pgid is still present after 30 seconds. No stronger signal was sent; retry Stop or inspect the session log."
  return 3
}

MODE=""
if (($#)); then MODE="$1"; fi
if [[ "$MODE" == stop ]]; then
  stop_tracked_session
  exit $?
fi
if [[ "$MODE" == status ]]; then
  if [[ -f "$STATE_FILE" ]]; then cat "$STATE_FILE"; else echo "NO_ACTIVE_PROJECT_SESSION"; fi
  exit 0
fi
if [[ "$MODE" != run || $# -ne 7 ]]; then
  echo "Usage: windows_demo_session.sh run phase6|lidar S00|S01|S02|S03|S04|S09 SEED true|false direct|detached LOG_PATH" >&2
  echo "       windows_demo_session.sh stop|status" >&2
  exit 2
fi
shift
profile="$1"
scenario="$2"
seed="$3"
rviz="$4"
display_mode="$5"
log_path="$6"
case "$profile" in phase6|lidar);; *) echo "ERROR: invalid profile." >&2; exit 2;; esac
case "$scenario" in S00|S01|S02|S03|S04|S09);; *) echo "ERROR: invalid scenario." >&2; exit 2;; esac
[[ "$seed" =~ ^-?[0-9]+$ ]] || { echo "ERROR: seed must be an integer." >&2; exit 2; }
[[ "$rviz" == true || "$rviz" == false ]] || { echo "ERROR: invalid RViz option." >&2; exit 2; }
[[ "$display_mode" == direct || "$display_mode" == detached ]] || { echo "ERROR: invalid display mode." >&2; exit 2; }

mkdir -p "$STATE_DIR" "$(dirname -- "$log_path")"
chmod 700 "$STATE_DIR"
exec > >(tee -a "$log_path") 2>&1
run_id="windows-$profile-$scenario-seed$seed-$(date -u +%Y%m%dT%H%M%SZ)-$$"
pgid=$(ps -o pgid= -p "$$" | tr -d ' ')
start_ticks=$(python3 - "$$" <<'PY'
from pathlib import Path
import sys
line = Path("/proc", sys.argv[1], "stat").read_text()
fields = line[line.rfind(")") + 2:].split()
print(fields[19])
PY
)
if [[ "$pgid" != "$$" ]]; then
  echo "ERROR: session was not isolated in its own process group; refusing to launch."
  exit 3
fi
if [[ -e "$STATE_FILE" ]]; then
  echo "ERROR: an active launcher session state file already exists; refusing a competing launch."
  exit 4
fi
python3 - "$STATE_FILE" "$$" "$pgid" "$start_ticks" "$SCRIPT_PATH" "$profile" "$scenario" "$seed" "$run_id" "$log_path" "$rviz" <<'PY'
import json, os, pathlib, sys
path = pathlib.Path(sys.argv[1])
state = {
    "pid": int(sys.argv[2]), "pgid": int(sys.argv[3]),
    "start_ticks": sys.argv[4], "script": sys.argv[5],
    "profile": sys.argv[6], "scenario": sys.argv[7],
    "seed": int(sys.argv[8]), "run_id": sys.argv[9],
    "log_path": sys.argv[10], "rviz": sys.argv[11] == "true",
}
tmp = path.with_name(path.name + ".tmp." + str(os.getpid()))
tmp.write_text(json.dumps(state, sort_keys=True) + "\n", encoding="utf-8")
os.chmod(tmp, 0o600)
tmp.replace(path)
PY
cleanup_state() {
  remove_owned_state "$$" "$SCRIPT_PATH"
}
trap cleanup_state EXIT

echo "Session ID: $run_id"
echo "Profile: $profile"
echo "Scenario: $scenario | seed: $seed | Gazebo GUI: true | RViz: $rviz"
echo "Parking: AUTONOMOUS PARKING — NOT YET LIVE-VALIDATED"
echo "Session log: $log_path"

set +u
source /opt/ros/jazzy/setup.bash || { echo "ERROR: sourcing ROS 2 Jazzy failed."; exit 5; }
source "$ROOT/simulation/ros2_ws/install/setup.bash" || { echo "ERROR: sourcing the installed project workspace failed."; exit 5; }
set -u
if [[ "$profile" == phase6 ]]; then
  echo "REFERENCE NAVIGATION — PREDEFINED MAP"
  echo "This mode uses the existing geometry-derived ORACLE_REFERENCE map for localization/planning; it does not validate RGB-only or online LiDAR navigation."
  if [[ "$rviz" == true ]]; then
    python3 "$ROOT/scripts/phase6_navigation_runner.py" --scenario "$scenario" --seed "$seed" --run-id "$run_id" --gui --rviz --hold-window
  else
    python3 "$ROOT/scripts/phase6_navigation_runner.py" --scenario "$scenario" --seed "$seed" --run-id "$run_id" --gui --hold-window
  fi
else
  echo "REALISTIC RGB + LiDAR — CURRENT VERIFIED CAPABILITIES"
  echo "Sensing, local mapping and fail-closed guarded straight movement only. Obstacle avoidance and Nav2 goal navigation remain unvalidated."
  if [[ "$scenario" == S00 ]]; then
    if [[ "$rviz" == true ]]; then
      bash "$ROOT/scripts/run_lidar_demo.sh" --scenario "$scenario" --seed "$seed" --gui --rviz --demo-status --demo-forward
    else
      bash "$ROOT/scripts/run_lidar_demo.sh" --scenario "$scenario" --seed "$seed" --gui --demo-status --demo-forward
    fi
  else
    if [[ "$rviz" == true ]]; then
      bash "$ROOT/scripts/run_lidar_demo.sh" --scenario "$scenario" --seed "$seed" --gui --rviz --demo-status
    else
      bash "$ROOT/scripts/run_lidar_demo.sh" --scenario "$scenario" --seed "$seed" --gui --demo-status
    fi
  fi
fi
