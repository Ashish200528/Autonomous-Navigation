#!/usr/bin/env bash
set -Ee -o pipefail
TOOLS_DIR="$(cd -- "$(dirname -- "$(readlink -f -- "$0")")" && pwd)"
ORCHESTRATOR="$TOOLS_DIR/wsl_orchestrator.py"
if [[ "$#" -gt 0 && "$1" == "--self-test" ]]; then
  exec python3 "$TOOLS_DIR/collect_results.py" self-test
fi
if [[ "$#" -eq 2 && "$1" == "--stop" ]]; then
  exec python3 "$ORCHESTRATOR" stop "$2"
fi
if [[ "$#" -ne 9 || "$1" != "--run" ]]; then
  echo "Usage: wsl_test_runner.sh --run RUN_ID RUN_DIR SETTINGS PROJECT CONTROLLER_PID DRY_RUN CONTROLLER_START LATEST_DIR" >&2
  echo "       wsl_test_runner.sh --stop RUN_DIR | --self-test" >&2
  exit 2
fi
shift
RUN_ID="$1"; RUN_DIR="$2"; SETTINGS="$3"; PROJECT="$4"
CONTROLLER_PID="$5"; DRY_RUN="$6"; CONTROLLER_START="$7"; LATEST_DIR="$8"
[[ "$DRY_RUN" == true || "$DRY_RUN" == false ]] || { echo "Invalid DRY_RUN value." >&2; exit 2; }
[[ -r /opt/ros/jazzy/setup.bash ]] || { echo "BLOCKED: ROS 2 Jazzy setup is missing."; exit 3; }
[[ -r "$PROJECT/simulation/ros2_ws/install/setup.bash" ]] || { echo "BLOCKED: project ROS workspace setup is missing."; exit 3; }
source /opt/ros/jazzy/setup.bash
source "$PROJECT/simulation/ros2_ws/install/setup.bash"
set -u
export PYTHONUNBUFFERED=1 ANC_TEST_RUN_ID="$RUN_ID"
exec python3 "$ORCHESTRATOR" run "$RUN_ID" "$RUN_DIR" "$SETTINGS" "$PROJECT" "$CONTROLLER_PID" "$DRY_RUN" "$CONTROLLER_START" "$LATEST_DIR"
