#!/usr/bin/env bash
set -Ee -o pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
usage(){ echo "Usage: ./scripts/run_lidar_demo.sh [--scenario S00|S01|S02|S03|S04|S09] [--seed INTEGER] [--gui] [--rviz] [--demo-status] [--demo-forward] [--hold-gui] [--goal] [--duration SECONDS]"; echo "Starts RGB + 2D LiDAR with online SLAM; Nav2 requires a checked-in readiness gate."; }
scenario=S00; seed=101; gui=false; rviz=false; demo_status=false; demo_forward=false; hold_gui=false; want_goal=false; duration=90
while (($#)); do case "$1" in
 --scenario) scenario="${2:?missing scenario}"; shift 2;;
 --seed) seed="${2:?missing seed}"; shift 2;;
 --gui) gui=true; shift;;
 --rviz) rviz=true; shift;;
 --demo-status) demo_status=true; shift;;
 --demo-forward) demo_forward=true; shift;;
 --hold-gui) hold_gui=true; shift;;
 --goal) want_goal=true; shift;;
 --duration) duration="${2:?missing duration}"; shift 2;;
 -h|--help) usage; exit 0;;
 *) echo "ERROR: unknown argument $1" >&2; usage >&2; exit 2;;
esac; done
case "$scenario" in S00|S01|S02|S03|S04|S09);; *) echo "ERROR: unsupported scenario $scenario" >&2; exit 2;; esac
[[ "$seed" =~ ^-?[0-9]+$ ]] || { echo "ERROR: seed must be an integer." >&2; exit 2; }
[[ "$duration" =~ ^[0-9]+$ ]] && ((duration>=1 && duration<=180)) || { echo "ERROR: duration must be 1..180 seconds." >&2; exit 2; }
[[ "$rviz" != true || "$gui" == true ]] || { echo "ERROR: RViz requires --gui." >&2; exit 2; }
[[ "$hold_gui" != true || "$gui" == true ]] || { echo "ERROR: --hold-gui requires --gui." >&2; exit 2; }
[[ "$demo_status" != true || "$gui" == true ]] || { echo "ERROR: --demo-status requires --gui." >&2; exit 2; }
[[ "$demo_forward" != true || ( "$scenario" == S00 && "$demo_status" == true && "$gui" == true ) ]] || { echo "ERROR: guarded forward display is S00-only and requires GUI status mode." >&2; exit 2; }
source /opt/ros/jazzy/setup.bash
source "$ROOT/simulation/ros2_ws/install/setup.bash"
set -u
command -v ros2 >/dev/null || { echo "ERROR: ROS 2 Jazzy is not sourced." >&2; exit 3; }

read_windows_available_mib(){
 local ps encoded value
 ps=$(cat <<'PS'
$src=@"
using System;
using System.Runtime.InteropServices;
public static class HostMemProbe {
 [StructLayout(LayoutKind.Sequential, CharSet=CharSet.Auto)]
 public class MEMORYSTATUSEX {
  public uint dwLength=(uint)Marshal.SizeOf(typeof(MEMORYSTATUSEX));
  public uint dwMemoryLoad; public ulong ullTotalPhys; public ulong ullAvailPhys;
  public ulong ullTotalPageFile; public ulong ullAvailPageFile;
  public ulong ullTotalVirtual; public ulong ullAvailVirtual; public ulong ullAvailExtendedVirtual;
 }
 [DllImport("kernel32.dll", CharSet=CharSet.Auto, SetLastError=true)]
 public static extern bool GlobalMemoryStatusEx([In,Out] MEMORYSTATUSEX lpBuffer);
}
"@
Add-Type -TypeDefinition $src
$m=[HostMemProbe+MEMORYSTATUSEX]::new()
[void][HostMemProbe]::GlobalMemoryStatusEx($m)
[math]::Floor($m.ullAvailPhys / 1MB)
PS
)
 encoded=$(printf '%s' "$ps" | iconv -f UTF-8 -t UTF-16LE | base64 -w0)
 value=$(powershell.exe -NoLogo -NoProfile -NonInteractive -EncodedCommand "$encoded" 2>/dev/null | tr -d '\r' | awk '/^[0-9]+$/ {v=$0} END {if(v!="") print v}')
 [[ "$value" =~ ^[0-9]+$ ]] || return 1
 printf '%s' "$value"
}
host_mib="$(read_windows_available_mib)" || { echo "BLOCKED: Windows RAM is unavailable through WSL interop; no simulation started." >&2; exit 4; }
min_host_mib=2048; [[ "$gui" == true ]] && min_host_mib=2560
wsl_kib="$(awk '/^MemAvailable:/ {print $2}' /proc/meminfo)"
if ((host_mib<min_host_mib || wsl_kib<1572864)); then
 echo "BLOCKED: insufficient memory; no simulator started."
 echo "Windows available: ${host_mib} MiB; required: ${min_host_mib} MiB."
 echo "WSL available: $((wsl_kib/1024)) MiB; required: 1536 MiB."
 exit 4
fi
nav2=false; block_reason="goal not requested"
if [[ "$want_goal" == true ]]; then
 readiness="$ROOT/config/phase8c_demo_readiness.json"
 if [[ -f "$readiness" ]]; then
  gate=$(python3 "$ROOT/scripts/phase8c_readiness.py" "$readiness" "$ROOT" --brief)
  gate_state="${gate%%$'\t'*}"; block_reason="${gate#*$'\t'}"
  [[ "$gate_state" == READY ]] && nav2=true
 else block_reason="readiness file missing"
 fi
fi
stamp="$(date -u +%Y%m%dT%H%M%SZ)"
run_dir="$ROOT/docs/evidence/phase8c/lidar/demos/$scenario-seed$seed-$stamp"
mkdir -p "$(dirname "$run_dir")"
[[ ! -e "$run_dir" ]] || { echo "ERROR: refusing to overwrite $run_dir" >&2; exit 5; }
mkdir "$run_dir"
python3 scripts/scenario_generator.py --scenario "$scenario" --seed "$seed" --output-dir "$run_dir/scenario" >"$run_dir/scenario-generation.log" 2>&1
layout="$run_dir/scenario/manifest.json"
world="$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['world_file'])" "$layout")"
read -r sx sy syaw < <(python3 -c "import json,sys; print(*json.load(open(sys.argv[1]))['start_xyyaw'])" "$layout")
cat >"$run_dir/demo-status.txt" <<TXT
Scenario: $scenario
Seed: $seed
GUI: $gui
RViz: $rviz
Goal requested: $want_goal
Nav2 enabled: $nav2
Readiness: $block_reason
Full LiDAR goal navigation: DISABLED by this demo unless readiness evidence independently passes; no readiness flags are changed.
Guarded forward request: $demo_forward (S00-only, request path /cmd_vel_nav; safety guard remains sole /cmd_vel owner)
Memory: Windows ${host_mib} MiB available; WSL $((wsl_kib/1024)) MiB available
TXT
if [[ "$want_goal" == true && "$nav2" != true ]]; then
 echo "GOAL BLOCKED: $block_reason"
 echo "Running sensor + online-SLAM only. This is not autonomous navigation."
fi
launch_log="$run_dir/ros2-launch.log"
ros2 launch anc_lidar realistic_rgb_lidar.launch.py "world_file:=$world" "start_x:=$sx" "start_y:=$sy" "start_yaw:=$syaw" "gui:=$gui" "rviz:=$rviz" "enable_slam:=true" "enable_nav2:=$nav2" >"$launch_log" 2>&1 &
launch_pid=$!
stop_launch(){ if [[ -n "${launch_pid:-}" ]]; then kill -INT "$launch_pid" 2>/dev/null || true; wait "$launch_pid" 2>/dev/null || true; launch_pid=""; fi; }
trap stop_launch EXIT
trap 'stop_launch; exit 130' INT
trap 'stop_launch; exit 143' TERM
sleep 3
if ! kill -0 "$launch_pid" 2>/dev/null; then
 set +e; wait "$launch_pid"; rc=$?; set -e; launch_pid=""
 echo "ERROR: ROS launch exited during startup (status $rc). See $launch_log" >&2
 exit 6
fi
result=0
if [[ "$demo_status" == true ]]; then
 echo "REALISTIC RGB + LiDAR — CURRENT VERIFIED CAPABILITIES"
 echo "Nav2 goal navigation stays disabled unless its readiness file and evidence independently pass."
 monitor_args=(--scenario "$scenario" --output "$run_dir/phase8c-demo-status.json"
               --launch-pid "$launch_pid" --monitor-duration-s "$duration")
 [[ "$demo_forward" != true ]] || monitor_args+=(--request-forward)
 set +e
 python3 scripts/phase8c_gui_demo.py "${monitor_args[@]}"
 result=$?
 set -e
 echo "Sensor/status monitor exit code: $result"
elif [[ "$nav2" == true ]]; then
 set +e
 python3 scripts/phase8c_navigation_runner.py --layout "$layout" --output-dir "$run_dir" --timeout-s "$duration" 2>&1 | tee "$run_dir/navigation-runner.log"
 result=${PIPESTATUS[0]}
 set -e
 echo "Navigation evaluator exit code: $result"
else
 echo "Demo runs for $duration seconds. ROS log: $launch_log"
 sleep "$duration"
fi
if [[ "$hold_gui" == true && "$demo_status" != true ]]; then
 echo "Gazebo remains visible. Press Ctrl+C here or use the launcher menu Stop option."
 while :; do sleep 3600 & wait $!; done
fi
stop_launch
trap - EXIT INT TERM
echo "Evidence saved: $run_dir"
echo "Review demo-status.txt, logs, and results before describing this run."
exit "$result"
