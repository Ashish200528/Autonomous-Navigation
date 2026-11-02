#!/usr/bin/env bash
set -Ee -o pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
usage(){ echo "Usage: ./scripts/run_lidar_demo.sh [--scenario S00|S01|S02|S03|S04|S09] [--seed INTEGER] [--gui] [--goal] [--duration SECONDS]"; echo "Starts RGB + 2D LiDAR with online SLAM; Nav2 requires a checked-in readiness gate."; }
scenario=S00; seed=101; gui=false; want_goal=false; duration=90
while (($#)); do case "$1" in
 --scenario) scenario="${2:?missing scenario}"; shift 2;;
 --seed) seed="${2:?missing seed}"; shift 2;;
 --gui) gui=true; shift;;
 --goal) want_goal=true; shift;;
 --duration) duration="${2:?missing duration}"; shift 2;;
 -h|--help) usage; exit 0;;
 *) echo "ERROR: unknown argument $1" >&2; usage >&2; exit 2;;
esac; done
case "$scenario" in S00|S01|S02|S03|S04|S09);; *) echo "ERROR: unsupported scenario $scenario" >&2; exit 2;; esac
[[ "$seed" =~ ^-?[0-9]+$ ]] || { echo "ERROR: seed must be an integer." >&2; exit 2; }
[[ "$duration" =~ ^[0-9]+$ ]] && ((duration>=1 && duration<=180)) || { echo "ERROR: duration must be 1..180 seconds." >&2; exit 2; }
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
  gate=$(python3 - "$readiness" "$ROOT" <<'PY'
import json, pathlib, sys
data=json.loads(pathlib.Path(sys.argv[1]).read_text()); root=pathlib.Path(sys.argv[2])
required=("production_turning_verified","moving_slam_verified","guarded_command_chain_verified")
checks=data.get("prerequisites",{}); evidence=data.get("evidence",{})
missing=[k for k in required if checks.get(k) is not True]
missing_files=[k for k in required if not evidence.get(k) or not (root/evidence[k]).is_file()]
why=[f"{k}=false" for k in missing]+[f"{k} evidence missing" for k in missing_files]
print(("BLOCKED\t"+"; ".join(why)) if why else "READY\tprerequisite evidence present")
PY
)
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
Goal requested: $want_goal
Nav2 enabled: $nav2
Readiness: $block_reason
Memory: Windows ${host_mib} MiB available; WSL $((wsl_kib/1024)) MiB available
TXT
if [[ "$want_goal" == true && "$nav2" != true ]]; then
 echo "GOAL BLOCKED: $block_reason"
 echo "Running sensor + online-SLAM only. This is not autonomous navigation."
fi
launch_log="$run_dir/ros2-launch.log"
ros2 launch anc_lidar realistic_rgb_lidar.launch.py "world_file:=$world" "start_x:=$sx" "start_y:=$sy" "start_yaw:=$syaw" "gui:=$gui" "enable_slam:=true" "enable_nav2:=$nav2" >"$launch_log" 2>&1 &
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
if [[ "$nav2" == true ]]; then
 set +e
 python3 scripts/phase8c_navigation_runner.py --layout "$layout" --output-dir "$run_dir" --timeout-s "$duration" 2>&1 | tee "$run_dir/navigation-runner.log"
 result=${PIPESTATUS[0]}
 set -e
 echo "Navigation evaluator exit code: $result"
else
 echo "Demo runs for $duration seconds. ROS log: $launch_log"
 sleep "$duration"
fi
stop_launch
trap - EXIT INT TERM
echo "Evidence saved: $run_dir"
echo "Review demo-status.txt, logs, and results before describing this run."
exit "$result"
