#!/usr/bin/env bash
set -Eeuo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "$BASH_SOURCE")" && pwd)
ROOT=$(cd -- "$SCRIPT_DIR/.." && pwd)
DISTRO="Ubuntu-24.04"
EXPECTED_BRANCH="feature/phase8c-lidar"
SESSION_SCRIPT="$ROOT/scripts/windows_demo_session.sh"
SESSION_STATE="$HOME/.cache/anc_demo_launcher/current.json"
WINDOWS_TOTAL_MIB=0
WINDOWS_AVAILABLE_MIB=0
WINDOWS_LOAD=0
WINDOWS_THREADS=0
WSL_AVAILABLE_MIB=0
DISK_AVAILABLE_MIB=0
SCENARIO="S00"
SEED="101"
RVIZ="false"

ps_eval() {
  local script="$1" encoded
  encoded=$(printf '%s' "$script" | iconv -f UTF-8 -t UTF-16LE | base64 -w0)
  powershell.exe -NoLogo -NoProfile -NonInteractive -EncodedCommand "$encoded" 2>/dev/null | tr -d '\r\000'
}

read_windows_memory() {
  local script encoded value
  script=$(cat <<'PS'
$src=@"
using System;
using System.Runtime.InteropServices;
public static class DemoHostMemory {
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
$m=[DemoHostMemory+MEMORYSTATUSEX]::new()
if (-not [DemoHostMemory]::GlobalMemoryStatusEx($m)) { exit 2 }
"{0}|{1}|{2}|{3}" -f [math]::Floor($m.ullTotalPhys/1MB),[math]::Floor($m.ullAvailPhys/1MB),$m.dwMemoryLoad,[Environment]::ProcessorCount
PS
)
  encoded=$(printf '%s' "$script" | iconv -f UTF-8 -t UTF-16LE | base64 -w0)
  value=$(powershell.exe -NoLogo -NoProfile -NonInteractive -EncodedCommand "$encoded" 2>/dev/null | tr -d '\r\000' | awk -F'|' '/^[0-9]+\|[0-9]+\|[0-9]+\|[0-9]+$/ {line=$0} END {if(line!="") print line}')
  [[ "$value" == *"|"* ]] || return 1
  IFS='|' read -r WINDOWS_TOTAL_MIB WINDOWS_AVAILABLE_MIB WINDOWS_LOAD WINDOWS_THREADS <<< "$value"
}

windows_process_summary() {
  local script
  script=$(cat <<'PS'
$processorCount=[Environment]::ProcessorCount
$before=@{}
Get-Process -ErrorAction SilentlyContinue | ForEach-Object { $before[$_.Id]=$_.CPU }
Start-Sleep -Milliseconds 400
Get-Process -ErrorAction SilentlyContinue |
 Sort-Object WorkingSet64 -Descending |
 Select-Object -First 10 |
 ForEach-Object {
  $cpuPct=0.0
  if ($before.ContainsKey($_.Id) -and $_.CPU -ne $null) {
   $cpuPct=[math]::Round([math]::Max(0,($_.CPU-$before[$_.Id]) / 0.4 / $processorCount * 100),1)
  }
  "{0} pid={1} ram={2}MiB cpu={3}%" -f $_.ProcessName,$_.Id,[math]::Round($_.WorkingSet64/1MB),$cpuPct
 }
PS
)
  ps_eval "$script"
}

registered_distro_path() {
  local script
  script=$(cat <<'PS'
$root='HKCU:\Software\Microsoft\Windows\CurrentVersion\Lxss'
try {
 $entry=Get-ChildItem -LiteralPath $root -ErrorAction Stop | ForEach-Object { Get-ItemProperty -LiteralPath $_.PSPath } | Where-Object { $_.DistributionName -eq 'Ubuntu-24.04' } | Select-Object -First 1
 if ($entry) { [string]$entry.BasePath } else { 'NOT_REGISTERED' }
} catch { 'REGISTRY_UNAVAILABLE' }
PS
)
  ps_eval "$script" | tail -1
}

protected_codex_inventory() {
  local script
  script=$(cat <<'PS'
$items=Get-Process -Name 'ChatGPT','Codex' -ErrorAction SilentlyContinue | Where-Object { $_.MainWindowHandle -ne 0 }
if (-not $items) { 'NO_PROTECTED_CODEX_WINDOW_FOUND'; exit }
$items | ForEach-Object {
 $packagePath=($_.Path -match '(?i)\\WindowsApps\\OpenAI\.Codex_[^\\]+\\app\\ChatGPT\.exe$')
 "{0}|{1}|{2}|main-window={3}|OpenAI-package-path={4}" -f $_.ProcessName,$_.Id,$_.Path,$_.MainWindowHandle,$packagePath
}
PS
)
  ps_eval "$script"
}

read_wsl_memory() {
  local kib
  kib=$(awk '/^MemAvailable:/ {print $2; exit}' /proc/meminfo)
  [[ "$kib" =~ ^[0-9]+$ ]] || return 1
  WSL_AVAILABLE_MIB=$((kib / 1024))
}

read_disk_memory() {
  local kib
  kib=$(df -Pk "$ROOT" | awk 'NR==2 {print $4}')
  [[ "$kib" =~ ^[0-9]+$ ]] || return 1
  DISK_AVAILABLE_MIB=$((kib / 1024))
}

show_resource_snapshot() {
  if ! read_windows_memory; then
    echo "Windows memory API unavailable through WSL interop."
    return 1
  fi
  read_wsl_memory || return 1
  read_disk_memory || return 1
  echo "Windows physical RAM: $(awk -v mib="$WINDOWS_TOTAL_MIB" 'BEGIN {printf "%.2f",mib/1024}') GiB | available: $WINDOWS_AVAILABLE_MIB MiB | load: $WINDOWS_LOAD% | logical CPUs: $WINDOWS_THREADS"
  echo "WSL memory available: $WSL_AVAILABLE_MIB MiB | swap: $(awk '/^SwapFree:/ {printf "%d MiB",$2/1024}' /proc/meminfo)"
  echo "WSL project filesystem free: $DISK_AVAILABLE_MIB MiB"
  echo "Windows processes by RAM (CPU shown as sampled percent):"
  windows_process_summary | sed -n '1,10p'
  echo "WSL processes by RSS:"
  ps -eo pid,pgid,pcpu,rss,comm,args --sort=-rss | head -n 9
  echo "WSL kernel: $(uname -r)"
  echo "WSLg display: WAYLAND_DISPLAY=$(printenv WAYLAND_DISPLAY 2>/dev/null || true) DISPLAY=$(printenv DISPLAY 2>/dev/null || true)"
}

print_preflight_state() {
  echo "Checking project-specific simulator processes..."
  local current
  current=$(ps -eo pid,ppid,pgid,pcpu,rss,comm,args | awk -v root="$ROOT" -v self="$$" -v parent="$PPID" '$1 != self && $1 != parent && $0 ~ root && ($6 == "gz" || $6 == "gzserver" || $6 == "rviz2" || ($6 == "ros2" && /launch/) || ($6 == "python3" && /ros2 launch/) || ($6 == "bash" && /run_lidar_demo.sh/)) {print}')
  if [[ -n "$current" ]]; then
    echo "$current"
    echo "These project processes are not owned by the launcher PID record. They will be left untouched and this launch will stop for safety."
    return 1
  fi
  echo "No untracked project Gazebo/RViz/ROS launch processes found."
}

offer_close_apps() {
  local protected
  protected=$(protected_codex_inventory || true)
  echo "Preserving the active ChatGPT/Codex execution and all user applications."
  if [[ -n "$protected" && "$protected" != *"NO_PROTECTED_CODEX_WINDOW_FOUND"* ]]; then
    echo "Protected ChatGPT/Codex process identity:"
    echo "$protected"
  fi
  echo "No desktop app is closed automatically because this launcher cannot prove that an app has no unsaved work."
  return 0
}

check_project_environment() {
  local branch base_path ros_prefix gz_prefix nav_prefix lidar_prefix
  base_path=$(registered_distro_path)
  echo "Ubuntu-24.04 registered WSL base path: $base_path"
  case "$base_path" in
    [Ee]:\\WSL\\Ubuntu-24.04*|[Ee]:/WSL/Ubuntu-24.04*) ;;
    *) echo "BLOCKED: expected Ubuntu-24.04 storage under E:\\WSL\\Ubuntu-24.04; no package or simulator was started."; return 1;;
  esac
  branch=$(git -C "$ROOT" rev-parse --abbrev-ref HEAD)
  if [[ "$branch" != "$EXPECTED_BRANCH" ]]; then
    echo "BLOCKED: expected branch $EXPECTED_BRANCH, found $branch."
    return 1
  fi
  [[ -r /opt/ros/jazzy/setup.bash ]] || { echo "BLOCKED: /opt/ros/jazzy/setup.bash is missing."; return 1; }
  [[ -r "$ROOT/simulation/ros2_ws/install/setup.bash" ]] || { echo "BLOCKED: installed project ROS workspace is missing."; return 1; }
  set +u
  source /opt/ros/jazzy/setup.bash || { echo "BLOCKED: sourcing ROS 2 Jazzy failed."; set -u; return 1; }
  source "$ROOT/simulation/ros2_ws/install/setup.bash" || { echo "BLOCKED: sourcing the installed project workspace failed."; set -u; return 1; }
  set -u
  command -v ros2 >/dev/null || { echo "BLOCKED: ROS 2 CLI is unavailable after sourcing."; return 1; }
  ros_prefix=$(ros2 pkg prefix ros_gz_sim 2>/dev/null) || { echo "BLOCKED: ros_gz_sim is not installed in the sourced environment."; return 1; }
  gz_prefix=$(ros2 pkg prefix anc_gazebo 2>/dev/null) || { echo "BLOCKED: project Gazebo package is not installed."; return 1; }
  nav_prefix=$(ros2 pkg prefix anc_navigation 2>/dev/null) || { echo "BLOCKED: project navigation package is not installed."; return 1; }
  lidar_prefix=$(ros2 pkg prefix anc_lidar 2>/dev/null) || { echo "BLOCKED: project LiDAR package is not installed."; return 1; }
  echo "ROS 2: $(ros2 --help >/dev/null 2>&1 && echo available); ros_gz_sim: $ros_prefix"
  echo "Project packages: anc_gazebo=$gz_prefix; anc_navigation=$nav_prefix; anc_lidar=$lidar_prefix"
  if ! command -v gz >/dev/null || ! gz sim --version >/dev/null 2>&1; then
    echo "BLOCKED: Gazebo Sim command is not available."
    return 1
  fi
  echo "Gazebo Sim command is available."
  if [[ -z "$(printenv WAYLAND_DISPLAY 2>/dev/null || true)" && -z "$(printenv DISPLAY 2>/dev/null || true)" ]]; then
    echo "BLOCKED: WSLg has no DISPLAY or WAYLAND_DISPLAY; graphical Gazebo cannot be shown."
    return 1
  fi
  echo "Project checkout: $ROOT"
  echo "Git branch: $branch"
  echo "No package installation or system configuration changes will be made by the launcher."
}

prepare_launch() {
  local profile="$1" selected_rviz="$2"
  echo "=== Safe resource preflight: $profile ==="
  echo "Parking remains NOT YET LIVE-VALIDATED."
  show_resource_snapshot || { echo "BLOCKED: could not read Windows/WSL memory."; return 1; }
  if offer_close_apps; then :; else return $?; fi
  if ! bash "$SESSION_SCRIPT" stop; then
    echo "BLOCKED: a tracked project session did not stop cleanly."
    return 1
  fi
  if ! print_preflight_state; then
    echo "BLOCKED: untracked project simulation session left running; no new launch started."
    return 1
  fi
  echo "Resource snapshot after preserving desktop apps and tracked-session cleanup:"
  show_resource_snapshot || { echo "BLOCKED: could not re-read host memory."; return 1; }
  if [[ "$WINDOWS_AVAILABLE_MIB" -lt 2560 || "$WSL_AVAILABLE_MIB" -lt 1536 ]]; then
    echo "BLOCKED: Gazebo GUI requires at least 2560 MiB available on Windows and 1536 MiB in WSL."
    echo "Current: Windows $WINDOWS_AVAILABLE_MIB MiB; WSL $WSL_AVAILABLE_MIB MiB. Free memory, then double-click again."
    return 1
  fi
  if [[ "$selected_rviz" == true && "$WINDOWS_AVAILABLE_MIB" -lt 3328 ]]; then
    echo "RViz is optional and the current Windows headroom is below the 3328 MiB threshold."
    read -r -p "Continue with Gazebo only (without RViz)? [Y/n] " reply
    reply=$(printf '%s' "$reply" | tr '[:upper:]' '[:lower:]')
    if [[ "$reply" == n || "$reply" == no ]]; then
      echo "BLOCKED: user chose not to launch without RViz."
      return 1
    fi
    selected_rviz=false
  fi
  if [[ "$DISK_AVAILABLE_MIB" -lt 3072 ]]; then
    echo "BLOCKED: keep at least 3072 MiB free in the WSL project filesystem for logs and generated worlds."
    return 1
  fi
  check_project_environment || return 1
  RVIZ="$selected_rviz"
}

choose_scenario() {
  echo
  echo "Choose reference-navigation scenario: 1) S00 empty  2) S01 box obstacle  3) S02  4) S03  5) S04  6) S09 no-route"
  local choice
  read -r -p "Scenario [1]: " choice
  case "$choice" in
    ""|1) SCENARIO=S00;; 2) SCENARIO=S01;; 3) SCENARIO=S02;;
    4) SCENARIO=S03;; 5) SCENARIO=S04;; 6) SCENARIO=S09;;
    *) echo "Invalid scenario selection."; return 1;;
  esac
}

choose_sensor_scenario() {
  echo
  echo "Choose a stationary RGB + LiDAR sensor scenario (these do not move):"
  echo "1) S01 box obstacle  2) S02 obstacle field  3) S03 narrow corridor  4) S04  5) S09 no-route"
  local choice
  read -r -p "Sensor scenario [2]: " choice
  case "$choice" in
    ""|2) SCENARIO=S02;; 1) SCENARIO=S01;; 3) SCENARIO=S03;;
    4) SCENARIO=S04;; 5) SCENARIO=S09;;
    *) echo "Invalid sensor scenario selection."; return 1;;
  esac
}

choose_seed() {
  local value
  read -r -p "Random seed [101] (or type random): " value
  if [[ -z "$value" ]]; then
    SEED=101
  elif [[ "$value" == random ]]; then
    SEED=$(od -An -N4 -tu4 /dev/urandom | tr -d ' ')
  elif [[ "$value" =~ ^-?[0-9]+$ ]]; then
    SEED="$value"
  else
    echo "Seed must be an integer or random."
    return 1
  fi
  echo "Selected scenario $SCENARIO, seed $SEED."
}

choose_rviz() {
  local answer
  read -r -p "Open optional RViz too? [y/N] " answer
  answer=$(printf '%s' "$answer" | tr '[:upper:]' '[:lower:]')
  if [[ "$answer" == y || "$answer" == yes ]]; then RVIZ=true; else RVIZ=false; fi
}

latest_evidence_dir() {
  local candidate
  candidate=$(
    {
      find "$ROOT/docs/evidence/phase6/runs" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' 2>/dev/null || true
      find "$ROOT/docs/evidence/phase8c/lidar/demos" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' 2>/dev/null || true
      find "$ROOT/docs/evidence/windows-demos" -mindepth 1 -maxdepth 1 -type f -printf '%T@ %h\n' 2>/dev/null || true
    } | sort -nr | head -n 1
  )
  [[ -n "$candidate" ]] || return 1
  printf '%s' "$candidate" | cut -d' ' -f2-
}

open_latest_evidence() {
  local folder win_path
  folder=$(latest_evidence_dir) || { echo "No demo evidence folder found yet."; return 0; }
  echo "Latest evidence folder: $folder"
  win_path=$(wslpath -w "$folder")
  if command -v explorer.exe >/dev/null; then
    explorer.exe "$win_path" >/dev/null 2>&1 || echo "Could not launch Explorer; use the WSL path above."
  else
    echo "Explorer interop is unavailable; use the WSL path above."
  fi
}

make_log_path() {
  local mode="$1" scenario="$2" seed="$3"
  mkdir -p "$ROOT/docs/evidence/windows-demos"
  printf '%s' "$ROOT/docs/evidence/windows-demos/$mode-$scenario-seed$seed-$(date -u +%Y%m%dT%H%M%SZ)-$$.log"
}

start_profile() {
  local profile="$1" detached="$2" log_path
  if ! prepare_launch "$profile" "$RVIZ"; then return 1; fi
  log_path=$(make_log_path "$profile" "$SCENARIO" "$SEED")
  if [[ "$detached" == true ]]; then
    echo "Starting the simulation in its own WSL process group. Its log is $log_path"
    setsid -f bash "$SESSION_SCRIPT" run "$profile" "$SCENARIO" "$SEED" "$RVIZ" detached "$log_path" >/dev/null 2>&1 </dev/null
    local n
    for n in $(seq 1 30); do
      if [[ -f "$SESSION_STATE" ]]; then
        echo "Demo session started. Use menu option 6 to stop it; option 7 follows its status log."
        return 0
      fi
      sleep 0.2
    done
    echo "The session did not create its tracking record. Review startup log: $log_path"
    tail -n 40 "$log_path" 2>/dev/null || true
    return 1
  fi
  echo "Live demo status follows in this window. Ctrl+C stops this tracked project session."
  env --default-signal=INT,QUIT setsid --wait bash "$SESSION_SCRIPT" run "$profile" "$SCENARIO" "$SEED" "$RVIZ" direct "$log_path" &
  local session_wait_pid=$! result=0
  trap 'bash "$SESSION_SCRIPT" stop || true' INT TERM
  if wait "$session_wait_pid"; then result=0; else result=$?; fi
  trap - INT TERM
  return "$result"
}

run_individual() {
  local profile="$1"
  if [[ "$profile" == phase6 ]]; then
    choose_scenario || return 1
  else
    SCENARIO=S00
    echo "Using S00 clear arena for the guarded LiDAR movement demonstration."
    echo "It requests a visible straight move up to 0.15 m or 4 seconds, with measured motion and stop confirmation."
    echo "Obstacle scenarios are sensing-only; Nav2 obstacle navigation is not yet readiness-approved."
  fi
  choose_seed || return 1
  choose_rviz
  start_profile "$profile" false
}

run_sensor_only() {
  local detached="$1"
  choose_sensor_scenario || return 1
  choose_seed || return 1
  choose_rviz
  start_profile lidar "$detached"
}

show_parking_status() {
  echo "AUTONOMOUS PARKING — NOT YET LIVE-VALIDATED"
  echo "The controller proposal has unit-test evidence only; no successful live Gazebo parking run exists."
}

follow_latest_log() {
  local log
  log=$(find "$ROOT/docs/evidence/windows-demos" -mindepth 1 -maxdepth 1 -type f -name '*.log' -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -n 1 | cut -d' ' -f2-)
  if [[ -z "$log" ]]; then echo "No launcher session log exists."; return 0; fi
  echo "Following $log (Ctrl+C returns to the menu)."
  tail -f "$log"
}

menu_loop() {
  local choice
  while true; do
    echo
    echo "=== AUTONOMOUS NAVIGATION CAR — GAZEBO DEMO MENU ==="
    echo "1) Phase 6 reference goal navigation (predefined map)"
    echo "2) RGB + LiDAR movement demo (S00 clear arena; live motion result)"
    echo "3) RGB + LiDAR sensor viewing (S01/S02/S03/S04/S09; stationary)"
    echo "4) Parking capability status"
    echo "5) Open latest evidence folder"
    echo "6) Stop tracked project simulation cleanly"
    echo "7) View latest launcher status/log"
    echo "8) Exit"
    read -r -p "Select [8]: " choice
    case "$choice" in
      1)
        choose_scenario || continue
        choose_seed || continue
        choose_rviz
        start_profile phase6 true || true
        ;;
      2)
        SCENARIO=S00
        echo "S00 movement: 0.05 m/s straight, up to 0.15 m or 4 seconds, after fresh safety authorization."
        echo "If the current LiDAR/safety state is not valid, the car remains stopped. Obstacle avoidance and Nav2 are not enabled."
        choose_seed || continue
        choose_rviz
        start_profile lidar false || true
        ;;
      3) run_sensor_only false || true;;
      4) show_parking_status;;
      5) open_latest_evidence;;
      6) bash "$SESSION_SCRIPT" stop || true;;
      7) follow_latest_log;;
      ""|8) echo "Exiting the menu. Any simulation remains tracked and can be stopped by reopening this menu."; break;;
      *) echo "Select a number from 1 to 8.";;
    esac
  done
}

diagnostics() {
  echo "Read-only environment diagnostics; no processes will be closed or launched."
  show_resource_snapshot || return 1
  echo "Ubuntu-24.04 registered WSL base path: $(registered_distro_path)"
  echo "Protected ChatGPT/Codex process identities (inspection only):"
  protected_codex_inventory
  check_project_environment || return 1
  bash "$SESSION_SCRIPT" status
}

main() {
  local command="$1"
  case "$command" in
    phase6) run_individual phase6;;
    lidar) run_individual lidar;;
    menu) menu_loop;;
    stop) bash "$SESSION_SCRIPT" stop;;
    latest) open_latest_evidence;;
    parking) show_parking_status;;
    diagnostics) diagnostics;;
    *) echo "Usage: windows_demo_launcher.sh phase6|lidar|menu|stop|latest|parking|diagnostics" >&2; return 2;;
  esac
}
if (($# != 1)); then
  echo "Usage: windows_demo_launcher.sh phase6|lidar|menu|stop|latest|parking|diagnostics" >&2
  exit 2
fi
main "$1"
