AUTONOMOUS NAVIGATION CAR - ONE-CLICK TESTER

Start a run
1. Close Codex and other applications you do not need. Do not close Windows services.
2. Open this folder on your Desktop.
3. Double-click RUN_AUTONOMOUS_CAR_TESTS.bat.
4. Let the checks finish. The default is headless; the runner will not start RViz.
5. Read latest\RUN_SUMMARY.txt and the run folder it names.
6. Reopen Codex and ask it to analyze the latest run folder.

For a visible LiDAR movement demonstration, use Desktop\02_WATCH_RGB_LIDAR.bat or option 2 in 03_GAZEBO_DEMO_MENU.bat. It uses the clear S00 arena and requests only a short safety-gated straight move. Obstacle scenarios are stationary; Nav2 obstacle navigation remains disabled until readiness is independently verified. This one-click tester itself is headless.

No terminal commands are required. The batch file opens its own readable Windows terminal and pauses on the final summary. The runner does not depend on Codex, ChatGPT, VS Code, temporary agent files, or an Internet connection during execution.

What it uses
- The existing Ubuntu-24.04 WSL2 distribution and /home/ashish/AutonomousNavigationCar.
- ROS 2 Jazzy, Gazebo Sim, SLAM Toolbox, Nav2 and the already-built project workspace.
- PowerShell and Bash scripts in this folder, plus Python's standard library and the existing WSL pytest installation.
- The actual Windows Desktop Known Folder path. No fixed user profile path is embedded.

Safety
- Default mode is headless S00. No Nav2 goal is sent unless the existing evidence-backed Phase 8C readiness validator and live safety checks permit it.
- The Phase 6 predefined-map reference path is excluded.
- The runner never changes production-turning authorization, safety bounds, UNKNOWN-space rules, WSL memory settings, Windows services, or other user applications.
- Only a session created by this test folder may be stopped automatically. Existing project-owned launcher sessions are left running if their live identity matches; ambiguous or untracked sessions block a new run.
- Every simulation and probe is time-bounded. The Windows PowerShell monitor can ask the matching WSL test process group to stop with SIGINT. It never sends SIGTERM or SIGKILL and never shuts down the WSL VM.
- Insufficient memory produces BLOCKED and no Gazebo launch. TIMEOUT, FAIL, BLOCKED, SKIPPED and NOT TESTED are reported separately.
- Existing run folders are preserved. Each run gets a unique timestamped folder; latest\ contains copies of the newest small summary files.

Configuration
Safe defaults are in config\test_settings.json. Keep mode=headless and scenario=S00 for ordinary runs. The GUI option is optional; do not enable it when the headless memory gate is not met. Screenshot, video and bag capture are off to limit resource and disk use.

Evidence
Each run is saved under runs\<timestamp>_S00_<id>\:
- RUN_SUMMARY.txt / RUN_SUMMARY.json: overall result and measured outcomes.
- ANALYZE_ME_FIRST.txt: strongest evidence, blockers, and a ready-to-copy follow-up request.
- environment\: Windows/WSL resources, versions, branch and commit.
- tests\: separate JSON result and complete log for each executed test.
- ros_logs\ and gazebo_logs\: per-run launch/system logs.
- maps\: sensor-generated map only if the ROS map saver succeeds.
- trajectories\: CSV extracted from an actual bounded motion probe.
- resource_monitoring\: Windows RAM/CPU, WSL RAM/swap, and WSL ROS/Gazebo process samples.

Creating this folder does not run a simulation. The Desktop BAT performs the full bounded suite when resources allow. For a report-only validation run, open PowerShell in this folder and run:

powershell.exe -NoProfile -ExecutionPolicy Bypass -File ".\tools\run_tests.ps1" -DryRun

A DRY_RUN performs environment/readiness checks and offline tests, records current resources, and never launches Gazebo or sends motion commands.

For the implementation details and the latest verified blocker, see PHASE_REPORT.md.
