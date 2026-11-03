# Autonomous Navigation Car — One-Click Tester Phase Report

**Date:** 2026-11-03  
**Status:** Desktop tester built and dry-run verified. A live Gazebo run remains blocked by the current Windows memory headroom and the existing Phase 8C readiness gate.

## Delivered

The standalone folder is:

C:\Users\ajayk\Desktop\AutonomousCar_OneClick_Tester

It contains exactly one launcher, RUN_AUTONOMOUS_CAR_TESTS.bat, plus local PowerShell, Bash and Python helpers, safe headless S00 settings, persistent runs folders, and a latest summary pointer. It does not require Codex, VS Code, or a browser.

The runner records Windows and WSL RAM, swap, disk space, CPU, GPU, WSL/WSLg versions, ROS/Gazebo package availability, project branch/commit, offline test logs, readiness output, and (when live tests are permitted) sensor, map, SLAM, motion, safety and navigation evidence. It preserves old run folders. Its WSL watchdog sends SIGINT only to the verified tester-owned process group; it does not shut down WSL or terminate unrelated apps.

## Final dry-run

- **Run ID:** 20261102T184143Z_S00_42f8c896
- **Run folder:** C:\Users\ajayk\Desktop\AutonomousCar_OneClick_Tester\runs\20261102T184143Z_S00_42f8c896
- **Overall:** BLOCKED for live simulation, as expected from the resource/readiness gates.
- **Offline tests:** 5 of 5 selected pytest files passed. Collector self-test passed.
- **Readiness:** BLOCKED. The checked-in validator reports missing production-turning evidence, three distinct moving-SLAM trials, guarded-command-chain evidence, and an S00 navigation trial.
- **Memory gate:** Windows had 986 MiB available at the gate; the required headless minimum is 2,048 MiB. WSL had 3,082 MiB available; the configured minimum is 1,536 MiB. The Windows minimum during this dry-run was 745 MiB.
- **GPU:** NVIDIA GeForce GTX 1650, 4096 MiB total, 3935 MiB free from nvidia-smi. WSLg and Direct3D versions are recorded in environment/wsl_status.txt.
- **Simulation:** environment/simulation_state.json says NOT_STARTED. No Gazebo, RViz, map-saver, or motion probe was launched.
- **Project:** branch feature/phase8c-lidar, commit 567ac430544fe7fb945f8fb9c7aac4f47e779804; tracked diff was clean. The existing 57 untracked evidence entries were preserved.
- **Evidence:** RUN_SUMMARY.txt, RUN_SUMMARY.json, ANALYZE_ME_FIRST.txt, environment snapshots, CSV resource logs, and per-test output are saved in the run folder.
- **Process monitor check:** a separate 15-second WSL Python process was captured at 9.88 MiB RSS by the installed resource monitor. The check record is resource_monitoring/process_sampling_validation.json; the probe exited naturally and was not a simulator process.

## Startup corrections recorded

Two earlier dry-run attempts stopped before any tests or simulation started. Their logs remain in runs:

- 20261102T183014Z_S00_e5972daf — path conversion failed because the resource-monitor script parameter shadowed the run-directory variable.
- 20261102T183351Z_S00_44b40906 — ROS setup sourcing ran under Bash nounset before ROS initialized its variables.

Both issues were corrected. The final run above completed, returned worker exit code 0, and updated latest.

## Remaining work

Free at least 2 GiB of Windows RAM before running the full BAT. The live sensor/SLAM/map/motion probes were not run in this dry-run. Nav2 stays blocked until the project’s evidence-backed readiness validator returns READY; the tester will not override that gate. Re-run RUN_AUTONOMOUS_CAR_TESTS.bat after memory is available, then review the new latest\RUN_SUMMARY.txt.

No ROS/Gazebo packages were installed or modified in this phase, and no project files were committed or pushed.

## LiDAR stationary behavior investigation and fix — 2026-11-03

### Root cause

The screenshot's active RGB + LiDAR run was scenario S02. Its saved `phase8c-demo-status.json` reports `SENSING_ONLY`, `requested=false`, `requested_command=null`, zero `/cmd_vel` output, and only 0.00012 m odometry drift. This is intentional for obstacle scenarios: the current Phase 8C visual demo issues a guarded forward request only in empty scenario S00. It does not yet implement autonomous obstacle avoidance or readiness-approved Nav2 goal movement.

The existing S00 evidence confirms that the guarded demonstration can move, but only a short distance: 0.05 m/s for at most 0.75 s produced a maximum guarded output of 0.05 m/s and 0.02875 m measured odometry displacement. Its ending safety state later failed closed when scan rate fell below the configured minimum (with localization also stale). This small movement can be easy to miss in a wide Gazebo view.

The standalone tester had a separate dispatch bug. Its latest full run had passing LiDAR sensor evidence and a valid, authorized forward safety state (0.18 m/s cap), but `command_guard_state.authorized` was false with `required_input_missing` because no command had been sent yet. The orchestrator required that guard flag before calling the motion probe, while the probe itself is what sends the first bounded request. As a result, all three moving-SLAM trials were recorded BLOCKED with `no velocity request issued`.

### Changes applied

- In the WSL project launcher `scripts/windows_demo_launcher.sh`, direct RGB + LiDAR launch and menu option 2 now use S00 for the guarded movement demonstration. Menu option 3 is explicitly for stationary S01/S02/S03/S04/S09 sensor viewing. The menu states that movement is only a few centimeters, is stopped by the safety gate, and does not provide obstacle avoidance or Nav2.
- In `tools/wsl_orchestrator.py`, motion dispatch now requires passing sensor checks plus a valid, explicitly authorized forward safety state with at least a 0.05 m/s cap and zero angular cap. It no longer requires the command guard to authorize a command before the motion probe issues the first command. The probe remains responsible for checking the guarded output.
- Updated `README_FIRST.txt` to distinguish the headless tester from the visible S00 movement launcher and stationary obstacle scenarios.

### Verification status

The saved prior S00 and S02 evidence was inspected; no new Gazebo session or test suite was started for this change. The existing tracked S02 session was left untouched. Runtime confirmation remains pending a fresh run after stopping that session. A full Nav2 obstacle course remains blocked by the project's readiness evidence and was not enabled or bypassed.
### Retest/resource note

A fresh read-only diagnostics snapshot reported Windows available RAM 606 MiB (91% load), WSL available RAM 2,915 MiB, and the tracked S02 launcher process alive (PID 7042). The GUI launcher requires at least 2,560 MiB Windows available RAM, so I did not launch another Gazebo/RViz session or interrupt the visible one. After the S02 session is stopped and Windows has at least 2,560 MiB available, run the updated S00 visual demo. A direct CIM query was denied by Windows; the project diagnostics supplied the resource figures above. No ROS packages or safety limits were changed.