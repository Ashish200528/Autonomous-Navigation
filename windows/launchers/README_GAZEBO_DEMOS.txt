AUTONOMOUS NAVIGATION CAR - GAZEBO WINDOWS DEMOS

Start with 03_GAZEBO_DEMO_MENU.bat. It opens the scenario, seed, and RViz menu and can start or cleanly stop a tracked simulation. The two numbered BAT files launch a selected mode directly.

MODES
1. 01_WATCH_ROBOT_NAVIGATE.bat runs the existing Phase 6 Nav2 ORACLE_REFERENCE demonstration. It uses a geometry-derived predefined map for localization and planning. It is labeled REFERENCE NAVIGATION - PREDEFINED MAP; it does not demonstrate RGB-only or online LiDAR goal navigation. S00 is the default; S01 has a box obstacle. S02, S03, S04, and S09 are also selectable. S09 is the expected no-route case.
2. 02_WATCH_RGB_LIDAR.bat shows the existing REALISTIC_RGB_LIDAR sensors, local map, safety status, and RGB camera output when RViz is enabled. In S00 only, it may request a short 0.05 m/s straight movement for at most 0.75 s through /cmd_vel_nav after a fresh safety state authorizes it. The existing command guard remains the sole /cmd_vel owner. Other scenarios are sensing-only. Full LiDAR goal navigation remains blocked unless the project's readiness gate and evidence independently pass; the launcher never changes readiness flags.
3. AUTONOMOUS PARKING is NOT YET LIVE-VALIDATED. Goal stopping is not precision parking.

SCENARIOS, SEEDS, AND RVIZ
Choose S00, S01, S02, S03, S04, or S09. Enter an integer seed or type random; blank uses 101. RViz is optional and can be skipped to reduce memory use. Gazebo graphical mode is required for these watch demos.

STARTUP AND SAFE STOP
The launcher reports Windows and WSL RAM, WSL disk, major process RAM and CPU, the WSL2 distro, ROS/Gazebo packages, and WSLg availability. It asks before attempting any approved ChatGPT/Codex close. Press C during the 10-second opportunity to cancel. Only a verified OpenAI.Codex Windows app package process with a visible main window receives a normal close request. Unsaved-work dialogs are left alone; no force-kill is used. Untracked simulator processes are protected and block a competing launch.

Use menu option 5 to stop a helper-tracked simulation. In a direct launch window, press Ctrl+C to clean up. Avoid closing the terminal window with its X while a simulation is active. Menu Exit leaves an active simulation running so you can reopen the menu and stop it; the menu reports the session log path.

MEMORY
The launcher blocks Gazebo below 2560 MiB available Windows RAM or 1536 MiB WSL RAM. Gazebo plus RViz requires 3328 MiB Windows RAM; if below that, the menu offers Gazebo-only. These are conservative laptop preflight thresholds, not changes to Windows or WSL settings. If blocked, close other work or wait for RAM to recover, then launch again. The current measured host had less than 1 GiB available, so a GUI start is not safe yet.

RESULTS
Phase 6 run evidence is saved under /home/ashish/AutonomousNavigationCar/docs/evidence/phase6/runs. RGB/LiDAR demo evidence and status JSON are saved under docs/evidence/phase8c/lidar/demos. Launcher console logs are saved under docs/evidence/windows-demos. The menu can open the latest evidence folder or follow the latest launcher log. The Linux project stays in Ubuntu's E: backed ext4 filesystem at /home/ashish/AutonomousNavigationCar; no ROS/Gazebo package install or Windows system setting change is performed by these launchers.