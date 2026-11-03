AUTONOMOUS NAVIGATION CAR - VISUAL DEMONSTRATION
=================================================

Start
-----
Double-click RUN_VISUAL_DEMO.bat. Keep the terminal window open; it will show the
final success/failure summary and the video paths.

The package runs the demonstrations sequentially:

  A1  Phase 6 ORACLE_REFERENCE Nav2, S00 empty arena
  A2  Phase 6 ORACLE_REFERENCE Nav2, S01 box obstacle
  B1  REALISTIC_RGB_LIDAR perception, S00, with one bounded forward request
  B2  REALISTIC_RGB_LIDAR perception, S02, sensing only

The Phase 6 demonstrations use a predefined geometry-derived map. They do not
build a map from the RGB camera or LiDAR. The RGB + LiDAR demonstrations do not
request Nav2 goals or production turning. S00's optional 0.05 m/s straight
request is sent only through the existing /cmd_vel_nav input when the current
safety state is fresh and authorizes it; the existing guard remains the sole
/cmd_vel publisher. S02 is sensing only.

Resource gate
-------------
Phase 6 Gazebo-only runs require at least 2560 MiB free Windows RAM. The RGB +
LiDAR runs with RViz require at least 3328 MiB free Windows RAM. Every run also
requires at least 1536 MiB available in WSL and 3072 MiB free on the WSL project
filesystem, plus 4096 MiB free on the Windows drive holding this package for
FFmpeg and recordings. If a gate is not met, that demo is marked BLOCKED and is
not launched. The full set needs the higher 3328 MiB threshold for the LiDAR
runs. Free memory yourself, then double-click the BAT again.

The launcher checks the Ubuntu-24.04 distro, the project's expected branch and
installed ROS packages. It sends SIGINT only to a session identified by the
project's own PID record. An untracked Gazebo, RViz, or ROS launch process causes
a block rather than a process kill. Unrelated Windows applications are never
closed.

Recording
---------
The launcher records the Windows desktop with FFmpeg gdigrab at 720p/15 fps.
It maximizes Gazebo for Phase 6 and tiles Gazebo + RViz side by side for LiDAR
when their window titles are detected. It reserves a bottom strip for the live
status terminal when the expected title is detected, so RGB, scan, map, and
safety-gate lines can be seen together. MP4 filenames and title metadata identify
the mode and scenario. The package tests H.264 NVENC with a short synthetic
color clip; if NVENC is unavailable or the probe fails, it tests and uses
software H.264. The probe clip is kept under logs and is not presented as
simulation footage.

FFmpeg is fetched into this package's tools folder the first time a simulation
is eligible to run. The launcher checks the archive against the SHA-256 checksum
published with that release, extracts the local executable, and does not add
FFmpeg to PATH or install it system-wide. This requires internet access on the
first eligible run; the Windows archive is about 200 MB.
FFmpeg upstream: https://ffmpeg.org/
Windows build and checksums: https://github.com/BtbN/FFmpeg-Builds/releases
FFmpeg source: https://github.com/FFmpeg/FFmpeg

Output
------
videos\     MP4 recordings. A video is saved even if its navigation result is
            a legitimate failure, provided capture itself worked.
screenshots\One actual frame extracted from each saved recording.
logs\       Launcher transcript, WSL session logs, FFmpeg logs and encoder probe.
results\    Machine-readable per-demo results, run-specific subfolders, and the
            latest human-readable RUN_SUMMARY.txt.
tools\      Package-local FFmpeg files downloaded on first run.

Each invocation has a timestamped run folder and timestamped videos/screenshots,
so another double-click does not overwrite earlier demonstration footage.

The filenames and MP4 title metadata identify the mode and scenario. Phase 6
videos are explicitly labeled as predefined-map reference navigation. The
LiDAR videos are labeled as perception demonstrations and do not imply LiDAR
Nav2 autonomy or parking. The optional combined presentation video is not
generated; the four source recordings remain separate.

Troubleshooting
---------------
- Low-memory block: free at least the listed host/WSL memory and run again.
- Environment/branch block: the launcher does not change the project branch or
  install ROS packages. Use the existing project launcher diagnostics to find
  the reported issue.
- Active simulation block: the launcher leaves an untracked process alone.
  Stop a tracked project run through the existing demo menu, then retry.
- Missing video: inspect the matching logs\ffmpeg_*.log and results\RUN_SUMMARY.txt.
- A failed/blocked run is reported as such; script exit codes alone are not
  counted as successful navigation.

No live autonomous parking, obstacle avoidance, or LiDAR Nav2 goal capability
is claimed by this package.

Current preparation status
--------------------------
The launcher passed PowerShell parser validation, embedded Windows API
compilation, Bash helper syntax validation, wrapper argument validation, and a
read-only WSL preflight. The latest snapshot observed 350 MiB
free Windows RAM (the diagnostics sample shortly after was 314 MiB), 3059 MiB
free WSL RAM, 970495 MiB free on the WSL project filesystem, and 35567 MiB free
on the package drive. It blocked before download, simulation, or recording. No
demonstration video has been made yet. Free at least 3328 MiB Windows RAM to
attempt all four runs; at 2560-3327 MiB only the two Phase 6 demos can run.
