@echo off
setlocal
title Autonomous Navigation Car - Gazebo Demo Menu
set "DISTRO=Ubuntu-24.04"
echo Select option 2 for the S00 LiDAR movement demo.
echo LiDAR progress, measured motion, and any blocker appear in this window.
echo Option 3 opens stationary sensor scenarios.
echo Checking registered WSL distributions...
wsl.exe --list --verbose
if errorlevel 1 (
  echo WSL distributions could not be read. No simulation was started.
  pause
  exit /b 1
)
rem Keep this fixed, space-free distro name unquoted: quoted names fail on this WSL build.
wsl.exe --distribution %DISTRO% --exec bash -lc "cd /home/ashish/AutonomousNavigationCar && exec bash scripts/windows_demo_launcher.sh menu"
set "DEMO_RC=%ERRORLEVEL%"
if not "%DEMO_RC%"=="0" echo Menu blocked or exited with code %DEMO_RC%.
pause
exit /b %DEMO_RC%