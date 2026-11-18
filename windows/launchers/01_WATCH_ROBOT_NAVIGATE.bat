@echo off
setlocal
title Autonomous Navigation Car - Phase 6 Reference Demo
set "DISTRO=Ubuntu-24.04"
echo Checking WSL status and registered distributions...
wsl.exe --status
if errorlevel 1 (
  echo WSL status could not be read. No simulation was started.
  pause
  exit /b 1
)
wsl.exe --list --verbose
if errorlevel 1 (
  echo WSL distributions could not be read. No simulation was started.
  pause
  exit /b 1
)
wsl.exe --distribution %DISTRO% --exec bash -lc "cd /home/ashish/AutonomousNavigationCar && exec bash scripts/windows_demo_launcher.sh phase6"
set "DEMO_RC=%ERRORLEVEL%"
if not "%DEMO_RC%"=="0" echo Demo blocked or exited with code %DEMO_RC%. See the session log path printed above.
pause
exit /b %DEMO_RC%