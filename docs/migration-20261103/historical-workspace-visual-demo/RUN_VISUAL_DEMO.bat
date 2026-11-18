@echo off
setlocal
title Autonomous Navigation Car - Visual Demonstration
if not exist "%~dp0scripts\Run-VisualDemo.ps1" (
  echo ERROR: Run-VisualDemo.ps1 is missing from this package.
  echo Package folder: %~dp0
  pause
  exit /b 2
)
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -NoExit -File "%~dp0scripts\Run-VisualDemo.ps1"
exit /b %ERRORLEVEL%
