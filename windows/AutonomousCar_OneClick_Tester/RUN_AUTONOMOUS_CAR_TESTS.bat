@echo off
setlocal EnableExtensions
title Autonomous Navigation Car - Standalone Tests
cd /d "%~dp0"
echo ============================================================
echo  Autonomous Navigation Car - Standalone Test Runner
echo  This run is independent of Codex, ChatGPT and VS Code.
echo ============================================================
echo.
if not exist "%~dp0tools\run_tests.ps1" (
  echo BLOCKED: tools\run_tests.ps1 is missing.
  set "ANC_EXIT=2"
  goto :finish
)
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\run_tests.ps1" -LauncherPath "%~f0"
set "ANC_EXIT=%ERRORLEVEL%"
:finish
echo.
echo Runner exit code: %ANC_EXIT%
echo Results folder: "%~dp0runs"
echo Latest summary: "%~dp0latest\RUN_SUMMARY.txt"
echo.
echo Keep this window open to review the summary. Press any key to close it.
pause >nul
exit /b %ANC_EXIT%
