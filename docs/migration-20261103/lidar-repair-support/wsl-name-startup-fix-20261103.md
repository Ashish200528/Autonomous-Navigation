
## Follow-up: WSL_E_DISTRO_NOT_FOUND startup fix — 3 November 2026

Reproduced the screenshot error through cmd.exe using the actual Downloads BAT.
On this machine, `wsl.exe --distribution "Ubuntu-24.04" --exec /bin/true` returned -1 / WSL_E_DISTRO_NOT_FOUND, while the same command with the space-free name unquoted returned 0. Direct project access also returned 0 and printed /home/ashish/AutonomousNavigationCar.

Updated Downloads and Desktop 03_GAZEBO_DEMO_MENU.bat to pass the fixed distro name unquoted. Removed the redundant wsl --status output, which printed an irrelevant WSL1 feature warning even though the registered Ubuntu is WSL2. Kept the registered-distribution listing. Backups are saved beside each BAT with suffix .before-wsl-name-fix-20261103.bak.

Verified each actual BAT through cmd.exe with LF-terminated option 8 input:
- Downloads: displayed the full eight-option menu; printed "Exiting the menu"; exit code 0.
- Desktop: displayed the full eight-option menu; printed "Exiting the menu"; exit code 0.

An initial echo-pipe check reached the menu but supplied Windows CRLF input; Bash rejected the extra carriage return and exited on EOF. Repeating with LF input verified both menu exits successfully. No simulator was started. No WSL settings, distro installations, packages, or system features were changed.

Microsoft command reference: https://learn.microsoft.com/en-us/windows/wsl/basic-commands