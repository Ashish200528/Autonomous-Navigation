# Resource optimization evidence

Collected 2026-11-02 on branch feature/phase8c-lidar.

- windows-baseline-20261102.log: Windows RAM, CPU, GPU, process and startup inventory at 22:49 IST. Interactive command lines omitted/redacted; performance-counter errors recorded.
- wsl-baseline-20261102.log: Ubuntu/WSL kernel, memory, swap, processes and filesystem readings.
- launcher-diagnostics-20261102.log: read-only diagnostics, including an earlier stale-function-name error that was fixed, successful diagnostics, and the fresh 22:54 IST prelaunch reading: Windows 1,665 MiB available, WSL 3,140 MiB available. Records glxinfo absent and nvidia-smi sees the GTX 1650 at 0%.
- verification-20261102.log: actual syntax-check, five policy-test and git diff check output.

No simulator benchmark was started because the latest host reading was below the 2,048 MiB headless Gazebo minimum. No apps were closed and no RAM recovery is attributed to this task.
