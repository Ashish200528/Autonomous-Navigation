"""Save the final checks and a user-readable report for the repaired launcher."""
import ast
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

root = Path('/home/ashish/AutonomousNavigationCar')
out = Path(sys.argv[1])
previous = Path(sys.argv[2])
stage = Path(__file__).parent
checks = json.loads((out/'smoke_summary.json').read_text())
assert len(checks) == 6 and all(c['passed'] for c in checks)
pytest = subprocess.run([sys.executable, '-m', 'pytest', '-q',
    'tests/phase8c_demo_policy_test.py', 'tests/windows_demo_resource_policy_test.py'],
    cwd=root, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30)
(out/'offline_checks.log').write_text(pytest.stdout)
assert pytest.returncode == 0, pytest.stdout
for name in ('phase8c_demo_policy.py', 'phase8c_gui_demo.py'):
    ast.parse((root/'scripts'/name).read_text(), filename=name)
for name in ('run_lidar_demo.sh', 'windows_demo_launcher.sh', 'windows_demo_session.sh'):
    subprocess.run(['bash', '-n', str(root/'scripts'/name)], check=True)
diff = subprocess.run(['git', 'diff', '--check'], cwd=root, text=True,
    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=10)
(out/'diff_check.log').write_text(diff.stdout or 'git diff --check: passed\n')
assert diff.returncode == 0, diff.stdout
menu = subprocess.run(['bash', 'scripts/windows_demo_launcher.sh', 'menu'], cwd=root,
    input='8\n', text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=10)
(out/'menu_exit_check.log').write_text(menu.stdout)
assert menu.returncode == 0 and 'S00 clear arena; live motion result' in menu.stdout
for name in ('monitor_sigint_summary.json', 'monitor_sigint.json', 'monitor_sigint.log'):
    shutil.copyfile(previous/name, out/name)
signal = out/'process_group_sigint_check.json'
assert signal.exists(), 'exact process-group SIGINT evidence is missing'
shutil.copyfile(stage/'ros_smoke_checks.py', out/'ros_smoke_checks.py')
files = [root/'scripts'/n for n in ('phase8c_demo_policy.py', 'phase8c_gui_demo.py',
                                  'run_lidar_demo.sh', 'windows_demo_launcher.sh')]
bat = Path('/mnt/c/Users/ajayk/Downloads/03_GAZEBO_DEMO_MENU.bat')
assert 'Select option 2 for the S00 LiDAR movement demo.' in bat.read_text()
assert '--distribution %DISTRO% --exec' in bat.read_text()
files.append(bat)
(out/'source_hashes.json').write_text(json.dumps({str(p): hashlib.sha256(p.read_bytes()).hexdigest()
    for p in files}, indent=2)+'\n')
report = f'''# LiDAR menu repair — 3 November 2026

The Downloads and Desktop BAT files invoke the same WSL project launcher. The repair is installed in that project and both BAT copies now identify the correct movement option.

## Cause

- The newest inspected obstacle run was S03, which explicitly issues no velocity request. Its result was SENSING_ONLY and its movement was only numerical drift.
- The old status monitor ran forever and kept displaying cached valid safety/scan values after ROS/Gazebo exited. The shell checked the ROS process only once during startup, and the configured duration did not limit status mode.
- The former S00 pulse was only 0.75 seconds at 0.05 m/s. Saved odometry showed 0.02875 m of movement, which was easy to miss.
- The menu launched LiDAR in the background, hiding startup failures and safety blockers from its visible console.

## Installed behavior

Option 2 always runs the empty S00 arena. It waits up to 30 seconds for fresh scan, odometry and explicit forward safety authorization stable for 0.5 seconds, then gives a three-second countdown. It requests only 0.05 m/s on /cmd_vel_nav, with zero angular request, until 0.15 m measured travel or four seconds. A 0.20 m hard distance bound, data freshness, speed caps and authorization are checked before every nonzero request. Any denial stops the request; there is no automatic retry.

The existing guard remains the production publisher of /cmd_vel. The result requires nonzero guard output, at least 0.02 m fresh measured odometry, and a confirmed zero output plus low linear/angular odometry velocity before reporting MOVEMENT_VERIFIED. The monitor sends zero in cleanup and records whether publication actually succeeded.

Live status and blockers appear in the BAT window. Missing sensors, expired startup, stopped streams, exited/zombie ROS launch, reused PIDs and total duration are handled with a recorded result and exit code. Stale safety values are displayed as invalid and stale scan rates are cleared. The demo no longer keeps an endless idle status process after ROS exits. Clean Ctrl+C stops only the verified tracked project session.

Option 3 provides stationary obstacle sensor scenes. Full LiDAR Nav2 obstacle avoidance remains blocked by existing readiness evidence; this change implements a visible guarded straight demonstration.

## Verification

- Final focused offline checks: {pytest.stdout.strip().splitlines()[-1]}.
- Final six private-graph ROS smoke checks: all passed. These used synthetic topic feedback and no Gazebo, renderer, world or robot bridge.
- Synthetic forward feedback: {next(c['displacement_m'] for c in checks if c['name']=='synthetic_guard_forward')} m, stop confirmed; maximum request 0.05 m/s and zero angular request.
- Synthetic mid-motion safety denial: stopped at {next(c['displacement_m'] for c in checks if c['name']=='synthetic_safety_stop')} m with stop confirmation and a BLOCKED result.
- Missing launch, missing sensors, total duration expiry and stopped sensor streams: all produced bounded failure exits and invalid displayed safety state.
- ROS monitor Ctrl+C: exit 130, STOPPED_BY_USER, no nonzero request.
- Exact synthetic process-group SIGINT check: passed, child trap ran, parent exited cleanly. No project simulation was signaled.
- Final Python/Bash syntax, menu exit and git diff whitespace checks: passed.

One earlier smoke harness ended synthetic publication after a fixed three seconds and missed late DDS subscription discovery on this memory-constrained host. The harness was corrected to wait for actual subscription discovery; the earlier logs were preserved and final checks above passed.

Actual Gazebo motion was not rerun in this repair. Windows had approximately 898 MiB available during diagnostics, below the preserved 2,560 MiB GUI gate. The user will run the updated launcher after freeing memory. The synthetic motion checks validate ROS control sequencing, not Gazebo physics or obstacle navigation.

## Run it

1. Close the old launcher window and free memory.
2. Double-click C:\\Users\\ajayk\\Downloads\\03_GAZEBO_DEMO_MENU.bat.
3. Choose 2, press Enter for seed 101, and choose N for optional RViz to reduce load.
4. Watch the countdown and measured result in that console. The S00 car should move a short straight distance and stop if the live safety checks pass. If blocked, the console reports the reason immediately.

Each real run writes docs/evidence/phase8c/lidar/demos/<run>/phase8c-demo-status.json and ros2-launch.log. The JSON contains the measured travel, stop confirmation and blocker or runtime error.

## Files and evidence

- /home/ashish/AutonomousNavigationCar/scripts/windows_demo_launcher.sh
- /home/ashish/AutonomousNavigationCar/scripts/run_lidar_demo.sh
- /home/ashish/AutonomousNavigationCar/scripts/phase8c_gui_demo.py
- /home/ashish/AutonomousNavigationCar/scripts/phase8c_demo_policy.py
- /home/ashish/AutonomousNavigationCar/tests/phase8c_demo_policy_test.py
- Windows launchers: C:\\Users\\ajayk\\Downloads\\03_GAZEBO_DEMO_MENU.bat and C:\\Users\\ajayk\\Desktop\\03_GAZEBO_DEMO_MENU.bat
- This repair's logs and source hashes: {out}

No packages, production controller limits, readiness approvals, system services or WSL settings were changed. No commit or push was performed.
'''
(out/'PHASE_REPORT.md').write_text(report)
(root/'docs/windows-gazebo-lidar-fix-20261103.md').write_text(report)
windows_report = Path('/mnt/c/Users/ajayk/Downloads/03_GAZEBO_DEMO_MENU_FIX_REPORT.md')
windows_report.write_text(report)
print(json.dumps({'checks': 'passed', 'offline_summary': pytest.stdout.strip().splitlines()[-1],
    'ros_checks': len(checks), 'report': str(windows_report), 'evidence': str(out)}, indent=2))
