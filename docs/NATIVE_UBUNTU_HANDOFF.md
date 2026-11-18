# Native Ubuntu handoff — 3 November 2026

Repository: https://github.com/Ashish200528/Autonomous-Navigation

Continue from **feature/phase8c-lidar**, not the older main branch. The immutable migration checkpoint is **pre-native-ubuntu-20261103**. The GitHub release with that tag contains the complete worktree archive (including ignored raw bags and historical build outputs), Windows tools, checksums, a file inventory, ROS/Gazebo user logs/configuration, and a Git bundle of all refs. Release upload verification is recorded separately in the local migration receipt after upload.

## What is preserved and what remains

- Source packages, launch files, worlds, robot/configuration files, maps, tests, reports, small evidence, original roadmap and project requests, and the latest Windows launchers are in Git.
- Large MCAP bags and other ignored worktree data are in the release archive, with file hashes. Historical failed attempts are retained as evidence, not counted as passes.
- Reference map-based Nav2 trials have earlier measured results. The LiDAR S00 demonstration now requests only guarded straight movement, bounded to 0.15 m / 4 seconds, and verifies stopping. Latest repair validation: 80 offline checks and six private ROS synthetic checks passed; actual Gazebo physics confirmation remains pending.
- Full RGB + LiDAR autonomous goal navigation remains **BLOCKED**. Production turning, moving SLAM, and the live guarded command chain still need measured verification. Do not flip readiness flags to bypass those requirements. Parking is not live validated.
- Windows BAT/PowerShell launchers and scripts/windows_demo_launcher.sh depend on Windows/WSL. scripts/run_lidar_demo.sh also currently probes Windows RAM through powershell.exe. Port that resource probe and launcher UI to native Linux before using that demo runner there. Do not expect BAT files or WSL paths to run on native Ubuntu.
- No full operating-system image, personal home directory, authentication tokens, shell history, or unrelated Arduino UNO Q application is included. Reinstall dependencies and log in to GitHub on the new system. A Git clone does not install ROS/Gazebo.

## Fresh installation target

Use **Ubuntu Desktop 24.04 LTS (64-bit)** to continue this project's ROS 2 Jazzy + Gazebo Harmonic stack without a distribution upgrade. Native NVIDIA graphics needs checking after Ubuntu installation; the WSL driver version is not a Linux driver installer.

Official installation guides:

- ROS 2 Jazzy: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- Supported ROS/Gazebo pairing: https://gazebosim.org/docs/harmonic/ros_installation/
- ROS dependency resolution: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html

After configuring the official ROS apt repository as instructed by that guide, install the project stack. These commands are future instructions; they were not run during the backup:

```bash
sudo apt update
sudo apt install --no-install-recommends git gh build-essential \
  python3-colcon-common-extensions python3-rosdep python3-pytest \
  python3-numpy python3-opencv python3-matplotlib python3-yaml \
  ros-jazzy-ros-base ros-jazzy-ros-gz ros-jazzy-rviz2 \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-robot-localization ros-jazzy-slam-toolbox \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-nav2-minimal-tb3-sim
```

The ros-jazzy-ros-gz package supplies the supported Harmonic vendor dependencies; avoid adding a conflicting Gazebo stack. Exact old package versions and manually installed packages are recorded in docs/migration-20261103/ for reference. Do not blindly install the entire old dpkg inventory: it includes WSL-specific system packages.

## Retrieve and build

```bash
git clone --branch feature/phase8c-lidar \
  https://github.com/Ashish200528/Autonomous-Navigation.git \
  "$HOME/AutonomousNavigationCar"
cd "$HOME/AutonomousNavigationCar"
git show --no-patch pre-native-ubuntu-20261103
source /opt/ros/jazzy/setup.bash
```

On a fresh machine, initialize rosdep once if its default sources file does not already exist, then resolve declared dependencies:

```bash
test -f /etc/ros/rosdep/sources.list.d/20-default.list || sudo rosdep init
rosdep update
rosdep install --from-paths simulation/ros2_ws/src --ignore-src --rosdistro jazzy -r -y
cd simulation/ros2_ws
colcon build --symlink-install --executor sequential
source install/setup.bash
cd ../..
python3 -m pytest -q tests/phase8c_demo_policy_test.py tests/windows_demo_resource_policy_test.py
```

Build from a fresh Git clone. Do not source the archived install/setup.bash or reuse archived build/install directories: their paths refer to the old /home/ashish environment. Use `$HOME/AutonomousNavigationCar` consistently if the new Ubuntu username differs. The historical evidence contains absolute Windows/WSL paths; keep it as history rather than treating those paths as new launch instructions.

Start subsequent verification with package discovery, graphics, memory and clock/sensor checks. A direct sensor-only launch after building is:

```bash
source /opt/ros/jazzy/setup.bash
source simulation/ros2_ws/install/setup.bash
ros2 launch anc_lidar realistic_rgb_lidar.launch.py \
  gui:=false rviz:=false enable_slam:=true enable_nav2:=false
```

That command starts sensing/SLAM, not motion. Close it with Ctrl+C before another run. Verify native graphics and available memory before enabling GUI/RViz. Resume development by porting the Windows memory probe, then perform the pending real S00 guarded movement test, production-turning assessment, moving SLAM assessment, and live command-chain validation in that order.

## Retrieve archived evidence

The release URL is:
https://github.com/Ashish200528/Autonomous-Navigation/releases/tag/pre-native-ubuntu-20261103

Download release assets into a separate directory and verify them:

```bash
mkdir -p "$HOME/AutonomousCar-Migration-Archive"
cd "$HOME/AutonomousCar-Migration-Archive"
gh release download pre-native-ubuntu-20261103 \
  --repo Ashish200528/Autonomous-Navigation
sha256sum -c SHA256SUMS
```

Extract the project-worktree archive into this separate archive directory when needed. It preserves raw evidence and old builds; keep the fresh source clone as the runnable workspace. The Git bundle can recreate the repository offline with `git clone Autonomous-Navigation.bundle AutonomousNavigationCar-offline` and then checking out the recorded feature branch. Windows application files and ROS/Gazebo settings are historical copies; do not copy old runtime PID state into the new home directory.

The backup does not erase or repartition anything. Confirm the GitHub release and its verified assets before deleting the WSL distribution. Keep another copy of the migration release on storage outside any partition you intend to format.
