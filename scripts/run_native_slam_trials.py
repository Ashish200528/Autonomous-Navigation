#!/usr/bin/env python3
"""Run three fresh S00 trials; source ROS and workspace setup before use.

The evaluator subscribes before Gazebo starts to capture SLAM's volatile initial
pose. Motion, timeout and localization acceptance remain in the existing probe.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=15)


def main():
    output = Path(sys.argv[1]).resolve()
    output.mkdir(parents=True, exist_ok=False)
    for index in range(3):
        env = dict(os.environ, ROS_DOMAIN_ID=str(50 + index),
                   OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
        samples = []
        with (output / f"trial{index}-probe.log").open("w") as plog, \
                (output / f"trial{index}-launch.log").open("w") as llog:
            probe = subprocess.Popen(
                [sys.executable, str(ROOT / "tests/phase8c_slam_timeout_probe.py"),
                 str(output / f"trial{index}.json")], env=env, cwd=ROOT,
                stdout=plog, stderr=subprocess.STDOUT, start_new_session=True)
            launch = None
            try:
                time.sleep(2)
                launch = subprocess.Popen([
                    "ros2", "launch", "anc_lidar", "realistic_rgb_lidar.launch.py",
                    f"world_file:={ROOT}/simulation/scenarios/worlds/s00_empty_open_arena.sdf",
                    "enable_slam:=true", "enable_nav2:=false",
                    "rotation_calibration_max_radps:=0.0"],
                    env=env, cwd=ROOT, stdout=llog, stderr=subprocess.STDOUT,
                    start_new_session=True)
                deadline = time.monotonic() + 65
                while probe.poll() is None and time.monotonic() < deadline:
                    memory = {line.split(":")[0]: int(line.split()[1])
                              for line in Path("/proc/meminfo").read_text().splitlines()
                              if line.startswith(("MemTotal:", "MemAvailable:", "SwapFree:"))}
                    rows = subprocess.check_output(
                        ["ps", "-eo", "pgid=,rss=,pcpu="], text=True)
                    selected = [row.split() for row in rows.splitlines()
                                if int(row.split()[0]) in (launch.pid, probe.pid)]
                    memory.update(wall_s=time.time(),
                                  project_rss_kib=sum(int(row[1]) for row in selected),
                                  project_lifetime_cpu_percent=sum(float(row[2]) for row in selected))
                    samples.append(memory)
                    time.sleep(0.5)
            finally:
                stop(probe)
                if launch is not None:
                    stop(launch)
                (output / f"trial{index}-resources.json").write_text(
                    json.dumps(samples, indent=2) + "\n")
        result = json.loads((output / f"trial{index}.json").read_text())
        print(index, result.get("status"), result.get("reason"), flush=True)


if __name__ == "__main__":
    main()
