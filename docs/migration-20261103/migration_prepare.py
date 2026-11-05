"""Prepare project-owned files and machine inventory for the authorized GitHub handoff."""
from pathlib import Path
import hashlib
import json
import re
import shutil
import subprocess

ROOT = Path('/home/ashish/AutonomousNavigationCar')
WIN = Path('/mnt/c/Users/ajayk')
STAGE = WIN / '.codex/.chatgpt-projects/g-p-6a9dbef62ef88191857a6ceddb4337ed'
OUT = ROOT / 'docs/migration-20261103'
OUT.mkdir(exist_ok=True)

def copy_tree(src, dst, skip_vendor=False):
    if not src.is_dir():
        raise FileNotFoundError(src)
    for p in sorted(src.rglob('*')):
        rel = p.relative_to(src)
        if '__pycache__' in rel.parts or '.git' in rel.parts:
            continue
        if skip_vendor and (rel.parts[:2] == ('tools', 'ffmpeg-latest') or p.suffix in ('.zip', '.exe', '.mp4')):
            continue
        target = dst / rel
        if p.is_symlink():
            target.parent.mkdir(parents=True, exist_ok=True)
            if not target.is_symlink():
                target.symlink_to(p.readlink())
        elif p.is_file():
            target.parent.mkdir(parents=True, exist_ok=True)
            if not target.is_file() or (target.stat().st_size, target.stat().st_mtime_ns) != (p.stat().st_size, p.stat().st_mtime_ns):
                shutil.copy2(p, target)

copy_tree(WIN/'Desktop/AutonomousCar_OneClick_Tester', ROOT/'windows/AutonomousCar_OneClick_Tester')
copy_tree(WIN/'Desktop/AutonomousCar_Visual_Demo', ROOT/'windows/AutonomousCar_Visual_Demo', True)
copy_tree(STAGE/'AutonomousCar_Visual_Demo', OUT/'historical-workspace-visual-demo', True)
copy_tree(STAGE/'lidar_repair', OUT/'lidar-repair-support')
launchers = ROOT/'windows/launchers'
launchers.mkdir(parents=True, exist_ok=True)
for name in ('01_WATCH_ROBOT_NAVIGATE.bat', '02_WATCH_RGB_LIDAR.bat', '03_GAZEBO_DEMO_MENU.bat', 'README_GAZEBO_DEMOS.txt'):
    shutil.copy2(WIN/'Desktop'/name, launchers/name)
shutil.copy2(WIN/'Downloads/03_GAZEBO_DEMO_MENU.bat', launchers/'03_GAZEBO_DEMO_MENU.downloads.bat')
shutil.copy2(WIN/'Downloads/03_GAZEBO_DEMO_MENU_FIX_REPORT.md', OUT/'03_GAZEBO_DEMO_MENU_FIX_REPORT.md')
req = OUT/'original-user-requirements'
req.mkdir(exist_ok=True)
shutil.copy2(WIN/'Downloads/Autonomous_Navigation_Car_Simulation_Master_Roadmap.docx', req/'Autonomous_Navigation_Car_Simulation_Master_Roadmap.docx')
request_ids = '''baa561b1-6e2d-4980-abbe-d3aaff4bd5f1 0d927160-8e8c-4cc0-8896-1d4b84d98eae 4ef6fa86-7257-4132-b340-26766ec87df3 75f67ae9-fc75-451e-a6dc-b3cca0bbcc05 3a84c3a5-879d-48d7-b933-24e604721b9a 9325ec96-f9f9-440f-9bb3-9a945c129356 136b5d9b-5d3c-46b1-a352-21fcdcf62027 df5dd716-21e9-4be7-b29a-bc07b5748927 26b6174b-717a-421b-8f33-ceffaf846264 673bdde6-48e5-4420-9949-61267f4fa880 fd5fb3b5-625a-4e63-81b5-bec6ee9ec297 8292d59f-c44f-4fe4-a780-3ebee9227d27 ebb312fc-c5d1-4478-8a85-10eec32f348f 97d84851-4f3e-4470-99cf-bcb137116549 a143662f-6342-4eb1-9966-d32c9718dd12 fc3b5918-58ce-4b70-8002-f82f388d0d24 1c86e1d1-2495-4e20-b0dc-c85c0601d494'''.split()
for rid in request_ids:
    shutil.copy2(WIN/'.codex/attachments'/rid/'Pasted text.txt', req/f'{rid}.txt')
for name in ('PHASE-0-ENVIRONMENT-REPORT.md', 'phase8c_invalid_scan_test.py', 'phase8c_localization_loss_test.py', 'phase3-rviz-view.jpg', 'phase8c_invalid_scan_test.py'):
    if (STAGE/name).is_file():
        shutil.copy2(STAGE/name, OUT/name)

def capture(name, args):
    result = subprocess.run(args, cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=60)
    (OUT/name).write_text(result.stdout)
    if result.returncode:
        raise RuntimeError(f'{name}: exit {result.returncode}')

capture('dpkg-packages.tsv', ['dpkg-query', '-W', '-f=${binary:Package}\t${Version}\n'])
capture('apt-manual-packages.txt', ['apt-mark', 'showmanual'])
capture('python-packages.txt', ['python3', '-c', 'import importlib.metadata as m; print("\\n".join(sorted(f"{d.metadata.get(chr(78)+chr(97)+chr(109)+chr(101), d.name)}=={d.version}" for d in m.distributions())))'])
capture('ubuntu-os-release.txt', ['cat', '/etc/os-release'])
capture('kernel.txt', ['uname', '-a'])
capture('colcon-packages.txt', ['colcon', 'list', '--base-paths', 'simulation/ros2_ws/src'])
capture('readiness-check.txt', ['python3', 'scripts/phase8c_readiness.py', 'config/phase8c_demo_readiness.json', str(ROOT), '--brief'])
shutil.copy2(STAGE/'NATIVE_UBUNTU_HANDOFF.md', ROOT/'docs/NATIVE_UBUNTU_HANDOFF.md')
readme = ROOT/'README.md'
text = readme.read_text()
notice = ('\n## Native Ubuntu migration checkpoint — 3 November 2026\n\n'
          'The latest working branch is `feature/phase8c-lidar`; use the `pre-native-ubuntu-20261103` tag for the migration snapshot. '
          'Start with [the native Ubuntu handoff](docs/NATIVE_UBUNTU_HANDOFF.md). '
          'The migration release preserves large ignored evidence and Windows tools as separate archives. '
          'Full realistic LiDAR goal navigation remains readiness-blocked; the recent monitor checks were synthetic ROS checks, and the repaired Gazebo movement demo awaits a real retest.\n')
if '## Native Ubuntu migration checkpoint' not in text:
    readme.write_text(text + notice)

# Scan the entire project, including raw archives, before publishing to this public repository.
patterns = [re.compile(rb'gh[pousr]_[A-Za-z0-9]{30,}'), re.compile(rb'github_pat_[A-Za-z0-9_]{30,}'),
            re.compile(rb'-----BEGIN (?:RSA |EC |OPENSSH |DSA )?PRIVATE KEY-----'),
            re.compile(rb'AKIA[A-Z0-9]{16}')]
hits = []
scanned = 0
for p in ROOT.rglob('*'):
    rel = p.relative_to(ROOT)
    if '.git' in rel.parts or p.is_symlink() or not p.is_file():
        continue
    if p.name in ('.env', 'hosts.yml', '.git-credentials', 'id_rsa', 'id_ed25519') or p.suffix in ('.pem', '.p12', '.pfx'):
        hits.append(str(rel))
        continue
    if p.stat().st_size > 40*1024*1024 or p.suffix in ('.mcap', '.mp4', '.avi', '.png', '.jpg', '.zip', '.exe', '.pyc'):
        continue
    data = p.read_bytes()
    if b'\0' in data[:4096]:
        continue
    scanned += 1
    if any(pattern.search(data) for pattern in patterns):
        hits.append(str(rel))
scan = {'scanned_text_files': scanned, 'suspect_paths': sorted(set(hits)), 'credential_directories_included': False}
(OUT/'publication-scan.json').write_text(json.dumps(scan, indent=2)+'\n')
print(json.dumps(scan), flush=True)
if hits:
    raise RuntimeError('Review credential-like content before publishing; values were not printed.')
print('Migration files prepared; no system packages or simulator processes changed.', flush=True)
