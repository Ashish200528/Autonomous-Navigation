"""Archive and hash only authorized simulation project data, outside the WSL VHD."""
from pathlib import Path
import gzip
import hashlib
import json
import os
import re
import stat
import subprocess
import tarfile
import time

ROOT = Path('/home/ashish/AutonomousNavigationCar')
BACKUP = Path('/mnt/e/AutonomousCar_Migration_Backup_20261103')
WIN = Path('/mnt/c/Users/ajayk')
BACKUP.mkdir(exist_ok=True)
TAG = 'pre-native-ubuntu-20261103'
sha = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
assert subprocess.check_output(['git', 'rev-parse', TAG+'^{}'], cwd=ROOT, text=True).strip() == sha

def digest(p):
    h = hashlib.sha256()
    with p.open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()

def archive(name, sources):
    out = BACKUP/name
    assert not out.exists(), f'Refusing to overwrite {out}'
    entries = []
    with tarfile.open(out, 'w:gz', compresslevel=1, dereference=False) as tf:
        for source, prefix, exclude_git in sources:
            assert source.exists(), source
            paths = [source] + sorted(source.rglob('*')) if source.is_dir() else [source]
            for p in paths:
                rel = p.relative_to(source) if source.is_dir() else Path()
                if exclude_git and '.git' in rel.parts:
                    continue
                # rglob does not traverse symlink directories. Never follow home links into other data.
                arc = str(Path(prefix)/rel)
                info = p.lstat()
                if stat.S_ISREG(info.st_mode):
                    if p.name in ('.env', 'hosts.yml', '.git-credentials', 'id_rsa', 'id_ed25519') or p.suffix in ('.pem', '.p12', '.pfx'):
                        raise RuntimeError(f'Credential filename requires review: {arc}')
                    if info.st_size <= 40*1024*1024 and p.suffix not in ('.mcap', '.mp4', '.avi', '.png', '.jpg', '.zip', '.exe', '.pyc'):
                        data = p.read_bytes()
                        if b'\0' not in data[:4096]:
                            for pattern in (rb'gh[pousr]_[A-Za-z0-9]{30,}', rb'github_pat_[A-Za-z0-9_]{30,}', rb'-----BEGIN (?:RSA |EC |OPENSSH |DSA )?PRIVATE KEY-----', rb'AKIA[A-Z0-9]{16}'):
                                if re.search(pattern, data):
                                    raise RuntimeError(f'Credential-like content requires review: {arc}')
                    row = {'path': arc, 'kind': 'file', 'size': info.st_size, 'sha256': digest(p)}
                elif stat.S_ISLNK(info.st_mode):
                    row = {'path': arc, 'kind': 'symlink', 'target': os.readlink(p)}
                elif stat.S_ISDIR(info.st_mode):
                    row = {'path': arc, 'kind': 'directory'}
                else:
                    raise RuntimeError(f'Unexpected socket/device in project archive: {arc}')
                tf.add(p, arcname=arc, recursive=False)
                if p.lstat().st_mtime_ns != info.st_mtime_ns:
                    raise RuntimeError(f'File changed during backup: {arc}')
                entries.append(row)
                if len(entries) % 1000 == 0:
                    print(f'{name}: {len(entries)} entries archived', flush=True)
    expected = {e['path']: e for e in entries}
    checked = 0
    with tarfile.open(out, 'r:gz') as tf:
        for member in tf:
            e = expected.pop(member.name)
            if member.isfile() or member.islnk():
                h = hashlib.sha256()
                stream = tf.extractfile(member)
                for block in iter(lambda: stream.read(1024*1024), b''):
                    h.update(block)
                assert h.hexdigest() == e['sha256'], member.name
                assert member.size == e['size'], member.name
            elif member.issym():
                assert member.linkname == e['target'], member.name
            checked += 1
    assert not expected
    print(f'{name}: all {checked} entries verified; {out.stat().st_size} bytes', flush=True)
    return {'asset': name, 'bytes': out.stat().st_size, 'sha256': digest(out), 'verified_entries': checked, 'entries': entries}

assets = []
assets.append(archive('project-worktree-20261103.tar.gz', [(ROOT, 'AutonomousNavigationCar', True)]))
assets.append(archive('windows-project-tools-20261103.tar.gz', [
    (WIN/'Desktop/AutonomousCar_Visual_Demo', 'AutonomousCar_Visual_Demo', False),
    (WIN/'Desktop/AutonomousCar_OneClick_Tester', 'AutonomousCar_OneClick_Tester', False),
    (WIN/'Desktop/03_GAZEBO_DEMO_MENU.bat', 'launchers/03_GAZEBO_DEMO_MENU.bat', False),
    (WIN/'Desktop/01_WATCH_ROBOT_NAVIGATE.bat', 'launchers/01_WATCH_ROBOT_NAVIGATE.bat', False),
    (WIN/'Desktop/02_WATCH_RGB_LIDAR.bat', 'launchers/02_WATCH_RGB_LIDAR.bat', False),
    (WIN/'Desktop/README_GAZEBO_DEMOS.txt', 'launchers/README_GAZEBO_DEMOS.txt', False),
]))
assets.append(archive('ros-gazebo-user-data-20261103.tar.gz', [
    (Path('/home/ashish/.ros'), '.ros', False),
    (Path('/home/ashish/.gz'), '.gz', False),
    (Path('/home/ashish/.rviz2'), '.rviz2', False),
    (Path('/home/ashish/.sdformat'), '.sdformat', False),
]))
bundle = BACKUP/'Autonomous-Navigation.bundle'
assert not bundle.exists()
subprocess.run(['git', 'bundle', 'create', str(bundle), '--all'], cwd=ROOT, check=True)
verification = subprocess.run(['git', 'bundle', 'verify', str(bundle)], cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=True)
(BACKUP/'git-bundle-verification.txt').write_text(verification.stdout)
assets.append({'asset': bundle.name, 'bytes': bundle.stat().st_size, 'sha256': digest(bundle)})
manifest = {'repository': 'Ashish200528/Autonomous-Navigation', 'branch': 'feature/phase8c-lidar',
            'tag': TAG, 'commit': sha, 'archives': assets,
            'scope': 'Simulation project worktree, Windows simulation applications, ROS/Gazebo user data; no personal credentials or full operating system image.'}
(BACKUP/'BACKUP-MANIFEST.json').write_text(json.dumps(manifest, indent=2)+'\n')
for source, name in [(ROOT/'docs/NATIVE_UBUNTU_HANDOFF.md', 'NATIVE_UBUNTU_HANDOFF.md'),
                     (ROOT/'docs/migration-20261103/publication-scan.json', 'publication-scan.json')]:
    (BACKUP/name).write_bytes(source.read_bytes())
checksums = []
for p in sorted(BACKUP.iterdir()):
    if p.is_file() and p.name != 'SHA256SUMS':
        assert p.stat().st_size < 2*1024**3, p
        checksums.append(f'{digest(p)}  {p.name}')
(BACKUP/'SHA256SUMS').write_text('\n'.join(checksums)+'\n')
print(json.dumps({'commit': sha, 'backup': str(BACKUP), 'assets': [dict((k,v) for k,v in a.items() if k!='entries') for a in assets]}), flush=True)
