# GitHub migration release verification — 3 November 2026

- Repository: `Ashish200528/Autonomous-Navigation`
- Migration tag: `pre-native-ubuntu-20261103`
- Source checkpoint commit: `a4a73f797fb0f64238ccabcdab9625b981c7900d`
- Published at: `2026-11-03 10:58:56 UTC`
- Release: https://github.com/Ashish200528/Autonomous-Navigation/releases/tag/pre-native-ubuntu-20261103

The commit and annotated tag were read back from GitHub. The published release is not a draft, and all eight assets report `uploaded`. GitHub's API reports these exact asset sizes and SHA-256 digests:

| Asset | Bytes | GitHub SHA-256 |
|---|---:|---|
| `Autonomous-Navigation.bundle` | 28,083,171 | `ad918b9ffebf95339a834ac8a06a52c322592b36c9c899ea0393443f2a75c696` |
| `BACKUP-MANIFEST.json` | 4,012,921 | `39094bf65d238c2597891e5bcc0feba178a18a361e990f7a483f9799ea96928c` |
| `NATIVE_UBUNTU_HANDOFF.md` | 6,869 | `c980cabbed398ef3c3608ad2c685e728068df181678c73042229d266b120d8ba` |
| `project-worktree-20261103.tar.gz` | 140,270,048 | `6a411c2a6a50fcf70df98108989a9b8c199e94c45ec1e56344fe5d361c0227b9` |
| `publication-scan.json` | 100 | `de07d00edf9bb5352dcbe1257767b3d169b6c9b5e9c96b2069c631bef6f4a19e` |
| `ros-gazebo-user-data-20261103.tar.gz` | 339,072 | `5402e450f390c0b810303cb13c8a003f509aa1ca82c81516bebe9144cc93ba99` |
| `SHA256SUMS` | 761 | `a42b97932da49a2bc783a8a0caa2b53840df27aa2e78bcd84a20ac9123ce0f6e` |
| `windows-project-tools-20261103.tar.gz` | 422,322,945 | `5e223d55d0a9e7d0d057e731a46b1372111b1b9693e4b1269a853b4378d9e93f` |

Before upload, the project archive was verified against 12,134 file/directory/link entries, the Windows archive against 1,274 entries, and the ROS/Gazebo archive against 2,024 entries. Local hashes for the four large archives and Git bundle match GitHub's reported SHA-256 digests. The source checkpoint passed 80 focused offline checks; this does not replace pending real Gazebo movement confirmation or full navigation readiness.

An additional local copy, `SHA256SUMS`, `BACKUP-MANIFEST.json`, and the offline Git bundle are under `E:\AutonomousCar_Migration_Backup_20261103`. Keep this separate copy until the release archives have been downloaded and checked on the native Ubuntu installation.
