# Autonomous Navigation Car roadmap status

Updated: 2026-11-02

The supplied master roadmap remains reference material. This status file records the repository's implemented phase gates.

| Phase | Status | Scope |
|---|---|---|
| 0 | PASS | Repository and simulation planning baseline |
| 1 | PASS / PASS WITH LIMITS | ROS 2, Gazebo, RViz and reference goal setup |
| 2 | PASS WITH LIMITS | Custom differential-drive robot |
| 3 | PASS WITH LIMITS | Camera, IMU, odometry, TF and contact interfaces |
| 4 | PASS WITH LIMITS | Wheel/IMU EKF and known-start localization |
| 5 | PASS WITH LIMITS | Deterministic worlds and scenario harness |
| 6 | PASS WITH LIMITS | ORACLE_REFERENCE Nav2 baseline |
| 7 | PASS WITH LIMITS | RGB FREE/OCCUPIED/UNKNOWN perception |
| 8 | PASS WITH LIMITS: stop-only baseline | Local RGB traversability and safe-stop supervisor |
| Phase 8 motion gate | BLOCKED | FREE evidence is disconnected and too short; hard-stop distance is unmeasured |

Phase 8 leaves every unsupported local cell UNKNOWN and outputs zero velocity. It does not report goal navigation or movement as successful. See [docs/phase8-feasibility.md](phase8-feasibility.md) and [docs/evidence/phase8/phase8-summary.json](evidence/phase8/phase8-summary.json).

The Phase 6 ORACLE_REFERENCE launch remains independent. No parking, hardware integration, or later phase is included in this status.
