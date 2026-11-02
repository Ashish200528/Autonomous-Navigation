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
| 8B | COMPLETE WITH LIMITS | Camera candidate sweep and isolated simulated stopping-distance measurements |
| Phase 8 motion gate | BLOCKED | Best connected full-footprint corridor 0.480 m vs 0.656429 m required; candidate occupied recall is inadequate |

Phase 8 leaves every unsupported local cell UNKNOWN and outputs zero velocity. It does not report goal navigation or movement as successful. See [docs/phase8-feasibility.md](phase8-feasibility.md) and [docs/evidence/phase8/phase8-summary.json](evidence/phase8/phase8-summary.json).

Phase 8B measured low-friction simulator stopping distance, but the conservative gate remains BLOCKED: required connected corridor is 0.656429 m versus a best observed 0.480 m, with zero occupied recall in the longest-corridor candidate. The Phase 8 stop-only lock remains active. See [Phase 8B report](evidence/phase8b/phase8b-report.md) and [machine-readable gate](evidence/phase8b/motion-gate.json).

The Phase 6 ORACLE_REFERENCE launch remains independent. No parking, hardware integration, or later phase is included in this status.
