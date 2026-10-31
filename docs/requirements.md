# Requirements and acceptance baseline

| ID | Requirement | Acceptance evidence | State |
|---|---|---|---|
| R-01 | Maintain a new simulation repository separate from the existing UNO Q application. | Repository path and Git history under `/home/ashish/AutonomousNavigationCar`. | Phase 0 |
| R-02 | Simulate differential-drive motion using configurable geometry and plausible collision/inertia. | Later model and motion tests. | Planned |
| R-03 | Support RGB camera, camera info, wheel odometry and IMU interfaces. | Later ROS topic/frequency/TF checks. | Planned |
| R-04 | Keep reference and realistic sensor profiles visibly distinct. | Launch/config audit and logged profile field. | Required |
| R-05 | Prevent truth, perfect depth/raycast and hidden obstacle locations from entering realistic navigation/perception. | Subscription/config audit. | Required |
| R-06 | Represent uncertain observations as UNKNOWN; stop/slow on stale required inputs. | Safe-stop and stale-input tests. | Required |
| R-07 | Use a map-frame goal with known initial map transform in the first baseline. | Reproducible S00 and navigation run. | Baseline chosen |
| R-08 | Define parking by position, heading and dwell thresholds before experiments. | Record thresholds and compare evaluator pose. | Provisional: 0.20 m, 10°, 5 s |
| R-09 | Use fixed scenario IDs and documented seeds for replay. | Registry in `simulation/scenarios/scenario_registry.yaml`. | Phase 0 |
| R-10 | Retain scenario results, failures, limitations and evidence honestly. | CSV/log records and phase reports. | Required |
| R-11 | Keep the first environment path in WSL Linux home on E: storage. | WSL distro location and project path. | Verified/planned |

## Acceptance discipline

Suggested numeric targets remain proposed until a test is run. A PASS requires recorded command output or artifacts for the exact gate; a planned test is not a passed test. The initial parking tolerance is a provisional test target, not a claim about physical performance.
