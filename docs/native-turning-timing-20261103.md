# Turning fault-stop timing diagnosis — 2026-11-03

Production turning and autonomous realistic S00/S01 remain NOT DONE. No runtime controller, timeout, readiness flag, motion cap or pass/fail threshold changed in this session.

The preceding sweep measured fault-to-confirmed-settled time, including the evaluator's mandatory 0.5 s stationary observation hold. This session separates three measurements: output first becomes zero, first subsequently verified slow pose-rate sample, and completion of stationary confirmation. Invalid/duplicate-time rate samples and rebounds reset the slow-interval candidate. Candidate time uses the truth-service response timestamp, a conservative sampling observation rather than the earlier time before requesting truth. These are sampled pose rates, not proof of continuous instantaneous velocity or a zero stopping angle.

Four fresh sequential headless Gazebo fault trials used the existing calibration cap and retained the original 0.75 s confirmed-settling acceptance rule. All four faults were injected; each completed with zero recorded non-ground contacts and the sole anc_safety_command_guard publisher. All four still FAIL that rule:

| Fault | Zero output s | First verified slow sample s | Observation hold s | Confirmed settled s |
|---|---:|---:|---:|---:|
| Command timeout | 0.328 | 0.455 | 0.539 | 0.994 |
| LiDAR loss | 0.347 | 0.474 | 0.540 | 1.014 |
| Localization loss | 0.189 | 0.220 | 0.540 | 0.760 |
| Unknown swept space | 0.131 | 0.258 | 0.540 | 0.798 |

The robot was observed slow before 0.75 s, but that does not convert these into production passes. The current criterion also requires the confirmation hold to finish before 0.75 s. A defensible production contract needs separate physical stopping-angle/time and verification-latency bounds, with sampling uncertainty and rebound coverage. This report does not redefine that contract, decrease the observation hold, shorten sensor validity timeouts to force a pass, or enable authority. Straight-line moving SLAM remains previously validated with its reported limits.

Added diagnostic fields and four regression tests covering the confirmation split, rebound, missing/duplicate-time rates and nonfinite rates. All 53 Phase 8C offline tests pass. Existing build remains applicable: only test/evaluation Python changed. Initial probe instrumentation was expanded during the sweep; every summary therefore also contains a consistently replayed diagnostic from the saved raw samples using the final helper. No prior pass/fail result was overwritten.

Evidence: docs/evidence/native-turn-timing-20261103/timing-summary.json, per-case summary.json, bounded launch logs, specs and world fixtures. Full result.json and original terminal logs remain ignored locally. Timing samples use monotonic wall time; no Gazebo truth was supplied to runtime localization. No package installation, driver change, unrelated-process shutdown or graphical demonstration was performed.

Reproduce after sourcing ROS and the workspace:

```bash
python3 scripts/run_turn_matrix.py docs/evidence/turn-timing-new-run \
  lidar_loss localization_loss command_timeout unknown_swept_region
python3 -m unittest discover -s tests -p 'phase8c*test.py'
```

The experiment exits nonzero because its acceptance criteria fail; it still preserves evidence and shuts down its project simulation. Follow the graphical commands in native-turning-matrix-20261103.md for existing Phase 6 reference navigation and realistic sensing-only views. Neither command demonstrates autonomous realistic LiDAR navigation.
