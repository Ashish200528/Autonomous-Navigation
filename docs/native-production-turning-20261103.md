# Evidence-bound production turning — 2026-11-03

## Result

The turning blocker is resolved for the tested simulator envelope: all 13 categories passed a fresh live matrix under the evidence-bound production-controller validation profile, following a separate 13/13 calibration-contract matrix. These are new production-controller tests, not renamed calibration results.

Normal production-sequential loading now verifies successfully using docs/evidence/native-turn-production-matrix-20261103/envelope.json (SHA256 79739d9b422649246431b8847abd6668c7a89d2a08f9b081ce94a8957723ddbd). Authority remains opt-in and bounded: pure turns <=0.10 rad/s and forward movement under the existing linear safety gate. Combined translation/yaw and reverse are denied. No hardware or general friction/obstacle-envelope certification is claimed.

The runtime requires complete hash-bound measured evidence, fresh sensor/localization inputs, a fully known-free swept disk for rotation, the existing speed/clearance gates and sole authoritative command guard. Missing/modified/incomplete evidence fails closed. Defaults and the normal demonstration readiness flags remain disabled; the new S00 commissioning entrypoint independently verifies production-turn and three moving-SLAM results rather than requiring a previous Nav2 success.

## Full production-controller matrix

| Category | Result | Physical stop upper s | Total stopping-angle upper rad |
|---|---|---:|---:|
| Clockwise | PASS | .112 | .01546 |
| Anticlockwise | PASS | .134 | .01352 |
| Multi-speed | PASS | .149 | .01301 |
| Repeated turns | PASS | .160 | .01928 |
| Extended rotation | PASS | .139 | .01559 |
| Near wall | PASS | .123 | .01069 |
| Near box | PASS | .108 | .00993 |
| Narrow passage | PASS | .120 | .00978 |
| Low friction | PASS WITH LIMITS | .093 | .00928 |
| LiDAR loss | PASS | .458 | .04453 |
| Localization loss | PASS | .263 | .02840 |
| Command timeout | PASS | .383 | .03792 |
| Unknown swept region | PASS | .164 | .01951 |

All completed trials: zero non-ground contacts; exactly one /cmd_vel publisher, anc_safety_command_guard; actual runtime production validation mode and measured envelope recorded. Maximum center drift .02872 m (eight-second rotation), minimum static circumdisk clearance .48642 m. Low-friction scope is the tested ground mu/mu2=.5 only. No simultaneous translation/rotation, moving obstacle or hardware trial was conducted.

## Fault timing decomposition

| Fault | Detection at source s | Guard decision suppression s | Observed zero output s | Zero-to-slow physical upper s | Post-zero angle upper rad | Confirmation s |
|---|---:|---:|---:|---:|---:|---:|
| lidar_loss | 0.3083 | 0.0435 | 0.3522 | 0.1057 | 0.01148 | 0.9381 |
| localization_loss | 0.1989 | 0.0128 | 0.2119 | 0.0512 | 0.00857 | 0.7731 |
| command_timeout | 0.3150 | 0.0000 | 0.3152 | 0.0683 | 0.00817 | 0.8966 |
| unknown_swept_region | 0.1078 | 0.0077 | 0.1161 | 0.0478 | 0.00794 | 0.6823 |

The mandatory conservative stationary observation lasted at least .508 s in all tests. Physical time bounds include measured source age and sample uncertainty; cumulative stopping angles include sampling margins and rebound travel. The source monotonic telemetry separates controller decisions from ROS callback delivery. Output delivery after guard decision was approximately .00018–.00065 s in these four fault measurements. Exact continuous-time deceleration is not inferred from samples; the table reports conservative pose-rate observation bounds. V2's explicit physical and confirmation rules are documented in phase8c-turn-stop-contract.md; historical V1 failures remain unchanged.

## Reproduction

After sourcing ROS and the workspace, calibration reproduction uses a NEW output directory:

```bash
python3 scripts/run_turn_matrix.py docs/evidence/new-calibration-run --stream-truth
python3 scripts/make_turn_envelope.py docs/evidence/new-calibration-run
python3 scripts/run_turn_matrix.py docs/evidence/new-production-run \
  --production-evidence docs/evidence/new-calibration-run/envelope.json --stream-truth
python3 scripts/make_turn_envelope.py docs/evidence/new-production-run
```

An invalid or failed matrix cannot authorize the next stage. The test profile blocks all translation. The accepted production matrix validates the same evidence-bound runtime yaw path used by normal sequential mode; normal mode additionally retains the independently tested forward gate. The generic guard rejects combined and reverse commands rather than projecting them into unvalidated motion.

## Nav2 commissioning status

scripts/commission_lidar_s00.py implements a separate first-goal procedure: production manifest verification, three distinct passing moving-SLAM trials, live single-guard audit, online SLAM/Nav2 launch, a short .75 m S00 destination, a 1.5 m evaluator travel abort, memory abort, collision abort and post-goal stop/localization checks. There is no reference occupancy map or evaluator truth input to navigation. It retains normal demo readiness rather than setting cached booleans to bypass it. The .75 m destination is intentionally a bounded first commissioning goal, not the longer Phase 6 course.

At this checkpoint the commissioning code is OFFLINE TESTED only; no realistic Nav2 goal result is claimed yet. 77 offline tests passed. The next live commissioning result will be recorded separately. S01, parking and exploration remain NOT VALIDATED. Existing Phase 6 source/configuration remains unchanged.

Evidence: docs/evidence/native-turn-production-matrix-20261103/matrix-summary.json, per-case summaries, source decision timing, resource samples and bounded launch logs. Full per-message records remain ignored locally. No ROS/Gazebo/NVIDIA installation or unrelated system configuration was changed.
