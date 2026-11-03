# Angular stopping contract continuation — 2026-11-03

## Completed milestone

Contract V2 is implemented and independently regression tested; see phase8c-turn-stop-contract.md for the explicit change from V1. Four fresh fault tests passed V2 under calibration authority. No historical V1 result was changed and no runtime watchdog timeout was relaxed.

| Fault | Fault to zero output s | Physical stop upper s | Total angle measured / upper rad | Post-zero angle upper rad | Stationary observation s | Confirmation s |
|---|---:|---:|---:|---:|---:|---:|
| Command timeout | .326 | .472 | .03305 / .04565 | .01590 | .534 | .994 |
| LiDAR loss | .339 | .400 | .03415 / .04683 | .01268 | .532 | .923 |
| Localization loss | .311 | .360 | .03158 / .04417 | .01479 | .532 | .882 |
| Unknown swept space | .089 | .129 | .01120 / .02378 | .01386 | .534 | .652 |

The source/query and sample-gap margins are included in upper bounds. Zero-output-to-physical-stop upper bounds were .146/.062/.049/.041 s respectively. This is conservative sampled angular-deceleration timing; reported simulator twist is not substituted for pose-rate measurement. All four injected faults completed with no recorded non-ground contacts and a sole guarded /cmd_vel publisher. Guard suppression observed from ROS callbacks was approximately .016–.040 s for sensor/map faults; source telemetry was added afterward to remove callback-order ambiguity. The first command-timeout observation had a -0.0004 s apparent callback-order offset; it is not a negative physical latency.

## Full matrix and subsequent debugging

The first full V2 service-based matrix passed 8/13 categories. Four categories failed evaluator truth capture (anticlockwise, multi-speed, repeated turns, near wall); near-box motion stopped on RGB timestamp safety revocation. These remain failures. A dedicated truth client reduced the next sweep to two failures (repeated-turn truth capture and narrow-passage gate/measurement failure); neither sweep is a complete production prerequisite.

An intermediate runner launch rejected an empty optional CLI value before simulation; that failed attempt is retained. Optional evidence arguments now use a nonempty sentinel when inactive. The native streaming smoke produced real named-model poses but failed its conservative hold check (0.486 s observed after source-age uncertainty). The collector now observes until a full conservative 0.50 s hold is available. A fresh full native-stream matrix is in progress; its results will be recorded separately.

## Status boundaries

- Physical/confirmation evaluator correction: IMPLEMENTED, OFFLINE TESTED, four fault trials LIVE PASS WITH LIMITS.
- Complete V2 calibration matrix: PENDING successful repeat; earlier complete sweeps failed as above.
- Evidence-bound production validation profile: IMPLEMENTED/OFFLINE TESTED, NOT LIVE VALIDATED yet.
- Production turning: DISABLED unless independently complete evidence is supplied; no valid production envelope exists at this milestone.
- Moving SLAM and Phase 6 reference navigation: earlier accepted evidence preserved; no reference map used in realistic trials.
- Realistic Nav2 S00/S01: NOT ATTEMPTED. Parking/exploration/hardware: unvalidated.

Builds for anc_lidar/anc_navigation_common and launch-argument inspection passed; 70 offline tests passed after the conservative observer update. The build predates later telemetry-only source edits; the next runtime milestone must rebuild/recheck before claiming production validation. GUI/NVIDIA/system configuration was not changed. New runner preflight requires >=1536 MiB available before startup; each trial is sequential and records sampled RAM/RSS. Full per-message captures remain local; compact results and bounded logs are versioned.

Evidence: docs/evidence/native-turn-contract-faults-final-20261103 (accepted four faults); native-turn-contract-faults-20261103, native-turn-contract-matrix-20261103, native-turn-contract-isolated-20261103 and native-turn-contract-isolated-final-20261103 retain earlier failures. The native stream evidence will be appended after completion. No new production or Nav2 success is claimed by this checkpoint.
