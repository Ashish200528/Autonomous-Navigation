# Phase 7 diagnostic findings

The authoritative Phase 7 result is dataset/accepted-phase7-final-verified/; earlier candidate logs here are development iterations and are not used for final metrics. The full release run is indexed by accepted-phase7-final-verified-console.log.

- A changing runtime floor reference could learn obstacle appearance as ground. The classifier now uses the fixed Phase 3 floor ROI calibration and requires calibrated seed support plus a contiguous floor component; otherwise it fails closed.
- RGB-identical obstacle/floor appearance made wider free-space projection unsafe. FREE was limited to the narrow 0.50-0.60 m camera-centered band; regions outside that justified band stay UNKNOWN.
- Invalid output originally risked losing the source image timestamp and confusing evaluator synchronization. Invalid masks now preserve the input image stamp; the no-image fallback is identified separately.
- Gazebo entity lookup can briefly fail during startup. The offline evaluator now performs bounded retries; this affects evaluation only, not runtime classification.
- The final P7C1 case still could not validate floor support in either view. It is retained as an all-UNKNOWN failure/uncertainty example. S03/S04 low-contrast occupied regions also remain largely UNKNOWN.

The final dataset has zero false-FREE pixels on known occupied regions, 26.67% occupied recall, and 80.98% UNKNOWN coverage. These limits justify PASS WITH LIMITS. Earlier iteration console logs are preserved locally for debugging but are not required for the accepted evidence bundle.
