#!/usr/bin/env python3
"""Compact an actual matrix and bind its measurements into a verified envelope."""
import hashlib
import json
from pathlib import Path
import sys
from anc_lidar.turn_envelope import CASES,load_envelope


def main():
    root=Path(sys.argv[1]).resolve()
    manifest=dict(schema='TURN_ENVELOPE_V2',angular_cap_radps=.10,cases={})
    for name in sorted(CASES):
        b=root/name
        result=json.loads((b/'result.json').read_text())
        for trial in result.get('trials',[]):
            truth=trial.get('truth_samples',[])
            if truth:
                trial['observed_rotation_sim_duration_s']=trial['first_zero_output_truth']['sim_stamp_s']-truth[0]['sim_stamp_s']
            for key in ('truth_samples','stop_truth_samples','output_samples','safety_trace','guard_trace'):
                trial.pop(key,None)
        (b/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
        data=(b/'summary.json').read_bytes()
        manifest['cases'][name]=dict(path=f'{name}/summary.json',sha256=hashlib.sha256(data).hexdigest())
    target=root/'envelope.json'
    if target.exists():raise SystemExit('Refusing to overwrite existing envelope')
    target.write_text(json.dumps(manifest,indent=2)+'\n')
    try:
        print(json.dumps(load_envelope(target,validation_only=True),indent=2))
    except Exception:
        # Keep the rejected manifest for diagnosis; loading it still fails closed.
        raise


if __name__=='__main__':main()
