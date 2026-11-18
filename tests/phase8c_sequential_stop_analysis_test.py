"""Synthetic fixtures test the evaluator; they are never live simulation evidence."""
import csv,gzip,json,math,sys,tempfile,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from analyze_sequential_stops import analyze

class SequentialStopAnalysisTests(unittest.TestCase):
    def run_fixture(self,rebound=False,clock=True,max_w=.05):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);rows=[]
            for i in range(30,126):
                t=i*.02;a=.05*min(t,1.)
                if rebound and t>=2.:a+=.015
                rows.append(dict(kind='truth_evaluator_only',stamp_s=t,receipt_wall_s=100+t,
                                 clock_s=t if clock else None,clock_receipt_wall_s=100+t,
                                 position=[0,0,0],orientation=[0,0,math.sin(a/2),math.cos(a/2)]))
            for t,w in [(.7,.05),(1.,0),(2.5,0)]:
                rows.append(dict(kind='guard_decision',data=dict(decision_monotonic_s=100+t,
                    output=dict(linear_x_mps=0,angular_z_radps=w,is_zero=w==0))))
            with gzip.open(root/'slam-source.jsonl.gz','wt') as f:
                for r in rows:f.write(json.dumps(r)+'\n')
            with (root/'guarded-commands.csv').open('w') as f:
                writer=csv.writer(f);writer.writerow(['topic','wall_s','linear_x_mps','angular_z_radps'])
                writer.writerow(['/cmd_vel_nav_raw',100.7,0,.05]);writer.writerow(['/cmd_vel_nav_raw',100.8,0,0])
            (root/'navigation-result.json').write_text(json.dumps(dict(status='PASS',command_summary={
                '/cmd_vel':dict(max_v=0,max_w=max_w)})))
            return analyze(root)

    def test_independent_stop_and_upstream_confirmation_pass(self):
        report=self.run_fixture();self.assertEqual(report['status'],'PASS')
        self.assertAlmostEqual(report['stops'][0]['upstream_zero_observed_to_guard_zero_s'],.2)
        self.assertEqual(report['stops'][0]['upstream_observation']['status'],'PASS')

    def test_rebound_after_confirmation_is_rejected(self):
        report=self.run_fixture(rebound=True);self.assertEqual(report['status'],'FAIL')
        self.assertIn('post_confirmation_rebound',report['stops'][0]['failures'])

    def test_missing_clock_or_recovery_velocity_cannot_qualify(self):
        self.assertEqual(self.run_fixture(clock=False)['status'],'FAIL')
        self.assertEqual(self.run_fixture(max_w=.1)['status'],'FAIL')
