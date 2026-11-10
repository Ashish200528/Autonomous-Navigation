"""Exercise monitor failure paths on a private ROS graph; no simulator or drive."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

ROOT = Path('/home/ashish/AutonomousNavigationCar')
OUT = Path(sys.argv[1])
OUT.mkdir(parents=True, exist_ok=True)
BASE = [sys.executable, str(ROOT/'scripts/phase8c_gui_demo.py')]
results = []


def invoke(name, extra):
    output = OUT/(name+'.json')
    started = time.monotonic()
    proc = subprocess.run(BASE+['--scenario', 'S00', '--output', str(output),
        '--request-forward', '--startup-timeout-s', '1', '--monitor-duration-s', '3']+extra,
        text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=12)
    (OUT/(name+'.log')).write_text(proc.stdout)
    result = json.loads(output.read_text())
    assert proc.returncode != 0, (name, proc.stdout)
    assert result['safety_state']['valid'] is False
    assert result['guarded_forward_demo']['requested'] is False
    assert result['runtime_error'], result
    results.append({'name': name, 'passed': True, 'exit_code': proc.returncode,
                    'elapsed_s': round(time.monotonic()-started, 3),
                    'reason': result['runtime_error']})


invoke('missing_launch', ['--launch-pid', '2147483647'])
invoke('missing_sensors', [])
invoke('demo_deadline', ['--startup-timeout-s', '10', '--monitor-duration-s', '1'])

# Publish genuine ROS topic samples, then stop the samples while retaining the
# publisher process. The display must invalidate cached state and exit.
rclpy.init()
driver = rclpy.create_node('demo_stream_loss_smoke_driver')
safety_pub = driver.create_publisher(String, '/realistic/safety_state', 10)
scan_pub = driver.create_publisher(LaserScan, '/realistic/scan', qos_profile_sensor_data)
odom_pub = driver.create_publisher(Odometry, '/odometry/filtered', qos_profile_sensor_data)
requests = []
driver.create_subscription(Twist, '/cmd_vel_nav',
    lambda m: requests.append((m.linear.x, m.angular.z)), 10)
target = OUT/'stopped_streams.json'
with (OUT/'stopped_streams.log').open('w') as log:
    monitor = subprocess.Popen(BASE+['--scenario', 'S02', '--output', str(target),
        '--startup-timeout-s', '12', '--monitor-duration-s', '25'], stdout=log,
        stderr=subprocess.STDOUT)
    started = time.monotonic()
    connected = None
    while time.monotonic()-started < 18:
        safety_pub.publish(String(data=json.dumps({'valid': True, 'authorized': True,
            'reason': 'smoke_test_fresh', 'max_linear_speed_mps': 0.18,
            'max_angular_speed_radps': 0.0})))
        scan_pub.publish(LaserScan())
        odom_pub.publish(Odometry())
        rclpy.spin_once(driver, timeout_sec=0.04)
        if all(p.get_subscription_count() > 0 for p in (safety_pub, scan_pub, odom_pub)):
            connected = connected or time.monotonic()
        if connected is not None and time.monotonic()-connected >= 3:
            break
    assert connected is not None, 'ROS subscription discovery did not complete'
    while monitor.poll() is None and time.monotonic()-started < 25:
        rclpy.spin_once(driver, timeout_sec=0.05)
    if monitor.poll() is None:
        monitor.send_signal(2)
        monitor.wait(timeout=4)
        raise AssertionError('monitor did not stop after streams ended')
    for _ in range(5):
        rclpy.spin_once(driver, timeout_sec=0.03)
result = json.loads(target.read_text())
assert monitor.returncode != 0, result
assert result['sensor_counts']['/realistic/scan'] > 0, result
assert result['last_received_safety_state']['valid'] is True, result
assert result['safety_state']['valid'] is False, result
assert result['safety_state']['authorized'] is False, result
assert result['scan_rate_hz_observed'] is None, result
assert result['runtime_error'].startswith('sensor_stream_stopped:'), result
assert all(abs(x) < 1e-9 and abs(z) < 1e-9 for x, z in requests), requests
results.append({'name': 'stopped_streams', 'passed': True, 'exit_code': monitor.returncode,
    'reason': result['runtime_error'], 'received_scans': result['sensor_counts']['/realistic/scan'],
    'observed_requests': len(requests), 'nonzero_requests': 0})


def synthetic_motion(name, deny_during_motion=False):
    """Check ROS control sequencing with synthetic feedback, not Gazebo physics."""
    state = {'request': 0.0, 'request_wall': 0.0, 'first_motion': None, 'x': 0.0}
    observed = []

    def on_request(msg):
        state['request'] = float(msg.linear.x)
        state['request_wall'] = time.monotonic()
        observed.append((float(msg.linear.x), float(msg.angular.z)))

    sub = driver.create_subscription(Twist, '/cmd_vel_nav', on_request, 10)
    guard_pub = driver.create_publisher(Twist, '/cmd_vel', 10)
    output_path = OUT/(name+'.json')
    with (OUT/(name+'.log')).open('w') as log:
        monitor = subprocess.Popen(BASE+['--scenario', 'S00', '--output', str(output_path),
            '--request-forward', '--startup-timeout-s', '12', '--monitor-duration-s', '18'],
            stdout=log, stderr=subprocess.STDOUT)
        started = previous = time.monotonic()
        while monitor.poll() is None and time.monotonic()-started < 25:
            rclpy.spin_once(driver, timeout_sec=0.02)
            now = time.monotonic()
            allowed = not (deny_during_motion and state['first_motion'] is not None
                           and now-state['first_motion'] >= 0.8)
            speed = (max(0.0, min(0.05, state['request']))
                     if allowed and now-state['request_wall'] <= 0.3 else 0.0)
            if speed and state['first_motion'] is None:
                state['first_motion'] = now
            state['x'] += speed*(now-previous)
            previous = now
            safety_pub.publish(String(data=json.dumps({'valid': allowed, 'authorized': allowed,
                'reason': 'smoke_fresh' if allowed else 'smoke_forced_denial',
                'max_linear_speed_mps': 0.18 if allowed else 0.0,
                'max_angular_speed_radps': 0.0})))
            scan_pub.publish(LaserScan())
            odom = Odometry()
            odom.pose.pose.position.x = state['x']
            odom.pose.pose.orientation.w = 1.0
            odom.twist.twist.linear.x = speed
            odom_pub.publish(odom)
            command = Twist()
            command.linear.x = speed
            guard_pub.publish(command)
        if monitor.poll() is None:
            monitor.send_signal(2)
            monitor.wait(timeout=4)
            raise AssertionError(name+': monitor did not finish within its duration bound')
    data = json.loads(output_path.read_text())
    motion = data['guarded_forward_demo']
    assert observed and any(x > 0 for x, _ in observed), (name, data)
    assert all(0 <= x <= 0.05 and abs(z) < 1e-9 for x, z in observed), observed
    assert motion['stop_confirmed'] is True, data
    assert 0.02 <= motion['odometry_displacement_m'] < 0.20, data
    if deny_during_motion:
        assert monitor.returncode == 4, data
        assert motion['status'] == 'STOPPED_SAFETY_STATE', data
        assert motion['block_reason'].startswith('safety_gate_denied:'), data
        assert motion['odometry_displacement_m'] < 0.10, data
    else:
        assert monitor.returncode == 0, data
        assert motion['status'] == 'MOVEMENT_VERIFIED', data
        assert 0.14 <= motion['odometry_displacement_m'] <= 0.17, data
    results.append({'name': name, 'passed': True, 'feedback': 'SYNTHETIC_ROS_NOT_GAZEBO',
        'exit_code': monitor.returncode, 'status': motion['status'],
        'displacement_m': motion['odometry_displacement_m'], 'stop_confirmed': True,
        'maximum_request_mps': max(x for x, _ in observed)})
    driver.destroy_subscription(sub)
    driver.destroy_publisher(guard_pub)


synthetic_motion('synthetic_guard_forward')
synthetic_motion('synthetic_safety_stop', deny_during_motion=True)
driver.destroy_node()
rclpy.shutdown()
(OUT/'smoke_summary.json').write_text(json.dumps(results, indent=2)+'\n')
print(json.dumps(results, indent=2))
