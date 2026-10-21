#!/usr/bin/env python3
"""Real TCPROS PVA/rate-thrust input and plant feedback on one private master."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import shutil
import socket
import subprocess
import tempfile
import threading
import xmlrpc.client

from frozen_session import robot, session
from world_client import Xsim, until


def scenario(x, rospy):
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from mavros_msgs.msg import AttitudeTarget, PositionTarget, State
    from mavros_msgs.srv import CommandBool, SetMode
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu

    latest, lock = {}, threading.Lock()
    subscriptions = []
    topics = {'pose': ('local_position/pose', PoseStamped),
              'velocity': ('local_position/velocity_local', TwistStamped),
              'odom': ('local_position/odom', Odometry), 'imu': ('imu/data', Imu),
              'target': ('setpoint_raw/target_attitude', AttitudeTarget), 'state': ('state', State)}
    for key, (suffix, kind) in topics.items():
        def record(message, key=key):
            with lock:
                latest[key] = message
        subscriptions.append(rospy.Subscriber('/uav/mavros/' + suffix, kind, record, queue_size=1))

    def last(key):
        with lock:
            return latest.get(key)

    def world_time():
        status, value = x.call('GET', '/v1/world')
        assert status == 200, (status, value)
        return int(value['time']['nanoseconds'])

    def step():
        result = x.operation('POST', '/v1/world/step', {'steps': 50})
        return int(result['result']['time']['nanoseconds'])

    def stamp(message):
        return message.header.stamp.to_nsec()

    def target_after(start, mask):
        target = last('target')
        return target if target and stamp(target) >= start and target.type_mask == mask else None

    x.start()
    until(lambda: all(s.get_num_connections() > 0 for s in subscriptions), 'feedback connections')
    # Frozen-session robots are enabled at construction. Observe an explicit
    # lifecycle edge after connecting; paused state messages are not latched.
    x.enable('uav', False)
    until(lambda: last('state') and not last('state').connected, 'disabled boundary')
    x.enable('uav')
    until(lambda: last('state') and last('state').connected, 'enabled state')
    assert not last('state').armed and last('state').mode == 'POSCTL'
    pva = rospy.Publisher('/uav/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    rates = rospy.Publisher('/uav/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
    until(lambda: pva.get_num_connections() and rates.get_num_connections(), 'both control inputs')
    rospy.wait_for_service('/uav/mavros/cmd/arming', timeout=5)
    arm = rospy.ServiceProxy('/uav/mavros/cmd/arming', CommandBool)
    mode = rospy.ServiceProxy('/uav/mavros/set_mode', SetMode)
    mode(0, 'OFFBOARD')
    assert last('state').mode == 'POSCTL', 'no valid input must not open OFFBOARD'
    assert arm(True).success
    until(lambda: last('state').armed, 'armed boundary')
    command = PositionTarget()
    command.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    command.type_mask = PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
    command.position.z = 1.0

    def pva_step():
        pva.publish(command)
        step()
        mode(0, 'OFFBOARD')
        return last('state').mode == 'OFFBOARD' and last('pose') and last('pose').pose.position.z > 0.5

    until(pva_step, 'PVA takeoff through World and original PX4', seconds=10)
    assert last('state').armed
    raw = AttitudeTarget()
    raw.type_mask = AttitudeTarget.IGNORE_ATTITUDE
    # The quaternion is deliberately zero: mask128 must not run an attitude correction.
    raw.body_rate.x, raw.body_rate.y, raw.body_rate.z = 0.1, -0.15, 0.2
    raw.thrust = 0.5
    raw_start = world_time()

    def raw_step():
        rates.publish(raw)
        step()
        return target_after(raw_start, AttitudeTarget.IGNORE_ATTITUDE)

    target = until(raw_step, 'rate/thrust ROS mailbox -> original PX4 target')
    expected = (raw.body_rate.x, raw.body_rate.y, raw.body_rate.z)

    def assert_rate_target(message):
        assert message.type_mask == 128
        assert (message.body_rate.x, message.body_rate.y, message.body_rate.z) == expected
        assert abs(message.thrust - 0.5) < 1e-7

    assert_rate_target(target)

    def rotating_feedback():
        raw_step()
        imu, pose = last('imu'), last('pose')
        return imu and pose and stamp(imu) >= raw_start and stamp(pose) >= raw_start and \
            imu.angular_velocity.x > 0.01 and imu.angular_velocity.y < -0.01 and imu.angular_velocity.z > 0.01

    until(rotating_feedback, 'plant angular motion appears in FLU IMU and pose')
    until(lambda: last('odom') and stamp(last('odom')) == stamp(last('imu')) == stamp(last('pose')),
          'matching integrated feedback sample')
    imu = last('imu')
    assert imu.header.frame_id == 'base_link'
    assert last('pose').header.frame_id == last('velocity').header.frame_id == 'map'
    assert last('odom').child_frame_id == 'base_link'
    assert (imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z) == (
        last('odom').twist.twist.angular.x, last('odom').twist.twist.angular.y,
        last('odom').twist.twist.angular.z)
    orientation = last('pose').pose.orientation
    assert orientation.x * orientation.x + orientation.y * orientation.y + orientation.z * orientation.z > 1e-10
    assert abs(sum(value * value for value in (orientation.x, orientation.y, orientation.z, orientation.w)) - 1) < 1e-10
    assert all(math.isfinite(value) for value in (
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z,
        last('pose').pose.position.z, last('velocity').twist.linear.z))
    assert math.sqrt(sum(value * value for value in (
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z))) > 0.1

    # Both a previous-generation-era stamp and >1s future stamp must be rejected
    # at the ROS boundary. The accepted raw target remains intact after a fresh step.
    for old_stamp in (1, world_time() + 2000000000):
        rejected = AttitudeTarget()
        rejected.header.stamp = rospy.Time(old_stamp // 1000000000, old_stamp % 1000000000)
        rejected.type_mask = AttitudeTarget.IGNORE_ATTITUDE
        rejected.body_rate.x = rejected.body_rate.y = rejected.body_rate.z = -2
        rejected.thrust = 0
        before = world_time()
        rates.publish(rejected)
        step()
        assert_rate_target(until(lambda: target_after(before, 128), 'stale/future input retains accepted target'))

    # A real airborne disarm refusal cannot be mistaken for a successful state change.
    assert not arm(False).success
    assert last('state').armed and last('state').mode == 'OFFBOARD'
    recover_start = world_time()

    # Publish once: a later predicate must not enqueue another PVA after the
    # post-recover target feedback that proves the stream changed to PVA.
    pva.publish(command)

    def recover_step():
        step()
        return target_after(recover_start + 1, 7)

    recovered = until(recover_step, 'PVA can replace the rate/thrust stream')
    assert stamp(recovered) > recover_start and recovered.type_mask == 7
    # No input after the observed PVA stage; retain the original 600-step loss gate.
    loss_start = world_time()
    loss_result = x.operation('POST', '/v1/world/step', {'steps': 600})
    loss_end = int(loss_result['result']['time']['nanoseconds'])
    assert loss_end - loss_start == 1200000000, '600 original 2ms steps must remain 1.2s'
    until(lambda: last('state').mode == 'AUTO.LOITER', 'existing OFFBOARD loss fallback')
    assert last('state').armed
    published = dict(rospy.get_published_topics())
    assert not any('hover_thrust' in name for name in published), 'XSim must not invent a truth estimator'
    return {'result': 'PASS', 'input': 'PVA -> rate/thrust(mask128) -> PVA',
            'raw_body_rate_flu_rad_s': expected, 'normalized_thrust': target.thrust,
            'imu_specific_force_body_m_s2': [imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z],
            'stale_and_future_rejected': True, 'armed_and_offboard_semantics': True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--xsim', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='xsim-rate-thrust-') as temporary:
        root = Path(temporary)
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 0))
            port = probe.getsockname()[1]
        os.environ.update(ROS_MASTER_URI='http://127.0.0.1:' + str(port), ROS_IP='127.0.0.1',
                          ROS_HOSTNAME='127.0.0.1', ROS_HOME=str(root / 'ros-home'), ROS_LOG_DIR=str(root / 'ros-log'))
        import rospy
        x = None
        with (output / 'roscore.log').open('w') as log:
            master = subprocess.Popen(['roscore', '-p', str(port)], stdout=log,
                                      stderr=subprocess.STDOUT, start_new_session=True)
            try:
                def ready():
                    try:
                        return xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/rate_thrust_test')[0] == 1
                    except OSError:
                        return False
                until(ready, 'private ROS master')
                rospy.init_node('xsim_rate_thrust_test', disable_signals=True)
                config = session([robot('uav', 'uav', 'px4_multirotor', [0, 0, 0])])
                x = Xsim(args.xsim, root / 'world', config, ros=True, paused=True)
                result = scenario(x, rospy)
                x.stop()
                (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
            finally:
                if x is not None:
                    x.kill()
                    if x.log_path.exists():
                        shutil.copy2(x.log_path, output / 'xsim.log')
                rospy.signal_shutdown('private rate/thrust fixture finished')
                if master.poll() is None:
                    os.killpg(master.pid, signal.SIGTERM)
                    try:
                        master.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(master.pid, signal.SIGKILL)
                        master.wait()
    print('PASS: real ROS PVA/rate-thrust/feedback; original PX4 control, no truth HTE')


if __name__ == '__main__':
    main()
