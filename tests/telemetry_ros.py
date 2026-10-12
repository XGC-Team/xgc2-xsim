#!/usr/bin/env python3
"""Verify ROS telemetry on a private master and a temporary xsim world."""
import argparse
import json
import os
import signal
import socket
import subprocess
import threading
import time
import xmlrpc.client
from http import client as http_client
from pathlib import Path

from native_http import NativeProvider, UnixHTTP


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--xsim', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    out.chmod(0o700)
    for name in ("ros-home", "ros-log"):
        (out / name).mkdir(mode=0o700, exist_ok=True)
    with socket.socket() as available:
        available.bind(('127.0.0.1', 0))
        port = available.getsockname()[1]
    os.environ.update(ROS_MASTER_URI=f'http://127.0.0.1:{port}',
                      ROS_IP='127.0.0.1', ROS_HOSTNAME='127.0.0.1',
                      ROS_HOME=str(out / 'ros-home'), ROS_LOG_DIR=str(out / 'ros-log'))
    children, logs = [], []

    def spawn(command, name):
        log = (out / name).open('w')
        logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        children.append(process)
        return process

    def until(predicate, label, seconds=15):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            try:
                result = predicate()
                if result:
                    return result
            except (OSError, http_client.HTTPException, xmlrpc.client.Error):
                pass
            time.sleep(.005)
        raise AssertionError('timeout: ' + label)

    rospy = None
    server = None
    try:
        master = spawn(['roscore', '-p', str(port)], 'roscore.log')
        master_rpc = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        until(lambda: master.poll() is None and master_rpc.getPid('/xsim_telemetry_test')[0] == 1,
              'private ROS master')
        import rospy
        from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
        from mavros_msgs.msg import AttitudeTarget, ExtendedState, State
        from mavros_msgs.srv import CommandBool
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu
        rospy.set_param('/use_sim_time', True)
        rospy.init_node('xsim_telemetry_test', disable_signals=True)
        epoch = 1700000000000000000
        instance = 'telemetry-private'
        config = {'instance_id': instance, 'epoch_ns': epoch, 'paused': True,
                  'publish_clock': True, 'output_period_ns': 8000000,
                  'entities': [{'name': 'uav', 'kind': 'fs150', 'position': [2, -1, 0],
                                'local_origin': [1, -2, 0],
                                'ros': {'frame': 'custom_local', 'mocap_seed': 41,
                                        'mocap_noise': [.05, 0, .02]}},
                               {'name': 'scout', 'kind': 'scout', 'position': [3, 4, 0], 'yaw': .7},
                               {'name': 'mecanum', 'kind': 'mecanum', 'position': [-3, 4, 0], 'yaw': -.9}]}
        (out / 'world.json').write_text(json.dumps(config, indent=2))
        socket_path = str(out / 'world.sock')
        server = spawn([args.xsim, '--config', str(out / 'world.json'), '--socket', socket_path, '--target-id', 'telemetry:world'],
                       'xsim.log')
        until(lambda: server.poll() is None and Path(socket_path).exists(), 'native socket')

        def http(method, path, body=None):
            connection = UnixHTTP(socket_path)
            try:
                connection.request(method, path, None if body is None else json.dumps(body),
                                   {'Content-Type': 'application/json'})
                response = connection.getresponse()
                return response.status, json.loads(response.read())
            finally:
                connection.close()

        def get(path):
            code, result = http('GET', path)
            assert code == 200, (code, result)
            return result

        serial = 0

        def mutate(path, extra=None, synchronous=False):
            nonlocal serial
            serial += 1
            request_id = 'telemetry-' + str(serial)
            body = {'instance_id': instance, 'request_id': request_id, **(extra or {})}
            code, response = http('POST', path, body)
            assert code == (200 if synchronous else 202), (code, response)
            if not synchronous:
                def terminal():
                    result = get('/requests/' + request_id)
                    return result if result['phase'] in ('applied', 'cancelled', 'failed') else None
                response = until(terminal, path)
            assert response['phase'] == 'applied' and response['result']['success'], response
            return response['result']

        messages, lock, subscriptions = {}, threading.Lock(), []
        topics = {
            'localization_pose': ('/uav/pose', PoseStamped, 125),
            'localization_twist': ('/uav/twist', TwistStamped, 125),
            'scout_pose': ('/scout/pose', PoseStamped, 125),
            'scout_twist': ('/scout/twist', TwistStamped, 125),
            'scout_imu': ('/scout/imu/data_raw', Imu, 30),
            'mecanum_pose': ('/mecanum/pose', PoseStamped, 125),
            'mecanum_twist': ('/mecanum/twist', TwistStamped, 125),
            'mecanum_imu': ('/mecanum/imu', Imu, 30),
            'local_pose': ('/uav/mavros/local_position/pose', PoseStamped, 30),
            'local_velocity': ('/uav/mavros/local_position/velocity_local', TwistStamped, 30),
            'local_odom': ('/uav/mavros/local_position/odom', Odometry, 30),
            'imu': ('/uav/mavros/imu/data', Imu, 30),
            'imu_raw': ('/uav/mavros/imu/data_raw', Imu, 30),
            'state': ('/uav/mavros/state', State, 1),
            'extended_state': ('/uav/mavros/extended_state', ExtendedState, 1),
            'target': ('/uav/mavros/setpoint_raw/target_attitude', AttitudeTarget, 10),
        }

        def subscribe(key, topic, message_type):
            messages[key] = []

            def record(message):
                with lock:
                    messages[key].append(message)

            subscriptions.append(rospy.Subscriber(topic, message_type, record,
                                                   queue_size=4096, tcp_nodelay=True))

        for key, (topic, message_type, _) in topics.items():
            subscribe(key, topic, message_type)
        subscribe('clock', '/clock', Clock)
        until(lambda: all(sub.get_num_connections() > 0 for sub in subscriptions),
              'all telemetry TCPROS connections')
        def assert_direct_topics():
            published = dict(rospy.get_published_topics())
            for name in ('uav', 'scout', 'mecanum'):
                assert published['/' + name + '/pose'] == 'geometry_msgs/PoseStamped'
                assert published['/' + name + '/twist'] == 'geometry_msgs/TwistStamped'
            assert published['/scout/imu/data_raw'] == 'sensor_msgs/Imu'
            assert published['/mecanum/imu'] == 'sensor_msgs/Imu'
            assert not any(topic.startswith('/vrpn_client_node/') or
                           '/vision_pose/' in topic or '/vision_speed/' in topic
                           for topic in published), published

        assert_direct_topics()
        provider = NativeProvider(socket_path, 'uav')
        assert provider.start(0)['generation'] == 1
        for name in ('scout', 'mecanum'):
            assert NativeProvider(socket_path, name).start(0)['generation'] == 1

        def samples(key):
            with lock:
                return list(messages[key])

        def counts():
            with lock:
                return {key: len(values) for key, values in messages.items()}

        def observe(expected):
            start = get('/status')['simulation_time_ns']
            mutate('/resume')
            until(lambda: get('/status')['simulation_time_ns'] >= start + 2400000000,
                  '2.4 seconds of integrated time', seconds=25)
            mutate('/pause')
            frozen = get('/status')['simulation_time_ns']
            until(lambda: samples('clock') and samples('clock')[-1].clock.to_nsec() == frozen,
                  'paused clock projection')
            time.sleep(.15)  # Drain messages sent before Pause on the private TCPROS links.
            report = {}
            for key, requested in expected.items():
                selected = [m for m in samples(key)
                            if start + 200000000 <= m.header.stamp.to_nsec() < start + 2200000000]
                stamps = [m.header.stamp.to_nsec() for m in selected]
                assert len(stamps) >= 2 and len(stamps) == len(set(stamps)), (key, stamps)
                observed = (len(stamps) - 1) * 1e9 / (stamps[-1] - stamps[0])
                assert .8 * requested <= observed <= 1.2 * requested, (key, requested, observed)
                if key == 'target' or key.startswith('local_'):
                    assert all(m.header.frame_id == 'custom_local' for m in selected), key
                if key in ('imu', 'imu_raw', 'scout_imu', 'mecanum_imu'):
                    assert all(m.header.frame_id == 'base_link' for m in selected), key
                elif key.startswith(('localization_', 'scout_', 'mecanum_')):
                    assert all(m.header.frame_id == 'world' for m in selected), key
                report[key] = {'requested_hz': requested, 'received_hz': observed,
                               'samples': len(stamps), 'first_stamp_ns': stamps[0],
                               'last_stamp_ns': stamps[-1]}
            settled = counts()
            time.sleep(.2)
            assert counts() == settled, 'paused telemetry repeated a frozen sample'
            return report, frozen

        defaults_start = get('/status')['simulation_time_ns']
        defaults, frozen = observe({key: spec[2] for key, spec in topics.items()})

        def first_window(key):
            return {m.header.stamp.to_nsec(): m for m in samples(key)
                    if defaults_start + 200000000 <= m.header.stamp.to_nsec() < defaults_start + 2200000000}

        def xyz(value):
            return (value.x, value.y, value.z)

        def quaternion(value):
            return (value.x, value.y, value.z, value.w)

        measured, local = first_window('localization_pose'), first_window('local_pose')
        paired = sorted(measured.keys() & local.keys())
        assert len(paired) >= 20, ('pose comparison samples', len(paired))
        errors = [tuple(a - b - origin for a, b, origin in
                        zip(xyz(measured[stamp].pose.position), xyz(local[stamp].pose.position), (1, -2, 0)))
                  for stamp in paired]
        assert all(abs(error[1]) < 1e-9 for error in errors), 'noise leaked into zero-noise axis'
        for axis, sigma in ((0, .05), (2, .02)):
            mean = sum(error[axis] for error in errors) / len(errors)
            variance = sum((error[axis] - mean) ** 2 for error in errors) / len(errors)
            assert abs(mean) < sigma and .2 * sigma ** 2 < variance < 3 * sigma ** 2, (axis, mean, variance)
        assert all(quaternion(measured[stamp].pose.orientation) == quaternion(local[stamp].pose.orientation)
                   for stamp in paired), 'localization noise changed orientation'
        measured_velocity, local_velocity = first_window('localization_twist'), first_window('local_velocity')
        paired_velocity = measured_velocity.keys() & local_velocity.keys()
        assert len(paired_velocity) >= 20
        assert all(xyz(measured_velocity[stamp].twist.linear) == xyz(local_velocity[stamp].twist.linear) and
                   xyz(measured_velocity[stamp].twist.angular) == xyz(local_velocity[stamp].twist.angular)
                   for stamp in paired_velocity), 'localization noise changed velocity'
        imu_messages, raw_messages = first_window('imu'), first_window('imu_raw')
        paired_imu = imu_messages.keys() & raw_messages.keys() & local.keys()
        assert len(paired_imu) >= 20
        assert all(xyz(imu_messages[stamp].angular_velocity) == xyz(raw_messages[stamp].angular_velocity) and
                   xyz(imu_messages[stamp].linear_acceleration) == xyz(raw_messages[stamp].linear_acceleration) and
                   quaternion(imu_messages[stamp].orientation) == quaternion(local[stamp].pose.orientation) and
                   raw_messages[stamp].orientation_covariance[0] == -1
                   for stamp in paired_imu), 'localization noise changed IMU truth'
        for name, position in (('scout', (3, 4, 0)), ('mecanum', (-3, 4, 0))):
            assert all(xyz(message.pose.position) == position for message in first_window(name + '_pose').values()), name
            assert all(xyz(message.twist.linear) == (0, 0, 0) and xyz(message.twist.angular) == (0, 0, 0)
                       for message in first_window(name + '_twist').values()), name
            assert all(xyz(message.angular_velocity) == (0, 0, 0) and
                       xyz(message.linear_acceleration) == (0, 0, 9.8066)
                       for message in first_window(name + '_imu').values()), name
        assert all(message.orientation_covariance[0] == -1
                   for message in first_window('scout_imu').values())

        code, rejected = http('POST', '/telemetry-rates',
                              {'instance_id': instance, 'request_id': 'retired-vrpn-rate',
                               'rates_hz': {'vrpn': 125}})
        assert code == 400 and 'unknown telemetry rate' in rejected['error'], (code, rejected)
        updated = mutate('/telemetry-rates', {'rates_hz': {'local': 15, 'imu': 10,
                                                         'state': 0, 'extended_state': 0}},
                         synchronous=True)['requested_rates_hz']
        assert updated == {'localization': 125, 'local': 15, 'imu': 10, 'imu_raw': 30,
                           'state': 0, 'extended_state': 0, 'target': 10}, updated
        assert get('/status')['simulation_time_ns'] == frozen
        boundary_counts = counts()
        rospy.wait_for_service('/uav/mavros/cmd/arming', timeout=5)
        assert rospy.ServiceProxy('/uav/mavros/cmd/arming', CommandBool)(True).success
        until(lambda: samples('state')[-1].armed and
              samples('state')[-1].header.stamp.to_nsec() == frozen, 'zero-Hz arm state boundary')
        until(lambda: counts()['extended_state'] > boundary_counts['extended_state'],
              'zero-Hz extended state boundary')
        before_stop = counts()['state']
        assert provider.stop(1)['success']
        until(lambda: counts()['state'] > before_stop and not samples('state')[-1].connected,
              'zero-Hz provider stop boundary')
        reset = mutate('/reset', {'entity_id': provider.entity_id, 'generation': 1})
        assert reset['generation'] == 2 and not reset['enabled'], reset
        assert provider.start(2)['generation'] == 3
        until(lambda: samples('state')[-1].connected and not samples('state')[-1].armed,
              'reset/start projection at frozen stamp')
        assert_direct_topics()
        expected = {key: spec[2] for key, spec in topics.items()
                    if key not in ('state', 'extended_state')}
        expected.update(local_pose=15, local_velocity=15, local_odom=15, imu=10, mecanum_imu=10)
        motion_publishers = {name: rospy.Publisher('/' + name + '/cmd_vel', Twist, queue_size=1)
                             for name in ('scout', 'mecanum')}
        until(lambda: all(p.get_num_connections() > 0 for p in motion_publishers.values()),
              'UGV velocity connections')
        for name, publisher in motion_publishers.items():
            command = Twist()
            command.linear.x, command.angular.z = .6, .5
            command.linear.y = .2 if name == 'mecanum' else 0
            publisher.publish(command)
        updated_start = get('/status')['simulation_time_ns']
        after_update, _ = observe(expected)
        for name in ('scout', 'mecanum'):
            moving = [m for m in samples(name + '_imu')
                      if updated_start + 200000000 <= m.header.stamp.to_nsec() < updated_start + 2200000000]
            assert moving, name + ' missing moving IMU'
            for message in moving:
                assert abs(message.angular_velocity.z - .5) < 1e-6, name
                assert abs(message.linear_acceleration.y - .3) < 1e-6, name
                assert abs(message.linear_acceleration.x - (-.1 if name == 'mecanum' else 0)) < 1e-6, name
                assert message.linear_acceleration.z == 9.8066, name
        moving_pose = {m.header.stamp.to_nsec(): m for m in samples('mecanum_pose')}
        paired_moving = [m for m in moving if m.header.stamp.to_nsec() in moving_pose]
        assert len(paired_moving) >= 2 and all(
            quaternion(m.orientation) == quaternion(moving_pose[m.header.stamp.to_nsec()].pose.orientation)
            for m in paired_moving), 'Mecanum IMU attitude truth'
        for key in ('state', 'extended_state'):
            assert not any(m.header.stamp.to_nsec() > updated_start + 200000000
                           for m in samples(key)), key + ' ignored the zero-Hz periodic rate'
        assert get('/telemetry-rates')['requested_rates_hz'] == updated
        (out / 'result.json').write_text(json.dumps({'result': 'PASS', 'defaults': defaults,
                                                    'after_rpc': after_update,
                                                    'zero_rate_boundaries': True,
                                                    'direct_localization_all_kinds': True,
                                                    'pose_only_noise': True,
                                                    'ugv_imu_rest_and_turn': True,
                                                    'paused_measurements_stable': True}, indent=2) + '\n')
    finally:
        if rospy is not None:
            rospy.signal_shutdown('private telemetry fixture complete')
        for process in reversed(children):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
        for log in logs:
            log.close()
    assert server is not None and server.returncode == 0, (out / 'xsim.log').read_text()
    print('PASS: direct localization for all kinds, pose-only noise, telemetry rates, pause, and lifecycle')


if __name__ == '__main__':
    main()
