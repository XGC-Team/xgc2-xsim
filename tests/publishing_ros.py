#!/usr/bin/env python3
"""Measure production ROS publication on an isolated master and TCPROS links."""
import argparse
import collections
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import selectors
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
import xmlrpc.client
from xmlrpc.server import SimpleXMLRPCServer

from native_http import UnixHTTP


U32 = struct.Struct('<I')
HEADER = struct.Struct('<III')
AGE_BINS_NS = [int(ms * 1e6) for ms in (0, 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024)]
MAX_MESSAGE_BYTES = 1024 * 1024


def until(predicate, label, seconds=20):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        result = predicate()
        if result:
            return result
        time.sleep(.01)
    raise AssertionError('timeout: ' + label)


def encoded_header(fields):
    pieces = [key.encode() + b'=' + value.encode() for key, value in fields.items()]
    data = b''.join(U32.pack(len(piece)) + piece for piece in pieces)
    return U32.pack(len(data)) + data


def read_exact(connection, length):
    data = bytearray()
    while len(data) < length:
        part = connection.recv(length - len(data))
        if not part:
            raise AssertionError('TCPROS connection closed during handshake')
        data.extend(part)
    return bytes(data)


def read_header(connection):
    length = U32.unpack(read_exact(connection, 4))[0]
    assert 0 < length <= MAX_MESSAGE_BYTES, length
    data, position, fields = read_exact(connection, length), 0, {}
    while position < length:
        size = U32.unpack_from(data, position)[0]
        position += 4
        assert size <= length - position
        key, value = data[position:position + size].decode().split('=', 1)
        fields[key] = value
        position += size
    return fields


class RosReceiver:
    """A fixed-thread ROS slave and selector for all registered TCPROS streams."""
    def __init__(self, master_uri, topic_specs):
        self.master = xmlrpc.client.ServerProxy(master_uri)
        self.caller = '/xsim_publishing_receiver'
        self.slave = SimpleXMLRPCServer(('127.0.0.1', 0), allow_none=True, logRequests=False)
        self.slave.register_function(lambda *args: [1, '', 0], 'publisherUpdate')
        self.slave.register_function(lambda *args: [1, '', os.getpid()], 'getPid')
        self.slave.register_function(lambda *args: [1, '', []], 'getPublications')
        self.slave.register_function(lambda *args: [1, '', [[topic, spec['type']._type]
                                      for topic, spec in topic_specs.items()]], 'getSubscriptions')
        self.uri = 'http://127.0.0.1:%d' % self.slave.server_address[1]
        self.slave_thread = threading.Thread(target=self.slave.serve_forever, daemon=True)
        self.slave_thread.start()
        self.selector = selectors.DefaultSelector()
        self.lock, self.stop = threading.Lock(), threading.Event()
        self.specs, self.streams, self.stats = topic_specs, [], {}
        self.error = None
        self.anchor_stamp, self.anchor_wall = None, None
        for topic in topic_specs:
            self.stats[topic] = {'messages': 0, 'payload_bytes': 0, 'last_stamp_ns': None,
                                 'stamp_regressions': 0, 'duplicate_stamps': 0,
                                 'latest': b'', 'frame': None, 'max_points': 0,
                                 'age_bins': [0] * (len(AGE_BINS_NS) + 1), 'closed': False}
        self.thread = threading.Thread(target=self.receive, daemon=True)
        self.thread.start()

    def connect(self, topic, slow=False):
        spec = self.specs[topic]
        code, _, publishers = self.master.registerSubscriber(self.caller, topic, spec['type']._type, self.uri)
        assert code == 1 and len(publishers) == 1, (topic, publishers)
        publisher = xmlrpc.client.ServerProxy(publishers[0])
        code, _, transport = publisher.requestTopic(self.caller, topic, [['TCPROS']])
        assert code == 1 and transport[0] == 'TCPROS', transport
        connection = socket.socket()
        if slow:
            connection.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024)
        connection.settimeout(10)
        connection.connect((transport[1], transport[2]))
        connection.sendall(encoded_header({'callerid': self.caller, 'topic': topic,
                                           'type': spec['type']._type, 'md5sum': spec['type']._md5sum,
                                           'tcp_nodelay': '1'}))
        header = read_header(connection)
        assert 'error' not in header and header.get('md5sum') == spec['type']._md5sum, header
        self.streams.append(connection)
        if not slow:
            connection.setblocking(False)
            self.selector.register(connection, selectors.EVENT_READ, [topic, bytearray()])
        return connection

    def receive(self):
        try:
            while not self.stop.is_set():
                for key, _ in self.selector.select(.02):
                    topic, buffer = key.data
                    try:
                        data = key.fileobj.recv(262144)
                    except BlockingIOError:
                        continue
                    except ConnectionResetError:
                        data = b''
                    if not data:
                        self.selector.unregister(key.fileobj)
                        with self.lock:
                            self.stats[topic]['closed'] = True
                        continue
                    buffer.extend(data)
                    position = 0
                    with self.lock:
                        while len(buffer) - position >= 4:
                            size = U32.unpack_from(buffer, position)[0]
                            assert 0 < size <= MAX_MESSAGE_BYTES, (topic, size)
                            if len(buffer) - position < size + 4:
                                break
                            payload = bytes(buffer[position + 4:position + size + 4])
                            position += size + 4
                            self.record(topic, payload)
                    if position:
                        del buffer[:position]
                    assert len(buffer) <= MAX_MESSAGE_BYTES + 4
        except Exception as error:
            self.error = repr(error)
            self.stop.set()

    def record(self, topic, payload):
        stats = self.stats[topic]
        _, seconds, nanoseconds = HEADER.unpack_from(payload)
        stamp = seconds * 1000000000 + nanoseconds
        frame_length = U32.unpack_from(payload, 12)[0]
        assert len(payload) >= 16 + frame_length
        frame = payload[16:16 + frame_length].decode()
        if stats['last_stamp_ns'] is not None:
            stats['stamp_regressions'] += stamp < stats['last_stamp_ns']
            stats['duplicate_stamps'] += stamp == stats['last_stamp_ns']
        stats['messages'] += 1
        stats['payload_bytes'] += len(payload)
        stats['last_stamp_ns'], stats['latest'], stats['frame'] = stamp, payload, frame
        if self.specs[topic]['group'] == 'cloud':
            height, width = struct.unpack_from('<II', payload, 16 + frame_length)
            points = height * width
            assert 0 < points <= self.specs[topic]['max_points'], (topic, points)
            stats['max_points'] = max(stats['max_points'], points)
        if self.anchor_stamp is not None:
            age = int((time.monotonic() - self.anchor_wall) * 1e9) - (stamp - self.anchor_stamp)
            index = 0
            while index < len(AGE_BINS_NS) and age > AGE_BINS_NS[index]:
                index += 1
            stats['age_bins'][index] += 1

    def anchor(self, stamp, wall):
        with self.lock:
            self.anchor_stamp, self.anchor_wall = stamp, wall

    def snapshot(self):
        assert self.error is None, self.error
        with self.lock:
            return {topic: {**value, 'age_bins': list(value['age_bins'])}
                    for topic, value in self.stats.items()}

    def close(self):
        self.stop.set()
        self.thread.join(3)
        for connection in self.streams:
            connection.close()
        self.selector.close()
        self.slave.shutdown()
        self.slave.server_close()
        self.slave_thread.join(3)


def process_sample(pid):
    fields = Path('/proc/%d/stat' % pid).read_text().rsplit(')', 1)[1].split()
    sample = {'cpu_seconds': (int(fields[11]) + int(fields[12])) / os.sysconf('SC_CLK_TCK'),
              'rss_bytes': int(fields[21]) * os.sysconf('SC_PAGE_SIZE'),
              'threads': int(fields[17]), 'fd_count': len(list(Path('/proc/%d/fd' % pid).iterdir()))}
    return sample


def distribution(values):
    values = sorted(values)
    if not values:
        return None
    return {'min': values[0], 'median': values[len(values)//2],
            'p95': values[min(len(values)-1, int(len(values)*.95))], 'max': values[-1]}


def status_summary(status):
    result = {key: value for key, value in status.items() if key != 'sensors'}
    sensors = status.get('sensors', [])
    if sensors:
        totals = ('scans', 'misses', 'throttled_samples', 'cache_hits', 'computed_scans',
                  'published', 'rate_degradations', 'rate_recoveries', 'errors',
                  'payload_grows', 'payload_misses')
        rates = ('requested_rate_hz', 'effective_rate_hz', 'source_rate_hz', 'observed_rate_hz',
                 'latency_ns', 'max_latency_ns')
        result['sensor_summary'] = {'count': len(sensors),
            'totals': {key: sum(s[key] for s in sensors) for key in totals if all(key in s for s in sensors)},
            'distributions': {key: distribution([s[key] for s in sensors]) for key in rates},
            'sample': sensors[:1]}
    return result


def publication_delta(before, after, elapsed):
    frame_growth = (after.get('frame_array_grows', 0) - before.get('frame_array_grows', 0))
    frame_slots = after.get('frame_slots')
    before, after = before.get('publication'), after.get('publication')
    if after is None:
        return None
    assert before is not None and after['errors'] == 0 and not after['error'], after
    assert after['workers'] == 2 and after.get('initial_buffer_slots_per_topic', after.get('buffer_slots_per_topic')) == 4, after
    result = {'submitted_frames': after['submitted_frames'] - before['submitted_frames'],
              'coalesced_worker_frames': after['coalesced_worker_frames'] - before['coalesced_worker_frames'],
              'prepare_ns': after['prepare_ns'] - before['prepare_ns'],
              'prepare_ns_lifetime_max': after['prepare_ns_max'],
              'frame_array_grows': frame_growth, 'frame_slots': frame_slots}
    for name in ('telemetry', 'cloud'):
        first, last = before[name], after[name]
        delta = {key: last[key] - first[key] for key in
                 ('accepted', 'bytes_estimate', 'no_subscribers', 'buffer_drops', 'serialize_ns',
                  'queue_ns', 'backpressure_ns', 'buffer_allocations')}
        delta.update(slot_capacity_bytes=last['slot_capacity_bytes'],
                     slot_capacity_growth_bytes=last['slot_capacity_bytes']-first['slot_capacity_bytes'],
                     serialize_ns_lifetime_max=last['serialize_ns_max'],
                     queue_ns_lifetime_max=last['queue_ns_max'])
        for timing in ('serialize_ns', 'queue_ns'):
            delta[timing+'_per_accepted'] = delta[timing]/delta['accepted'] if delta['accepted'] else None
        delta['bytes_estimate_per_wall_second'] = delta['bytes_estimate']/elapsed
        result[name] = delta
    return result


def window(receiver, api, pid, duration):
    start_messages, start_status = receiver.snapshot(), api.get('/v1/world')['diagnostics']
    start_cpu, fixture_cpu, started = process_sample(pid), time.process_time(), time.monotonic()
    resources = [start_cpu]
    while time.monotonic() - started < duration:
        time.sleep(min(.25, max(.001, duration - (time.monotonic() - started))))
        resources.append(process_sample(pid))
        assert receiver.error is None, receiver.error
    ended, fixture_end_cpu = time.monotonic(), time.process_time()
    end_messages, end_status = receiver.snapshot(), api.get('/v1/world')['diagnostics']
    elapsed = ended - started
    topics, groups = {}, collections.defaultdict(lambda: {'messages': 0, 'payload_bytes': 0,
                                                         'wall_hz_sum': 0, 'topics': 0})
    for topic, spec in receiver.specs.items():
        before, after = start_messages[topic], end_messages[topic]
        count = after['messages'] - before['messages']
        byte_count = after['payload_bytes'] - before['payload_bytes']
        stamps = ((after['last_stamp_ns'] - before['last_stamp_ns'])
                  if before['last_stamp_ns'] is not None and after['last_stamp_ns'] is not None else 0)
        assert after['stamp_regressions'] == before['stamp_regressions'], topic
        assert count > 0, ('no received samples', topic)
        if spec['group'] not in ('state', 'extended_state'):
            assert after['duplicate_stamps'] == before['duplicate_stamps'], ('duplicate measurement', topic)
        topics[topic] = {'group': spec['group'], 'requested_hz': spec['hz'], 'samples': count,
                         'wall_hz': count / elapsed, 'stamp_hz': count * 1e9 / stamps if stamps > 0 else None,
                         'payload_bytes': byte_count, 'tcpros_message_bytes': byte_count + 4 * count,
                         'latest_simulation_age_ns': (end_status['simulation_time_ns'] - after['last_stamp_ns']
                                                      if after['last_stamp_ns'] is not None else None),
                         'receive_delay_proxy_bins': [b - a for a, b in zip(before['age_bins'], after['age_bins'])],
                         'duplicate_stamps': after['duplicate_stamps'] - before['duplicate_stamps'],
                         'max_cloud_points': after['max_points']}
        group = groups[spec['group']]
        group['messages'] += count; group['payload_bytes'] += byte_count
        group['wall_hz_sum'] += count / elapsed; group['topics'] += 1
    for group in groups.values():
        group['mean_topic_wall_hz'] = group['wall_hz_sum'] / group['topics']
    for name, group in groups.items():
        selected = [stats for stats in topics.values() if stats['group'] == name]
        group['topic_wall_hz'] = distribution([stats['wall_hz'] for stats in selected])
        group['latest_simulation_age_ns'] = distribution([stats['latest_simulation_age_ns'] for stats in selected
                                                         if stats['latest_simulation_age_ns'] is not None])
        group['zero_sample_topics'] = sum(stats['samples'] == 0 for stats in selected)
        group['max_cloud_points'] = max(stats['max_cloud_points'] for stats in selected)
        group['receive_delay_proxy_bins'] = [sum(stats['receive_delay_proxy_bins'][i] for stats in selected)
                                             for i in range(len(AGE_BINS_NS)+1)]
    sample = {topic:stats for topic,stats in topics.items()
              if topic.startswith(('/robot000/', '/robot%03d/' % (len(receiver.specs)//11-1)))}
    return {'wall_seconds': elapsed, 'groups': dict(groups), 'sample_topics': sample,
            'received_payload_bytes': sum(g['payload_bytes'] for g in groups.values()),
            'xsim_cpu_core_equivalents': (resources[-1]['cpu_seconds'] - start_cpu['cpu_seconds']) / elapsed,
            'receiver_cpu_core_equivalents': (fixture_end_cpu - fixture_cpu) / elapsed,
            'xsim_rss_min_bytes': min(r['rss_bytes'] for r in resources),
            'xsim_rss_max_bytes': max(r['rss_bytes'] for r in resources),
            'xsim_rss_end_bytes': resources[-1]['rss_bytes'],
            'xsim_threads_max': max(r['threads'] for r in resources),
            'simulation_wall_ratio': (end_status['simulation_time_ns'] - start_status['simulation_time_ns']) / (elapsed * 1e9),
            'publication_delta': publication_delta(start_status, end_status, elapsed),
            'status_start': status_summary(start_status), 'status_end': status_summary(end_status)}


class Api:
    def __init__(self, path):
        self.path, self.instance, self.serial = path, None, 0
        self.instance = self.get('/v1/describe')['instance_id']

    def request(self, method, path, body=None):
        self.serial += 1
        headers = {'Content-Type': 'application/json', 'X-Xrpc-Timeout-Ms': '5000',
                   'X-Request-ID': 'publishing-' + str(self.serial)}
        if self.instance:
            headers['X-Xrpc-Instance-ID'] = self.instance
        connection = UnixHTTP(self.path)
        try:
            connection.request(method, path, None if body is None else json.dumps(body),
                               headers)
            response = connection.getresponse()
            return response.status, json.loads(response.read())
        finally:
            connection.close()

    def get(self, path):
        code, result = self.request('GET', path)
        assert code == 200, (code, result)
        return result

    def mutate(self, path, extra=None, method='POST'):
        code, result = self.request(method, path, {'operation_timeout_ms': 5000, **(extra or {})})
        assert code in (200, 201, 202), (code, result)
        def terminal():
            receipt = self.get('/v1/operations/' + result['id'])
            return receipt if receipt['state'] in ('succeeded', 'failed', 'cancelled') else None
        receipt = until(terminal, path)
        assert receipt['state'] == 'succeeded', receipt
        return receipt


def topic_specs(count):
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from mavros_msgs.msg import AttitudeTarget, ExtendedState, State
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, PointCloud2
    specs = {}
    definitions = [('pose', PoseStamped, 'localization', 125), ('twist', TwistStamped, 'localization', 125),
                   ('mavros/local_position/pose', PoseStamped, 'local', 30),
                   ('mavros/local_position/velocity_local', TwistStamped, 'local', 30),
                   ('mavros/local_position/odom', Odometry, 'local', 30),
                   ('mavros/imu/data', Imu, 'imu', 30), ('mavros/imu/data_raw', Imu, 'imu_raw', 30),
                   ('mavros/state', State, 'state', 1), ('mavros/extended_state', ExtendedState, 'extended_state', 1),
                   ('mavros/setpoint_raw/target_attitude', AttitudeTarget, 'target', 10),
                   ('cloud', PointCloud2, 'cloud', 10)]
    for index in range(count):
        for suffix, message_type, group, hz in definitions:
            specs['/robot%03d/%s' % (index, suffix)] = {'type': message_type, 'group': group, 'hz': hz,
                                                     'max_points': 360 * 32 if index == 0 else 72 * 16}
    return specs


def lifecycle(receiver, api):
    from geometry_msgs.msg import PoseStamped, TwistStamped
    api.mutate('/v1/world/pause')
    frozen = api.get('/v1/world')['diagnostics']['simulation_time_ns']
    time.sleep(.6)
    first = receiver.snapshot(); time.sleep(.4); second = receiver.snapshot()
    assert api.get('/v1/world')['diagnostics']['simulation_time_ns'] == frozen
    assert all(first[t]['messages'] == second[t]['messages'] for t in first), 'paused samples repeated'
    entity_path = '/v1/entities/robot000'
    ref = api.get(entity_path)['ref']
    api.mutate(entity_path + '/state', {'generation': ref['generation'], 'state': {'enabled': False}})
    api.mutate(entity_path + '/reset', {'generation': ref['generation']})
    reset = api.get(entity_path)
    assert reset['ref'] == ref and not reset['state']['enabled'], reset
    code, stale = api.request('POST', entity_path + '/state',
                              {'operation_timeout_ms': 5000, 'generation': ref['generation'] + 1,
                               'state': {'enabled': True}})
    assert code == 409, ('stale generation start succeeded', code, stale)
    api.mutate(entity_path + '/state', {'generation': ref['generation'], 'state': {'enabled': True}})
    started = time.monotonic(); receiver.anchor(frozen, started); api.mutate('/v1/world/resume')
    def new_measurements():
        snapshot = receiver.snapshot()
        return all(snapshot[topic]['last_stamp_ns'] > frozen for topic in ('/robot000/pose', '/robot000/twist'))
    until(new_measurements, 'new generation pose/twist')
    snapshot = receiver.snapshot()
    pose = PoseStamped();pose.deserialize(snapshot['/robot000/pose']['latest'])
    twist = TwistStamped();twist.deserialize(snapshot['/robot000/twist']['latest'])
    assert pose.header.frame_id == twist.header.frame_id == 'world'
    assert pose.header.stamp.to_nsec() > frozen and twist.header.stamp.to_nsec() > frozen
    assert api.get(entity_path)['ref'] == ref
    api.mutate(entity_path, {'generation': ref['generation']}, method='DELETE')
    assert not any(e['ref']['id'] == ref['id'] for e in api.get('/v1/entities')['entities'])
    time.sleep(.5); first = receiver.snapshot(); time.sleep(.3); second = receiver.snapshot()
    assert all(first[t]['messages'] == second[t]['messages'] for t in first if t.startswith('/robot000/'))
    assert second['/robot001/pose']['messages'] > first['/robot001/pose']['messages']
    api.mutate('/v1/world/pause')
    return {'pause_samples_stable': True, 'reset_generation': ref['generation'], 'restart_generation': ref['generation'],
            'stale_start_rejected': True, 'removed_topics_stable': True, 'remaining_pose_continues': True,
            'ros_generation_note': 'Public EntityRef generation is stable through reset/start and checked via v1; ROS messages do not carry generation.'}


def verify_windows(steady, congested, recovered, args):
    if steady['publication_delta'] is None:
        assert not args.require_publication
        return {'publication_metrics_available': False}
    assert args.warmup >= 5, 'four slots of 1 Hz state need at least five wall seconds of warmup'
    for phase in (steady, congested, recovered):
        assert phase['publication_delta']['frame_slots'] == 16
        assert phase['publication_delta']['frame_array_grows'] == 0, 'fixed fleet Frame arrays grew'
        first, last = phase['status_start']['sensor_summary'], phase['status_end']['sensor_summary']
        assert last['sample'][0]['payload_slots'] == 4, last
        assert last['totals']['payload_grows'] == first['totals']['payload_grows'], 'fixed scene payload vectors grew'
        for group in ('telemetry', 'cloud'):
            metrics = phase['publication_delta'][group]
            if phase is not congested or group != 'cloud':
                assert metrics['buffer_allocations'] == 0, ('stable payload allocated after warmup', group, metrics)
                assert metrics['slot_capacity_growth_bytes'] == 0, ('stable slot capacity grew', group, metrics)
    normal_pose = steady['groups']['localization']['topic_wall_hz']['median']
    slow_pose = congested['groups']['localization']['topic_wall_hz']['median']
    recovery_pose = recovered['groups']['localization']['topic_wall_hz']['median']
    assert slow_pose >= .75 * normal_pose, ('slow subscriber stalled pose', normal_pose, slow_pose)
    assert recovery_pose >= .8 * normal_pose, ('pose failed to recover', normal_pose, recovery_pose)
    assert congested['groups']['localization']['latest_simulation_age_ns']['max'] < 500000000
    assert congested['sample_topics']['/robot000/cloud']['latest_simulation_age_ns'] < 500000000, 'stopped connections starved the healthy cloud receiver'
    margin = max(16*1024*1024, 4*(steady['xsim_rss_max_bytes']-steady['xsim_rss_min_bytes']))
    assert congested['xsim_rss_max_bytes'] <= steady['xsim_rss_end_bytes'] + margin, 'slow subscriber RSS grew beyond bounded allowance'
    checks = {'publication_metrics_available': True, 'steady_and_recovered_buffers_stable': True,
              'pose_slow_to_steady_ratio': slow_pose/normal_pose,
              'pose_recovery_to_steady_ratio': recovery_pose/normal_pose,
              'slow_extra_rss_allowance_bytes': margin, 'slow_requires_positive_drops': False}
    return checks


def run(args):
    out = Path(args.output);out.mkdir(parents=True, exist_ok=True)
    out.chmod(0o700)
    for name in ("ros-home", "ros-log"):
        (out / name).mkdir(mode=0o700, exist_ok=True)
    nofile, _ = resource.getrlimit(resource.RLIMIT_NOFILE)
    assert nofile >= args.count * 11 + 64, ('TCPROS benchmark needs --ulimit nofile=4096:4096', nofile)
    with socket.socket() as available:
        available.bind(('127.0.0.1', 0));port = available.getsockname()[1]
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % port, ROS_IP='127.0.0.1',
                      ROS_HOSTNAME='127.0.0.1', ROS_HOME=str(out/'ros-home'), ROS_LOG_DIR=str(out/'ros-log'))
    children, logs, receiver, server = [], [], None, None
    def spawn(command, name):
        log = (out/name).open('w');logs.append(log)
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(child);return child
    result = None
    try:
        master = spawn(['roscore', '-p', str(port)], 'roscore.log')
        master_rpc = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        def master_ready():
            try:return master.poll() is None and master_rpc.getPid('/xsim_publishing_test')[0] == 1
            except OSError:return False
        until(master_ready, 'private ROS master')
        config = {'instance_id': 'publishing-private', 'epoch_ns': 1700000000000000000, 'paused': True,
                  'publish_clock': False, 'model_step_ns': 2000000, 'output_period_ns': 8000000,
                  'sensor_workers': 2, 'scene': {'obstacles': [{'type': 'box', 'position': [0,0,-.5], 'size': [256,256,1]}]},
                  'entities': [{'name': 'robot%03d' % index, 'kind': 'fs150',
                                'position': [index % 10, index // 10, 0],
                                'ros': {'mocap_noise': [0,0,0]},
                                'sensor': {'backend': 'cpu', 'mode': 'raycast', 'rate_hz': 10,
                                           'h_res': 360 if index == 0 else 72,
                                           'v_res': 32 if index == 0 else 16,
                                           'v_fov_deg': 90, 'h_fov_deg': 360, 'range': 20,
                                           'translation': [0,0,1], 'world_bodies': False, 'noise_std': 0}}
                               for index in range(args.count)]}
        if args.scene:
            config['scene'] = json.loads(Path(args.scene).read_text())
        (out/'world.json').write_text(json.dumps(config, indent=2)+'\n')
        (out/'world.json').chmod(0o600)
        socket_path = str(out/'world.sock')
        server = spawn([args.xsim, '--socket', socket_path, '--target-id', 'publishing:world',
                        '--config', str(out/'world.json')], 'xsim.log')
        until(lambda: server.poll() is None and Path(socket_path).exists(), 'xsim native socket', 60)
        api = Api(socket_path)
        if args.require_publication:
            assert 'publication' in api.get('/v1/world')['diagnostics'], 'publication status is required'
        roster = api.get('/v1/entities')['entities']
        assert len(roster) == args.count
        receiver = RosReceiver(os.environ['ROS_MASTER_URI'], topic_specs(args.count))
        for topic in receiver.specs:receiver.connect(topic)
        for entity in roster:
            ref = entity['ref']
            api.mutate('/v1/entities/%s/state' % ref['id'], {'generation': ref['generation'], 'state': {'enabled': True}})
        frozen = api.get('/v1/world')['diagnostics']['simulation_time_ns']
        receiver.anchor(frozen, time.monotonic());api.mutate('/v1/world/resume')
        def receiving():
            snapshot = receiver.snapshot()
            return all(stats['messages'] for stats in snapshot.values())
        until(receiving, 'ROS telemetry and bounded clouds receiving', 30)
        time.sleep(args.warmup)
        print('steady: %d entities, %d TCPROS streams' % (args.count, len(receiver.specs)), flush=True)
        steady = window(receiver, api, server.pid, args.duration)
        slow_peers = []
        for _ in range(4):
            slow_peers.append(receiver.connect('/robot000/cloud', slow=True))
            time.sleep(.17)
        slow_buffer = slow_peers[0].getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        time.sleep(.5)
        print('slow subscriber: %d entities' % args.count, flush=True)
        congested = window(receiver, api, server.pid, args.slow_duration)
        for slow in slow_peers: slow.close()
        time.sleep(.5)
        recovered = window(receiver, api, server.pid, max(2., args.duration/2))
        publication_checks = verify_windows(steady, congested, recovered, args)
        boundaries = lifecycle(receiver, api)
        result = {'result': 'PASS', 'binary_sha256': hashlib.sha256(Path('/proc/%d/exe' % server.pid).read_bytes()).hexdigest(),
                  'world_config_sha256': hashlib.sha256((out/'world.json').read_bytes()).hexdigest(),
                  'entities': args.count, 'fixture': {'normal_cloud_rays': 72*16, 'first_cloud_rays': 360*32,
                                                    'receiver_threads': 2, 'static_scene': True, 'noise_std': 0,
                                                    'slow_receive_buffer_bytes': slow_buffer, 'extra_slow_connections': len(slow_peers),
                                                    'slow_subscriber': 'four staggered extra TCPROS cloud sockets with SO_RCVBUF=1024; no payload reads'},
                  'receive_delay_proxy_upper_bounds_ns': AGE_BINS_NS + [None],
                  'metrics_note': 'Payload/TCPROS framing byte counters exclude handshake, IP/TCP headers and retransmission. Receive delay is wall since Resume minus stamp advance and includes world lag/RTF; it is not pure network latency. Each stream retains only its latest payload and bounded partial frame.',
                  'steady': steady, 'slow_subscriber': congested, 'recovered': recovered,
                  'publication_checks': publication_checks, 'lifecycle': boundaries}
    finally:
        if receiver is not None:receiver.close()
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGTERM)
                try:child.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL);child.wait()
        for log in logs:log.close()
    assert server is not None and server.returncode == 0, ('xsim exit', server.returncode if server else None)
    result['xsim_exit_code'] = server.returncode
    (out/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print('PASS: %d entities, measured steady/slow/recovery and lifecycle' % args.count, flush=True)


def comparison_world_identity(directory, result):
    raw = (directory/'world.json').read_bytes()
    raw_sha = hashlib.sha256(raw).hexdigest()
    if 'world_config_sha256' in result:
        assert result['world_config_sha256'] == raw_sha, 'recorded world hash differs from source bytes'
    config = json.loads(raw)
    assert isinstance(config, dict), 'world config must be an object'
    omitted = []
    if 'cloud_bandwidth_mbps' in config:
        value = config.pop('cloud_bandwidth_mbps')
        assert type(value) in (int, float) and value == 0, 'legacy test format accepts only numeric zero cloud_bandwidth_mbps'
        omitted.append('cloud_bandwidth_mbps')
    canonical = json.dumps(config, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    return {'world_sha256': raw_sha,
            'comparison_input_sha256': hashlib.sha256(canonical).hexdigest(),
            'test_format_compatibility': {'canonicalization': 'sorted JSON keys, compact separators',
                'recorded_world_sha256_checked': 'world_config_sha256' in result,
                'omitted_legacy_zero_fields': omitted,
                'scope': 'test input comparison only; no runtime configuration compatibility'}}


def compare(args):
    out = Path(args.output);out.mkdir(parents=True, exist_ok=True)
    runs, origins = {}, {}
    current_sha = hashlib.sha256(Path(args.current).read_bytes()).hexdigest()
    for count in (int(value) for value in args.counts.split(',')):
        assert 2 <= count <= 100
        runs[str(count)] = {}
        origins[str(count)] = {}
        for label, binary in (('baseline', args.baseline), ('current', args.current)):
            if label == 'baseline' and args.baseline_results:
                directory = Path(args.baseline_results)/('baseline-%d' % count)
                result_path = directory/'result.json'
                result = json.loads(result_path.read_text())
                assert result['result'] == 'PASS' and result['xsim_exit_code'] == 0 and result['entities'] == count
                if args.baseline:
                    assert result['binary_sha256'] == hashlib.sha256(Path(args.baseline).read_bytes()).hexdigest()
                runs[str(count)][label] = result
                origins[str(count)][label] = {'result':os.path.relpath(result_path,out),
                    'result_sha256':hashlib.sha256(result_path.read_bytes()).hexdigest(),
                    **comparison_world_identity(directory, result),
                    'reused_existing_measurements':True}
                continue
            directory = out/('%s-%d' % (label,count))
            command = [sys.executable, str(Path(__file__).resolve()), '--xsim', binary, '--count', str(count),
                       '--output', str(directory), '--duration', str(args.duration),
                       '--slow-duration', str(args.slow_duration), '--warmup', str(args.warmup)]
            if label == 'current':command.append('--require-publication')
            if args.scene:command.extend(['--scene', str(Path(args.scene).resolve())])
            subprocess.run(command, check=True)
            runs[str(count)][label] = json.loads((directory/'result.json').read_text())
            assert label != 'current' or runs[str(count)][label]['binary_sha256'] == current_sha, 'current binary changed during comparison'
            origins[str(count)][label] = {'result':os.path.relpath(directory/'result.json',out),
                'result_sha256':hashlib.sha256((directory/'result.json').read_bytes()).hexdigest(),
                **comparison_world_identity(directory, runs[str(count)][label]),
                'reused_existing_measurements':False}
        assert origins[str(count)]['baseline']['comparison_input_sha256'] == origins[str(count)]['current']['comparison_input_sha256'], 'comparison world input differs'
    summary = {}
    for count, pair in runs.items():
        first, last = pair['baseline']['steady'], pair['current']['steady']
        summary[count] = {'baseline_pose_median_wall_hz': first['groups']['localization']['topic_wall_hz']['median'],
                          'current_pose_median_wall_hz': last['groups']['localization']['topic_wall_hz']['median'],
                          'baseline_cloud_mean_wall_hz': first['groups']['cloud']['mean_topic_wall_hz'],
                          'current_cloud_mean_wall_hz': last['groups']['cloud']['mean_topic_wall_hz'],
                          'baseline_xsim_cpu_core_equivalents': first['xsim_cpu_core_equivalents'],
                          'current_xsim_cpu_core_equivalents': last['xsim_cpu_core_equivalents'],
                          'baseline_simulation_wall_ratio': first['simulation_wall_ratio'],
                          'current_simulation_wall_ratio': last['simulation_wall_ratio'],
                          'current_publication_delta': last['publication_delta']}
    (out/'comparison.json').write_text(json.dumps({'summary':summary, 'case_results':origins}, indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--xsim')
    parser.add_argument('--baseline')
    parser.add_argument('--baseline-results')
    parser.add_argument('--current')
    parser.add_argument('--count', type=int, default=20)
    parser.add_argument('--counts', default='20,100')
    parser.add_argument('--scene', help='JSON scene replacing the default one-box scene')
    parser.add_argument('--duration', type=float, default=8)
    parser.add_argument('--slow-duration', type=float, default=6)
    parser.add_argument('--warmup', type=float, default=5)
    parser.add_argument('--require-publication', action='store_true')
    parser.add_argument('--output', required=True)
    arguments = parser.parse_args()
    assert all(math.isfinite(value) for value in (arguments.duration, arguments.slow_duration,
                                                 arguments.warmup))
    assert 2 <= arguments.duration <= 60 and 2 <= arguments.slow_duration <= 30 and 0 <= arguments.warmup <= 30
    if arguments.baseline or arguments.baseline_results or arguments.current:
        assert (arguments.baseline or arguments.baseline_results) and arguments.current and not arguments.xsim
        compare(arguments)
    else:
        assert arguments.xsim and 2 <= arguments.count <= 100
        run(arguments)
