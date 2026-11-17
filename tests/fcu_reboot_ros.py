#!/usr/bin/env python3
"""A real CommandLong FCU reboot on a private paused two-vehicle ROS world."""
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import tempfile
import threading
import time
import xmlrpc.client

from frozen_session import robot, session
from world_client import Xsim, until


def scenario(x, rospy):
    from mavros_msgs.msg import PositionTarget, State
    from mavros_msgs.srv import CommandBool, CommandLong, CommandLongRequest, SetMode

    latest, lock = {}, threading.Lock()

    def record(name, message):
        with lock:
            latest[name] = message

    subscriptions = [rospy.Subscriber('/' + name + '/mavros/state', State,
        lambda message, name=name: record(name, message), queue_size=1)
        for name in ('uav', 'peer')]

    def state(name='uav'):
        with lock:
            return latest.get(name)

    def read(route):
        status, value = x.call('GET', route)
        assert status == 200, (status, value)
        return value

    def cut():
        world = read('/v1/world')
        world.pop('diagnostics')  # wall-time RTF and service-publication counters are not physical state
        return {'world': world, 'uav': read('/v1/entities/uav'),
                'peer': read('/v1/entities/peer'), 'instance': x.instance}

    x.start()
    until(lambda: all(s.get_num_connections() for s in subscriptions), 'state subscriptions')
    # Observe ordinary lifecycle edges; state is intentionally not latched.
    for name in ('uav', 'peer'):
        x.enable(name, False)
        until(lambda: state(name) and not state(name).connected, name + ' disabled')
        x.enable(name)
        until(lambda: state(name) and state(name).connected, name + ' enabled')
    initial = cut()
    x.operation('POST', '/v1/world/step', {'steps': 25})
    moved = cut()
    assert moved['uav']['state']['pose'] != initial['uav']['state']['pose']
    pva = rospy.Publisher('/uav/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    until(lambda: pva.get_num_connections(), 'PVA receiver')
    rospy.wait_for_service('/uav/mavros/cmd/command', timeout=5)
    command = rospy.ServiceProxy('/uav/mavros/cmd/command', CommandLong, persistent=True)
    mode = rospy.ServiceProxy('/uav/mavros/set_mode', SetMode)
    arm = rospy.ServiceProxy('/uav/mavros/cmd/arming', CommandBool)
    target = PositionTarget(coordinate_frame=1, type_mask=3527)
    target.velocity.x = 0.4

    def enter_offboard():
        pva.publish(target)
        time.sleep(0.05)  # finite operator pacing between publishing and mode request
        assert mode(0, 'OFFBOARD').mode_sent
        until(lambda: state().mode == 'OFFBOARD', 'accepted OFFBOARD input')

    def request(**fields):
        values = {'command': 246, 'param1': 1}
        values.update(fields)
        return command(CommandLongRequest(**values))

    try:
        enter_offboard()
        before = cut()
        response = request()
        assert response.success and response.result == 0, response
        until(lambda: state().mode == 'POSCTL', 'reboot clears OFFBOARD mode')
        after = cut()
        assert after == before, 'FCU reboot changed physics, world time, identity, peer or provider'
        assert state().connected and not state().armed and state('peer').connected
        assert mode(0, 'OFFBOARD').mode_sent
        time.sleep(0.05)
        assert state().mode == 'POSCTL', 'reboot retained obsolete OFFBOARD availability'
        # A persistent service connection survives repeated FCU reboot.
        assert request().success
        assert cut() == before
        malformed = [({'command': 22}, 3), ({'param1': 0}, 2), ({'param1': 2}, 2),
                     ({'param1': float('nan')}, 2), ({'param2': 21196}, 3),
                     ({'broadcast': True}, 3), ({'confirmation': 1}, 3)]
        malformed += [({'param' + str(i): 1}, 3) for i in range(3, 8)]
        for fields, expected in malformed:
            refused = request(**fields)
            assert not refused.success and refused.result == expected, (fields, refused)
            assert cut() == before
        enter_offboard()  # fresh control input works after reboot
        assert arm(True).success
        until(lambda: state().armed, 'fresh arming')
        armed_cut = cut()
        refused = request()
        assert not refused.success and refused.result == 2, refused
        assert state().armed and state().mode == 'OFFBOARD'
        assert cut() == armed_cut, 'armed reboot refusal mutated the running vehicle'
        disarmed = request(command=400, param1=0, param2=21196)
        assert disarmed.success and disarmed.result == 0
        until(lambda: not state().armed, 'ordinary force disarm test cleanup')
        assert request().success
        until(lambda: state().mode == 'POSCTL', 'final reboot')
        assert cut() == before
        return {'result': 'PASS', 'accepted': 'disarmed FCU-only reboot',
                'armed_result': refused.result, 'malformed_cases': len(malformed),
                'retained': ['physical pose/twist', 'world clock', 'peer', 'identity/generation',
                             'enabled provider', 'persistent ROS service connection'],
                'fresh_control': True, 'cleanup': 'world process and private ROS master joined'}
    finally:
        command.close()
        pva.unregister()
        for subscription in subscriptions:
            subscription.unregister()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--xsim', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='xsim-fcu-reboot-') as temporary:
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
                        return xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/fcu_reboot_test')[0] == 1
                    except OSError:
                        return False
                until(ready, 'private ROS master')
                rospy.init_node('xsim_fcu_reboot_test', disable_signals=True)
                config = session([robot('uav', 'uav', 'px4_multirotor', [1, -2, 1.5]),
                                  robot('peer', 'peer', 'px4_multirotor', [-3, 4, 2])])
                config['scene']['parameters']['fcuParameters'] = {'MC_ROLLRATE_I': 0.3, 'MPC_XY_VEL_I_ACC': 0.7}
                x = Xsim(args.xsim, root / 'world', config, ros=True, paused=True)
                result = scenario(x, rospy)
                x.stop()
                (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
            finally:
                if x is not None:
                    x.kill()
                    if x.log_path.exists():
                        shutil.copy2(x.log_path, output / 'xsim.log')
                rospy.signal_shutdown('private FCU reboot fixture finished')
                if master.poll() is None:
                    os.killpg(master.pid, signal.SIGTERM)
                    try:
                        master.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(master.pid, signal.SIGKILL)
                        master.wait()
    print('PASS: real ROS FCU reboot, unchanged physical/world/peer state, fresh controls and refusal semantics')


if __name__ == '__main__':
    main()
