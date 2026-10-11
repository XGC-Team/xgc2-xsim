#!/usr/bin/env python3
"""Native startup before ROS, bounded master discovery and Run binding replacement.

The long CTest holds the master down for 60 seconds and measures xsim, including
its native world and HTTP threads. Each invocation owns its port and ROS storage.
"""
import argparse
import json
import os
import random
import signal
import socket
import subprocess
import tempfile
import threading
import time
import uuid
import xmlrpc.client
from pathlib import Path

from world_client import Xsim, until


def free_port():
    while True:
        port = random.randint(20000, 40000)
        with socket.socket() as probe:
            try:
                probe.bind(("127.0.0.1", port))
            except OSError:
                continue
            return port


def resources(pid):
    stat = Path("/proc/%d/stat" % pid).read_text().rsplit(")", 1)[1].split()
    cpu = (int(stat[11]) + int(stat[12])) / os.sysconf("SC_CLK_TCK")
    return cpu, len(list(Path("/proc/%d/task" % pid).iterdir())), len(list(Path("/proc/%d/fd" % pid).iterdir()))


def stop_master(master):
    os.killpg(master.pid, signal.SIGCONT)
    os.killpg(master.pid, signal.SIGTERM)
    try:
        master.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(master.pid, signal.SIGKILL)
        master.wait(timeout=10)


def describe(x):
    code, result = x.call("GET", "/v1/describe?wait_ready_ms=1000", bound=False)
    assert code == 200 and result["ready"] and result["facts"]["frames_flowing"], (code, result)
    return result["facts"]["ros"]


def config():
    return {"epoch_ns": 1700000000000000000, "paused": True, "publish_clock": True,
            "reference_cloud_topic": "/startup_reference",
            "scene": {"obstacles": [{"type": "box", "position": [4, 0, 0], "size": [1, 4, 2]}]},
            "entities": [{"name": "late_robot", "public_id": "late-ugv", "kind": "scout",
                          "sensor": {"backend": "cpu", "h_res": 16, "v_res": 4}}]}


def stalled_master(binary, root):
    """A TCP peer that accepts but never answers must not hold native startup or stop."""
    listener = socket.socket()
    listener.bind(("127.0.0.1", free_port()))
    listener.listen(1)
    listener.settimeout(.1)
    stopping, accepted = threading.Event(), threading.Event()

    def serve():
        while not stopping.is_set():
            try:
                peer, _ = listener.accept()
            except socket.timeout:
                continue
            with peer:
                accepted.set()
                stopping.wait(3)

    thread = threading.Thread(target=serve)
    thread.start()
    x = Xsim(binary, root, config(), ros=True)
    x.env["ROS_MASTER_URI"] = "http://127.0.0.1:%d" % listener.getsockname()[1]
    unused = Xsim(binary, root / "unused", {"epoch_ns": 1700000000000000000, "paused": True}, ros=True)
    unused.env["ROS_MASTER_URI"] = x.env["ROS_MASTER_URI"]
    try:
        unused.start(timeout=3)
        time.sleep(.2)
        assert describe(unused)["connect_attempts"] == 0 and not accepted.is_set(), "unused ROS IO connected eagerly"
        unused.stop()
        started = time.monotonic()
        x.start(timeout=3)
        assert not describe(x)["ready"]
        assert time.monotonic() - started < 2, "native startup waited for a stalled master"
        assert accepted.wait(2), "the worker did not attempt master discovery"
        code, result = x.hold("Describe")
        assert code == 200 and result["robots"] == ["late-ugv"], result
        started = time.monotonic()
        x.stop()
        assert time.monotonic() - started < 2, "master discovery did not stop promptly"
    finally:
        unused.kill()
        x.kill()
        stopping.set()
        thread.join(timeout=5)
        listener.close()
        assert not thread.is_alive()


def late_master(binary, root, port, long_test):
    x = Xsim(binary, root / "xsim", config(), ros=True)
    masters, logs = [], []
    paused_master_pid = None

    def start_master():
        log = (root / ("master-%d.log" % len(masters))).open("w")
        logs.append(log)
        master = subprocess.Popen(["roscore", "-p", str(port)], stdout=log, stderr=subprocess.STDOUT,
                                  start_new_session=True)
        masters.append(master)
        return master

    try:
        started = time.monotonic()
        x.start(timeout=3)
        waiting = describe(x)
        assert not waiting["ready"] and waiting["state"] == "waiting_for_master", waiting
        assert time.monotonic() - started < 2, "native startup waited for the master"
        # Missing optional GPU support refuses only the requested GPU sensor,
        # with the loader's reason visible through the native operation status.
        gpu = {"name": "unavailable_gpu", "kind": "scout", "sensor": {
            "backend": "gpu", "mode": "lidar_scan", "range": 20, "min_range": .2,
            "h_fov_deg": 120, "v_fov_deg": 60, "h_res": 240, "v_res": 120,
            "point_cover_spacing": .1, "rate_hz": 10}}
        code, operation = x.call("POST", "/v1/entities", {"operation_timeout_ms": 4000, "entity": {
            "id": "gpu-refused", "role": "robot", "pose": {"position": [0, 0, 0], "orientation": [0, 0, 0, 1]},
            "asset": {"id": "gpu-test", "realization": {
                "media_type": "application/vnd.xgc2.xsim.entity+json", "content": gpu}}}})
        assert code in (200, 201, 202), (code, operation)
        if operation["state"] in ("accepted", "running"):
            code, operation = x.call("POST", "/v1/operations/" + operation["id"] + "/wait", {}, timeout_ms=6000)
            assert code == 200, (code, operation)
        assert operation["state"] == "failed" and "GPU module not found" in operation["error"]["message"], operation
        assert "no CPU fallback" in operation["error"]["message"]
        assert x.hold("Describe")[1]["robots"] == ["late-ugv"]
        # Native robot controls stay available, with only the latest desired ROS state retained.
        x.enable("late-ugv")
        x.enable("late-ugv", False)
        x.enable("late-ugv")
        code, held = x.hold("Engage", {"robot_ids": ["late-ugv"]})
        assert code == 200, held
        state = x.hold_state("late-ugv")["late-ugv"]
        x.release("late-ugv", state["revision"])
        time.sleep(.2)  # Let completed HTTP connections retire before the resource baseline.
        before_ros = describe(x)
        before = resources(x.process.pid)
        window = 60 if long_test else 1
        wall = time.monotonic()
        time.sleep(window)
        elapsed = time.monotonic() - wall
        after = resources(x.process.pid)
        after_ros = describe(x)
        assert not after_ros["ready"] and x.process.poll() is None, after_ros
        assert after[1] == before[1] and after[2] <= before[2] + 1, (before, after)
        lines = x.log_path.read_text().splitlines()
        if long_test:
            cpu = after[0] - before[0]
            attempts = after_ros["connect_attempts"] - before_ros["connect_attempts"]
            print(json.dumps({"peer_down_seconds": elapsed, "cpu_seconds": cpu, "connect_attempts": attempts,
                              "outage_log_lines": len(lines), "threads_before_after": [before[1], after[1]],
                              "fds_before_after": [before[2], after[2]]}), flush=True)
            assert elapsed >= 60, elapsed
            assert attempts <= 16, attempts
            assert len(lines) <= 7, lines
        master = start_master()
        bound = until(lambda: (value if value["ready"] else False) if (value := describe(x)) else False,
                      "late master binding", 35, .1)
        assert bound["master_pid"] > 0 and bound["master_run_id"], bound
        assert bound["state_replays"] == bound["applied_revision"] == bound["desired_revision"] == 1, bound
        assert bound["entity_bindings"] == 1, bound
        if long_test:
            # The world keeps simulating during the outage, so the outage may cost no more CPU per second than the
            # same process spends idle with a bound master (an absolute bound would measure the runner, not the wait).
            idle_start = resources(x.process.pid)[0]
            time.sleep(10)
            idle_rate = (resources(x.process.pid)[0] - idle_start) / 10
            assert cpu / elapsed <= idle_rate * 1.5 + 0.02, (cpu, elapsed, idle_rate)

        # Topic peers start after xsim: roscpp publisher updates establish their connections.
        import rospy
        from geometry_msgs.msg import PoseStamped, Twist
        from sensor_msgs.msg import PointCloud2
        from rosgraph_msgs.msg import Clock

        rospy.init_node("xsim_start_order_test", disable_signals=True)
        received = {"reference": [], "pose": [], "cloud": [], "clock": []}
        lock = threading.Lock()

        def record(name):
            def callback(message):
                with lock:
                    received[name].append(message)
                    del received[name][:-2]
            return callback

        subscriptions = [rospy.Subscriber(topic, message_type, record(name), queue_size=1)
                         for name, topic, message_type in
                         [("reference", "/startup_reference", PointCloud2), ("pose", "/late_robot/pose", PoseStamped),
                          ("cloud", "/late_robot/cloud", PointCloud2), ("clock", "/clock", Clock)]]
        driver = rospy.Publisher("/late_robot/cmd_vel", Twist, queue_size=1)
        until(lambda: received["reference"], "one latched frozen-scene replay")
        until(lambda: driver.get_num_connections() == 1, "late command publisher")
        x.operation("POST", "/v1/world/resume")
        until(lambda: all(received.values()), "clock, CPU sensing and pose after late binding")
        command = Twist()
        command.linear.x = .4
        driver.publish(command)
        until(lambda: received["pose"][-1].pose.position.x > .01, "late command input")
        x.operation("POST", "/v1/world/pause")
        time.sleep(.2)
        assert len(received["reference"]) == 1, "frozen scene was replayed more than once"
        assert describe(x)["state_replays"] == 1
        for subscription in subscriptions:
            subscription.unregister()
        driver.unregister()
        rospy.signal_shutdown("start-order topic checks complete")

        # A stopped master can return with the same identity. Apply only the
        # latest desired command subscription state, without another scene replay.
        paused_master_pid = bound["master_pid"]
        os.kill(paused_master_pid, signal.SIGSTOP)
        until(lambda: not describe(x)["ready"], "temporarily stopped master", 15, .1)
        down_resources = resources(x.process.pid)
        x.enable("late-ugv", False)
        x.enable("late-ugv")
        os.kill(paused_master_pid, signal.SIGCONT)
        paused_master_pid = None
        until(lambda: describe(x)["ready"], "same master returns", 35, .1)
        assert describe(x)["state_replays"] == 1
        assert resources(x.process.pid)[1] <= down_resources[1] + 2, "publication threads grew after recovery"

        # A disappearance reports peer status while the native world stays usable.
        stop_master(master)
        unavailable = until(lambda: value if not (value := describe(x))["ready"] else False,
                            "master disappearance", 15, .1)
        assert unavailable["state"] == "master_unavailable", unavailable
        assert x.hold("Describe")[0] == 200
        replacement = start_master()
        until(lambda: x.process.poll() is not None, "replaced master ends xsim", 35, .1)
        assert x.process.wait(timeout=5) == 0
        assert not Path(x.endpoint).exists()
        assert x.log_path.read_text().count("ROS master identity changed") == 1
        x.log.close()

        # Run-id replacement on the same PID must also end a newly bound xsim.
        second = Xsim(binary, root / "same-pid", config(), ros=True)
        try:
            second.start()
            until(lambda: describe(second)["ready"], "second master binding", 35, .1)
            proxy = xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"])
            proxy.setParam("/start_order_test", "/run_id", str(uuid.uuid4()))
            until(lambda: second.process.poll() is not None, "changed run id ends xsim", 15, .1)
            assert second.process.wait(timeout=5) == 0 and not Path(second.endpoint).exists()
        finally:
            second.kill()
        assert replacement.poll() is None
    finally:
        if paused_master_pid is not None:
            os.kill(paused_master_pid, signal.SIGCONT)
        x.kill()
        for master in masters:
            if master.poll() is None:
                stop_master(master)
            else:
                master.wait(timeout=5)
        for log in logs:
            log.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--xsim", required=True)
    parser.add_argument("--long", action="store_true")
    args = parser.parse_args()
    definition = json.loads((Path(__file__).resolve().parents[1] / "process-definitions/xsim-world.json").read_text())["definitions"][0]
    assert definition["readiness"]["kind"] == "describe"
    uri = definition["parameters"]["properties"]["rosMasterUri"]
    assert uri["fixedOnly"] and "default" not in uri and "output" not in uri
    assert "rosMasterUri" in definition["parameters"]["required"]
    with tempfile.TemporaryDirectory(prefix="xsim-start-order-") as directory:
        root = Path(directory)
        port = free_port()
        for name in ("ros-home", "ros-log"):
            (root / name).mkdir(mode=0o700)
        os.environ.update(ROS_MASTER_URI="http://127.0.0.1:%d" % port, ROS_IP="127.0.0.1", ROS_HOSTNAME="127.0.0.1",
                          ROS_HOME=str(root / "ros-home"), ROS_LOG_DIR=str(root / "ros-log"),
                          XSIM_LIDAR_GPU_MODULE="/nonexistent/xsim-lidar-gpu.so")
        if not args.long:
            stalled_master(args.xsim, root / "stalled")
        late_master(args.xsim, root, port, args.long)
    print("PASS: ROS start order, native readiness and master replacement")


if __name__ == "__main__":
    main()
