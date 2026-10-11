#!/usr/bin/env python3
"""Chassis HOLD with ROS IO on: two Scouts and a Mecanum driven over cmd_vel under a private ROS master.

One held robot stops (zero written, then stopped) while the others keep moving and its own sensors keep
publishing. A release does not replay what was commanded during the hold. Engage and release work while the
world is paused. A held robot that is removed and created again is still held, and a restarted xsim is a new
instance that rejects the old one's release. Also checks the readiness facts of the ROS-bound service.
Needs a ROS Noetic environment (roscore, rospy); never uses the default master port or ROS_HOME.
"""
import argparse
import os
import random
import signal
import socket
import subprocess
import tempfile
import threading
import time
import xmlrpc.client
from pathlib import Path

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, PointCloud2

from world_client import HOLD, Xsim, until, write_private

UNIT = "application/vnd.xgc2.xsim.entity+json"
SPEED = 0.4  # m/s forward
ROBOTS = {"ugv-a": "scout_a", "ugv-b": "scout_b", "ugv-m": "mec_m"}  # public ID -> ROS namespace


def free_port():
    while True:
        port = random.randint(20000, 40000)
        with socket.socket() as probe:
            try:
                probe.bind(("127.0.0.1", port))
            except OSError:
                continue
            return port


def world_config():
    scout = lambda public_id, y: {"name": ROBOTS[public_id], "public_id": public_id, "kind": "scout", "position": [0, y, 0]}
    first = scout("ugv-a", 0)
    first["sensor"] = {"backend": "cpu", "rate_hz": 20, "h_res": 72, "v_res": 16, "h_fov_deg": 90, "v_fov_deg": 30}
    return {"epoch_ns": 1700000000000000000, "model_step_ns": 2000000, "output_period_ns": 8000000, "paused": False,
            "publish_clock": True, "scene": {"obstacles": [{"type": "box", "position": [6, 0, 0], "size": [1, 8, 4]}]},
            "entities": [first, scout("ugv-b", 3),
                         {"name": "mec_m", "public_id": "ugv-m", "kind": "mecanum", "position": [0, -3, 0]}]}


class Feed:
    """The latest message and the message count of one topic."""

    def __init__(self, topic, message_type):
        self.lock = threading.Lock()
        self.count, self.last = 0, None
        self.subscriber = rospy.Subscriber(topic, message_type, self._record, queue_size=1000)

    def _record(self, message):
        with self.lock:
            self.count += 1
            self.last = message

    def snapshot(self):
        with self.lock:
            return self.count, self.last

    def x(self):
        return self.snapshot()[1].pose.position.x

    def fresh_x(self):
        """x of a message that arrived after this call."""
        before = self.snapshot()[0]
        until(lambda: self.snapshot()[0] >= before + 5, "fresh messages", 10)
        return self.x()


class Driver(threading.Thread):
    """Publishes the same forward cmd_vel at 20 Hz to every robot in `active`."""

    def __init__(self):
        super().__init__(daemon=True)
        self.publishers = {name: rospy.Publisher("/%s/cmd_vel" % name, Twist, queue_size=1) for name in ROBOTS.values()}
        self.message = Twist()
        self.message.linear.x = SPEED
        self.active, self.lock, self.stopping = set(ROBOTS.values()), threading.Lock(), False

    def drive(self, name, on=True):
        with self.lock:
            (self.active.add if on else self.active.discard)(name)

    def run(self):
        while not self.stopping:
            with self.lock:
                names = list(self.active)
            for name in names:
                self.publishers[name].publish(self.message)
            time.sleep(0.05)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--xsim", required=True)
    parser.add_argument("--slow-service-test")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="xsim-hold-ros-") as directory:
        root = Path(directory)
        root.chmod(0o700)
        port = free_port()
        os.environ.update(ROS_MASTER_URI="http://127.0.0.1:%d" % port, ROS_IP="127.0.0.1", ROS_HOSTNAME="127.0.0.1",
                          ROS_HOME=str(root / "test-ros-home"), ROS_LOG_DIR=str(root / "test-ros-log"))
        for name in ("test-ros-home", "test-ros-log"):
            (root / name).mkdir(mode=0o700)
        xsim = Xsim(args.xsim, root / "xsim", world_config(), ros=True)
        driver = None
        with (root / "roscore.log").open("w") as log:
            master = subprocess.Popen(["roscore", "-p", str(port)], stdout=log, stderr=subprocess.STDOUT,
                                      start_new_session=True)
            try:
                proxy = xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"])

                def master_up():
                    try:
                        return proxy.getPid("/hold_test")[0] == 1
                    except OSError:
                        return False

                until(master_up, "private ROS master", 30)
                rospy.init_node("xsim_hold_test", disable_signals=True)
                driver = Driver()
                scenario(xsim, driver)
                if args.slow_service_test:
                    result = subprocess.run([args.slow_service_test], capture_output=True, text=True, timeout=30)
                    assert result.returncode == 0, result.stdout + result.stderr
            finally:
                if driver is not None:
                    driver.stopping = True
                xsim.kill()
                rospy.signal_shutdown("test finished")
                os.killpg(master.pid, signal.SIGTERM)
                try:
                    master.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(master.pid, signal.SIGKILL)
    print("PASS: chassis HOLD with ROS IO")


def scenario(x, driver):
    # ---- readiness: the facts of a ROS-bound service
    described = x.start()
    assert described["facts"]["ros_master_uri"] == os.environ["ROS_MASTER_URI"], described["facts"]
    assert described["facts"]["world_generation"] == 1
    status, ready = x.call("GET", "/v1/describe?wait_ready_ms=10000")
    assert status == 200 and ready["ready"] is True and ready["facts"]["frames_flowing"] is True, ready
    assert ready["facts"]["capabilities"] == [{"name": HOLD, "entities": ["ugv-a", "ugv-b", "ugv-m"]}], ready["facts"]
    status, roster = x.hold("Describe")
    assert status == 200 and roster["robots"] == ["ugv-a", "ugv-b", "ugv-m"], roster

    pose = {name: Feed("/%s/pose" % name, PoseStamped) for name in ROBOTS.values()}
    imu = Feed("/scout_a/imu/data_raw", Imu)
    cloud = Feed("/scout_a/cloud", PointCloud2)
    for public_id in ROBOTS:
        x.enable(public_id)
    until(lambda: all(feed.snapshot()[1] for feed in pose.values()), "localization of every robot")
    until(lambda: all(p.get_num_connections() for p in driver.publishers.values()), "cmd_vel subscribers")
    driver.start()

    def moved(name, seconds=0.3):
        start = pose[name].x()
        time.sleep(seconds)
        return pose[name].x() - start

    # ---- everything drives
    for name in pose:
        until(lambda: moved(name) > 0.05, name + " drives", 10)

    # ---- engage one: zero written, then stopped; the others keep moving; its sensors keep publishing
    status, engaged = x.hold("Engage", {"robot_ids": ["ugv-a"]})
    assert status == 200 and engaged["robots"][0]["outcome"] == "engaged", engaged
    assert engaged["robots"][0]["held"] is True and engaged["robots"][0]["revision"] == 1, engaged
    until(lambda: x.hold_state("ugv-a")["ugv-a"]["stage"] in ("zero_written", "stopped"), "zero written")
    until(lambda: x.hold_state("ugv-a")["ugv-a"]["stage"] == "stopped", "stopped (its own twist at rest)", 10)
    held = x.hold_state("ugv-a")["ugv-a"]
    assert held["stopped_at_ms"] >= held["zero_written_at_ms"] >= held["gated_at_ms"], held
    def refused():
        return x.call("GET", "/v1/world")[1]["diagnostics"]["hold_refused_commands"]

    before = {"pose": pose["scout_a"].snapshot()[0], "imu": imu.snapshot()[0], "cloud": cloud.snapshot()[0]}
    b_x, m_x, a_x, refusals = pose["scout_b"].x(), pose["mec_m"].x(), pose["scout_a"].x(), refused()
    time.sleep(1.5)  # the driver keeps commanding the held robot; HOLD refuses every command
    assert refused() - refusals >= 10, "HOLD did not refuse the commands of the held robot"
    assert abs(pose["scout_a"].x() - a_x) < 0.01, "held robot moved: %f" % (pose["scout_a"].x() - a_x)
    published = {"pose": pose["scout_a"].snapshot()[0] - before["pose"], "imu": imu.snapshot()[0] - before["imu"],
                 "cloud": cloud.snapshot()[0] - before["cloud"]}
    assert published["pose"] >= 60 and published["imu"] >= 15 and published["cloud"] >= 8, published
    assert pose["scout_b"].x() - b_x > 0.15 and pose["mec_m"].x() - m_x > 0.15, "the other robots keep moving"
    assert x.hold_state("ugv-b", "ugv-m")["ugv-b"]["held"] is False
    assert x.call("GET", "/v1/entities/ugv-a")[1]["state"]["enabled"] is True

    # ---- release: compare and set, and nothing commanded during the hold comes back
    assert x.release("ugv-a", 9)["outcome"] == "conflict"
    assert x.release("ugv-a", 1, instance="f" * 32)["outcome"] == "conflict"
    driver.drive("scout_a", False)
    time.sleep(0.3)  # the commands of the hold were all refused, none is left in flight
    released = x.release("ugv-a", 1)
    assert released["outcome"] == "released" and released["revision"] == 2 and released["held"] is False, released
    a_x = pose["scout_a"].x()
    time.sleep(0.8)
    assert abs(pose["scout_a"].x() - a_x) < 0.01, "a command of the hold was replayed"
    driver.drive("scout_a")
    until(lambda: moved("scout_a") > 0.05, "a fresh command drives the released robot", 10)

    # ---- a paused world: the state changes at once and the zero follows at the next boundary
    driver.drive("scout_a", False)  # no other command may wake the paused world: only the engage can
    driver.drive("mec_m", False)
    x.operation("POST", "/v1/world/pause")
    steps = x.call("GET", "/v1/world")[1]["steps"]
    status, engaged = x.hold("Engage", {"robot_ids": ["ugv-b"]})
    assert status == 200 and engaged["robots"][0]["held"] is True and engaged["robots"][0]["revision"] == 1, engaged
    until(lambda: x.hold_state("ugv-b")["ugv-b"]["stage"] in ("zero_written", "stopped"), "zero written while paused")
    assert x.call("GET", "/v1/world")[1]["steps"] == steps, "HOLD does not advance a paused world"
    x.operation("POST", "/v1/world/resume")
    b_x = pose["scout_b"].x()
    time.sleep(1.0)
    assert abs(pose["scout_b"].x() - b_x) < 0.03, "the zero written while paused was lost"
    x.operation("POST", "/v1/world/pause")
    released = x.release("ugv-b", 1)
    assert released["outcome"] == "released" and released["revision"] == 2, released
    assert x.hold_state("ugv-b")["ugv-b"]["held"] is False
    time.sleep(0.3)  # the cmd_vel stream is admitted again and waits in the paused world
    x.operation("POST", "/v1/world/resume")
    until(lambda: moved("scout_b") > 0.05, "released robot drives after the resume", 10)
    driver.drive("scout_a")
    driver.drive("mec_m")

    # ---- a held robot that is removed and created again is still held
    assert x.hold("Engage", {"robot_ids": ["ugv-m"]})[1]["robots"][0]["revision"] == 1
    until(lambda: x.hold_state("ugv-m")["ugv-m"]["stage"] in ("zero_written", "stopped"), "mecanum zero")
    x.operation("DELETE", "/v1/entities/ugv-m", {"generation": x.ref("ugv-m")["generation"]})
    status, gone = x.hold("State", {"robot_ids": ["ugv-m"]})
    assert status == 404 and gone["error"]["details"]["robot_ids"] == ["ugv-m"], gone
    assert x.chassis() == ["ugv-a", "ugv-b"]
    assert x.hold("Describe")[1]["robots"] == ["ugv-a", "ugv-b"]
    x.operation("POST", "/v1/entities", {"entity": {
        "id": "ugv-m", "role": "robot", "pose": {"position": [0, -3, 0], "orientation": [0, 0, 0, 1]},
        "asset": {"id": "mecanum-test", "realization": {"media_type": UNIT, "content": {"kind": "mecanum", "name": "mec_m"}}}}})
    again = x.hold_state("ugv-m")["ugv-m"]
    assert again["held"] is True and again["revision"] == 1, again
    x.enable("ugv-m")
    until(lambda: x.hold_state("ugv-m")["ugv-m"]["stage"] in ("zero_written", "stopped"), "zero on the new model")
    until(lambda: driver.publishers["mec_m"].get_num_connections(), "cmd_vel subscriber of the new entity")
    m_x = pose["mec_m"].fresh_x()
    time.sleep(1.0)
    assert abs(pose["mec_m"].x() - m_x) < 0.01, "the re-created robot moved under HOLD"
    assert x.release("ugv-m", 1)["outcome"] == "released"
    until(lambda: moved("mec_m") > 0.05, "released mecanum drives", 10)

    # ---- restart: a new instance, no persisted HOLD state, the old instance's release is rejected
    x.hold("Engage", {"robot_ids": ["ugv-a"]})
    old = x.instance
    x.stop()
    x.start()
    assert x.instance != old
    status, fresh = x.hold("State")
    assert status == 200 and fresh["instance"] == x.instance, fresh
    assert all(r["held"] is False and r["revision"] == 0 for r in fresh["robots"]), fresh
    stale = x.hold("Release", {"expected_instance": old, "changes": [{"robot_id": "ugv-a", "expected_revision": 1}]})
    assert stale[0] == 200 and stale[1]["robots"][0]["outcome"] == "conflict", stale
    x.hold("Engage", {"robot_ids": ["ugv-a"]})
    assert x.release("ugv-a", 1, instance=old)["outcome"] == "conflict"
    assert x.hold_state("ugv-a")["ugv-a"]["held"] is True
    x.stop()

    # ---- a command that waits for the input thread during the hold is not replayed by the release
    # A slow input poll keeps the cmd_vel callbacks queued for seconds. The release has to run them while the
    # robot is still held: a callback that ran after it would stamp the command as received after the release.
    for name in ROBOTS.values():
        driver.drive(name, False)
    x.config["input_poll_ns"] = 3000000000
    write_private(x.world, x.config)
    for attempt in range(5):
        x.start()
        x.enable("ugv-a")
        until(lambda: driver.publishers["scout_a"].get_num_connections(), "cmd_vel subscriber")
        a_x = pose["scout_a"].fresh_x()
        assert x.hold("Engage", {"robot_ids": ["ugv-a"]})[1]["robots"][0]["held"] is True
        before = refused()
        driver.publishers["scout_a"].publish(driver.message)
        time.sleep(0.4)  # the message is in the callback queue of the input thread
        if refused() != before:  # an input poll ran it while held: the window did not open, try again
            x.stop()
            continue
        assert x.release("ugv-a", 1)["outcome"] == "released"
        assert refused() > before, "the release did not run the queued callback while the robot was held"
        time.sleep(4.0)  # more than one input poll after the release
        assert abs(pose["scout_a"].x() - a_x) < 0.01, "a command received during the hold was replayed"
        x.stop()
        break
    else:
        raise AssertionError("an input poll interrupted every attempt")


if __name__ == "__main__":
    main()
