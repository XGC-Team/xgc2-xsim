"""One xsim process behind a private bootstrap, driven through its v1 management service.

Shared by the native and ROS integration tests: process start and stop, the XRPC request headers, operations
that are awaited to their terminal state, and capability calls (chassis HOLD).
"""
import http.client
import json
import os
import signal
import subprocess
import time
import uuid
from pathlib import Path

from native_http import UnixHTTP

HOLD = "xgc2.chassis.hold"


def write_private(path, value):
    path.write_text(json.dumps(value))
    path.chmod(0o600)


class Xsim:
    def __init__(self, binary, root, config, ros=False):
        self.binary = str(Path(binary).resolve())
        self.root = Path(root)
        self.root.mkdir(mode=0o700, exist_ok=True)
        self.config, self.ros = config, ros
        self.endpoint = str(self.root / "world.sock")
        self.instance = None
        self.process = None
        self.log = None
        self.log_path = self.root / "xsim.log"
        binding = {"schema_version": 1, "target_id": "test:world", "service": "xgc2.simulation",
                   "api_version": "v1", "profile": "http.v1", "endpoint": {"kind": "unix", "address": self.endpoint},
                   "runtime_grant": "test:runtime", "authentication": "local_private",
                   "secret_handles": {}, "storage_grants": []}
        bootstrap = {"schema_version": 1, "binding": binding, "grants": {}}
        self.env = dict(os.environ)
        if ros:
            for name in ("ros-home", "ros-log"):
                (self.root / name).mkdir(mode=0o700, exist_ok=True)
            binding["storage_grants"] = ["test:ros-home", "test:ros-log"]
            bootstrap["application"] = {"rosHomeGrant": "test:ros-home", "rosLogGrant": "test:ros-log"}
            self.env.update(ROS_HOME=str(self.root / "ros-home"), ROS_LOG_DIR=str(self.root / "ros-log"))
        self.bootstrap = self.root / "bootstrap.json"
        write_private(self.bootstrap, bootstrap)
        self.world = self.root / "world.json"
        write_private(self.world, config)

    def start(self, timeout=15):
        """Start the process and return its first unbound describe (the discovery call)."""
        assert self.process is None or self.process.poll() is not None
        self.log = self.log_path.open("ab")
        self.process = subprocess.Popen([self.binary, "--bootstrap-input", str(self.bootstrap), "--config", str(self.world)],
                                        stdout=self.log, stderr=subprocess.STDOUT, env=self.env)
        self.instance = None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert self.process.poll() is None, self.log_path.read_text(errors="replace")
            try:
                status, described = self.call("GET", "/v1/describe", bound=False)
                if status == 200:
                    self.instance = described["instance_id"]
                    return described
            except (OSError, http.client.HTTPException):
                pass
            time.sleep(0.01)
        raise AssertionError("xsim did not answer describe\n" + self.log_path.read_text(errors="replace"))

    def stop(self):
        """SIGTERM; the process must exit cleanly and remove its endpoint."""
        self.process.send_signal(signal.SIGTERM)
        assert self.process.wait(timeout=15) == 0, self.log_path.read_text(errors="replace")
        assert not os.path.exists(self.endpoint)
        self.log.close()

    def kill(self):
        if self.process is not None and self.process.poll() is None:
            self.process.kill()
            self.process.wait(timeout=10)
        if self.log is not None and not self.log.closed:
            self.log.close()

    def call(self, method, target, body=None, bound=True, instance=None, timeout_ms=3000):
        """One request with the XRPC headers; returns (status, parsed body)."""
        headers = {"Content-Type": "application/json", "X-Xrpc-Timeout-Ms": str(timeout_ms),
                   "X-Request-ID": str(uuid.uuid4())}
        if bound and (instance or self.instance):
            headers["X-Xrpc-Instance-ID"] = instance or self.instance
        client = UnixHTTP(self.endpoint)
        try:
            try:
                client.request(method, target,
                               None if body is None else (body if isinstance(body, str) else json.dumps(body)), headers)
            except (BrokenPipeError, ConnectionResetError):
                pass  # the host answered from the headers (an instance mismatch) and closed before the body
            response = client.getresponse()
            data = response.read()
            if bound and (instance or self.instance) and response.status != 409:
                assert response.getheader("X-Xrpc-Instance-ID") == (instance or self.instance)
            return response.status, json.loads(data) if data else None
        finally:
            client.close()

    def operation(self, method, target, body=None):
        """A mutation awaited to its terminal state; it must have succeeded."""
        body = dict(body or {})
        body.setdefault("operation_timeout_ms", 4000)
        status, operation = self.call(method, target, body)
        assert status in (200, 201, 202), (status, operation)
        if operation["state"] in ("accepted", "running"):
            status, operation = self.call("POST", "/v1/operations/" + operation["id"] + "/wait", {}, timeout_ms=6000)
            assert status == 200, (status, operation)
        assert operation["state"] == "succeeded", operation
        return operation

    def ref(self, public_id):
        status, entity = self.call("GET", "/v1/entities/" + public_id)
        assert status == 200, (status, entity)
        return entity["ref"]

    def enable(self, public_id, enabled=True):
        self.operation("POST", "/v1/entities/%s/state" % public_id,
                       {"generation": self.ref(public_id)["generation"], "state": {"enabled": enabled}})

    def capability(self, service, method, body=None, instance=None):
        """A method of a capability the world serves: POST /v1/call/<service>/<Method> with the JSON request."""
        return self.call("POST", "/v1/call/%s/%s" % (service, method), {} if body is None else body, instance=instance)

    def hold(self, method, body=None, instance=None):
        """A method of the chassis HOLD capability: Describe, State, Engage or Release."""
        return self.capability(HOLD, method, body, instance)

    def hold_state(self, *robot_ids):
        status, state = self.hold("State", {"robot_ids": list(robot_ids)} if robot_ids else {})
        assert status == 200, (status, state)
        return {robot["robot_id"]: robot for robot in state["robots"]}

    def release(self, robot_id, revision, instance=None):
        status, reply = self.hold("Release", {"expected_instance": instance or self.instance,
                                              "changes": [{"robot_id": robot_id, "expected_revision": revision}]})
        assert status == 200, (status, reply)
        assert len(reply["robots"]) == 1
        return reply["robots"][0]

    def chassis(self):
        """The entities the world lists for the chassis HOLD capability in its describe facts."""
        status, described = self.call("GET", "/v1/describe")
        assert status == 200, (status, described)
        listed = [c for c in described["facts"]["capabilities"] if c["name"] == HOLD]
        assert len(listed) == 1, described["facts"]
        return listed[0]["entities"]


def until(predicate, label, seconds=10, interval=0.01):
    deadline = time.monotonic() + seconds
    while True:
        result = predicate()
        if result:
            return result
        if time.monotonic() >= deadline:
            raise AssertionError("timeout: " + label)
        time.sleep(interval)
