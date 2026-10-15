#!/usr/bin/env python3
"""Exercise the actual native server through its private XRPC Unix endpoint."""
import argparse
import copy
import http.client
import json
import math
import os
import signal
import socket
import subprocess
import tempfile
import threading
import time
import uuid
from pathlib import Path


class UnixHTTP(http.client.HTTPConnection):
    def __init__(self, path, timeout=6):
        super().__init__("localhost", timeout=timeout)
        self.path = path

    def connect(self):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect(self.path)


def write_private(path, value):
    path.write_text(json.dumps(value))
    path.chmod(0o600)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", required=True)
    parser.add_argument("--test-host", action="store_true",
                        help="validate native domain via the explicit temporary host, not the production launcher")
    parser.add_argument("--output")
    args = parser.parse_args()
    binary = str(Path(args.binary).resolve())
    evidence = {"launcher": "temporary native test host" if args.test_host else "direct private Unix http.v1",
                "checks": []}
    with tempfile.TemporaryDirectory(prefix="xsim-native-v1-") as directory:
        root = Path(directory)
        root.chmod(0o700)
        endpoint = str(root / "world.sock")
        configuration = root / "world.json"
        frozen = json.loads((Path(__file__).parent / "experiment-fixtures" / "empty-world.input.json").read_text())
        frozen['scene']['parameters'] = {'modelStepMs': 1, 'outputPeriodMs': 4, 'commandPollMs': 1}
        write_private(configuration, frozen)
        process = None
        instance = None
        log_path = root / "server.log"
        log = log_path.open("wb")

        def call(method, target, body=None, request_id=None, bound=True, timeout_ms=3000):
            headers = {"Content-Type": "application/json", "X-Xrpc-Timeout-Ms": str(timeout_ms),
                       "X-Request-ID": request_id or str(uuid.uuid4())}
            if bound and instance:
                headers["X-Xrpc-Instance-ID"] = instance
            client = UnixHTTP(endpoint)
            try:
                client.request(method, target, None if body is None else json.dumps(body), headers)
                response = client.getresponse()
                result = json.loads(response.read())
                if bound and instance and response.status != 409:
                    assert response.getheader("X-Xrpc-Instance-ID") == instance
                return response.status, result
            finally:
                client.close()

        def start():
            nonlocal process, instance
            command = [binary, "--experiment-stdin", "--run-id", "native-fixture:run", "--socket", endpoint, "--target-id", "native-fixture:world"]
            process = subprocess.Popen(command, stdin=subprocess.PIPE,
                                       stdout=log, stderr=subprocess.STDOUT)
            process.stdin.write(configuration.read_bytes())
            process.stdin.close()
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise AssertionError(log_path.read_text(errors="replace"))
                try:
                    code, description = call("GET", "/v1/describe?wait_ready_ms=5000", bound=False, timeout_ms=6000)
                    if code == 200:
                        instance = description["instance_id"]
                        code, health = call("GET", "/v1/health")
                        if code == 200 and health["state"] == "ready":
                            mutation("POST", "/v1/world/pause", {})
                            return description, health
                except (OSError, http.client.HTTPException):
                    pass
                time.sleep(0.01)
            raise AssertionError("native readiness did not complete")

        def await_operation(code, operation):
            assert code in (200, 201, 202), (code, operation)
            if operation["state"] in ("accepted", "running"):
                code, operation = call("POST", "/v1/operations/" + operation["id"] + "/wait", {}, timeout_ms=5000)
                assert code == 200, (code, operation)
            return operation

        def mutation(method, target, body, request_id=None):
            body = copy.deepcopy(body)
            body.setdefault("operation_timeout_ms", 3000)
            operation = await_operation(*call(method, target, body, request_id))
            assert operation["state"] == "succeeded", operation
            assert operation["effects"]["applied"] is True
            return operation

        def result_entity(operation, public_id):
            return next(item for item in operation["result"]["entities"] if item["ref"]["id"] == public_id)

        def state(public_id):
            code, result = call("GET", "/v1/entities/" + public_id)
            assert code == 200, result
            return result["state"]

        try:
            description, health = start()
            assert set(description) == {"service", "api_version", "instance_id", "ready", "facts"}, description
            code, immediate = call("GET", "/v1/describe", bound=False)
            assert code == 200 and set(immediate) == set(description), immediate
            assert immediate["facts"]["simulation"] == description["facts"]["simulation"]
            assert description["facts"]["capabilities"] == [{"name": "xgc2.chassis.hold", "entities": []}]
            ref = description["facts"]["service_ref"]
            assert ref["api_version"] == "v1" and ref["target_id"] == "native-fixture:world"
            assert ref["service"] == "xgc2.simulation" and ref["endpoint"]["address"] == endpoint
            assert ref["instance_id"] == description["instance_id"] == immediate["instance_id"]
            simulation = description["facts"]["simulation"]
            assert set(simulation) == {"describe_path", "engine", "capabilities", "limits", "world_frame",
                                       "artifact_types", "asset_schemas"}, simulation
            assert simulation["describe_path"] == "/v1/describe" and simulation["engine"] == {"name": "xsim"}
            assert simulation["capabilities"]["world.reset"] == {
                "all_entities": True, "entities": True, "reset_time": False, "atomic": True}
            assert simulation["capabilities"]["entities.set_state"] == {"enabled": True, "pose": False, "twist": False}
            assert simulation["limits"]["operations"] == 256
            assert simulation["limits"]["inflight_operations"] == 32
            assert simulation["limits"]["operation_payload_bytes"] == 16 * 1024 * 1024
            assert simulation["world_frame"] == {"id": "world", "axes": "right-handed ENU", "gravity": [0, 0, -9.81]}
            assert simulation["artifact_types"] == ["application/vnd.xgc2.xsim.world+json",
                                                    "application/vnd.xgc2.xsim.entity+json"]
            assert simulation["asset_schemas"] == {"application/vnd.xgc2.xsim.entity+json": {
                "kinds": ["fs150", "scout", "mecanum"], "initial_orientation": "yaw-only"}}
            assert call("GET", "/v1/world", bound=False)[0] == 409
            assert call("GET", "/status")[0] == 404
            assert call("POST", "/v1/health/observe", {"after_revision": 0})[1]["state"] == "ready"
            # The shared HTTP host cancels an admitted held reply by closing
            # its transport at the caller deadline; pre-admission expiry may
            # instead return the explicit deadline response.
            before_timeout = time.monotonic()
            try:
                assert call("POST", "/v1/health/observe", {"after_revision": health["revision"]}, timeout_ms=40)[0] == 504
            except http.client.RemoteDisconnected:
                pass
            assert time.monotonic() - before_timeout < 1
            assert call("GET", "/v1/health")[1]["state"] == "ready"
            evidence["checks"].append("strict five-field describe, typed capabilities, simulation facts, formal identity, declared bounds, real health and held observation deadline")

            create = {"entity": {"id": "frozen-id.alpha", "role": "robot",
                "pose": {"position": [1, 2, 3], "orientation": [0, 0, 0, 1]},
                "asset": {"id": "fs150-native", "realization": {
                    "media_type": "application/vnd.xgc2.xsim.entity+json",
                    "content": {"kind": "fs150", "name": "private_model_name"}}}}, "operation_timeout_ms": 3000}
            first = mutation("POST", "/v1/entities", create, "create-first")
            first_entity = result_entity(first, "frozen-id.alpha")
            entity_ref = first_entity["ref"]
            assert entity_ref["generation"] > 0 and first_entity["state"]["enabled"] is False
            assert call("GET", "/v1/entities")[1]["entities"][0]["ref"] == entity_ref
            assert call("GET", "/v1/entities/frozen-id.alpha/sensors")[0] == 404
            path = "/v1/entities/frozen-id.alpha"
            generation = {"generation": entity_ref["generation"]}
            mutation("POST", path + "/state", {**generation, "state": {"enabled": True}})
            step = mutation("POST", "/v1/world/step", {"steps": 40})
            moved = result_entity(step, "frozen-id.alpha")["state"]
            assert moved["pose"]["position"][2] < 3, moved
            before_toggle = copy.deepcopy(moved["pose"])
            disabled = mutation("POST", path + "/state", {**generation, "state": {"enabled": False}})
            assert result_entity(disabled, "frozen-id.alpha")["state"]["pose"] == before_toggle
            enabled = mutation("POST", path + "/state", {**generation, "state": {"enabled": True}})
            assert result_entity(enabled, "frozen-id.alpha")["state"]["pose"] == before_toggle
            before_reset = call("GET", "/v1/world")[1]
            reset = mutation("POST", path + "/reset", generation)
            reset_entity = result_entity(reset, "frozen-id.alpha")
            assert reset_entity["ref"] == entity_ref and reset_entity["state"]["enabled"] is True
            assert reset_entity["state"]["pose"]["position"] == [1, 2, 3]
            assert reset["result"]["time"] == before_reset["time"]
            assert call("POST", path + "/state", {**generation, "enabled": False, "operation_timeout_ms": 3000})[0] == 400
            assert call("POST", path + "/state", {**generation, "state": {"pose": {"position": [0, 0, 0]}}, "operation_timeout_ms": 3000})[0] == 422
            evidence["checks"].append("real FS150 motion, SetEnabled preserves plant, reset preserves enable/time/public generation")

            for index, invalid in enumerate((0, -1, True, 1.5, "3", None, 5001)):
                request_id = "invalid-timeout-" + str(index)
                assert call("POST", "/v1/world/pause", {"operation_timeout_ms": invalid}, request_id)[0] == 400
                mutation("POST", "/v1/world/pause", {}, request_id)
            for index, invalid in enumerate((0, -1, True, 1.5, "3", None)):
                request_id = "invalid-steps-" + str(index)
                assert call("POST", "/v1/world/step", {"steps": invalid, "operation_timeout_ms": 3000}, request_id)[0] == 400
                mutation("POST", "/v1/world/step", {"steps": 1}, request_id)
                assert call("POST", path + "/reset", {"generation": invalid, "operation_timeout_ms": 3000})[0] == 400
            evidence["checks"].append("strict integer validation and rejected requests do not poison idempotency IDs")

            guard_create = copy.deepcopy(create)
            guard_create["entity"]["id"] = "guard-beta"
            guard_create["entity"]["asset"]["realization"]["content"] = {"kind": "scout", "name": "guard_model"}
            guard = mutation("POST", "/v1/entities", guard_create)
            guard_ref = result_entity(guard, "guard-beta")["ref"]
            step = mutation("POST", "/v1/world/step", {"steps": 20})
            before_invalid_reset = result_entity(step, "frozen-id.alpha")["state"]
            # GET observes the native output frame, delivered separately from
            # the operation receipt. Establish the completed step's baseline
            # before checking that a rejected reset has no effects.
            deadline = time.monotonic() + 1
            while state("frozen-id.alpha") != before_invalid_reset and time.monotonic() < deadline:
                time.sleep(.001)
            assert state("frozen-id.alpha") == before_invalid_reset
            bad_reset = {"scope": "entities", "entities": [entity_ref, {"id": guard_ref["id"], "generation": guard_ref["generation"] + 1}],
                         "reset_time": False, "operation_timeout_ms": 3000}
            assert call("POST", "/v1/world/reset", bad_reset, "atomic-reset")[0] == 409
            assert state("frozen-id.alpha") == before_invalid_reset
            selected_reset = mutation("POST", "/v1/world/reset", {"scope": "entities", "entities": [entity_ref], "reset_time": False}, "atomic-reset")
            assert result_entity(selected_reset, "frozen-id.alpha")["state"]["pose"]["position"] == [1, 2, 3]
            mutation("POST", "/v1/world/reset", {"scope": "all_entities", "reset_time": False})
            assert call("POST", "/v1/world/reset", {"scope": "all_entities", "reset_time": True, "operation_timeout_ms": 3000})[0] == 422
            assert call("POST", "/v1/world/reset", {"scope": "all_entities", "entities": [], "reset_time": False, "operation_timeout_ms": 3000})[0] == 400
            assert call("POST", "/v1/world/reset", {"scope": "all_entities", "operation_timeout_ms": 3000})[0] == 400
            evidence["checks"].append("atomic reset validation, explicit scope and no time rewind")

            before_step = call("GET", "/v1/world")[1]
            code, stepping = call("POST", "/v1/world/step", {"steps": 5000, "operation_timeout_ms": 5000}, "cancellable-step")
            assert code == 202
            code, cancelled = call("POST", "/v1/operations/cancellable-step/cancel", {})
            assert code == 200
            terminal = await_operation(code, cancelled)
            if terminal["state"] == "cancelled":
                assert terminal["effects"]["applied"] is False
                expected_steps = 0
            else:
                assert terminal["state"] == "succeeded", terminal
                expected_steps = 5000
            after_step = call("GET", "/v1/world")[1]
            assert after_step["steps"] - before_step["steps"] == expected_steps
            assert int(after_step["time"]["nanoseconds"]) - int(before_step["time"]["nanoseconds"]) == expected_steps * 1000000
            evidence["checks"].append("cancel reports actual boundary outcome and step completes exact count")

            removed = mutation("DELETE", path, generation, "remove-first")
            assert result_entity(removed, "frozen-id.alpha")["lifecycle"] == "removed"
            assert call("GET", path)[0] == 404
            assert call("DELETE", path, {**generation, "operation_timeout_ms": 3000}, "remove-first")[1] == removed
            replacement = mutation("POST", "/v1/entities", create)
            assert result_entity(replacement, "frozen-id.alpha")["ref"]["generation"] != entity_ref["generation"]
            assert call("POST", path + "/reset", {**generation, "operation_timeout_ms": 3000})[0] == 409
            assert call("POST", "/v1/entities", create, "create-first")[1] == first
            assert call("POST", "/v1/operations/create-first/cancel", {})[1] == first
            conflicting = copy.deepcopy(create)
            conflicting["operation_timeout_ms"] = 2000
            assert call("POST", "/v1/entities", conflicting, "create-first")[0] == 409
            evidence["checks"].append("native release, replacement fences and immutable retained receipts")

            old_instance = instance
            process.send_signal(signal.SIGTERM)
            assert process.wait(timeout=5) == 0
            assert not os.path.exists(endpoint)
            description, _ = start()
            assert instance != old_instance
            current = instance
            instance = old_instance
            assert call("GET", "/v1/world")[0] == 409
            instance = current
            assert call("GET", "/v1/entities")[1]["entities"] == []
            evidence["checks"].append("owned graceful exit removes endpoint; restart invalidates old instance")

            # A claimed Step cannot be reported as an effect-free cancellation
            # when shutdown interrupts actual integration.
            before_stop_step = call("GET", "/v1/world")[1]
            requested_steps = 10000000
            code, step = call("POST", "/v1/world/step", {
                "steps": requested_steps, "operation_timeout_ms": 5000}, "stop-active-step")
            assert code == 202
            mutation("POST", "/v1/world/pause", {})
            observed = call("GET", "/v1/world")[1]
            assert 0 < observed["steps"] < requested_steps
            waited = {}
            ready = threading.Event()

            def wait_for_stop():
                ready.set()
                try:
                    waited["reply"] = call("POST", "/v1/operations/stop-active-step/wait", {}, timeout_ms=5000)
                except Exception as error:
                    waited["error"] = repr(error)

            waiter = threading.Thread(target=wait_for_stop)
            waiter.start()
            assert ready.wait(1)
            time.sleep(0.05)
            process.send_signal(signal.SIGTERM)
            waiter.join(6)
            assert not waiter.is_alive() and "reply" in waited, waited
            code, interrupted = waited["reply"]
            assert code == 200 and interrupted["state"] == "failed", interrupted
            assert interrupted["error"]["code"] == "interrupted", interrupted
            completed = interrupted["result"]["steps_completed"]
            assert observed["steps"] <= completed < requested_steps
            assert interrupted["effects"]["applied"] is True
            assert int(interrupted["result"]["time"]["nanoseconds"]) == int(before_stop_step["time"]["nanoseconds"]) + completed * 1000000
            assert process.wait(timeout=5) == 0
            assert not os.path.exists(endpoint)
            evidence["checks"].append("shutdown of claimed Step reports exact partial integration and applied effects")
            print(json.dumps(evidence, sort_keys=True))
            if args.output:
                Path(args.output).write_text(json.dumps(evidence, indent=2) + "\n")
        finally:
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
            log.close()
            if process is not None and process.returncode:
                print(log_path.read_text(errors="replace"))


if __name__ == "__main__":
    main()
