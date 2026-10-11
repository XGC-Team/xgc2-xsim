#!/usr/bin/env python3
"""Chassis HOLD and the readiness contract of the actual headless server (no ROS).

HOLD is the capability `xgc2.chassis.hold`, called as POST /v1/call/xgc2.chassis.hold/<Method> on the management
socket. Covers the service semantics over that socket: Describe, State, Engage and Release with revisions and
instances, engage and release while the world is paused, removal and re-creation of a held entity, a restart,
the capability listing in the describe facts, and `wait_ready_ms`. Motion under HOLD is checked by
chassis_hold_ros.py.
"""
import argparse
import tempfile
import time
from pathlib import Path

from world_client import HOLD, Xsim, until, write_private

HEX32 = "0123456789abcdef" * 2
UNIT = "application/vnd.xgc2.xsim.entity+json"


def world_config():
    scout = lambda name, public_id, y: {"name": name, "public_id": public_id, "kind": "scout", "position": [0, y, 0]}
    return {"epoch_ns": 1000000000, "model_step_ns": 1000000, "output_period_ns": 4000000, "paused": False,
            "entities": [scout("scout_a", "ugv-a", 0), scout("scout_b", "ugv-b", 2),
                         {"name": "mec", "kind": "mecanum", "position": [0, 4, 0]},
                         {"name": "uav", "kind": "fs150", "position": [0, 6, 1]}]}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--xsim", required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="xsim-hold-native-") as directory:
        xsim = Xsim(args.xsim, Path(directory), world_config())
        try:
            run(xsim)
        finally:
            xsim.kill()


def run(x):
    # ---- readiness: identity, facts, the bounded hold and its argument checks
    described = x.start()
    assert described["service"] == "xgc2.simulation" and described["api_version"] == "v1", described
    assert described["instance_id"] == x.instance and described["service_ref"]["instance_id"] == x.instance
    assert described["facts"]["world_generation"] == 1 and "ros_master_uri" not in described["facts"], described
    started = time.monotonic()
    status, ready = x.call("GET", "/v1/describe?wait_ready_ms=5000")
    assert status == 200 and ready["ready"] is True and ready["facts"]["frames_flowing"] is True, ready
    assert time.monotonic() - started < 2, "a ready service answers a waiting describe at once"
    assert x.call("GET", "/v1/describe?wait_ready_ms=0")[1]["ready"] is True
    for bad in ("wait_ready_ms=-1", "wait_ready_ms=30001", "wait_ready_ms=1.5", "wait_ready_ms=01", "wait_ready_ms=",
                "wait_ready_ms=1&wait_ready_ms=2", "other=1"):
        assert x.call("GET", "/v1/describe?" + bad)[0] == 400, bad
    assert x.call("GET", "/v1/health?wait_ready_ms=1")[0] == 404

    # ---- the capability: listed in the facts with the chassis entities, called by generic method addressing
    assert ready["facts"]["capabilities"] == [{"name": HOLD, "entities": ["mec", "ugv-a", "ugv-b"]}], ready["facts"]
    status, roster = x.hold("Describe")
    assert status == 200 and roster["instance"] == x.instance, roster
    assert roster["robots"] == ["mec", "ugv-a", "ugv-b"], roster
    assert roster["feedback"] == {"mec": True, "ugv-a": True, "ugv-b": True}
    assert roster["capabilities"] == ["engage", "release", "state"]
    assert x.call("POST", "/v1/call/%s/Describe" % HOLD)[0] == 200, "an empty request is {}"
    assert set(x.hold_state()) == {"mec", "ugv-a", "ugv-b"}
    assert all(r["held"] is False and r["revision"] == 0 and r["stage"] == "released"
               for r in x.hold_state().values())

    # ---- engage: immediate, idempotent, zero written by the world, stopped from the entity's own twist
    x.enable("ugv-a")
    status, engaged = x.hold("Engage", {"robot_ids": ["ugv-a", "nobody"]})
    assert status == 200 and engaged["instance"] == x.instance, engaged
    first, missing = engaged["robots"]
    assert first["outcome"] == "engaged" and first["held"] is True and first["revision"] == 1, first
    assert first["stage"] in ("gated", "zero_written"), first
    assert missing == {"robot_id": "nobody", "outcome": "unknown_robot"}, missing
    state = until(lambda: x.hold_state("ugv-a")["ugv-a"] if x.hold_state("ugv-a")["ugv-a"]["stage"] != "gated" else None,
                  "the world thread writes zero")
    assert state["stage"] in ("zero_written", "stopped") and state["zero_written_at_ms"] is not None, state
    until(lambda: x.hold_state("ugv-a")["ugv-a"]["stage"] == "stopped", "stopped after the dwell at rest", 5)
    stopped = x.hold_state("ugv-a")["ugv-a"]
    assert stopped["stopped_at_ms"] >= stopped["zero_written_at_ms"] >= stopped["gated_at_ms"], stopped
    again = x.hold("Engage", {"robot_ids": ["ugv-a"]})[1]["robots"][0]
    assert again["outcome"] == "already_held" and again["revision"] == 1, again
    assert x.hold_state("ugv-b")["ugv-b"]["held"] is False, "other robots are not held"
    # HOLD is not a disable: the entity stays enabled, so its sensors and publication keep running.
    assert x.call("GET", "/v1/entities/ugv-a")[1]["state"]["enabled"] is True
    # The flight entity is not a chassis.
    assert x.hold("Engage", {"robot_ids": ["uav"]})[1]["robots"] == [{"robot_id": "uav", "outcome": "unknown_robot"}]
    status, unknown = x.hold("State", {"robot_ids": ["uav"]})
    assert status == 404 and unknown["error"]["code"] == "not_found", unknown
    assert unknown["error"]["details"]["robot_ids"] == ["uav"], unknown

    # ---- release: compare and set on instance and revision
    wrong = x.release("ugv-a", 9)
    assert wrong["outcome"] == "conflict" and wrong["held"] is True and wrong["revision"] == 1, wrong
    stale = x.release("ugv-a", 1, instance=HEX32)
    assert stale["outcome"] == "conflict" and stale["held"] is True, stale
    assert x.release("ugv-b", 0)["outcome"] == "not_held"
    released = x.release("ugv-a", 1)
    assert released["outcome"] == "released" and released["held"] is False and released["revision"] == 2, released
    assert released["stage"] == "released"
    assert x.release("ugv-a", 1)["outcome"] == "not_held", "a replayed release cannot apply twice"
    assert x.hold("Engage", {"robot_ids": ["ugv-a"]})[1]["robots"][0]["revision"] == 3
    assert x.release("ugv-a", 3)["revision"] == 4

    # ---- request validation: closed schemas, nothing changes on a rejected request
    for body in ({}, {"robot_ids": []}, {"robot_ids": ["ugv-a"], "all": True}, {"robot_ids": "ugv-a"},
                 {"robot_ids": ["bad id"]}, {"robot_ids": ["ugv-a", "ugv-a"]}, {"all": False}, {"extra": 1}):
        status, rejected = x.hold("Engage", body)
        assert status == 400 and rejected["error"]["code"] == "invalid_argument", (body, status, rejected)
    assert x.hold("Release", {"expected_instance": "short", "changes": []})[0] == 400
    assert x.hold("Engage", "not json")[0] == 400
    assert x.hold("Engage", '{"robot_ids":["ugv-a"],"robot_ids":["ugv-b"]}')[0] == 400, "duplicate fields"
    assert x.hold("Engage", "[]")[0] == 400, "a request is an object"
    assert not x.hold_state("ugv-a")["ugv-a"]["held"]

    # ---- the generic route: only POST, bound to the instance, unknown services and methods are not found
    assert x.call("GET", "/v1/call/%s/Describe" % HOLD)[0] == 404
    assert x.call("POST", "/v1/call/xgc2.nothing/Describe", {})[0] == 404
    assert x.call("POST", "/v1/call/%s" % HOLD, {})[0] == 404
    status, no_method = x.hold("Frobnicate", {})
    assert status == 404 and no_method["error"]["code"] == "not_found", no_method
    assert x.call("POST", "/v1/call/%s/Describe" % HOLD, {}, bound=False)[0] == 409, "calls are bound to the instance"

    # ---- a paused world: state and gate at once, the zero is written at the next boundary
    x.operation("POST", "/v1/world/pause")
    steps = x.call("GET", "/v1/world")[1]["steps"]
    status, engaged = x.hold("Engage", {"robot_ids": ["ugv-b"]})
    paused = engaged["robots"][0]
    assert status == 200 and paused["held"] is True and paused["revision"] == 1, paused
    assert x.hold_state("ugv-b")["ugv-b"]["held"] is True
    until(lambda: x.hold_state("ugv-b")["ugv-b"]["stage"] == "zero_written", "zero written while paused")
    assert x.call("GET", "/v1/world")[1]["steps"] == steps, "HOLD does not advance a paused world"
    released = x.release("ugv-b", 1)
    assert released["outcome"] == "released" and released["revision"] == 2, released
    assert x.hold_state("ugv-b")["ugv-b"]["held"] is False
    x.operation("POST", "/v1/world/resume")

    # ---- a removed entity keeps its HOLD state: the re-created one starts held
    x.hold("Engage", {"robot_ids": ["mec"]})
    ref = x.ref("mec")
    x.operation("DELETE", "/v1/entities/mec", {"generation": ref["generation"]})
    status, gone = x.hold("State", {"robot_ids": ["mec"]})
    assert status == 404 and gone["error"]["details"]["robot_ids"] == ["mec"], gone
    assert x.hold("Describe")[1]["robots"] == ["ugv-a", "ugv-b"]
    assert x.chassis() == ["ugv-a", "ugv-b"], "the capability lists the entities that are in the world"
    assert x.hold("Engage", {"robot_ids": ["mec"]})[1]["robots"][0]["outcome"] == "unknown_robot"
    x.operation("POST", "/v1/entities", {"entity": {
        "id": "mec", "role": "robot", "pose": {"position": [0, 4, 0], "orientation": [0, 0, 0, 1]},
        "asset": {"id": "mecanum-test", "realization": {"media_type": UNIT, "content": {"kind": "mecanum", "name": "mec2"}}}}})
    until(lambda: x.hold_state("mec")["mec"]["stage"] in ("zero_written", "stopped"), "zero written to the new model")
    back = x.hold_state("mec")["mec"]
    assert back["held"] is True and back["revision"] == 1, back
    assert x.hold("Describe")[1]["robots"] == ["mec", "ugv-a", "ugv-b"]
    assert x.chassis() == ["mec", "ugv-a", "ugv-b"]
    # A live public ID cannot be taken by a second entity.
    status, clash = x.call("POST", "/v1/entities", {"operation_timeout_ms": 4000, "entity": {
        "id": "mec", "role": "robot", "pose": {"position": [0, 8, 0], "orientation": [0, 0, 0, 1]},
        "asset": {"id": "mecanum-test", "realization": {"media_type": UNIT, "content": {"kind": "mecanum", "name": "mec3"}}}}})
    assert status == 409, (status, clash)
    assert x.release("mec", 1)["outcome"] == "released"

    # ---- restart: a new instance, no persisted HOLD state, a stale release is rejected
    x.hold("Engage", {"robot_ids": ["ugv-a", "mec"]})
    old = x.instance
    status, held = x.hold("State", {"robot_ids": ["ugv-a"]})
    assert held["robots"][0]["held"] is True
    x.stop()
    x.start()
    assert x.instance != old
    assert x.hold("State", {}, instance=old)[0] == 409, "the old incarnation is fenced by the transport"
    status, fresh = x.hold("State")
    assert status == 200 and fresh["instance"] == x.instance != old, fresh
    assert all(r["held"] is False and r["revision"] == 0 for r in fresh["robots"]), fresh
    status, reply = x.hold("Release", {"expected_instance": old,
                                       "changes": [{"robot_id": "ugv-a", "expected_revision": 1}]})
    assert status == 200 and reply["instance"] == x.instance, reply
    assert reply["robots"][0]["outcome"] == "conflict" and reply["robots"][0]["held"] is False, reply
    x.hold("Engage", {"robot_ids": ["ugv-a"]})
    assert x.release("ugv-a", 1, instance=old)["outcome"] == "conflict"
    assert x.hold_state("ugv-a")["ugv-a"]["held"] is True
    x.stop()

    # ---- a world that starts paused is ready as well: its first frame is delivered without a step
    x.config["paused"] = True
    write_private(x.world, x.config)
    x.start()
    status, ready = x.call("GET", "/v1/describe?wait_ready_ms=5000")
    assert status == 200 and ready["ready"] is True and ready["facts"]["frames_flowing"] is True, ready
    assert x.call("GET", "/v1/world")[1]["paused"] is True
    x.stop()
    print("PASS: chassis HOLD and readiness over the headless management service")


if __name__ == "__main__":
    main()
