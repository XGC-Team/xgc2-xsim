"""Finite native CLI/config and Unix startup checks; no ROS graph or experiment."""
import argparse
import json
import os
from pathlib import Path
import re
import socket
import stat
import subprocess
import tempfile
import time

from native_http import UnixHTTP


def main(binary):
    with tempfile.TemporaryDirectory(prefix="xsim-cli-config-") as temporary:
        root = Path(temporary)
        config = root / "world.json"
        scene = root / "scene.yaml"
        scene.write_text("obstacles: []\n")
        original = {"instance_id": "cli-config-test", "epoch_ns": 1791250000000000000,
                    "paused": True, "entities": []}
        config.write_text(json.dumps(original))
        socket_path = root / "new" / "nested" / "world.sock"
        socket_path.parent.mkdir(parents=True, mode=0o700)
        socket_path.parent.parent.chmod(0o700)
        prefix = [binary, "--target-id", "native-fixture:cli"]
        command = prefix + ["--config", str(config), "--socket", str(socket_path)]

        def rejected(arguments, reason, payload=None):
            result = subprocess.run(arguments, input=payload, capture_output=True, text=True, timeout=5)
            assert result.returncode == 1, (arguments, result)
            assert reason in result.stderr, result.stderr
            assert not socket_path.exists(), "rejected config bound the socket"

        help_text = subprocess.check_output([binary, "--help"], text=True)
        assert all(flag in help_text for flag in ("--scene-file", "--socket", "--target-id"))
        assert "--bootstrap-input" not in help_text
        rejected(command + ["--scene-file"], "missing value")
        rejected(command + ["--scene-file", "", "--scene-file", ""], "more than once")
        rejected(command + ["--config", str(config)], "more than once")
        rejected(command + ["--socket", str(socket_path)], "more than once")
        rejected([binary, "--config", "--socket", str(socket_path)], "missing value")
        rejected([binary, "--config", str(config), "--socket"], "missing value")
        rejected(command + ["--target-id", "other:world"], "more than once")
        rejected(command + ["--bootstrap-input", str(root / "retired.json")], "unknown argument")
        rejected([binary, "--config", str(config), "--socket", str(socket_path)], "--target-id")
        rejected(prefix + ["--config", str(config)], "--socket")
        rejected(prefix + ["--config", str(config), "--socket", ""], "--socket")
        for path in ("relative.sock", "tcp://127.0.0.1:30000", str(root) + "/"):
            rejected(prefix + ["--config", str(config), "--socket", path], "absolute filesystem path")
        absent = root / "unallocated"
        rejected(prefix + ["--config", str(config), "--socket", str(absent / "world.sock")], "directory allocation")
        assert not absent.exists(), "launcher created an unallocated runtime directory"
        socket_path.parent.chmod(0o755)
        rejected(command, "bootstrap")  # DirectoryGrant rejects a non-private runtime owner.
        assert stat.S_IMODE(socket_path.parent.stat().st_mode) == 0o755
        socket_path.parent.chmod(0o700)
        alias = root / "runtime-alias"
        alias.symlink_to(socket_path.parent, target_is_directory=True)
        rejected(prefix + ["--config", str(config), "--socket", str(alias / "world.sock")], "directory allocation")
        ancestor = root / "ancestor-alias"
        ancestor.symlink_to(socket_path.parent.parent, target_is_directory=True)
        rejected(prefix + ["--config", str(config), "--socket", str(ancestor / "nested" / "world.sock")], "symbolic links")
        for target in ("", "bad target"):
            rejected([binary, "--target-id", target, "--config", str(config), "--socket", str(socket_path)], "target")
        rejected(command + ["--scene-file", str(root / "absent.yaml")], "bad file")
        configured = dict(original, scene_file=str(scene))
        config.write_text(json.dumps(configured))
        rejected(command + ["--scene-file", str(root / "different.yaml")], "conflicts")
        scene.write_text("obstacles: [unterminated\n")
        rejected(command + ["--scene-file", str(scene)], "end of")
        for epoch in [0, -1, 1.5, 18446744073709551615, 9223372036854775807]:
            config.write_text(json.dumps(dict(original, epoch_ns=epoch)))
            rejected(command, "epoch")
        missing_epoch = dict(original)
        del missing_epoch["epoch_ns"]
        config.write_text(json.dumps(missing_epoch))
        rejected(command, "epoch_ns")
        for text in ['{} {}', '{"epoch_ns":1,"epoch_ns":2}', '{"x":{"a":1,"a":2}}']:
            config.write_text(text)
            rejected(command, "configuration" if 'epoch_ns' in text or '"x"' in text else "parse")
        config.write_text(json.dumps(original))

        def start(arguments, payload=None, native_epoch=False):
            before_start = time.time_ns()
            process = subprocess.Popen(arguments, stdin=subprocess.PIPE if payload is not None else subprocess.DEVNULL,
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            if payload is not None:
                process.stdin.write(payload.encode())
                process.stdin.close()
                process.stdin = None
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline and not socket_path.exists() and process.poll() is None:
                time.sleep(.01)
            assert process.poll() is None and socket_path.exists(), process.communicate(timeout=1)
            connection = UnixHTTP(str(socket_path))
            try:
                headers = {"X-Request-ID": "cli-describe", "X-Xrpc-Timeout-Ms": "2000"}
                connection.request("GET", "/v1/describe", headers=headers)
                description = connection.getresponse()
                assert description.status == 200
                described = json.loads(description.read())
                assert set(described) == {"service", "api_version", "instance_id", "ready", "facts"}, described
                assert described["facts"]["service_ref"]["target_id"] == "native-fixture:cli"
                assert described["facts"]["service_ref"]["endpoint"] == {"kind": "unix", "address": str(socket_path)}
                connection.request("GET", "/v1/describe?wait_ready_ms=1000", headers=headers)
                readiness = connection.getresponse()
                assert readiness.status == 200
                ready = json.loads(readiness.read())
                assert set(ready) == {"service", "api_version", "instance_id", "ready", "facts"}, ready
                assert ready["ready"] and ready["facts"]["service_ref"] == described["facts"]["service_ref"], ready
                headers["X-Xrpc-Instance-ID"] = described["instance_id"]
                headers["X-Request-ID"] = "cli-world"
                connection.request("GET", "/v1/world", headers=headers)
                response = connection.getresponse()
                assert response.status == 200
                value = json.loads(response.read())
                epoch = value["diagnostics"]["epoch_ns"]
                if native_epoch:
                    assert before_start <= epoch <= time.time_ns(), value
                    headers["X-Request-ID"] = "cli-world-again"
                    connection.request("GET", "/v1/world", headers=headers)
                    response = connection.getresponse()
                    assert response.status == 200
                    assert json.loads(response.read())["diagnostics"]["epoch_ns"] == epoch
                else:
                    assert epoch == original["epoch_ns"], value
            finally:
                connection.close()
            return process

        # The shipped definition supplies its optional scene argument, even empty.
        process = start(command + ["--scene-file", ""])
        try:
            assert stat.S_IMODE(socket_path.parent.stat().st_mode) == 0o700
            assert stat.S_IMODE(socket_path.parent.parent.stat().st_mode) == 0o700
            assert stat.S_IMODE(socket_path.stat().st_mode) == 0o600
        finally:
            process.terminate()
            _, stderr = process.communicate(timeout=5)
            assert process.returncode == 0, stderr
        assert not socket_path.exists(), "normal shutdown leaked owned socket"
        assert socket_path.parent.exists(), "shutdown removed its parent directory"

        scene.write_text("obstacles: []\n")
        for arguments, value in [(command + ["--scene-file", str(scene)], original),
                                 (command, dict(original, scene_file=str(scene)))]:
            config.write_text(json.dumps(value))
            process = start(arguments)
            process.terminate()
            _, stderr = process.communicate(timeout=5)
            assert process.returncode == 0, stderr
            assert not socket_path.exists()
        config.write_text(json.dumps(original))

        # Pre-existing foreign sockets/files must not be removed even on a failed bind.
        with socket.socket(socket.AF_UNIX) as foreign:
            foreign.bind(str(socket_path))
            identity = socket_path.lstat().st_ino
            result = subprocess.run(command, capture_output=True, timeout=5)
            assert result.returncode == 1, result.stderr
            assert socket_path.lstat().st_ino == identity
        socket_path.unlink()
        socket_path.write_text("foreign-file")
        result = subprocess.run(command, capture_output=True, timeout=5)
        assert result.returncode == 1 and socket_path.read_text() == "foreign-file"
        socket_path.unlink()

        # A pathname replacement after bind remains foreign at our RAII cleanup.
        process = start(command)
        moved = socket_path.with_name("moved-original.sock")
        try:
            socket_path.rename(moved)
            with socket.socket(socket.AF_UNIX) as replacement:
                replacement.bind(str(socket_path))
                identity = socket_path.lstat().st_ino
                process.terminate()
                _, stderr = process.communicate(timeout=5)
                assert process.returncode == 0, stderr
                assert socket_path.lstat().st_ino == identity
        finally:
            if process.poll() is None:
                process.kill()
                process.communicate(timeout=5)
        socket_path.unlink()  # prior replacement-fence fixture owns this foreign socket
        # The independent product compiles the current flat frozen Session
        # through the same bounded reader before owning any socket/ROS effects.
        experiment = root / "experiment.json"
        fixture = Path(__file__).parent / "experiment-fixtures" / "empty-world.input.json"
        frozen = json.loads(fixture.read_text())
        experiment.write_text(json.dumps(frozen))
        experiment_command = prefix + ["--experiment-file", str(experiment), "--socket", str(socket_path)]
        rejected(experiment_command + ["--config", str(config)], "exactly one")
        rejected(experiment_command + ["--experiment-file", str(experiment)], "more than once")
        rejected([binary, "--experiment-file", "--socket", str(socket_path)], "missing value")
        for invalid in ['{"sessionId":"one","sessionId":"two"}', '{} {}']:
            experiment.write_text(invalid)
            rejected(experiment_command, "configuration" if "sessionId" in invalid else "parse")
        experiment.write_text(json.dumps(frozen))
        process = start(experiment_command + ["--scene-file", str(scene)], native_epoch=True)
        process.terminate()
        _, stderr = process.communicate(timeout=5)
        assert process.returncode == 0, stderr
        assert not socket_path.exists(), "experiment-file shutdown leaked owned socket"
        stdin_command = prefix + ["--experiment-stdin", "--socket", str(socket_path), "--scene-file", str(scene)]
        rejected(stdin_command + ["--config", str(config)], "exactly one", "{}")
        rejected(stdin_command + ["--experiment-stdin"], "more than once", "{}")
        rejected(stdin_command, "parse", "{} {}")
        rejected(stdin_command, "duplicate configuration", '{"sessionId":"one","sessionId":"two"}')
        rejected(stdin_command, "exceeds 8 MiB", " " * (8 * 1024 * 1024 + 1))
        process = start(stdin_command, json.dumps(frozen), native_epoch=True)
        process.terminate()
        _, stderr = process.communicate(timeout=5)
        assert process.returncode == 0, stderr
        assert not socket_path.exists(), "experiment-stdin shutdown leaked owned socket"
        definition_path = Path(__file__).parent.parent / "process-definitions" / "xsim-world.json"
        definition = json.loads(definition_path.read_text())["definitions"][0]
        assert not {"bootstrapInput", "startupInputPath", "rosHome", "rosLogDir"} & set(definition["parameters"]["properties"])
        assert not {"ROS_HOME", "ROS_LOG_DIR"} & set(definition["command"]["env"])
        assert "targetId" in definition["parameters"]["required"]
        values = {"targetId": "native-fixture:cli", "socketPath": str(socket_path), "sceneFile": ""}
        definition_command = [binary] + [re.sub(r"\$\{(\w+)\}", lambda match: values[match.group(1)], argument)
                                         for argument in definition["command"]["args"]]
        process = start(definition_command, json.dumps(frozen), native_epoch=True)
        process.terminate()
        _, stderr = process.communicate(timeout=5)
        assert process.returncode == 0, stderr
        assert not socket_path.exists(), "shipped definition shutdown leaked owned socket"
        print(json.dumps({"ok": True, "emptySceneFlag": True, "validSceneFlagAndConfig": True,
                          "sceneConflictRefused": True,
                          "invalidSceneUsesOriginalParser": True, "explicitEpochUnchanged": True,
                          "nativeStartupEpochStable": True,
                          "shippedDefinitionArgv": True, "retiredBootstrapRefused": True, "explicitTargetIdentity": True,
                          "unboundDescribeReadiness": True,
                          "experimentFile": True, "experimentStdin": True, "privateParents": True, "foreignSocketAndReplacementPreserved": True}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--xsim", required=True)
    main(os.path.abspath(parser.parse_args().xsim))
