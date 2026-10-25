"""Finite frozen-session CLI and private Unix endpoint ownership checks."""
import argparse
import copy
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
    with tempfile.TemporaryDirectory(prefix='xsim-cli-session-') as directory:
        root = Path(directory)
        fixture = Path(__file__).parent / 'experiment-fixtures'
        frozen = json.loads((fixture / 'empty-world.input.json').read_text())
        endpoint = root / 'runtime' / 'world.sock'
        endpoint.parent.mkdir(mode=0o700)
        command = [binary, '--experiment-stdin', '--run-id', 'world-run:cli',
                   '--target-id', 'native-fixture:cli', '--socket', str(endpoint)]

        def rejected(arguments, reason, payload=None):
            result = subprocess.run(arguments, input=json.dumps(frozen) if payload is None else payload,
                                    capture_output=True, text=True, timeout=5)
            assert result.returncode == 1 and reason in result.stderr, result
            assert not endpoint.exists(), 'invalid session bound the endpoint'

        help_text = subprocess.check_output([binary, '--help'], text=True)
        assert all(flag in help_text for flag in ('--experiment-stdin', '--run-id', '--socket', '--target-id', '--slot', '--phase-t0', '--scene-file'))
        for retired in ('--config', '--manifest', '--experiment-file', '--bootstrap-input'):
            assert retired not in help_text
            rejected(command + [retired, 'retired'], 'unknown argument')
        for flag in ('--run-id', '--target-id', '--socket'):
            rejected(command + [flag], 'more than once')
            short = command[:command.index(flag)] + command[command.index(flag)+2:]
            rejected(short, 'are required')
        rejected(command + ['--experiment-stdin'], 'more than once')
        rejected(command + ['--sensor-init-timeout-ms', '0'], 'initialization timeout')
        rejected(command + ['--sensor-init-timeout-ms', '600001'], 'initialization timeout')
        for flag in ('--run-id', '--target-id'):
            invalid = command.copy()
            invalid[invalid.index(flag)+1] = 'invalid identity'
            rejected(invalid, 'bounded identifiers')
        for value in ('relative.sock', 'tcp://127.0.0.1:30000', str(root)+'/'):
            invalid = command.copy()
            invalid[-1] = value
            rejected(invalid, 'absolute filesystem path')
        absent = root / 'unallocated'
        invalid = command.copy()
        invalid[-1] = str(absent/'world.sock')
        rejected(invalid, 'directory allocation')
        assert not absent.exists()
        endpoint.parent.chmod(0o755)
        rejected(command, 'bootstrap')
        endpoint.parent.chmod(0o700)
        alias = root/'runtime-alias'
        alias.symlink_to(endpoint.parent, target_is_directory=True)
        invalid[-1] = str(alias/'world.sock')
        rejected(invalid, 'directory allocation')
        for payload in ('{} {}', '{"sessionId":"one","sessionId":"two"}', ' '*(8*1024*1024+1)):
            rejected(command, 'exceeds 8 MiB' if len(payload)>8*1024*1024 else 'parse' if payload=='{} {}' else 'duplicate configuration', payload)
        empty_scene = {'schema': 'xgc2.scene.compiled.v1', 'id': 'cli-fixture',
                       'frame': 'world', 'geometries': [], 'models': {}, 'obstacles': []}
        scene = root/'scene.json'
        selected = copy.deepcopy(frozen)
        selected['sceneAsset'] = {'sceneFile': str(scene)}
        rejected(command, 'cannot read scene document', json.dumps(selected))
        scene.write_text('{"schema":')
        rejected(command, 'parse error', json.dumps(selected))
        scene.write_text(json.dumps(empty_scene))

        # scene.parameters keys: XSim accepts the shared and xsim lists of the product contract
        # (contracts/scene-parameters.json) and nothing else. The probe session carries a malformed
        # sceneAsset, which is validated after the parameters, so an accepted key ends at that check.
        configured = os.environ.get('XGC_SCENE_PARAMETERS_CONTRACT')
        if configured:
            contract = Path(configured)
            assert contract.is_file(), 'configured scene parameters contract is missing: ' + configured
        else:
            contract = next((parent / 'xgc2' / 'contracts' / 'scene-parameters.json'
                             for parent in Path(__file__).resolve().parents
                             if (parent / 'xgc2' / 'contracts' / 'scene-parameters.json').is_file()), None)
        if contract is not None:
            table = json.loads(contract.read_text())
            probe = copy.deepcopy(frozen)
            probe['sceneAsset'] = {'sceneFile': 5}

            def parameter_accepted(key, value):
                probe['scene']['parameters'] = {key: value}
                result = subprocess.run(command, input=json.dumps(probe), capture_output=True, text=True, timeout=5)
                assert result.returncode == 1 and not endpoint.exists(), result
                if 'unknown scene parameter: ' + key in result.stderr:
                    return False
                assert 'sceneAsset.sceneFile must be a string' in result.stderr, result.stderr
                return True

            accepted = {**table['shared'], **table['simulators']['xsim']}
            for key, spec in accepted.items():
                assert parameter_accepted(key, spec['example']), 'XSim rejects the contract key ' + key
            for key, spec in table['simulators']['gazebo'].items():
                assert not parameter_accepted(key, spec['example']), 'XSim accepts the Gazebo-only key ' + key
            assert not parameter_accepted('typo', 1)
            assert 'pointCloudBackend' in accepted and accepted['pointCloudBackend']['default'] == 'auto'
        else:
            print('SKIP scene parameter contract:', contract, 'not found')

        def start(arguments, document, expected_ids=None):
            before = time.time_ns()
            process = subprocess.Popen(arguments, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            try:
                process.stdin.write(json.dumps(document).encode())
                process.stdin.close()
                process.stdin = None
                deadline = time.monotonic()+5
                while not endpoint.exists() and process.poll() is None and time.monotonic()<deadline:
                    time.sleep(.01)
                assert process.poll() is None and endpoint.exists(), process.communicate(timeout=1)
                connection = UnixHTTP(str(endpoint))
                try:
                    headers = {'X-Request-ID':'cli-describe', 'X-Xrpc-Timeout-Ms':'2000'}
                    connection.request('GET', '/v1/describe?wait_ready_ms=1000', headers=headers)
                    response = connection.getresponse()
                    assert response.status == 200
                    described = json.loads(response.read())
                    assert described['ready'], described
                    assert described['facts']['run_id'] == 'world-run:cli', described
                    ref = described['facts']['service_ref']
                    assert ref['target_id']=='native-fixture:cli'
                    assert ref['endpoint']=={'kind':'unix','address':str(endpoint)}
                    headers['X-Xrpc-Instance-ID']=described['instance_id']
                    connection.request('GET','/v1/world',headers=headers)
                    response = connection.getresponse()
                    assert response.status == 200
                    epoch = json.loads(response.read())['diagnostics']['epoch_ns']
                    assert before <= epoch <= time.time_ns()
                    connection.request('GET','/v1/entities',headers=headers)
                    response = connection.getresponse()
                    assert response.status == 200
                    entities = json.loads(response.read())['entities']
                    wanted = {r['slotId'] for r in document['robots']} if expected_ids is None else expected_ids
                    assert {e['ref']['id'] for e in entities} == wanted
                    assert all(e['state']['enabled'] for e in entities)
                finally:
                    connection.close()
                assert stat.S_IMODE(endpoint.stat().st_mode)==0o600
                return process
            except BaseException:
                process.kill()
                process.communicate(timeout=5)
                raise

        def stop(process):
            process.terminate()
            _, error = process.communicate(timeout=5)
            assert process.returncode==0, error

        mixed = json.loads((fixture/'default-mixed.input.json').read_text())
        for document in (frozen, selected, mixed):
            process = start(command, document)
            stop(process)
            assert not endpoint.exists() and endpoint.parent.exists()
        onboard = copy.deepcopy(mixed)
        for robot in onboard['robots']:
            if robot['slotId'] == 'scout-1':
                robot['binding']['executionHost'] = {'kind': 'agent', 'agentId': 'agent-1'}
        rejected(command, 'station', json.dumps(onboard))
        rejected(command + ['--slot', 'missing-slot'], 'roster', json.dumps(mixed))
        rejected(command + ['--slot', 'slot id'], 'bounded identifier', json.dumps(mixed))
        rejected(command + ['--phase-t0', '1.5'], 'phase T0', json.dumps(mixed))
        process = start(command + ['--slot', 'scout-1', '--phase-t0', '1'], onboard, {'scout-1'})
        stop(process)
        assert not endpoint.exists() and endpoint.parent.exists()
        with socket.socket(socket.AF_UNIX) as foreign:
            foreign.bind(str(endpoint))
            identity=endpoint.lstat().st_ino
            result=subprocess.run(command,input=json.dumps(frozen),capture_output=True,text=True,timeout=5)
            assert result.returncode==1 and endpoint.lstat().st_ino==identity
        endpoint.unlink()
        endpoint.write_text('foreign-file')
        result=subprocess.run(command,input=json.dumps(frozen),capture_output=True,text=True,timeout=5)
        assert result.returncode==1 and endpoint.read_text()=='foreign-file'
        endpoint.unlink()
        process=start(command,frozen)
        moved=endpoint.with_name('moved.sock')
        endpoint.rename(moved)
        with socket.socket(socket.AF_UNIX) as replacement:
            replacement.bind(str(endpoint))
            identity=endpoint.lstat().st_ino
            stop(process)
            assert endpoint.lstat().st_ino==identity
        endpoint.unlink()
        definition=json.loads((fixture.parent.parent/'process-definitions/xsim-world.json').read_text())['definitions'][0]
        required=set(definition['parameters']['required'])
        assert {'targetId','runId','rosIp','configJson'} <= required
        assert 'default' not in definition['parameters']['properties']['rosIp']
        assert definition['parameters']['properties']['sceneFile']['default'] == ''
        values={'targetId':'native-fixture:cli','runId':'world-run:cli','socketPath':str(endpoint),
                'slotId':'','phaseT0':'','sceneFile':'','sensorInitTimeoutMs':'10000'}
        arguments=[binary]+[re.sub(r'\$\{(\w+)\}',lambda m:values[m.group(1)],a) for a in definition['command']['args']]
        process=start(arguments,selected)
        stop(process)
        assert not endpoint.exists()
        missing = root/'core-scene.json'
        host = root/'host-scene.json'
        host.write_text(json.dumps(empty_scene))
        remote = copy.deepcopy(selected)
        remote['sceneAsset']['sceneFile'] = str(missing)
        payload = json.dumps(remote)
        assert str(missing) in payload and str(host) not in payload
        rejected(command + ['--scene-file', ''], 'cannot read scene document', payload)
        rejected(command + ['--scene-file', str(host), '--scene-file', str(host)], 'more than once', payload)
        process = start(command + ['--scene-file', str(host)], remote)
        stop(process)
        rejected(command + ['--scene-file', str(missing)], 'cannot read scene document', json.dumps(selected))
        assert not endpoint.exists()
        print(json.dumps({'frozenSessionStdin':True,'explicitRunAndTarget':True,'sessionSceneAsset':True,
                          'retiredInputsRejected':True,'shippedDefinitionArgv':True,'nativeEpoch':True,
                          'rosterEnabled':True,'privateEndpointOwnership':True,
                          'emptySceneFileKeepsFrozen':True,'sceneFileOverride':True}))


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--xsim',required=True)
    main(os.path.abspath(parser.parse_args().xsim))
