"""Frozen-session fixtures for the production stdin boundary."""
import json
from pathlib import Path


def robot(slot, namespace, kind, position, *, sensor=False):
    return {
        'slotId': slot, 'kind': kind, 'namespace': '/'+namespace,
        'asset': {'px4': {'modelId': 'fs150'}} if kind == 'px4_multirotor' else {},
        'profile': {'parameters': {}},
        'binding': {'id': slot, 'namespace': '/'+namespace, 'executionHost': {'kind': 'station'},
                    'initialPose': dict(zip(('x', 'y', 'z'), position), yaw=0),
                    'simulationSensors': {'simpleLidar': sensor}},
    }


def session(robots=(), *, model_step_ms=2, output_period_ms=8, scene_file=None, reference=False):
    frozen = json.loads((Path(__file__).parent/'experiment-fixtures/empty-world.input.json').read_text())
    frozen['robots'] = list(robots)
    frozen['scene']['parameters'] = {'modelStepMs': model_step_ms, 'outputPeriodMs': output_period_ms}
    if scene_file:
        frozen['sceneAsset'] = {'sceneFile': str(scene_file)}
    if reference:
        frozen['visualizationTopics'] = [{'topic': '/xgc/scene/reference_cloud', 'messageType': 'sensor_msgs/PointCloud2',
                                          'role': 'semantic', 'visible3d': True}]
    return frozen


def scene_file(root, obstacles):
    root = Path(root)
    root.mkdir(mode=0o700, parents=True, exist_ok=True)
    path = root/'scene.yaml'
    document = {'schema': 'xgc2.scene.v1', 'id': 'test-scene', 'frame': 'world', 'obstacles': []}
    for index, (position, size) in enumerate(obstacles):
        document['obstacles'].append({'id': 'box'+str(index), 'name': 'Fixture box',
            'pose': {'position': position, 'orientation': [0, 0, 0, 1]}, 'motion': {'type': 'hold'},
            'parts': [{'id': 'body', 'pose': {'position': [0, 0, 0], 'orientation': [0, 0, 0, 1]},
                       'color': [1, 1, 1, 1], 'geometry': {'type': 'box', 'size': size}}]})
    path.write_text(json.dumps(document))
    return path
