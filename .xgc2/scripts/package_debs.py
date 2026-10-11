#!/usr/bin/env python3
"""Split the installed core and optional GPU renderer into two Debian packages."""
import argparse
import pathlib
import shutil
import subprocess
import tempfile


def package(stage, output_dir, dependencies, gpu_dependencies):
    stage = pathlib.Path(stage)
    output_dir = pathlib.Path(output_dir)
    prefix = stage / 'opt/xgc2/xsim'
    required = ('bin/xsim', 'lib/xsim/libxsim-lidar-gpu.so',
                'share/xsim-lidar-gpu/LICENSE', 'share/xsim-lidar-gpu/source-pin.json',
                'share/xsim-lidar-gpu/marsim-memory-entry.patch',
                'share/xsim-lidar-gpu/marsim/include/360camera.vs',
                'share/xsim-lidar-gpu/marsim/include/camera.fs',
                'share/xsim-lidar-gpu/source/src/xsim-lidar-gpu/module.cpp')
    for name in required:
        if not (prefix / name).is_file():
            raise ValueError('required installed payload missing: ' + name)
    definitions = stage / 'usr/share/xgc2/process-definitions'
    if not (definitions / 'xsim-world.json').is_file():
        raise ValueError('installed xsim-world process definition is required')
    if (stage / 'opt/ros').exists():
        raise ValueError('ROS-owned payload is not part of xsim')
    output_dir.mkdir(parents=True, exist_ok=True)
    arch = subprocess.check_output(['dpkg', '--print-architecture'], text=True).strip()
    packages = (
        ('xsim', dependencies, ('bin', 'share/xsim'),
         'Standalone ECS world and CPU point cloud server', 'Suggests: xsim-lidar-gpu\n'),
        ('xsim-lidar-gpu', gpu_dependencies, ('lib/xsim', 'share/xsim-lidar-gpu'),
         'Optional GPL GPU lidar renderer for xsim', ''))
    with tempfile.TemporaryDirectory(prefix='xsim-debs-', dir=output_dir) as directory:
        for name, deps, paths, description, extra in packages:
            root = pathlib.Path(directory) / name
            installed = root / 'opt/xgc2/xsim'
            for path in paths:
                shutil.copytree(prefix / path, installed / path)
            if name == 'xsim':
                shutil.copytree(definitions, root / 'usr/share/xgc2/process-definitions')
            control = root / 'DEBIAN'
            control.mkdir()
            (control / 'control').write_text(f'''Package: {name}
Version: 1.3.0-2
Architecture: {arch}
Maintainer: XGC Team <dev@xgc.team>
Depends: {deps}
{extra}Description: {description}
''')
            subprocess.run(['dpkg-deb', '--build', '--root-owner-group', str(root),
                            str(output_dir / f'{name}_1.3.0-2_{arch}.deb')], check=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--stage', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--depends', required=True)
    parser.add_argument('--gpu-depends', required=True)
    args = parser.parse_args()
    package(args.stage, args.output_dir, args.depends, args.gpu_depends)


if __name__ == '__main__':
    main()
