#!/usr/bin/env python3
"""Package the simulator separately from the shared optional sensing library."""
import argparse
import pathlib
import shutil
import subprocess
import tempfile


def package(stage, output_dir, dependencies, gpu_dependencies, version):
    stage, output_dir = pathlib.Path(stage), pathlib.Path(output_dir)
    required = ('opt/xgc2/xsim/bin/xsim', 'usr/lib/xgc2-sensing/libxgc2-sensing-gpu.so',
                'usr/include/xgc2_sensing/gpu.h', 'usr/libexec/xgc2-sensing-probe',
                'usr/share/xgc2-sensing/LICENSE', 'usr/share/xgc2-sensing/source-pin.json',
                'usr/share/xgc2/process-definitions/xsim-world.json')
    for path in required:
        if not (stage / path).is_file():
            raise ValueError('required installed payload missing: ' + path)
    if (stage / 'opt/ros').exists():
        raise ValueError('ROS-owned payload is not part of these native packages')
    output_dir.mkdir(parents=True, exist_ok=True)
    arch = subprocess.check_output(['dpkg', '--print-architecture'], text=True).strip()
    packages = (
        ('xsim', dependencies, ('opt/xgc2/xsim', 'usr/share/xgc2/process-definitions'),
         'Standalone ECS world and native observation server', f'Suggests: libxgc2-sensing-gpu (= {version})\n'),
        ('libxgc2-sensing-gpu', gpu_dependencies,
         ('usr/lib/xgc2-sensing', 'usr/share/xgc2-sensing', 'usr/libexec'),
         'Shared native GPU sensing with explicit occlusion semantics', ''),
        ('libxgc2-sensing-dev', 'libc6-dev', ('usr/include/xgc2_sensing',),
         'Native sensing ABI headers and optional module loader', ''),
    )
    with tempfile.TemporaryDirectory(prefix='xgc2-debs-', dir=output_dir) as directory:
        for name, deps, paths, description, extra in packages:
            root = pathlib.Path(directory) / name
            for path in paths:
                shutil.copytree(stage / path, root / path)
            control = root / 'DEBIAN';control.mkdir()
            (control / 'control').write_text(f'''Package: {name}
Version: {version}
Architecture: {arch}
Maintainer: XGC Team <dev@xgc.team>
Depends: {deps}
{extra}Description: {description}
''')
            subprocess.run(['dpkg-deb', '--build', '--root-owner-group', str(root),
                            str(output_dir / f'{name}_{version}_{arch}.deb')], check=True)


def main():
    parser = argparse.ArgumentParser()
    for flag in ('version','stage','output-dir','depends','gpu-depends'):
        parser.add_argument('--' + flag, required=True)
    args = parser.parse_args()
    package(args.stage, args.output_dir, args.depends, args.gpu_depends, args.version)


if __name__ == '__main__': main()
