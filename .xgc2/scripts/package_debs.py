#!/usr/bin/env python3
"""Package the installed standalone server and its native runtime assets."""
import argparse, pathlib, subprocess

def package(stage, output, dependencies):
    stage=pathlib.Path(stage);output=pathlib.Path(output)
    server=stage/'opt/xgc2/xsim/bin/xsim'
    if not server.is_file():raise ValueError('installed xsim executable is required')
    if not (stage/'usr/share/xgc2/process-definitions/xsim-world.json').is_file():
        raise ValueError('installed xsim-world process definition is required')
    if (stage/'opt/ros').exists():raise ValueError('ROS-owned payload is not part of xsim')
    arch=subprocess.check_output(['dpkg','--print-architecture'],text=True).strip()
    control=stage/'DEBIAN';control.mkdir(exist_ok=True)
    (control/'control').write_text(f'''Package: xsim
Version: 1.2.0-1
Architecture: {arch}
Maintainer: XGC Team <dev@xgc.team>
Depends: {dependencies}
Description: Standalone ECS world and integrated CPU/GPU point cloud server
''')
    subprocess.run(['dpkg-deb','--build','--root-owner-group',str(stage),str(output)],check=True)

def main():
    p=argparse.ArgumentParser();p.add_argument('--stage',required=True);p.add_argument('--output',required=True);p.add_argument('--depends',required=True);a=p.parse_args()
    package(a.stage,a.output,a.depends)
if __name__=='__main__':main()
