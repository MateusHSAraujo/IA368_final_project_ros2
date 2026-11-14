#!/usr/bin/env python3
"""Prepare scene_house.ttt (with /myRobot pasted in) for the ROS 2 stack.

1. Makes the static house shapes (walls, floor, furniture, doors) respondable, so the
   robot collides with them. In the original scene only 5 shapes were respondable and
   the robot could drive through the walls. Invisible shapes (roof) and the robot are
   left untouched.
2. Enables CoppeliaSim's real-time mode, so simulation time does not run ahead of the
   wall clock used to stamp the ROS messages.
3. Installs the fast depth-camera laser (see install_fast_laser.py).

The scene is modified in memory only: save it in CoppeliaSim afterwards (File > Save).

Usage (CoppeliaSim on Windows with scene_house.ttt open, simulation stopped):
    python3 -u scripts/coppelia/prepare_scene_house.py [--host IP] [--dry-run]
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
import install_fast_laser as fast_laser  # noqa: E402

ROOTS = ('/house', '/objects')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', default=None)
    parser.add_argument('--dry-run', action='store_true', help='Only report what would change')
    args = parser.parse_args()
    sim = fast_laser.connect(args.host or fast_laser.detect_windows_host_ip() or '127.0.0.1')
    if sim.getSimulationState() != sim.simulation_stopped:
        sys.exit('Stop the simulation first.')
    scene = sim.getStringParam(sim.stringparam_scene_path_and_name)
    if not scene.lower().endswith('scene_house.ttt'):
        sys.exit(f'Expected scene_house.ttt to be open, found: {scene}')

    changed, skipped_invisible, already = [], [], 0
    for root_path in ROOTS:
        root = sim.getObject(root_path, {'noError': True})
        if root == -1:
            continue
        for h in sim.getObjectsInTree(root, sim.sceneobject_shape):
            alias = sim.getObjectAlias(h, 2)
            if not sim.getBoolProperty(h, 'visible'):
                skipped_invisible.append(alias)
                continue
            if not sim.getObjectInt32Param(h, sim.shapeintparam_static):
                continue  # dynamic shapes keep their settings
            if sim.getObjectInt32Param(h, sim.shapeintparam_respondable):
                already += 1
                continue
            changed.append(alias)
            if not args.dry_run:
                sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 1)
    print(f'Respondable: {len(changed)} shapes changed, {already} already respondable, '
          f'{len(skipped_invisible)} invisible skipped {skipped_invisible}')

    realtime = sim.getBoolParam(sim.boolparam_realtime_simulation)
    if not realtime and not args.dry_run:
        sim.setBoolParam(sim.boolparam_realtime_simulation, True)
    print(f'Real-time mode: {"already on" if realtime else "enabled"}')

    if not args.dry_run:
        fast_laser.install(sim)
    print('Done. Save the scene in CoppeliaSim to keep the changes.' if not args.dry_run else 'Dry run: nothing changed.')


if __name__ == '__main__':
    main()
