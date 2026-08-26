#!/usr/bin/env python3
"""Spawn and optionally move an intruder drone model in a Gazebo world.

Modes:
  flying - spawn the static-flagged `intruder_drone` model and teleport it
           along a crossing route through the patrol area (cheap: no physics,
           no flight controller).
  static - spawn it once, parked in the patrol path.
  none   - do nothing (exit immediately).

Uses the `gz service` CLI, so it works against a running gz sim server.
"""

import argparse
import subprocess
import sys
import time


def gz_service(world, service, reqtype, reptype, req, timeout=3000):
    cmd = [
        'gz', 'service', '-s', service,
        '--reqtype', reqtype, '--reptype', reptype,
        '--timeout', str(timeout), '--req', req]
    return subprocess.run(cmd, capture_output=True, text=True)


def wait_for_world(world, timeout_s=120):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        out = subprocess.run(['gz', 'service', '-l'],
                             capture_output=True, text=True)
        if f'/world/{world}/create' in out.stdout:
            return True
        time.sleep(1.0)
    return False


def spawn(world, name, x, y, z):
    req = (f'name: "{name}", file: "intruder_drone", '
           f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}')
    r = gz_service(world, f'/world/{world}/create',
                   'gz.msgs.EntityFactory', 'gz.msgs.Boolean', req)
    if r.returncode != 0:
        print(f'spawn failed: {r.stderr.strip()}', file=sys.stderr)
        return False
    print(f'intruder spawned at ({x}, {y}, {z})')
    return True


def set_pose(world, name, x, y, z):
    req = f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}'
    gz_service(world, f'/world/{world}/set_pose',
               'gz.msgs.Pose', 'gz.msgs.Boolean', req, timeout=1000)


def main():
    ap = argparse.ArgumentParser(description='Intruder drone simulator')
    ap.add_argument('--mode', choices=['flying', 'static', 'none'],
                    default='flying')
    ap.add_argument('--world', default='default')
    ap.add_argument('--name', default='intruder')
    ap.add_argument('--alt', type=float, default=65.0,
                    help='cruise altitude (m)')
    ap.add_argument('--y', type=float, default=60.0,
                    help='crossing line offset east of origin (m)')
    ap.add_argument('--speed', type=float, default=12.0, help='m/s')
    args = ap.parse_args()

    if args.mode == 'none':
        print('intruder disabled')
        return

    if not wait_for_world(args.world):
        print('gz world never appeared - aborting', file=sys.stderr)
        sys.exit(1)

    # start west of the patrol box, fly east through it
    x = -400.0
    if not spawn(args.world, args.name, x, args.y, args.alt):
        sys.exit(1)

    if args.mode == 'static':
        set_pose(args.world, args.name, 0.0, args.y, args.alt)
        print('intruder parked (static mode)')
        return

    step_dt = 0.2
    step_m = args.speed * step_dt
    print('intruder flying east at %.0f m/s' % args.speed)
    try:
        while True:
            x += step_m
            set_pose(args.world, args.name, x, args.y, args.alt)
            time.sleep(step_dt)
            if x > 400.0:          # loop back for continuous testing
                x = -400.0
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
