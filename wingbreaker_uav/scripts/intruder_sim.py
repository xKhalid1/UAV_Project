#!/usr/bin/env python3
"""Spawn N intruder drones and fly each in a fixed circular orbit.

Modes:
  flying - spawn the requested number of static-flagged `intruder_x500` models
           and teleport each along its own circular orbit around the patrol
           area (cheap: no physics, no flight controller).
  static - spawn them once, parked at their orbit start points.
  none   - do nothing (exit immediately).

Uses the `gz service` CLI, so it works against a running gz sim server.
"""

import argparse
import os
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


def find_model_sdf():
    """Locate intruder_x500/model.sdf - works from source tree and install."""
    candidates = [
        os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     '..', '..', 'sim', 'models', 'intruder_x500',
                     'model.sdf'),
        os.path.expanduser(
            '~/UAV_Project/sim/models/intruder_x500/model.sdf'),
    ]
    for c in candidates:
        c = os.path.abspath(c)
        if os.path.isfile(c):
            return c
    print('model.sdf not found, tried:\n  ' + '\n  '.join(candidates),
          file=sys.stderr)
    return None


def spawn(world, name, x, y, z):
    model_sdf = find_model_sdf()
    if model_sdf is None:
        return False
    req = (f'name: "{name}", '
           f'sdf_filename: "{model_sdf}", '
           f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}')
    r = gz_service(world, f'/world/{world}/create',
                   'gz.msgs.EntityFactory', 'gz.msgs.Boolean', req)
    if r.returncode != 0:
        print(f'[{name}] spawn failed: {r.stderr.strip()}', file=sys.stderr)
        return False
    print(f'[{name}] spawned at ({x:.0f}, {y:.0f}, {z:.0f})')
    return True


def set_pose(world, name, x, y, z):
    req = f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}'
    gz_service(world, f'/world/{world}/set_pose',
               'gz.msgs.Pose', 'gz.msgs.Boolean', req, timeout=1000)


class Orbiter:
    """One intruder on a fixed circular orbit."""

    def __init__(self, idx, cx, cy, radius, alt, speed, phase, clockwise):
        self.name = f'intruder_{idx}'
        self.cx = cx
        self.cy = cy
        self.radius = radius
        self.alt = alt
        self.speed = speed            # tangential m/s
        self.omega = speed / radius   # rad/s
        sign = -1.0 if clockwise else 1.0
        self.theta0 = phase
        self.sign = sign
        self.t0 = None
        self.last = None

    def spawn_pos(self):
        th = self.theta0
        return (self.cx + self.radius * math.cos(th),
                self.cy + self.radius * math.sin(th),
                self.alt)

    def pos_at(self, t):
        th = self.theta0 + self.sign * self.omega * t
        return (self.cx + self.radius * math.cos(th),
                self.cy + self.radius * math.sin(th),
                self.alt)


import math


def main():
    ap = argparse.ArgumentParser(description='Intruder drone simulator (fixed orbit)')
    ap.add_argument('--mode', choices=['flying', 'static', 'none'],
                    default='flying')
    ap.add_argument('--world', default='runway_world')
    # orbit parameters (centre near the patrol box)
    ap.add_argument('--cx', type=float, default=37.0,
                    help='orbit centre east (m) - near patrol box centre')
    ap.add_argument('--cy', type=float, default=33.0,
                    help='orbit centre north (m)')
    ap.add_argument('--radius', type=float, default=70.0,
                    help='orbit radius (m)')
    ap.add_argument('--alt', type=float, default=65.0,
                    help='orbit altitude (m); match brain.patrol_alt for '
                         'a level camera view')
    ap.add_argument('--speed', type=float, default=12.0,
                    help='tangential orbit speed (m/s)')
    ap.add_argument('--num-intruders', type=int, default=3,
                    help='how many intruders to spawn, each on its own orbit')
    args = ap.parse_args()

    if args.mode == 'none':
        print('intruder disabled')
        return

    if not wait_for_world(args.world):
        print('gz world never appeared - aborting', file=sys.stderr)
        sys.exit(1)

    # build orbiters: same centre/radius/alt/speed, spread evenly in phase,
    # alternating direction so they don't bunch up.
    orbiters = []
    n = max(1, int(args.num_intruders))
    for i in range(n):
        phase = 2.0 * math.pi * i / n
        clockwise = (i % 2 == 1)
        orbiters.append(Orbiter(i, args.cx, args.cy, args.radius,
                                args.alt, args.speed, phase, clockwise))

    # spawn each at its phase-0 position so the very first set_pose (if any)
    # doesn't teleport unexpectedly.
    t_now = 0.0
    for o in orbiters:
        x, y, z = o.pos_at(t_now)
        if not spawn(args.world, o.name, x, y, z):
            sys.exit(1)
        o.t0 = time.time()

    if args.mode == 'static':
        # leave them parked at their phase-0 positions
        print(f'{n} intruders parked (static mode)')
        return

    step_dt = 0.2
    print(f'{n} intruders orbiting at r={args.radius:.0f}m '
          f'v={args.speed:.0f}m/s around ({args.cx:.0f},{args.cy:.0f}) '
          f'at alt {args.alt:.0f}m')
    try:
        while True:
            t_now = time.time()
            for o in orbiters:
                x, y, z = o.pos_at(t_now - o.t0)
                set_pose(args.world, o.name, x, y, z)
            time.sleep(step_dt)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
