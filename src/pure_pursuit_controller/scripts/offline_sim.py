#!/usr/bin/env python3
"""Offline check of the controller math, without ROS or Gazebo.

Integrates a kinematic bicycle model (L = 2.7 m, |delta| <= 0.5 rad, first
order steering lag) driven by the same PurePursuit class the ROS node uses,
on a synthetic closed circuit with straights, a chicane and tight corners.
It writes CSV files in the same format as path_recorder so analyze_tracking.py
can be tested before the Gazebo runs, and to sweep the look-ahead gain k.

  python3 offline_sim.py --out ~/pp_data/offline --k 0.0 0.3 0.6 1.0 1.5
"""
import argparse
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from pure_pursuit_controller.pure_pursuit_core import PurePursuit, save_points  # noqa: E402


def build_circuit(step=0.5):
    """Piecewise (length, curvature) circuit, closed by solving two straights."""
    def trace(segments):
        x = y = th = 0.0
        pts = [(x, y)]
        for length, kappa in segments:
            n = max(1, int(round(length / step)))
            ds = length / n
            for _ in range(n):
                if abs(kappa) < 1e-9:
                    x += ds * math.cos(th)
                    y += ds * math.sin(th)
                else:
                    th_new = th + kappa * ds
                    x += (math.sin(th_new) - math.sin(th)) / kappa
                    y -= (math.cos(th_new) - math.cos(th)) / kappa
                    th = th_new
                pts.append((x, y))
        return pts

    q = math.pi / 2

    def segs(a, b):
        return [
            (60.0, 0.0),                       # main straight
            (q * 14, 1 / 14), (25.0, 0.0),     # left hairpin half
            (q * 14, 1 / 14),
            (15.0, 0.0),                       # back straight with chicane
            (q / 2 * 12, -1 / 12), (q * 12, 1 / 12), (q / 2 * 12, -1 / 12),
            (a, 0.0),
            (q * 20, 1 / 20), (b, 0.0),        # wide corner
            (q * 10, 1 / 10),                  # tight corner back to start
        ]
    # solve a, b so the loop closes (the end point is linear in a and b)
    p0 = trace(segs(10.0, 10.0))[-1]
    pa = trace(segs(11.0, 10.0))[-1]
    pb = trace(segs(10.0, 11.0))[-1]
    ja = (pa[0] - p0[0], pa[1] - p0[1])
    jb = (pb[0] - p0[0], pb[1] - p0[1])
    det = ja[0] * jb[1] - ja[1] * jb[0]
    da = (-p0[0] * jb[1] + p0[1] * jb[0]) / det
    db = (-ja[0] * p0[1] + ja[1] * p0[0]) / det
    pts = trace(segs(10.0 + da, 10.0 + db))
    return pts[:-1]  # last point equals the first one


def teleop_like(path, spacing=0.5, noise=0.08, seed=1):
    """Resample at the recorder spacing and add small lateral noise, like a human driver."""
    rnd = random.Random(seed)
    out = [path[0]]
    for p in path[1:]:
        if math.dist(p, out[-1]) >= spacing:
            out.append(p)
    wobble = 0.0
    noisy = []
    for i, (x, y) in enumerate(out):
        a, b = out[i - 1], out[(i + 1) % len(out)]
        th = math.atan2(b[1] - a[1], b[0] - a[0])
        wobble = 0.9 * wobble + rnd.gauss(0.0, noise)
        noisy.append((x - wobble * math.sin(th), y + wobble * math.cos(th)))
    return noisy


def simulate(ref, v=4.0, k=0.6, ld_min=4.0, L=2.7, tau=0.15, dt=0.01, rate=20.0,
             max_steer=0.5):
    pp = PurePursuit(ref, wheelbase=L, ld_min=ld_min, k=k, max_steer=max_steer,
                     closed_loop=True)
    x, y, th = ref[0][0], ref[0][1], math.atan2(ref[1][1] - ref[0][1], ref[1][0] - ref[0][0])
    delta = 0.0
    speed = 0.0
    omega_cmd = 0.0
    t = 0.0
    next_ctrl = 0.0
    out = [(0.0, x, y)]
    start = None
    while t < 400.0:
        if t >= next_ctrl:  # controller runs on its own timer
            _, omega_cmd, _, _ = pp.compute(x, y, th, v, speed)
            if start is None:
                start = pp.nearest_idx
            if pp.nearest_idx - start >= pp.n - 2:
                break
            next_ctrl += 1.0 / rate
        # plant: speed and steering follow the command with first order lag
        speed += (v - speed) * dt / 0.5
        delta_cmd = math.atan(omega_cmd * L / v)  # what the Ackermann plugin does
        delta += (max(-max_steer, min(max_steer, delta_cmd)) - delta) * dt / tau
        x += speed * math.cos(th) * dt
        y += speed * math.sin(th) * dt
        th += speed * math.tan(delta) / L * dt
        t += dt
        if math.dist((x, y), out[-1][1:]) >= 0.5:
            out.append((t, x, y))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='offline')
    ap.add_argument('--k', type=float, nargs='+', default=[0.0, 0.3, 0.6, 1.0, 1.5])
    ap.add_argument('--speed', type=float, default=4.0)
    ap.add_argument('--ld-min', type=float, default=4.0)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    ref = teleop_like(build_circuit())
    save_points(os.path.join(args.out, 'waypoints.csv'), ref)
    for k in args.k:
        traj = simulate(ref, v=args.speed, k=k, ld_min=args.ld_min)
        name = f'actual_trajectory_k{k:.1f}'.replace('.', '')
        save_points(os.path.join(args.out, name + '.csv'), traj)
        print(f'k = {k:.1f}: {len(traj)} points, lap time {traj[-1][0]:.1f} s -> {name}.csv')


if __name__ == '__main__':
    main()
