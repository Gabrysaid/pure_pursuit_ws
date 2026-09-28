#!/usr/bin/env python3
"""Tracking performance analysis for M4 Activity 2.

Compares the reference path (waypoints.csv) with one or more executed
trajectories (actual_trajectory*.csv) and reports the cross-track error:
the distance from every executed point to the closest reference segment.

Example
  python3 analyze_tracking.py --ref ~/pp_data/waypoints.csv \
      --run "k = 0.6" ~/pp_data/actual_trajectory.csv --out ~/pp_data/results

With several --run entries the script also plots the look-ahead gain sweep.
"""
import argparse
import csv
import json
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

# categorical order (fixed): blue, orange, aqua, yellow, magenta
COLORS = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4']
REF_COLOR = '#52514e'
CURVE_SHADE = '#ecebe6'

plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'Liberation Serif', 'DejaVu Serif'],
    'font.size': 9, 'axes.titlesize': 10, 'axes.labelsize': 9,
    'legend.fontsize': 8, 'axes.spines.top': False, 'axes.spines.right': False,
    'axes.edgecolor': '#8a8984', 'axes.grid': True, 'grid.color': '#e4e3de',
    'grid.linewidth': 0.6, 'lines.linewidth': 1.6, 'savefig.bbox': 'tight',
})


def read_xy(path):
    with open(os.path.expanduser(path), newline='') as f:
        rows = list(csv.DictReader(f))
    return np.array([[float(r['x']), float(r['y'])] for r in rows])


def arc_length(p):
    return np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))])


def curvature(p, window=10.0):
    """Menger curvature using points about +/- window/2 metres apart.

    A wide stencil filters the small lateral wobble of a teleop recording,
    so straights are not misclassified as curves.
    """
    n = len(p)
    spacing = np.median(np.linalg.norm(np.diff(p, axis=0), axis=1))
    span = max(2, int(round(window / 2 / max(spacing, 1e-3))))
    k = np.zeros(n)
    for i in range(n):
        a, b, c = p[(i - span) % n], p[i], p[(i + span) % n]
        ab, bc, ca = np.linalg.norm(b - a), np.linalg.norm(c - b), np.linalg.norm(a - c)
        area2 = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
        k[i] = 2.0 * area2 / max(ab * bc * ca, 1e-9)
    return k


def curve_mask(ref, threshold, min_length=4.0):
    """Reference points that belong to a curve; runs shorter than min_length
    metres (straights or curves) are absorbed by their neighbours."""
    mask = curvature(ref) > threshold
    step = np.median(np.linalg.norm(np.diff(ref, axis=0), axis=1))
    min_pts = max(1, int(round(min_length / max(step, 1e-3))))
    for value in (False, True):  # fill short straight gaps, then drop short curves
        i = 0
        while i < len(mask):
            if mask[i] == value:
                j = i
                while j < len(mask) and mask[j] == value:
                    j += 1
                if j - i < min_pts and i > 0 and j < len(mask):
                    mask[i:j] = not value
                i = j
            else:
                i += 1
    return mask


def cross_track(ref, pts, closed):
    """For each point: (distance to reference polyline, index of closest segment)."""
    a = ref if closed else ref[:-1]
    b = np.roll(ref, -1, axis=0) if closed else ref[1:]
    d = b - a
    seg2 = np.maximum((d ** 2).sum(axis=1), 1e-12)
    errors, idx = np.empty(len(pts)), np.empty(len(pts), dtype=int)
    for i, p in enumerate(pts):
        t = np.clip(((p - a) * d).sum(axis=1) / seg2, 0.0, 1.0)
        proj = a + t[:, None] * d
        dist = np.linalg.norm(p - proj, axis=1)
        j = int(np.argmin(dist))
        errors[i], idx[i] = dist[j], j
    return errors, idx


def metrics(e):
    if len(e) == 0:
        return {'mean': float('nan'), 'rms': float('nan'), 'p95': float('nan'),
                'max': float('nan'), 'n': 0}
    return {'mean': float(np.mean(e)), 'rms': float(np.sqrt(np.mean(e ** 2))),
            'p95': float(np.percentile(e, 95)), 'max': float(np.max(e)), 'n': int(len(e))}


def analyze(ref, run, closed, curve_threshold):
    e, seg = cross_track(ref, run, closed)
    is_curve = curve_mask(ref, curve_threshold)[seg]
    return {
        'errors': e, 'seg': seg, 'is_curve': is_curve,
        'all': metrics(e), 'straight': metrics(e[~is_curve]), 'curve': metrics(e[is_curve]),
    }


def plot_overlay(ref, runs, results, out, title):
    fig, ax = plt.subplots(figsize=(6.4, 4.2))
    ax.plot(ref[:, 0], ref[:, 1], '--', color=REF_COLOR, lw=1.4,
            label='Reference (waypoints.csv)')
    for i, (label, pts) in enumerate(runs):
        m = results[label]['all']
        ax.plot(pts[:, 0], pts[:, 1], color=COLORS[i % len(COLORS)],
                label=f'Executed, {label} (mean CTE {m["mean"]:.2f} m)')
    ax.plot(*ref[0], 'o', color=REF_COLOR, ms=6, label='Start')
    ax.set_aspect('equal', adjustable='datalim')
    ax.set_xlabel('x [m] (odom frame)')
    ax.set_ylabel('y [m] (odom frame)')
    ax.set_title(title)
    ax.legend(loc='best', frameon=False)
    for ext in ('pdf', 'svg', 'png'):
        fig.savefig(os.path.join(out, f'overlay.{ext}'), dpi=200)
    plt.close(fig)


def plot_error_profile(ref, runs, results, out, closed):
    s_ref = arc_length(ref if not closed else np.vstack([ref, ref[:1]]))
    fig, ax = plt.subplots(figsize=(6.4, 2.4))
    # shade curved sections of the reference
    first = results[runs[0][0]]
    kappa_mask = curve_mask(ref, first['curve_threshold'])
    start = None
    for i, c in enumerate(np.append(kappa_mask, False)):
        if c and start is None:
            start = i
        elif not c and start is not None:
            ax.axvspan(s_ref[start], s_ref[min(i, len(s_ref) - 1)], color=CURVE_SHADE, lw=0)
            start = None
    for i, (label, _) in enumerate(runs):
        r = results[label]
        s = s_ref[r['seg']]
        order = np.argsort(s, kind='stable')
        ax.plot(s[order], r['errors'][order], color=COLORS[i % len(COLORS)], lw=1.2,
                label=label)
    ax.set_xlabel('Distance along reference path [m]  (shaded = curve)')
    ax.set_ylabel('|Cross-track error| [m]')
    ax.set_ylim(bottom=0)
    if len(runs) > 1:
        ax.legend(frameon=False, ncol=min(len(runs), 5))
    for ext in ('pdf', 'svg', 'png'):
        fig.savefig(os.path.join(out, f'error_profile.{ext}'), dpi=200)
    plt.close(fig)


def plot_sweep(results, labels, values, out, xlabel):
    fig, ax = plt.subplots(figsize=(3.4, 2.4))
    for key, col, name in (('straight', COLORS[0], 'Straights'),
                           ('curve', COLORS[1], 'Curves')):
        ys = [results[lab][key]['mean'] for lab in labels]
        ax.plot(values, ys, '-o', color=col, ms=5, label=name)
    ax.set_xlabel(xlabel)
    ax.set_ylabel('Mean |CTE| [m]')
    ax.set_ylim(bottom=0)
    ax.legend(frameon=False)
    for ext in ('pdf', 'svg', 'png'):
        fig.savefig(os.path.join(out, f'sweep.{ext}'), dpi=200)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ref', required=True, help='reference CSV (x,y)')
    ap.add_argument('--run', nargs=2, action='append', metavar=('LABEL', 'CSV'),
                    required=True, help='executed trajectory, can be repeated')
    ap.add_argument('--sweep-values', type=float, nargs='*',
                    help='numeric value of each run (e.g. the k used) for the sweep plot')
    ap.add_argument('--sweep-label', default='Look-ahead gain k [s]')
    ap.add_argument('--open', action='store_true', help='reference is not a closed loop')
    ap.add_argument('--curve-threshold', type=float, default=0.02,
                    help='curvature [1/m] above which a section counts as curve (R < 50 m)')
    ap.add_argument('--title', default='Reference vs executed trajectory')
    ap.add_argument('--out', default='results')
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    ref = read_xy(args.ref)
    closed = not args.open
    runs = [(label, read_xy(path)) for label, path in args.run]

    results = {}
    for label, pts in runs:
        results[label] = analyze(ref, pts, closed, args.curve_threshold)
        results[label]['curve_threshold'] = args.curve_threshold

    plot_overlay(ref, runs, results, args.out, args.title)
    plot_error_profile(ref, runs, results, args.out, closed)
    if args.sweep_values and len(args.sweep_values) == len(runs) and len(runs) > 1:
        plot_sweep(results, [r[0] for r in runs], args.sweep_values, args.out,
                   args.sweep_label)

    summary = {
        'reference_points': int(len(ref)),
        'reference_length_m': float(arc_length(ref)[-1]),
        'curve_threshold_1_per_m': args.curve_threshold,
        'runs': {lab: {k: results[lab][k] for k in ('all', 'straight', 'curve')}
                 for lab, _ in runs},
    }
    with open(os.path.join(args.out, 'metrics.json'), 'w') as f:
        json.dump(summary, f, indent=2)

    print(f'Reference: {len(ref)} points, {summary["reference_length_m"]:.1f} m')
    print(f'{"run":<14}{"mean":>8}{"rms":>8}{"p95":>8}{"max":>8}'
          f'{"mean straight":>15}{"mean curve":>12}   [m]')
    for lab, _ in runs:
        r = results[lab]
        print(f'{lab:<14}{r["all"]["mean"]:8.3f}{r["all"]["rms"]:8.3f}'
              f'{r["all"]["p95"]:8.3f}{r["all"]["max"]:8.3f}'
              f'{r["straight"]["mean"]:15.3f}{r["curve"]["mean"]:12.3f}')
    print(f'Figures and metrics.json written to {os.path.abspath(args.out)}')


if __name__ == '__main__':
    main()
