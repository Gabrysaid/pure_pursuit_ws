#!/usr/bin/env python3
"""Builds report.pdf (cover + 3 page body + references) from the analysis output.

  python3 make_report.py                      # draft with offline results only
  python3 make_report.py --gazebo ~/pp_data/results --gazebo-sweep ~/pp_data/results_sweep

Anything that still depends on Gazebo data is highlighted in yellow in the PDF.
"""
import argparse
import json
import os
import shutil

HERE = os.path.dirname(os.path.abspath(__file__))

TEAM = [
    ('Rebeca Saldaña Espinoza', 'A01735498'),
    ('José Alfredo Guerrero Fernández', 'A01736556'),
    ('Jose David Bautista Zuñiga', 'A01734266'),
    ('Sina Jovy', 'A01833561'),
    ('Gabriel Said Vera Carballo', 'A00839097'),
    ('Luis Héctor Muñoz Del Razo', 'A01738595'),
]
REPO = 'https://github.com/Gabrysaid/pure_pursuit_ws'
DATE = 'September 27, 2026'


def pending(text):
    return f'<span class="pending">{text}</span>'


def load(folder, name='metrics.json'):
    if not folder:
        return None
    path = os.path.join(os.path.expanduser(folder), name)
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return json.load(f)


def fmt(v):
    return f'{v:.2f}'


def table_rows(runs):
    rows = []
    for label, r in runs.items():
        rows.append(
            f'<tr><td>{label}</td><td>{fmt(r["all"]["mean"])}</td><td>{fmt(r["all"]["rms"])}'
            f'</td><td>{fmt(r["all"]["p95"])}</td><td>{fmt(r["all"]["max"])}</td>'
            f'<td>{fmt(r["straight"]["mean"])}</td><td>{fmt(r["curve"]["mean"])}</td></tr>')
    return '\n'.join(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gazebo', help='analyze_tracking output for the main Gazebo run')
    ap.add_argument('--gazebo-sweep', help='analyze_tracking output for the Gazebo k sweep')
    ap.add_argument('--offline', default=os.path.join(HERE, '..', 'results', 'offline'))
    ap.add_argument('--out', default=os.path.join(HERE, 'report.pdf'))
    args = ap.parse_args()

    build = os.path.join(HERE, 'build')
    os.makedirs(build, exist_ok=True)

    gz = load(args.gazebo)
    gz_sweep = load(args.gazebo_sweep)

    def copy_fig(folder, name, new):
        src = os.path.join(os.path.expanduser(folder), name)
        shutil.copy(src, os.path.join(build, new))
        return new

    # ---------- figures ----------
    if gz:
        fig_overlay = copy_fig(args.gazebo, 'overlay.svg', 'gz_overlay.svg')
        fig_profile = copy_fig(args.gazebo, 'error_profile.svg', 'gz_profile.svg')
        main_run = next(iter(gz['runs'].values()))
        ref_len = gz['reference_length_m']
        ref_pts = gz['reference_points']
        src_txt = 'Gazebo Sim 10, Prius model'
    else:
        fig_overlay = copy_fig(args.offline, 'overlay.svg', 'off_overlay.svg')
        fig_profile = copy_fig(args.offline, 'error_profile.svg', 'off_profile.svg')
        main_run = None
        ref_len = ref_pts = None
        src_txt = pending('OFFLINE placeholder: replace with the Gazebo run')
    sweep_folder = args.gazebo_sweep if gz_sweep else args.offline
    fig_sweep = copy_fig(sweep_folder, 'sweep.svg', 'sweep.svg')

    # ---------- numbers used in the text ----------
    if main_run:
        m_all, m_st, m_cu = main_run['all'], main_run['straight'], main_run['curve']
        mean_txt = fmt(m_all['mean'])
        rms_txt = fmt(m_all['rms'])
        max_txt = fmt(m_all['max'])
        st_txt, cu_txt = fmt(m_st['mean']), fmt(m_cu['mean'])
        ratio_txt = f'{m_cu["mean"] / max(m_st["mean"], 1e-9):.1f}'
        ref_txt = f'{ref_pts} waypoints ({ref_len:.0f} m)'
    else:
        mean_txt = rms_txt = max_txt = st_txt = cu_txt = ratio_txt = pending('X.XX')
        ref_txt = pending('N waypoints (L m)')

    gz_table = (table_rows(gz_sweep['runs']) if gz_sweep else
                (table_rows(gz['runs']) if gz else
                 f'<tr><td colspan="7">{pending("Gazebo runs pending (k = 0.0, 0.4, 1.0)")}'
                 '</td></tr>'))
    sw = {}
    if gz_sweep:
        for label, r in gz_sweep['runs'].items():
            sw[float(label.split('=')[1].split()[0])] = r
    ks = sorted(sw)

    def sw_mean(k, part='all'):
        return fmt(sw[k][part]['mean'])
    best_k = min(ks, key=lambda k: sw[k]['all']['mean']) if ks else None
    if ks:
        sweep_txt = (
            'The sweep shows the expected U shape. With a short look-ahead (k = 0, '
            f'L<sub>d</sub> = 3 m) the geometric gain 2L/L<sub>d</sub><sup>2</sup> is high and, '
            'combined with the steering lag, the car overshoots every corner and loops around the '
            f'path: the mean CTE is {sw_mean(0.0)} m and the maximum {fmt(sw[0.0]["all"]["max"])} m. '
            'That run also stopped before closing the lap, because the forward index search '
            'advanced while the car was looping. Increasing k damps the response: at k = 0.2 s '
            f'the mean drops to {sw_mean(0.2)} m. Between k = 0.4 and 0.8 s the curve is flat '
            f'({sw_mean(0.4)} and {sw_mean(0.8)} m); the longer preview keeps improving the '
            f'straights ({sw_mean(0.4, "straight")} to {sw_mean(0.8, "straight")} m) but the curve '
            f'error already grows ({sw_mean(0.4, "curve")} to {sw_mean(0.8, "curve")} m). At '
            f'k = 1.2 s the chord to the target cuts the inside of the corners, the curve error '
            f'rises to {sw_mean(1.2, "curve")} m and the mean to {sw_mean(1.2)} m. Making '
            'L<sub>d</sub> proportional to speed keeps the preview time constant, so the same k '
            f'transfers across speeds. The lowest mean CTE was at k = {best_k} s, but its '
            'difference with k = 0.4 s (0.02 m) is smaller than what a single run can resolve. '
            'We selected k = 0.4 s with L<sub>d,min</sub> = 3 m because it gives the lowest '
            'curve error and the lowest maximum error, i.e. the best compromise between '
            'oscillation and corner cutting.')
    else:
        sweep_txt = pending('Gazebo sweep pending.')
    sweep_src = ('Gazebo' if gz_sweep else
                 'offline kinematic bicycle (v = 8 m/s, L<sub>d,min</sub> = 1.5 m, '
                 '150 ms steering lag)')

    team_html = '<br>'.join(f'{n} &nbsp;&nbsp; {m}' for n, m in TEAM)

    html = f"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<title>M4 Activity 2 Pure Pursuit</title>
<style>
@page {{ size: Letter; margin: 2.2cm 2.2cm 2.0cm 2.2cm; }}
body {{ font-family: 'Times New Roman', 'Liberation Serif', serif; font-size: 12pt;
       line-height: 1.22; color: #111; margin: 0; }}
h1 {{ font-size: 13pt; margin: 9pt 0 3pt; break-after: avoid; page-break-after: avoid; }}
p {{ margin: 0 0 5pt; text-align: justify; }}
.cover {{ height: 23cm; display: flex; flex-direction: column; justify-content: center;
          text-align: center; page-break-after: always; }}
.cover .inst {{ font-size: 18pt; font-weight: bold; }}
.cover .campus {{ font-size: 15pt; margin-bottom: 1.6cm; }}
.cover .lbl {{ font-weight: bold; margin-top: 0.55cm; }}
.cover a {{ color: #1a4fa0; }}
.eq {{ text-align: center; margin: 3pt 0 5pt; font-style: italic; }}
.eq span.n {{ float: right; font-style: normal; }}
figure {{ margin: 4pt 0 6pt; text-align: center; }}
figure img {{ width: 100%; }}
figcaption {{ font-size: 10pt; text-align: left; margin-top: 2pt; }}
.row {{ display: flex; gap: 10pt; align-items: flex-start; }}
.row > * {{ flex: 1; }}
table {{ border-collapse: collapse; font-size: 10pt; width: 100%; margin: 3pt 0 4pt; }}
th, td {{ border-bottom: 0.6pt solid #999; padding: 1.5pt 4pt; text-align: center; }}
th {{ border-top: 0.9pt solid #333; border-bottom: 0.9pt solid #333; }}
.tcap {{ font-size: 10pt; margin-top: 4pt; }}
.pending {{ background: #fff2a8; }}
code {{ font-family: 'DejaVu Sans Mono', monospace; font-size: 9.5pt; }}
.refs p {{ padding-left: 18pt; text-indent: -18pt; text-align: left; font-size: 11.5pt; }}
.break {{ page-break-before: always; }}
</style></head><body>

<section class="cover">
  <div class="inst">Tecnológico de Monterrey</div>
  <div class="campus">Campus Puebla</div>
  <div class="lbl">Course</div><div>Movilidad Inteligente (MR3004C.601)</div>
  <div class="lbl">Assignment</div>
  <div>M4 Activity 2: Implementing a Pure Pursuit Controller in ROS 2</div>
  <div class="lbl">Professor</div><div>Arturo Daniel Sosa Cerón</div>
  <div class="lbl">Team</div><div>{team_html}</div>
  <div class="lbl">GitHub repository</div><div><a href="{REPO}">{REPO}</a></div>
  <div class="lbl">Date</div><div>{DATE}</div>
</section>

<h1>1. Implementation</h1>
<p>We created the <code>ament_python</code> package <code>pure_pursuit_controller</code> with two
nodes and a ROS free module (<code>pure_pursuit_core.py</code>) that holds all the math, so it can be
unit tested without a simulator (6 pytest cases: yaw conversion, straight line, turn direction,
bicycle mapping and index monotonicity over two laps). <b>path_recorder</b> subscribes to
<code>/odom</code> (<code>nav_msgs/Odometry</code>) and appends (x, y) only when the Euclidean
displacement from the last stored point exceeds <code>min_distance</code> = 0.5 m. The list is
written to CSV from a <code>finally</code> block and from a context <code>on_shutdown</code> hook,
so Ctrl+C always produces the file. The same node, launched with another output name, logs the
executed path during autonomous runs.</p>
<p><b>pure_pursuit_node</b> keeps clean callback boundaries: the odometry callback only stores the
state (x, y, &theta;, v) and a timestamp, while a 20 Hz timer runs the controller and publishes
<code>/cmd_vel</code> (<code>geometry_msgs/Twist</code>). This decouples the command rate from the
odometry rate. If odometry is older than 0.5 s the node commands zero velocity, it stops by itself
after one lap, and on Ctrl+C it sends a final stop because we disable the default rclpy signal handler.
All gains are ROS parameters loaded from <code>config/pure_pursuit.yaml</code> and can be changed live
through a parameter callback (<code>ros2 param set /pure_pursuit_node lookahead_gain 0.8</code>).
For debugging, the node also publishes the target point and the signed cross track error.
<code>launch/pure_pursuit.launch.py</code> includes <code>prius_bringup/gz_sim.launch.py</code> and,
after a short delay, starts the controller and the trajectory logger. Command-line arguments set
the speed, look-ahead and output file name. The reference was recorded with
<code>teleop_twist_keyboard</code> using <code>record_path.launch.py</code>: {ref_txt}.</p>

<h1>2. Control formulation</h1>
<p>The planar heading is extracted from the odometry quaternion with the ZYX convention:</p>
<div class="eq">&theta; = atan2( 2(q<sub>w</sub>q<sub>z</sub> + q<sub>x</sub>q<sub>y</sub>),
 1 &minus; 2(q<sub>y</sub><sup>2</sup> + q<sub>z</sub><sup>2</sup>) ) <span class="n">(1)</span></div>
<p>The look-ahead distance grows with speed, L<sub>d</sub> = L<sub>d,min</sub> + k&middot;v, clipped
to 15 m. To avoid index jumps, the closest waypoint is searched only forward from the previous one
inside a 60 point window, using an unwrapped counter so the closed loop is crossed without
jumping back to index 0. Starting there, we advance until the first waypoint at a distance
&ge; L<sub>d</sub> from the car; that is the target (x<sub>t</sub>, y<sub>t</sub>). With
&alpha; = atan2(y<sub>t</sub> &minus; y, x<sub>t</sub> &minus; x) &minus; &theta; (wrapped to
(&minus;&pi;, &pi;]), the arc through the car and the target has curvature
&kappa; = 2 sin &alpha; / L<sub>d</sub>. For a bicycle model with wheelbase L, tan &delta; = L&kappa;,
which gives the steering law and, since d&theta;/dt = v&kappa;, the yaw rate sent as
<code>angular.z</code>:</p>
<div class="eq">&delta; = tan<sup>&minus;1</sup>( 2L sin &alpha; / L<sub>d</sub> ), &nbsp;&nbsp;
|&delta;| &le; 0.5 rad <span class="n">(2)</span></div>
<div class="eq">d&theta;/dt = v tan &delta; / L <span class="n">(3)</span></div>
<p>L = 2.7 m and the 0.5 rad limit are taken from the <code>&lt;wheel_base&gt;</code> and
<code>&lt;steering_limit&gt;</code> tags of the Prius <code>AckermannSteering</code> plugin. The
plugin inverts (3) with the same L, so the commanded &delta; is exactly what reaches the front
wheels. The commanded speed is used in (3) and the measured speed in L<sub>d</sub>. Defaults:
v = 4 m/s, L<sub>d,min</sub> = 3 m, k = 0.4 s.</p>

<h1>3. Tracking performance</h1>
<p>The cross track error (CTE) is the distance from every logged point to the closest segment of
the reference polyline (<code>scripts/analyze_tracking.py</code>). Reference points whose curvature,
computed over a 10 m stencil to filter the teleoperation wobble, exceeds 0.02 m<sup>&minus;1</sup>
(R &lt; 50 m) are labeled curve; the rest are straights. All numbers come from Gazebo runs
({src_txt}, v = 4 m/s) against the recorded reference, {ref_txt}. The teleoperated drive was
trimmed where it rejoins the start straight (0.19 m gap), so the loop closes with a sharp
junction.</p>
<figure><img src="{fig_overlay}" style="width:68%">
<figcaption><b>Fig. 1.</b> Reference path (waypoints.csv) against the executed trajectory
(actual_trajectory_main.csv) in the odometry frame, equal axis scale.</figcaption></figure>
<figure><img src="{fig_profile}" style="width:88%">
<figcaption><b>Fig. 2.</b> Absolute CTE along the reference path (k = 0.4 s); shaded regions are
curves. The spikes near 45 m are points of the S-bend excursion that lie closest to the start
straight.</figcaption></figure>
<p>For the default gains the mean CTE over the whole lap is {mean_txt} m (RMS {rms_txt} m,
maximum {max_txt} m). On straights the mean is {st_txt} m and on curves {cu_txt} m, {ratio_txt}
times larger. Two effects explain the difference. First, corner cutting: the target point is
L<sub>d</sub> ahead, so it enters the curve before the car and the controller turns early, following
a chord inside the arc. Second, steering lag: when we commanded &omega; = 0.3 rad/s in Gazebo
the odometry reported only 0.275 rad/s after the transient, because the steering joints need time to
reach the commanded angle. In the tight S-bend (R &asymp; 6 m, close to the minimum radius
L/tan 0.5 = 4.9 m) the car saturates the steering, overshoots and loops once before
rejoining the path, which produces the maximum error.</p>

<h1>4. Look-ahead tuning</h1>
<div class="row">
<figure style="flex:0.9"><img src="{fig_sweep}">
<figcaption><b>Fig. 3.</b> Mean CTE versus look-ahead gain k (L<sub>d,min</sub> = 3 m,
v = 4 m/s), {sweep_src}.</figcaption></figure>
<div style="flex:1.1">
<table><tr><th>Run</th><th>Mean</th><th>RMS</th><th>P95</th><th>Max</th><th>Straight</th>
<th>Curve</th></tr>
{gz_table}
</table>
<div class="tcap"><b>Table 1.</b> CTE in metres, one Gazebo run per k, simulation restarted
before each run.</div>
</div></div>
<p>{sweep_txt}</p>
<p><b>Conclusion.</b> The package builds with colcon without warnings, runs the controller on a
fixed-rate timer and reproduces the recorded path with a mean error of {mean_txt} m at 4 m/s.
The main limitation is the tight S-bend, where the steering limit and lag dominate. A
curvature-dependent speed profile or a Stanley term for the lateral error would reduce it.</p>

<section class="refs break">
<h1>References</h1>
<p>[1] R. C. Coulter, "Implementation of the Pure Pursuit Path Tracking Algorithm," Carnegie
Mellon University, Robotics Institute, Tech. Rep. CMU-RI-TR-92-01, 1992.</p>
<p>[2] J. M. Snider, "Automatic Steering Methods for Autonomous Automobile Path Tracking,"
Carnegie Mellon University, Robotics Institute, Tech. Rep. CMU-RI-TR-09-08, 2009.</p>
<p>[3] R. Rajamani, <i>Vehicle Dynamics and Control</i>, 2nd ed. New York: Springer, 2012.</p>
<p>[4] S. Macenski, T. Foote, B. Gerkey, C. Lalancette, and W. Woodall, "Robot Operating System 2:
Design, architecture, and uses in the wild," <i>Science Robotics</i>, vol. 7, no. 66, 2022.</p>
<p>[5] Open Robotics, "ROS 2 Lyrical Luth documentation," docs.ros.org/en/lyrical, 2026.</p>
<p>[6] Open Robotics, "Gazebo Sim AckermannSteering system," gazebosim.org/api/sim/10, 2026.</p>
<p>[7] A. D. Sosa Cerón, "movilidad_inteligente" course repository (prius_bringup),
github.com/dsosa114/movilidad_inteligente, 2026.</p>
<h1>Appendix: use of AI</h1>
<p>Claude (Anthropic) was used as a programming assistant to draft the package code, the
analysis scripts and a first version of this text. The team reviewed, tested and edited all of
it, ran the simulations and is responsible for the results.</p>
</section>
</body></html>"""
    html_path = os.path.join(build, 'report.html')
    with open(html_path, 'w') as f:
        f.write(html)

    from weasyprint import HTML
    HTML(html_path).write_pdf(args.out)
    print('written', args.out)


if __name__ == '__main__':
    main()
