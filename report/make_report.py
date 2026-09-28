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
        ref_txt = f'{ref_pts} waypoints over a total distance of {ref_len:.0f} m'
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
        k0_max = fmt(sw[0.0]['all']['max'])
        sweep_txt = (
            '<p>With a short look-ahead, such as k = 0, the look-ahead distance stays at 3 m. '
            'This makes the geometric gain 2L/L<sub>d</sub><sup>2</sup> relatively high. '
            'Together with the steering lag, this causes the vehicle to overshoot the corners '
            f'and loop around the path. The mean CTE in this case was {sw_mean(0.0)} m and the '
            f'maximum error was {k0_max} m. The vehicle also stopped before completing the lap '
            'because the forward index continued advancing while the vehicle was looping.</p>'
            f'<p>Increasing k reduces this behavior. At k = 0.2 s, the mean CTE decreased to '
            f'{sw_mean(0.2)} m. Between k = 0.4 and k = 0.8 s, the mean error changed only '
            f'slightly, from {sw_mean(0.4)} to {sw_mean(0.8)} m. During this range, the longer '
            'look-ahead improved the performance on straight sections, reducing the error from '
            f'{sw_mean(0.4, "straight")} to {sw_mean(0.8, "straight")} m, but the curve error '
            f'increased from {sw_mean(0.4, "curve")} to {sw_mean(0.8, "curve")} m. At k = 1.2 s, '
            'the target point is farther ahead and the controller starts cutting the inside of '
            f'the corners more. As a result, the curve error increases to {sw_mean(1.2, "curve")} '
            f'm and the mean CTE increases to {sw_mean(1.2)} m.</p>'
            '<p>Because L<sub>d</sub> is proportional to speed, the preview time remains '
            'approximately constant. This means the same value of k can be used at different '
            f'speeds. The lowest mean CTE in the tests was obtained with k = {best_k} s, but the '
            f'difference between k = {best_k} and k = 0.4 was only '
            f'{abs(sw[0.4]["all"]["mean"] - sw[best_k]["all"]["mean"]):.2f} m, which is smaller '
            'than what can be reliably distinguished from a single run. For this reason, we '
            'selected k = 0.4 s with L<sub>d,min</sub> = 3 m. This configuration gave a lower '
            'curve error and a lower maximum error while keeping a good balance between '
            'oscillation and corner cutting.</p>')
    else:
        sweep_txt = '<p>' + pending('Gazebo sweep pending.') + '</p>'
    sweep_src = ('Gazebo' if gz_sweep else
                 'offline kinematic bicycle (v = 8 m/s, L<sub>d,min</sub> = 1.5 m, '
                 '150 ms steering lag)')

    team_html = '<br>'.join(f'{n} &nbsp;&nbsp; {m}' for n, m in TEAM)

    html = f"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<title>M4 Activity 2 Pure Pursuit</title>
<style>
@page {{ size: Letter; margin: 2.0cm; }}
body {{ font-family: 'Times New Roman', 'Liberation Serif', serif; font-size: 12pt;
       line-height: 1.15; color: #111; margin: 0; }}
sub, sup {{ line-height: 0; }}
h1 {{ font-size: 13pt; margin: 6pt 0 2pt; break-after: avoid; page-break-after: avoid; }}
p {{ margin: 0 0 4pt; text-align: justify; }}
.cover {{ height: 23cm; display: flex; flex-direction: column; justify-content: center;
          text-align: center; page-break-after: always; }}
.cover .inst {{ font-size: 18pt; font-weight: bold; }}
.cover .campus {{ font-size: 15pt; margin-bottom: 1.6cm; }}
.cover .lbl {{ font-weight: bold; margin-top: 0.55cm; }}
.cover a {{ color: #1a4fa0; }}
.eq {{ text-align: center; margin: 2pt 0 3pt; font-style: italic; }}
.eq span.n {{ float: right; font-style: normal; }}
figure {{ margin: 2pt 0 4pt; text-align: center; }}
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
<p>We created the <code>ament_python</code> package <code>pure_pursuit_controller</code>, which
contains two nodes and a ROS-free module called <code>pure_pursuit_core.py</code>. This module
contains the main mathematical calculations, which allowed us to test them without running the
simulator. We used 6 pytest cases to check yaw conversion, straight-line motion, turn direction,
bicycle model mapping, and index monotonicity over two laps.</p>
<p>The <b>path_recorder</b> node subscribes to <code>/odom</code> (<code>nav_msgs/Odometry</code>)
and stores the (x, y) position only when the Euclidean distance from the last stored point is
greater than <code>min_distance</code> = 0.5 m. The points are saved to a CSV file both from a
<code>finally</code> block and an <code>on_shutdown</code> hook, so the file is still generated
when the program is stopped with Ctrl+C. The same node can also be launched with a different output
name to record the path followed during autonomous runs.</p>
<p>The <b>pure_pursuit_node</b> separates the odometry callback from the controller. The callback
only stores the current state (x, y, &theta;, v) and its timestamp, while a 20 Hz timer runs the
controller and publishes <code>/cmd_vel</code> using <code>geometry_msgs/Twist</code>. This keeps
the controller update rate independent from the odometry rate. If the odometry data is older than
0.5 s, the node commands zero velocity. It also stops automatically after one lap and sends a final
stop command when Ctrl+C is pressed. For this, the default <code>rclpy</code> signal handler is
disabled. All controller gains are loaded as ROS parameters from
<code>config/pure_pursuit.yaml</code> and can also be changed while the node is running, for
example with <code>ros2 param set /pure_pursuit_node lookahead_gain 0.8</code>. The node also
publishes the target point and the signed cross-track error to help with debugging.</p>
<p>The launch file <code>launch/pure_pursuit.launch.py</code> includes
<code>prius_bringup/gz_sim.launch.py</code>. After a short delay, it starts the controller and the
trajectory logger. The speed, look-ahead parameters, and output file name can be changed using
command-line arguments. The reference path was recorded using <code>teleop_twist_keyboard</code>
and <code>record_path.launch.py</code>, resulting in {ref_txt}.</p>

<h1>2. Control formulation</h1>
<p>The vehicle heading in the plane is obtained from the odometry quaternion using the ZYX
convention:</p>
<div class="eq">&theta; = atan2( 2(q<sub>w</sub>q<sub>z</sub> + q<sub>x</sub>q<sub>y</sub>),
 1 &minus; 2(q<sub>y</sub><sup>2</sup> + q<sub>z</sub><sup>2</sup>) ) <span class="n">(1)</span></div>
<p>The look-ahead distance changes with the vehicle speed according to
L<sub>d</sub> = L<sub>d,min</sub> + k&middot;v, with a maximum value of 15 m. To avoid jumping
between different parts of the path, we search for the closest waypoint only in the forward
direction from the previous waypoint and within a 60-point window. An unwrapped index is used so
the controller can pass through the end of the closed path without going back to index 0. From the
closest point, we move forward until finding the first waypoint whose distance from the vehicle is
at least L<sub>d</sub>. This waypoint is used as the target point (x<sub>t</sub>, y<sub>t</sub>).
The angle to the target, &alpha; = atan2(y<sub>t</sub> &minus; y, x<sub>t</sub> &minus; x)
&minus; &theta;, is wrapped to the interval (&minus;&pi;, &pi;]. Using this angle, the curvature
of the arc between the vehicle and the target point is &kappa; = 2 sin &alpha; / L<sub>d</sub>.
For the bicycle model, the relationship between curvature and steering angle is
tan &delta; = L&kappa;. Therefore, the steering angle used by the controller is:</p>
<div class="eq">&delta; = tan<sup>&minus;1</sup>( 2L sin &alpha; / L<sub>d</sub> ), &nbsp;&nbsp;
|&delta;| &le; 0.5 rad <span class="n">(2)</span></div>
<p>Since d&theta;/dt = v&kappa;, the yaw rate sent as <code>angular.z</code> is:</p>
<div class="eq">d&theta;/dt = v tan &delta; / L <span class="n">(3)</span></div>
<p>The wheelbase L = 2.7 m and the 0.5 rad steering limit were taken from the
<code>&lt;wheel_base&gt;</code> and <code>&lt;steering_limit&gt;</code> tags in the Prius
<code>AckermannSteering</code> plugin. The plugin uses the same value of L when converting the
command, so the steering angle calculated by the controller is the same one applied to the front
wheels. The commanded speed is used in the yaw-rate equation (3), while the measured speed is used
to calculate L<sub>d</sub>. The default parameters are v = 4 m/s, L<sub>d,min</sub> = 3 m, and
k = 0.4 s.</p>

<h1>3. Tracking performance</h1>
<p>The cross-track error (CTE) is calculated as the distance between each logged point and the
closest segment of the reference path. This calculation is done in
<code>scripts/analyze_tracking.py</code>. To separate curves from straight sections, we calculated
the curvature of the reference path using a 10 m stencil. Points with a curvature greater than
0.02 m<sup>&minus;1</sup>, equivalent to R &lt; 50 m, were classified as curves. The remaining
points were classified as straights. All results were obtained from {src_txt} at 4 m/s,
comparing the autonomous trajectory with the recorded reference path of {ref_txt}. The
teleoperated path was trimmed at the point where it returned to the starting straight, leaving a
0.19 m gap. This creates a sharp connection when the loop is closed.</p>
<figure><img src="{fig_overlay}" style="width:72%">
<figcaption><b>Fig. 1.</b> Reference path (waypoints.csv) against the executed trajectory
(actual_trajectory_main.csv) in the odometry frame, with equal axis scale.</figcaption></figure>
<figure><img src="{fig_profile}" style="width:74%">
<figcaption><b>Fig. 2.</b> Absolute CTE along the reference path (k = 0.4 s). Shaded regions
represent curves. The spikes near 45 m correspond to points from the S-bend excursion that are
closest to the starting straight.</figcaption></figure>
<p>For the default parameters, the mean CTE over the complete lap was {mean_txt} m, with an RMS
error of {rms_txt} m and a maximum error of {max_txt} m. On straight sections, the mean CTE was
{st_txt} m, while on curves it increased to {cu_txt} m, which is {ratio_txt} times higher. There
are two main reasons for this difference. The first one is corner cutting. Since the target point
is located L<sub>d</sub> ahead of the vehicle, it can already be inside a curve before the vehicle
reaches it. This makes the controller start turning earlier and follow a chord inside the curve
instead of following the reference path exactly. The second factor is steering lag. When we
commanded &omega; = 0.3 rad/s in Gazebo, the odometry reported only 0.275 rad/s after the
transient. This happens because the steering joints need some time to reach the commanded angle.
The effect becomes more important in the tight S-bend, where the radius is approximately 6 m. This
is close to the minimum radius of L/tan(0.5) = 4.9 m. Because of the steering limit, the vehicle
saturates the steering, overshoots the path, and makes one loop before returning to the reference.
This produces the maximum tracking error.</p>

<h1>4. Look-ahead tuning</h1>
<p>We tested different values of the look-ahead gain k, keeping L<sub>d,min</sub> = 3 m and the
speed at 4 m/s. Each value was tested in one Gazebo run, and the simulation was restarted before
every run.</p>
<div class="row">
<figure style="flex:0.8"><img src="{fig_sweep}">
<figcaption><b>Fig. 3.</b> Mean CTE versus look-ahead gain k (L<sub>d,min</sub> = 3 m,
v = 4 m/s), {sweep_src}.</figcaption></figure>
<div style="flex:1.2">
<table><tr><th>Run</th><th>Mean</th><th>RMS</th><th>P95</th><th>Max</th><th>Straight</th>
<th>Curve</th></tr>
{gz_table}
</table>
<div class="tcap"><b>Table 1.</b> CTE in metres. One Gazebo run was performed for each value of
k, with the simulation restarted before each run.</div>
</div></div>
{sweep_txt}
<p><b>Conclusion.</b> The package builds successfully with <code>colcon</code> without warnings
and runs the controller using a fixed-rate timer. At 4 m/s, the controller was able to reproduce
the recorded path with a mean tracking error of {mean_txt} m. The main limitation was the tight
S-bend, where the steering limit and the delay in the steering response had the greatest effect on
the trajectory. A speed profile that changes according to the curvature or the addition of a
Stanley term for lateral error could help reduce the tracking error in this section.</p>

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
<p>[8] Anthropic, Claude [AI assistant], claude.ai, 2026. Used mainly for translation of the
report and for programming support (code drafting, debugging and simulation scripts). The team recorded the
reference path by teleoperation and reviewed the results and the final content.</p>
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
