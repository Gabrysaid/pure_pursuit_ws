"""Pure Pursuit controller node.

/odom  (nav_msgs/Odometry)  -> odometry callback only stores the latest state
timer  (control_rate Hz)    -> Pure Pursuit step, publishes /cmd_vel (Twist)

The control loop runs on its own timer so the command rate does not depend on
how fast odometry arrives.
"""
import math
import os

from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64

from pure_pursuit_controller.pure_pursuit_core import (
    load_waypoints, PurePursuit, quaternion_to_yaw)


class PurePursuitNode(Node):

    def __init__(self):
        super().__init__('pure_pursuit_node')
        # -------- parameters --------
        self.declare_parameter('waypoints_file', 'waypoints.csv')
        self.declare_parameter('wheelbase', 2.7)          # Prius <wheel_base> in the SDF
        self.declare_parameter('target_speed', 4.0)       # m/s
        self.declare_parameter('lookahead_min', 3.0)      # Ld at v = 0 (m)
        self.declare_parameter('lookahead_gain', 0.4)     # k in Ld = Ld_min + k v (s)
        self.declare_parameter('lookahead_max', 15.0)     # m
        self.declare_parameter('max_steer', 0.5)          # rad, SDF <steering_limit>
        self.declare_parameter('closed_loop', True)
        self.declare_parameter('laps', 1)                 # closed loop: laps before stop
        self.declare_parameter('control_rate', 20.0)      # Hz
        self.declare_parameter('odom_timeout', 0.5)       # s without odom -> stop
        self.declare_parameter('goal_tolerance', 1.5)     # m, open paths only

        path_file = os.path.expanduser(self.get_parameter('waypoints_file').value)
        waypoints = load_waypoints(path_file)
        self.closed_loop = bool(self.get_parameter('closed_loop').value)
        self.laps = int(self.get_parameter('laps').value)
        self.target_speed = float(self.get_parameter('target_speed').value)
        self.odom_timeout = float(self.get_parameter('odom_timeout').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)

        self.pp = PurePursuit(
            waypoints,
            wheelbase=float(self.get_parameter('wheelbase').value),
            ld_min=float(self.get_parameter('lookahead_min').value),
            k=float(self.get_parameter('lookahead_gain').value),
            ld_max=float(self.get_parameter('lookahead_max').value),
            max_steer=float(self.get_parameter('max_steer').value),
            closed_loop=self.closed_loop)

        # tuning without restarting: ros2 param set /pure_pursuit_node lookahead_gain 0.8
        self.add_on_set_parameters_callback(self.on_params)

        # -------- state --------
        self.x = self.y = self.yaw = self.v = 0.0
        self.last_odom_time = None
        self.odom_frame = 'odom'
        self.finished = False
        self.start_index = None

        # -------- ROS interfaces --------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_pub = self.create_publisher(PointStamped, '~/target', 10)
        self.cte_pub = self.create_publisher(Float64, '~/cross_track_error', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback,
                                 qos_profile_sensor_data)
        rate = float(self.get_parameter('control_rate').value)
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info(
            f'Loaded {len(waypoints)} waypoints from {path_file}; '
            f'v = {self.target_speed} m/s, Ld = {self.pp.ld_min} + '
            f'{self.pp.k} v, L = {self.pp.wheelbase} m, {rate:.0f} Hz')

    # ---------------- callbacks ----------------
    def odom_callback(self, msg):
        """Only state extraction here; no control logic."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.v = msg.twist.twist.linear.x
        self.odom_frame = msg.header.frame_id or self.odom_frame
        self.last_odom_time = self.get_clock().now()

    def control_loop(self):
        if self.finished:
            return
        if self.last_odom_time is None:
            self.get_logger().info('Waiting for /odom ...', throttle_duration_sec=2.0)
            return
        age = (self.get_clock().now() - self.last_odom_time).nanoseconds * 1e-9
        if age > self.odom_timeout:
            self.get_logger().warn(f'/odom is {age:.2f} s old, stopping',
                                   throttle_duration_sec=1.0)
            self.publish_cmd(0.0, 0.0)
            return

        delta, omega, target, ld = self.pp.compute(
            self.x, self.y, self.yaw, self.target_speed, self.v)
        if self.start_index is None:
            self.start_index = self.pp.nearest_idx

        # stop conditions
        progress = self.pp.nearest_idx - self.start_index
        if self.closed_loop and self.laps > 0 and progress >= self.laps * self.pp.n - 2:
            self.stop('Lap(s) completed')
            return
        if self.pp.goal_reached(self.x, self.y, self.goal_tolerance):
            self.stop('Goal reached')
            return

        self.publish_cmd(self.target_speed, omega)

        pt = PointStamped()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = self.odom_frame
        pt.point.x, pt.point.y = target
        self.target_pub.publish(pt)
        self.cte_pub.publish(Float64(data=self.pp.cross_track_error(self.x, self.y)))

        self.get_logger().debug(
            f'delta = {math.degrees(delta):.1f} deg, omega = {omega:.3f} rad/s, '
            f'Ld = {ld:.2f} m')

    def on_params(self, params):
        for p in params:
            if p.name == 'lookahead_gain':
                self.pp.k = float(p.value)
            elif p.name == 'lookahead_min':
                self.pp.ld_min = float(p.value)
            elif p.name == 'lookahead_max':
                self.pp.ld_max = float(p.value)
            elif p.name == 'target_speed':
                self.target_speed = float(p.value)
            elif p.name == 'max_steer':
                self.pp.max_steer = float(p.value)
            elif p.name == 'wheelbase':
                self.pp.wheelbase = float(p.value)
            elif p.name in ('waypoints_file', 'closed_loop', 'control_rate'):
                return SetParametersResult(
                    successful=False, reason=f'{p.name} is read only at runtime')
            self.get_logger().info(f'{p.name} -> {p.value}')
        return SetParametersResult(successful=True)

    # ---------------- helpers ----------------
    def publish_cmd(self, v, w):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    def stop(self, reason):
        self.publish_cmd(0.0, 0.0)
        self.finished = True
        self.get_logger().info(f'{reason}, vehicle stopped')


def main(args=None):
    # we handle Ctrl+C ourselves so the context is still alive to send a stop
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # leave the car stopped if we are interrupted mid-run
        if rclpy.ok():
            node.publish_cmd(0.0, 0.0)
            node.get_logger().info('Stop command sent')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
