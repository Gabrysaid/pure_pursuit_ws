"""Path recorder: stores (x, y) from /odom every time the car moves more than
min_distance, and dumps the list to CSV when the node shuts down (Ctrl+C).

Used twice: once to record the reference path (waypoints.csv) while driving
by teleop, and again during autonomous tracking (actual_trajectory.csv).
"""
import math
import os

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from pure_pursuit_controller.pure_pursuit_core import save_points


class PathRecorder(Node):

    def __init__(self):
        super().__init__('path_recorder')
        self.declare_parameter('output_file', 'waypoints.csv')
        self.declare_parameter('min_distance', 0.5)      # m between stored points
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('save_time', False)       # add a t column

        self.output_file = os.path.expanduser(
            self.get_parameter('output_file').value)
        self.min_distance = float(self.get_parameter('min_distance').value)
        self.save_time = bool(self.get_parameter('save_time').value)
        topic = self.get_parameter('odom_topic').value

        self.points = []
        self.saved = False
        self.t0 = None

        self.sub = self.create_subscription(
            Odometry, topic, self.odom_callback, qos_profile_sensor_data)
        # log progress once per second without touching the callback rate
        self.timer = self.create_timer(1.0, self.report)
        # safe exit: save even if the context is shut down from outside
        self.context.on_shutdown(self.save)

        self.get_logger().info(
            f'Recording {topic} -> {self.output_file} '
            f'(min_distance = {self.min_distance} m). Ctrl+C to save.')

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = stamp
        if self.points:
            last = self.points[-1]
            if math.hypot(x - last[-2], y - last[-1]) < self.min_distance:
                return
        if self.save_time:
            self.points.append((stamp - self.t0, x, y))
        else:
            self.points.append((x, y))

    def report(self):
        self.get_logger().info(f'{len(self.points)} points stored',
                               throttle_duration_sec=5.0)

    def save(self):
        if self.saved:
            return
        self.saved = True
        if not self.points:
            print('[path_recorder] no points received, nothing saved')
            return
        folder = os.path.dirname(self.output_file)
        if folder:
            os.makedirs(folder, exist_ok=True)
        save_points(self.output_file, self.points)
        # the logger may already be gone at shutdown, so print as well
        print(f'[path_recorder] saved {len(self.points)} points to '
              f'{os.path.abspath(self.output_file)}')


def main(args=None):
    rclpy.init(args=args)
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
