import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import numpy as np
from scipy.interpolate import splprep, splev
import tf_transformations
import os


class BSplinePathSmoother(Node):
    def __init__(self):
        super().__init__('path_smoother')

        # Publishers / Subscribers
        self.pub = self.create_publisher(Path, '/smooth_path', 10)
        self.sub_done = self.create_subscription(Bool, '/mission_complete', self.done_cb, 10)

        # Parameters
        self.timer_period = 2.0  # publish every 2 sec
        self.timer = self.create_timer(self.timer_period, self.publish_smooth_path)
        self._shutdown_flag = False

        self.get_logger().info("B-spline path smoother initialized.")

    def done_cb(self, msg):
        """Handle mission complete signal."""
        if msg.data:
            self.get_logger().info("Mission complete signal received. Shutting down path_smoother.")
            self._shutdown_flag = True
            self.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    def publish_smooth_path(self):
        """Read waypoints.txt → smooth using cubic B-spline → publish /smooth_path."""
        if self._shutdown_flag:
            return

        try:
            waypoints_path = os.path.join(os.path.expanduser('~'), 'robot10x_ws', 'src', 'path_smoothing', 'waypoints.txt')
            waypoints = np.loadtxt(waypoints_path)
        except Exception as e:
            self.get_logger().error(f"Failed to read waypoints.txt: {e}")
            return

        if waypoints.shape[1] < 2:
            self.get_logger().warn("Waypoints file must have at least two columns (x, y).")
            return

        x_points, y_points = waypoints[:, 0], waypoints[:, 1]
        if len(x_points) < 3:
            self.get_logger().warn("Not enough points for B-spline smoothing.")
            return

        try:
            # --- Fit cubic B-spline (clamped ends) ---
            tck, u = splprep([x_points, y_points], s=0.05, k=3)

            # --- Evaluate spline densely ---
            u_dense = np.linspace(0, 1, 200)
            x_dense, y_dense = splev(u_dense, tck)

            # --- Compute arc-length and re-sample uniformly ---
            dx, dy = np.gradient(x_dense), np.gradient(y_dense)
            ds = np.hypot(dx, dy)
            s = np.cumsum(ds)
            s /= s[-1]
            s_target = np.linspace(0, 1, len(u_dense))
            x_s, y_s = np.interp(s_target, s, x_dense), np.interp(s_target, s, y_dense)

            # --- Compute heading from tangent ---
            dx, dy = np.gradient(x_s), np.gradient(y_s)
            yaw = np.arctan2(dy, dx)

            # --- Build nav_msgs/Path message ---
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            for i in range(len(x_s)):
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = float(x_s[i])
                pose.pose.position.y = float(y_s[i])
                q = tf_transformations.quaternion_from_euler(0, 0, yaw[i])
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
                path_msg.poses.append(pose)

            # --- Publish smoothed path ---
            self.pub.publish(path_msg)
            self.get_logger().info(f"Published B-spline path with {len(path_msg.poses)} poses.")

        except Exception as e:
            self.get_logger().error(f"B-spline smoothing failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = BSplinePathSmoother()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C received — shutting down smoother.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

