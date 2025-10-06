import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import numpy as np


class TrajectoryController(Node):
    def __init__(self):
        super().__init__('trajectory_controller')

        # ---- parameters ----
        self.lookahead = 0.4          # m
        self.linear_speed = 0.2       # m/s
        self.goal_tolerance = 0.1     # m
        self.goal_reached = False

        # ---- data holders ----
        self.traj = None
        self.current_pose = None

        # ---- publishers & subscribers ----
        self.sub_traj = self.create_subscription(
            Float64MultiArray, '/trajectory', self.traj_cb, 10)
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ---- control loop ----
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        self.get_logger().info('Trajectory controller node started.')

    # ----------------------------------------------------------
    def traj_cb(self, msg):
        """Receive trajectory array (x, y, t)."""
        try:
            self.traj = np.array(msg.data).reshape(-1, 3)
            self.goal_reached = False
            self.get_logger().info(f"Received trajectory with {len(self.traj)} points.")
        except Exception as e:
            self.get_logger().error(f"Trajectory reshape failed: {e}")

    # ----------------------------------------------------------
    def odom_cb(self, odom):
        """Update current position."""
        self.current_pose = odom.pose.pose.position

    # ----------------------------------------------------------
    def control_loop(self):
        """Compute and publish velocity commands at 10 Hz."""
        if self.traj is None or self.current_pose is None or self.goal_reached:
            return

        x, y = self.current_pose.x, self.current_pose.y
        goal_x, goal_y, _ = self.traj[-1]
        dist_to_goal = np.hypot(goal_x - x, goal_y - y)

        # ---- check if goal reached ----
        if dist_to_goal < self.goal_tolerance:
            self.get_logger().info('Goal reached! Stopping robot.')
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)
            self.goal_reached = True
            return

        # ---- pure pursuit steering ----
        dists = np.hypot(self.traj[:, 0] - x, self.traj[:, 1] - y)
        idx = np.argmin(dists)
        lookahead_idx = min(idx + 5, len(self.traj) - 1)
        goal = self.traj[lookahead_idx]

        dx, dy = goal[0] - x, goal[1] - y
        alpha = np.arctan2(dy, dx)

        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = 2 * cmd.linear.x * np.sin(alpha) / self.lookahead
        self.cmd_pub.publish(cmd)


# ----------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down trajectory controller.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

