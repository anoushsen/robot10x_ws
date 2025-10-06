import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64MultiArray, Bool, ColorRGBA
from visualization_msgs.msg import Marker
import numpy as np
import tf_transformations
import time


class PurePursuitController(Node):
    def __init__(self):
        super().__init__('trajectory_pure_pursuit')

        # Subscribers
        self.sub_traj = self.create_subscription(Float64MultiArray, '/trajectory', self.traj_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odometry/wheels', self.odom_cb, 10)

        # Publishers
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_done = self.create_publisher(Bool, '/mission_complete', 10)

        # RViz visualization publishers
        self.path_pub = self.create_publisher(Path, '/visualized_path', 10)
        self.lookahead_pub = self.create_publisher(Marker, '/lookahead_point', 10)
        self.robot_marker_pub = self.create_publisher(Marker, '/robot_marker', 10)

        # Parameters
        self.min_lookahead = 0.6
        self.max_lookahead = 1.2
        self.linear_speed = 0.25
        self.goal_tolerance = 0.2
        self.max_angular = 0.5
        self.steering_smooth = 0.8

        # State
        self.trajectory = None
        self.current_pose = None
        self.prev_omega = 0.0

        self.get_logger().info("Pure Pursuit Controller initialized.")
        self.create_timer(0.05, self.control_loop)

    # ---------- Callbacks ----------
    def traj_cb(self, msg):
        try:
            self.trajectory = np.array(msg.data).reshape(-1, 6)
            self.publish_path_marker()
        except Exception as e:
            self.get_logger().error(f"Invalid trajectory format: {e}")

    def odom_cb(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.current_pose = np.array([p.x, p.y, yaw])
        self.publish_robot_marker(p.x, p.y, yaw)

    # ---------- Visualization ----------
    def publish_path_marker(self):
        if self.trajectory is None:
            return
        path = Path()
        path.header.frame_id = 'map'
        for pt in self.trajectory:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            path.poses.append(pose)
        self.path_pub.publish(path)

    def publish_robot_marker(self, x, y, yaw):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.orientation.z = np.sin(yaw / 2.0)
        marker.pose.orientation.w = np.cos(yaw / 2.0)
        marker.scale.x = 0.3
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        self.robot_marker_pub.publish(marker)

    def publish_lookahead_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        marker.scale.x = marker.scale.y = marker.scale.z = 0.15
        marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        self.lookahead_pub.publish(marker)

    # ---------- Control ----------
    def control_loop(self):
        if self.trajectory is None or self.current_pose is None:
            return

        traj = self.trajectory
        x, y, yaw = self.current_pose
        x_goal, y_goal, _ = traj[-1, :3]
        dist_to_goal = np.hypot(x - x_goal, y - y_goal)

        # Stop when close to goal
        if dist_to_goal < self.goal_tolerance:
            self.pub_cmd.publish(Twist())
            self.pub_done.publish(Bool(data=True))
            self.get_logger().info("Goal reached. Stopping robot.")
            self.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            return

        L_d = np.clip(dist_to_goal * 0.6, self.min_lookahead, self.max_lookahead)
        x_ld, y_ld = self.find_lookahead_point(x, y, traj, L_d)
        if x_ld is None:
            return

        # Publish lookahead marker
        self.publish_lookahead_marker(x_ld, y_ld)

        dx = np.cos(-yaw) * (x_ld - x) - np.sin(-yaw) * (y_ld - y)
        dy = np.sin(-yaw) * (x_ld - x) + np.cos(-yaw) * (y_ld - y)
        dy = -dy  # coordinate fix for Gazebo’s ENU frame

        curvature = (2 * dy) / (L_d ** 2)
        omega = np.clip(self.linear_speed * curvature, -self.max_angular, self.max_angular)
        omega = self.steering_smooth * self.prev_omega + (1 - self.steering_smooth) * omega
        self.prev_omega = omega

        cmd = Twist()
        cmd.linear.x = float(self.linear_speed)
        cmd.angular.z = float(omega)
        self.pub_cmd.publish(cmd)

    def find_lookahead_point(self, x, y, traj, L_d):
        dists = np.hypot(traj[:, 0] - x, traj[:, 1] - y)
        idx = np.where(dists >= L_d)[0]
        if len(idx) == 0:
            return None, None
        return traj[idx[0], 0], traj[idx[0], 1]


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C received — stopping robot.")
        try:
            node.pub_cmd.publish(Twist())
            time.sleep(0.1)
        except Exception:
            pass
    finally:
        node.get_logger().info("Shutting down controller cleanly.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

