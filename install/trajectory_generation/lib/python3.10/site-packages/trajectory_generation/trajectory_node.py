import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray, Bool
import numpy as np
import tf_transformations


class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')

        # Subscribers
        self.sub = self.create_subscription(Path, '/smooth_path', self.path_cb, 10)
        self.sub_done = self.create_subscription(Bool, '/mission_complete', self.done_cb, 10)

        # Publisher
        self.pub = self.create_publisher(Float64MultiArray, '/trajectory', 10)

        # Parameters
        self.dt = 0.1  # time step for derivative and resampling
        self.v_nominal = 0.4  # nominal linear velocity
        self._shutdown_flag = False

        self.get_logger().info("Trajectory Generator initialized (with yaw unwrap and safety clipping).")

    # -------------------- Mission Complete Handler --------------------
    def done_cb(self, msg):
        if msg.data:
            self.get_logger().info("Mission complete signal received. Shutting down trajectory_generator.")
            self._shutdown_flag = True
            self.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    # -------------------- Path Callback --------------------
    def path_cb(self, msg):
        if self._shutdown_flag:
            return
        if len(msg.poses) < 2:
            self.get_logger().warn("Received empty or short path.")
            return

        try:
            # Extract (x, y) coordinates
            x = np.array([p.pose.position.x for p in msg.poses])
            y = np.array([p.pose.position.y for p in msg.poses])

            # Compute yaw from path tangent direction
            dx = np.gradient(x)
            dy = np.gradient(y)
            yaw = np.unwrap(np.arctan2(dy, dx))  # unwrap prevents large jumps
            yaw = -yaw
            
            # Arc-length parametrization
            ds = np.hypot(dx, dy)
            s = np.cumsum(ds)
            s -= s[0]

            # Compute time along path (assuming nominal velocity)
            t = s / self.v_nominal
            v = np.full_like(x, self.v_nominal)  # constant linear velocity
            dyaw_dt = np.gradient(yaw, t)
            w = -dyaw_dt

            # Clean up angular rates
            w = np.nan_to_num(w, nan=0.0, posinf=0.0, neginf=0.0)
            w = np.clip(w, -1.0, 1.0)  # saturate angular velocity

            # Stack trajectory [x, y, yaw, v, w, t]
            traj = np.column_stack((x, y, yaw, v, w, t))

            # Convert to Python float list for publishing
            msg_out = Float64MultiArray()
            msg_out.data = [float(v) for v in traj.flatten()]

            self.pub.publish(msg_out)
            self.get_logger().info(f"Published trajectory with {len(traj)} points.")

        except Exception as e:
            self.get_logger().error(f"Error generating trajectory: {e}")

# -------------------- Safe Shutdown --------------------
def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C received — shutting down generator.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

