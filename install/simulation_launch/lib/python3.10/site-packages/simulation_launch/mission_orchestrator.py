import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64MultiArray
import time


class MissionOrchestrator(Node):
    def __init__(self):
        super().__init__('mission_orchestrator')
        self.get_logger().info("Mission orchestrator starting...")

        # Clients
        self.cli_smooth = self.create_client(Trigger, '/generate_path')
        self.cli_traj = self.create_client(Trigger, '/generate_trajectory')
        self.cli_track = self.create_client(Trigger, '/start_tracking')

        # Data flags
        self.path_received = False
        self.traj_received = False
        self.odom_received = False

        # Lightweight topic listeners
        self.sub_path = self.create_subscription(Path, '/smooth_path', self._path_cb, 10)
        self.sub_traj = self.create_subscription(Float64MultiArray, '/trajectory', self._traj_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odometry/wheels', self._odom_cb, 10)

        # Wait for service availability
        for cli, name in [
            (self.cli_smooth, '/generate_path'),
            (self.cli_traj, '/generate_trajectory'),
            (self.cli_track, '/start_tracking')
        ]:
            self.get_logger().info(f"Waiting for {name}...")
            cli.wait_for_service()
            self.get_logger().info(f"{name} available.")

        # Start mission
        self.call_generate_path()

    # ---- topic callbacks ----
    def _path_cb(self, msg):
        if not self.path_received:
            self.path_received = True
            self.get_logger().info("Received first smoothed path message.")

    def _traj_cb(self, msg):
        if not self.traj_received:
            self.traj_received = True
            self.get_logger().info("Received first trajectory message.")

    def _odom_cb(self, msg):
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info("Received first odometry message.")

    # ---- services ----
    def call_generate_path(self):
        self.get_logger().info("Calling /generate_path...")
        req = Trigger.Request()
        fut = self.cli_smooth.call_async(req)
        fut.add_done_callback(self.path_done_cb)

    def path_done_cb(self, fut):
        res = fut.result()
        if res.success:
            self.get_logger().info(f"Path OK: {res.message}")
            self.verify_path_ready_then_traj()
        else:
            self.get_logger().error(f"Path FAILED: {res.message}")
            self.shutdown()

    def verify_path_ready_then_traj(self):
        """Wait for smoothed path data before calling trajectory generator."""
        self.get_logger().info("Waiting for smoothed path data to arrive...")
        timeout = 5.0
        start = time.time()
        while time.time() - start < timeout:
            if self.path_received:
                self.get_logger().info("Smoothed path confirmed. Proceeding to trajectory generation.")
                self.call_generate_trajectory()
                return
            time.sleep(0.2)
        self.get_logger().warn("No smoothed path data received within timeout. Proceeding anyway.")
        self.call_generate_trajectory()

    def call_generate_trajectory(self):
        self.get_logger().info("Calling /generate_trajectory...")
        req = Trigger.Request()
        fut = self.cli_traj.call_async(req)
        fut.add_done_callback(self.traj_done_cb)

    def traj_done_cb(self, fut):
        res = fut.result()
        if res.success:
            self.get_logger().info(f"Trajectory OK: {res.message}")
            self.call_start_tracking()
        else:
            self.get_logger().error(f"Trajectory FAILED: {res.message}")
            self.shutdown()

    def call_start_tracking(self):
        """Wait until /trajectory and /odometry/wheels have both produced at least one message."""
        self.get_logger().info("Verifying that trajectory and odometry data are flowing...")

        timeout = 10.0
        start = time.time()
        while time.time() - start < timeout:
            if self.traj_received and self.odom_received:
                self.get_logger().info("Trajectory and odometry data confirmed. Proceeding to /start_tracking.")
                break
            time.sleep(0.2)
        else:
            self.get_logger().warn("No data received on required topics within timeout. Proceeding anyway (may fail).")

        # Call tracking
        req = Trigger.Request()
        fut = self.cli_track.call_async(req)
        fut.add_done_callback(self.track_done_cb)

    def track_done_cb(self, fut):
        res = fut.result()
        if res.success:
            self.get_logger().info("Tracking started successfully.")
        else:
            self.get_logger().error(f"Tracking failed: {res.message}")
        self.get_logger().info("Mission orchestrator finished.")
        self.shutdown()

    def shutdown(self):
        self.get_logger().info("Shutting down orchestrator.")
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

