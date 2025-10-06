import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger

class MissionOrchestrator(Node):
    def __init__(self):
        super().__init__('mission_orchestrator')
        self.get_logger().info("Mission orchestrator starting...")

        self.cli_smooth = self.create_client(Trigger, '/generate_path')
        self.cli_traj = self.create_client(Trigger, '/generate_trajectory')
        self.cli_track = self.create_client(Trigger, '/start_tracking')

        for cli, name in [
            (self.cli_smooth, '/generate_path'),
            (self.cli_traj, '/generate_trajectory'),
            (self.cli_track, '/start_tracking')
        ]:
            self.get_logger().info(f"Waiting for {name}...")
            cli.wait_for_service()
            self.get_logger().info(f"{name} available.")

        self.call_generate_path()

    def call_generate_path(self):
        self.get_logger().info("Calling /generate_path...")
        req = Trigger.Request()
        fut = self.cli_smooth.call_async(req)
        fut.add_done_callback(self.path_done_cb)

    def path_done_cb(self, fut):
        res = fut.result()
        if res.success:
            self.get_logger().info(f"Path OK: {res.message}")
            self.call_generate_trajectory()
        else:
            self.get_logger().error(f"Path FAILED: {res.message}")
            self.shutdown()

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
        self.get_logger().info("Calling /start_tracking...")
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
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MissionOrchestrator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

