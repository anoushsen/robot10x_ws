import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray
import numpy as np

class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        self.sub = self.create_subscription(Path, '/smooth_path', self.callback, 10)
        self.pub = self.create_publisher(Float64MultiArray, '/trajectory', 10)

    def callback(self, path):
        v = 0.2  # constant m/s
        t = 0.0
        traj = Float64MultiArray()
        for i in range(1, len(path.poses)):
            p1, p2 = path.poses[i-1], path.poses[i]
            dx = p2.pose.position.x - p1.pose.position.x
            dy = p2.pose.position.y - p1.pose.position.y
            dist = np.hypot(dx, dy)
            t += dist / v
            traj.data.extend([p2.pose.position.x, p2.pose.position.y, t])
        self.pub.publish(traj)
        self.get_logger().info('Published trajectory with time stamps')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    rclpy.spin(node)
    rclpy.shutdown()



