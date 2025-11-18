import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
from ament_index_python.packages import get_package_share_directory

class UrdfPublisher(Node):
    def __init__(self):
        super().__init__('urdf_publisher')
        # 发布到/robot_description
        self.publisher = self.create_publisher(String, '/robot_description', 10)
        pkg_share = get_package_share_directory('lidar')
        urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')
        with open(urdf_path, 'r') as f:
            urdf_str = f.read()
        msg = String()
        msg.data = urdf_str
        self.publisher.publish(msg)
        self.get_logger().info('Published URDF to /robot_description')

def main(args=None):
    rclpy.init(args=args)
    node = UrdfPublisher()
    rclpy.spin_once(node, timeout_sec=1.0)  # 只发布一次
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()