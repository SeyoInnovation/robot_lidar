#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from rclpy.time import Time
import math

class CmdVelToJointStates(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_joint_states')

        # === 参数（可通过 launch 文件覆盖）===
        self.declare_parameter('wheel_separation', 0.20)   # 两轮中心距（米）
        self.declare_parameter('wheel_diameter', 0.064)    # 轮子直径（米）
        self.declare_parameter('cmd_vel_topic', '/robot/cmd_vel')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('rate', 50.0)  # 发布频率

        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_diameter').value / 2.0
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.joint_states_topic = self.get_parameter('joint_states_topic').value
        self.rate_hz = self.get_parameter('rate').value

        # === 状态变量 ===
        self.left_vel = 0.0
        self.right_vel = 0.0
        self.left_pos = 0.0
        self.right_pos = 0.0
        self.last_time = self.get_clock().now()

        # === 订阅与发布 ===
        self.subscription = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10)

        self.joint_pub = self.publisher = self.create_publisher(
            JointState, self.joint_states_topic, 10)

        self.timer = self.create_timer(1.0 / self.rate_hz, self.publish_joint_states)

        self.get_logger().info(f"启动成功！监听 {self.cmd_vel_topic} → 发布 {self.joint_states_topic}")
        self.get_logger().info(f"轮距: {self.wheel_separation}m, 轮径: {self.wheel_radius*2:.4f}m")

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # 经典差速反解
        self.right_vel = linear_x + 0.5 * self.wheel_separation * angular_z
        self.left_vel  = linear_x - 0.5 * self.wheel_separation * angular_z

    def publish_joint_states(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt > 0.5:  # 防止长时间停机导致位置爆炸
            dt = 0.0

        # 积分得到位置（弧度）
        self.left_pos  += self.left_vel  * dt / self.wheel_radius
        self.right_pos += self.right_vel * dt / self.wheel_radius

        js = JointState()
        js.header.stamp = now.to_msg()
        js.header.frame_id = ''

        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [self.left_pos, self.right_pos]
        js.velocity = [self.left_vel / self.wheel_radius, self.right_vel / self.wheel_radius]
        js.effort = [0.0, 0.0]

        self.joint_pub.publish(js)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToJointStates()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()