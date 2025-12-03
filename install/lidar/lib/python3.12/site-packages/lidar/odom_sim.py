#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
from geometry_msgs.msg import Twist

class FakeOdomNode(Node):
    def __init__(self):
        super().__init__('fake_odom_publisher')

        # 参数：可以用键盘或 joy 控制速度
        self.declare_parameter('linear_speed', 0.2)   # m/s
        self.declare_parameter('angular_speed', 0.0)  # rad/s

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        # 机器人当前位置（模拟积分）
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = self.get_clock().now()

        # 发布者
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 可选：订阅 cmd_vel 来控制（真实机器人就是这个话题）
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # 50Hz 发布假里程计
        self.timer = self.create_timer(0.01, self.publish_odom)

        self.get_logger().info("假里程计节点已启动！用键盘或 joy 发 /cmd_vel 就能动了")

    def cmd_vel_callback(self, msg):
        # 实时更新目标速度（可以用 teleop_twist_keyboard 控制）
        self.linear_speed = msg.linear.x
        self.angular_speed = msg.angular.z

    def publish_odom(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 简单积分得到位姿（带一点噪声更真实，可删）
        distance = self.linear_speed * dt
        theta_dot = self.angular_speed * dt

        self.x += distance * math.cos(self.theta + theta_dot / 2.0)
        self.y += distance * math.sin(self.theta + theta_dot / 2.0)
        self.theta += theta_dot
        self.theta = self.theta % (2 * math.pi)

        # === 发布 Odometry 消息 ===
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        q = self.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # 速度（body 坐标系）
        odom.twist.twist.linear.x = self.linear_speed
        odom.twist.twist.angular.z = self.angular_speed

        # 随便填个协方差（不影响建图）
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 0.01
        odom.pose.covariance[21] = 0.01
        odom.pose.covariance[28] = 0.01
        odom.pose.covariance[35] = 0.01

        self.odom_pub.publish(odom)

        # === 发布 TF (odom → base_link) ===
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return [x, y, z, w]


def main(args=None):
    rclpy.init(args=args)
    node = FakeOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()