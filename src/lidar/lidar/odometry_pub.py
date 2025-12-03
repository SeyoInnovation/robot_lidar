#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from uart_msg.msg import WheelData

import math
from math import sin, cos

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # 参数
        self.declare_parameter('wheel_base', 0.395)        # 左右轮距 m
        self.declare_parameter('wheel_diameter', 0.130)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('gyro_weight', 0.15)        # 0=纯轮子，1=纯陀螺仪

        self.L = self.get_parameter('wheel_base').value
        self.gyro_weight = self.get_parameter('gyro_weight').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # 状态
        self.x = self.y = self.theta = 0.0
        self.last_time = self.get_clock().now()

        # 发布
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 订阅原始数据
        self.sub = self.create_subscription(
            WheelData, 'wheel_raw_data', self.callback, 10)

    def callback(self, msg: WheelData):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt < 1e-6:
            return
        self.last_time = current_time

        v_l = msg.left_velocity
        v_r = msg.right_velocity
        omega_gyro = math.radians(msg.gyro_z)          # °/s → rad/s

        # 差速模型计算线速度和角速度
        v = (v_l + v_r) / 2.0
        omega_wheel = (v_r - v_l) / self.L

        # 简单融合（一阶互补）
        omega = (1.0 - self.gyro_weight) * omega_wheel + self.gyro_weight * omega_gyro

        # 位姿积分
        self.theta += omega * dt
        self.x += v * cos(self.theta) * dt
        self.y += v * sin(self.theta) * dt

        # 四元数
        cy = cos(self.theta * 0.5)
        sy = sin(self.theta * 0.5)
        qx = qy = 0.0
        qz = sy
        qw = cy

        # 发布 Odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        # 协方差（可后期标定）
        odom.pose.covariance[0]  = 0.005
        odom.pose.covariance[7]  = 0.005
        odom.pose.covariance[35] = 0.03
        odom.twist.covariance[0]  = 0.01
        odom.twist.covariance[35] = 0.03

        self.odom_pub.publish(odom)

        # 发布 TF
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()