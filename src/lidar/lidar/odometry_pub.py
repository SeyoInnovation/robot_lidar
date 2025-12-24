#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from uart_msg.msg import WheelData

import math

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # 参数
        self.declare_parameter('wheel_base', 0.177)          # 实际轮距，和 raw 节点保持一致！
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        self.L = self.get_parameter('wheel_base').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # 状态初始化
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_left_distance = 0.0
        self.last_right_distance = 0.0
        self.last_time = self.get_clock().now()

        # 发布者
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # 订阅原始轮子数据
        self.subscription = self.create_subscription(
            WheelData, 'wheel_raw_data', self.callback, 10)

        self.get_logger().info("OdometryNode 已启动（纯轮式里程计）")

    def callback(self, msg: WheelData):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        # 防止 dt 过小或首次异常
        if dt < 1e-6:
            return

        # 首次接收数据，只记录初始里程，不积分
        if not hasattr(self, 'initialized'):
            self.last_left_distance = msg.left_distance
            self.last_right_distance = msg.right_distance
            self.last_time = current_time
            self.initialized = True
            self.get_logger().info("里程计初始化完成")
            return

        # 计算本次左右轮行走增量（单位：米）
        d_l = msg.left_distance - self.last_left_distance
        d_r = msg.right_distance - self.last_right_distance

        self.last_left_distance = msg.left_distance
        self.last_right_distance = msg.right_distance
        self.last_time = current_time

        # 差速模型
        d_center = (d_l + d_r) / 2.0                    # 前进位移
        d_theta = (d_r - d_l) / self.L                  # 旋转角度

        # 位姿积分（使用中间姿态，精度更高）
        theta_mid = self.theta + d_theta / 2.0
        self.x += d_center * math.cos(theta_mid)
        self.y += d_center * math.sin(theta_mid)
        self.theta += d_theta

        # 角度归一化到 [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # 时间戳
        stamp = current_time.to_msg()

        # 发布 TF（如果启用）
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0

            # 四元数转换
            q = self.euler_to_quaternion(0, 0, self.theta)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(t)

        # 发布 odom 消息
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # 速度（使用真实 dt 计算）
        if dt > 0:
            odom.twist.twist.linear.x = d_center / dt
            odom.twist.twist.angular.z = d_theta / dt
        else:
            odom.twist.twist.linear.x = 0.0
            odom.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom)

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return [x, y, z, w]


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()