#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from uart_msg.msg import WheelData
import serial
import struct

class RawSerialNode(Node):
    def __init__(self):
        super().__init__('raw_serial_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('motor1_is_left', True)   # 改这里决定谁是左轮

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        self.motor1_is_left = self.get_parameter('motor1_is_left').value

        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.get_logger().info(f"串口 {port} 已打开 @ {baud}")

        self.pub = self.create_publisher(WheelData, 'wheel_raw_data', 10)

        self.create_timer(0.002, self.read_loop)  # 500Hz 轮询

    def read_loop(self):
        # 找帧头
        if self.ser.read(2) != b'\xAA\x55':
            return

        data = self.ser.read(31)
        if len(data) < 31 or data[-2:] != b'\x55\xAA':
            return

        payload = data[:-3]
        recv_chksum = data[-3]

        calc_chksum = 0
        for b in payload[:28]:
            calc_chksum ^= b
        if calc_chksum != recv_chksum:
            return

        # 解包 7 个 float（小端）
        m1_v, m1_s, m2_v, m2_s, gx, gy, gz = struct.unpack('<7f', payload[:28])

        msg = WheelData()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        # 根据参数决定左右轮映射
        if self.motor1_is_left:
            msg.left_velocity  = m1_v
            msg.right_velocity = m2_v
            msg.left_distance  = m1_s
            msg.right_distance = m2_s
        else:
            msg.left_velocity  = m2_v
            msg.right_velocity = m1_v
            msg.left_distance  = m2_s
            msg.right_distance = m1_s

        msg.gyro_x = gx
        msg.gyro_y = gy
        msg.gyro_z = gz

        self.pub.publish(msg)
        self.get_logger().info(f"Published raw data: vL={msg.left_velocity:.3f}, vR={msg.right_velocity:.3f}")

def main():
    rclpy.init()
    node = RawSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()