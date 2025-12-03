#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ByteMultiArray
import serial
import time

class SerialReaderNode(Node):
    def __init__(self):
        super().__init__('serial_reader_node')

        # ==================== 参数 ====================
        self.declare_parameter('port', '/dev/ttyUSB0')          # 串口设备
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('topic_name', '/serial/data')     # 发布话题名
        self.declare_parameter('publish_string', True)          # True=String, False=ByteMultiArray

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        timeout = self.get_parameter('timeout').get_parameter_value().double_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        publish_string = self.get_parameter('publish_string').get_parameter_value().bool_value

        # ==================== 串口初始化 ====================
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            self.get_logger().info(f"串口 {port} 已打开，波特率 {baudrate}")
        except serial.SerialException as e:
            self.get_logger().error(f"无法打开串口: {e}")
            raise e

        # ==================== 发布者 ====================
        if publish_string:
            self.publisher = self.create_publisher(String, topic_name, 10)
            self.get_logger().info(f"将以 String 类型发布到 {topic_name}")
        else:
            self.publisher = self.create_publisher(ByteMultiArray, topic_name, 10)
            self.get_logger().info(f"将以 ByteMultiArray 类型发布到 {topic_name}")

        # 100Hz 轮询（可根据实际需求调高调低）
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    ### 这个地方需要调整读取格式
    def timer_callback(self):
        if self.ser.in_waiting > 0:
            # 方法1：按行读取（推荐，大多数设备以\n结尾）
            try:
                line = self.ser.readline()          # 包含 \n
                # line = self.ser.read(self.ser.in_waiting)  # 如果你想一次性读所有字节

                if len(line) > 0:
                    if isinstance(self.publisher.msg_type(), String):
                        msg = String()
                        # 去掉换行符并解码，或者直接发布原始字节转str
                        try:
                            msg.data = line.decode('utf-8', errors='ignore').strip()
                        except:
                            msg.data = str(line)
                        self.publisher.publish(msg)
                        self.get_logger().info(f"Published String: {msg.data}")

                    else:  # ByteMultiArray
                        msg = ByteMultiArray()
                        msg.data = list(line)  # bytes → list[int]
                        self.publisher.publish(msg)
                        self.get_logger().debug(f"Published {len(msg.data)} bytes")

            except Exception as e:
                self.get_logger().error(f"读取串口出错: {e}")

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("串口已关闭")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialReaderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()