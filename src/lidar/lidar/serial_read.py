#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from uart_msg.msg import WheelData       
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist, TransformStamped
import serial
import struct
from datetime import datetime

WHEELBASE = 0.177

class RawSerialNode(Node):
    def __init__(self):
        super().__init__('raw_serial_node')

        # 参数（可以通过 launch 文件或命令行覆盖）
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 38400)        
        self.declare_parameter('motor1_is_left', True)    

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        # 初始化变量
        self.vx = 0.0
        self.vth = 0.0
        self.mode1 = 0
        self.mode2 = 0
        self.water = 0
        self.fan = 0
        self.sweep = 0
        # 打开串口
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=None                     
        )

        self.get_logger().info(f"串口已打开 → {port} @ {baud} bps")

        # 发布者
        self.pub = self.create_publisher(WheelData, 'wheel_raw_data', 10)

        # 订阅待发送的数据
        self.sub_pubdata = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info("已订阅 /cmd_vel 话题（Twist），收到即下发")
        self.sub_pubdata = self.create_subscription(Int32MultiArray, '/mode_select', self.mode_vel_callback, 10)
        self.sub_pubdata = self.create_subscription(Int32MultiArray, '/function', self.function_vel_callback, 10)
        # 用定时器发送
        self.create_timer(0.02, self.hardware_sync_loop)
        # 缓冲区
        self.buffer = bytearray()
        self.create_timer(0.001, self.read_loop)

    
    # Ros2 -> Stm32    
    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vth = msg.angular.z
    def mode_vel_callback(self, msg):
        self.mode1 = msg.data[0]
        self.mode2 = msg.data[1]
        # self.get_logger().info(f"收到模式选择指令 → 模式1: {self.mode1}  模式2: {self.mode2}")
    def function_vel_callback(self, msg):
        self.water = msg.data[0]
        self.fan = msg.data[1]
        self.sweep = msg.data[2]
        self.get_logger().info(f"收到功能控制指令 → 水泵: {self.water}  风扇: {self.fan}  扫地机: {self.sweep}")

    def hardware_sync_loop(self):
        """核心循环：计算差速并将所有状态打包发送至 STM32"""
        v_l = self.vx - self.vth * (WHEELBASE / 2.0)
        v_r = self.vx + self.vth * (WHEELBASE / 2.0)
        
        # 物理补偿 (使用你最新的调试值)
        v_r = -v_r * 1
        v_l = -v_l * 1
        
        try:
            header = bytearray([0xAA, 0x55])
            # 打包数据
            payload = struct.pack('<ffiii', v_l, v_r, self.water, self.fan, self.sweep)
            
            check = 0
            for b in payload: check ^= b
            
            tail = bytearray([check, 0x55, 0xAA])
            self.get_logger().info(
                f"发送数据 → VL: {v_l:.3f} m/s  VR: {v_r:.3f} m/s  水泵: {self.water}  风扇: {self.fan}  扫地机: {self.sweep}"
            )
            self.ser.write(header + payload + tail)
        except Exception as e:
            self.get_logger().error(f"发送异常: {e}")

    # Stm32 -> Ros2
    def read_loop(self):
        # 把串口里所有数据一次性读进来
        if self.ser.in_waiting > 0:
            self.buffer.extend(self.ser.read(self.ser.in_waiting))

        while len(self.buffer) >= 33:
            # 1. 找帧头 AA55
            idx = self.buffer.find(b'\xAA\x55')
            if idx == -1:                        
                self.buffer.clear()
                return

            if idx > 0:                              
                self.buffer = self.buffer[idx:]
                if len(self.buffer) < 33:
                    return

            frame = self.buffer[:33]

            # 2. 检查帧尾 55AA
            if frame[-2:] != b'\x55\xAA':
                self.buffer.pop(0)                     
                continue

            # 3. 校验（异或 28 字节数据，索引 2~29）
            calc_sum = 0
            for b in frame[2:30]:
                calc_sum ^= b

            if calc_sum != frame[30]:
                self.get_logger().warn(
                    f"校验失败 → 收到 0x{frame[30]:02X}  计算 0x{calc_sum:02X}"
                )
                self.buffer.pop(0)
                continue
            self.buffer = self.buffer[33:]             
            # 4. 解析 7 个 float（小端）
            m1_v, m1_s, m2_v, m2_s, gx, gy, gz = struct.unpack('<7f', frame[2:30])

            # 5. 填充并发布 ROS2 消息
            msg = WheelData()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"

            msg.left_velocity   = m1_v
            msg.right_velocity  = m2_v # 右轮数据相反
            msg.left_distance   = -m1_s
            msg.right_distance  = -m2_s

            msg.gyro_x = gx
            msg.gyro_y = gy
            msg.gyro_z = gz

            self.pub.publish(msg)

            now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            self.get_logger().info(
                f"[{now}] "
                f"Lv:{msg.left_velocity:7.3f} m/s  "
                f"Rv:{msg.right_velocity:7.3f} m/s  |  "
                f"Ld:{msg.left_distance:8.3f} m  "
                f"Rd:{msg.right_distance:8.3f} m  |  "
                f"GX:{gx:7.2f}  GY:{gy:7.2f}  GZ:{gz:7.2f} °/s"
            )


def main(args=None):
    rclpy.init(args=args)
    node = RawSerialNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点被用户手动停止")
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()