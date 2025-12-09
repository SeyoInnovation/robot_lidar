#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import tkinter as tk
from tkinter import messagebox
import threading


class PubNode(Node):
    def __init__(self):
        super().__init__('pub_node')
        self.pub = self.create_publisher(Int32MultiArray, '/pub_data', 10)
        self.get_logger().info("发布节点已启动，话题: /pub_data (Int32MultiArray)")

    def publish_data(self, int1: int, int2: int):
        msg = Int32MultiArray()
        msg.data = [int1, int2]
        self.pub.publish(msg)
        self.get_logger().info(f"已发布: {msg.data}")


def ros_spin(node):
    """ROS spin 线程"""
    rclpy.spin(node)


def main():
    rclpy.init()

    node = PubNode()

    # 启动 ROS spin 线程
    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    # ==================== GUI 界面 ====================
    root = tk.Tk()
    root.title("Pub Data 发送器")
    root.geometry("300x200")

    # Int1 输入
    tk.Label(root, text="Int1:").pack(pady=5)
    entry1 = tk.Entry(root)
    entry1.pack()

    # Int2 输入
    tk.Label(root, text="Int2:").pack(pady=5)
    entry2 = tk.Entry(root)
    entry2.pack()

    def send_button_clicked():
        try:
            int1 = int(entry1.get() or 0)
            int2 = int(entry2.get() or 0)
            node.publish_data(int1, int2)
            messagebox.showinfo("成功", f"已发送: [{int1}, {int2}]")
        except ValueError:
            messagebox.showerror("错误", "请输入有效整数！")

    tk.Button(root, text="发布到 /pub_data", command=send_button_clicked).pack(pady=20)

    # 运行 GUI 主循环
    root.mainloop()

    # 清理
    rclpy.shutdown()
    ros_thread.join()


if __name__ == '__main__':
    main()