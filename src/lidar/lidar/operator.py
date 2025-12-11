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

    def publish_data(self, int1: int):
        msg = Int32MultiArray()
        msg.data = [int1]
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
    root.title("Pub Data 持续发送器")
    root.geometry("350x250")

    # 发送状态控制变量
    sending = False
    timer_id = None
    SEND_INTERVAL_MS = 100  # 发送间隔（毫秒），可修改为 10（100Hz）、50（20Hz） 等

    # Int1 输入
    tk.Label(root, text="Int1 值（持续发送的内容）:").pack(pady=10)
    entry1 = tk.Entry(root, width=15, font=("Arial", 12), justify='center')
    entry1.pack()
    entry1.insert(0, "0")  # 默认值

    def start_sending():
        nonlocal sending, timer_id
        if sending:
            return

        try:
            int1 = int(entry1.get())
        except ValueError:
            messagebox.showerror("错误", "请输入有效的整数！")
            return

        sending = True
        button_start.config(state="disabled")
        button_stop.config(state="normal")
        entry1.config(state="disabled")

        messagebox.showinfo("开始发送", f"开始持续发送 [{int1}]\n频率约 {1000 // SEND_INTERVAL_MS} Hz")

        def repeat_send():
            nonlocal timer_id
            if sending:
                node.publish_data(int1)
                timer_id = root.after(SEND_INTERVAL_MS, repeat_send)

        repeat_send()

    def stop_sending():
        nonlocal sending, timer_id
        if not sending:
            return

        sending = False
        if timer_id:
            root.after_cancel(timer_id)
            timer_id = None

        button_start.config(state="normal")
        button_stop.config(state="disabled")
        entry1.config(state="normal")

        messagebox.showinfo("停止发送", "已停止持续发送")

    # 按钮区域
    button_frame = tk.Frame(root)
    button_frame.pack(pady=20)

    button_start = tk.Button(button_frame, text="开始持续发送", width=15, height=2,
                             bg="green", fg="white", command=start_sending)
    button_start.pack(side=tk.LEFT, padx=10)

    button_stop = tk.Button(button_frame, text="停止发送", width=15, height=2,
                            bg="red", fg="white", command=stop_sending, state="disabled")
    button_stop.pack(side=tk.LEFT, padx=10)

    # 显示当前发送频率（可选）
    tk.Label(root, text=f"当前发送间隔: {SEND_INTERVAL_MS} ms（约 {1000 // SEND_INTERVAL_MS} Hz）",
             fg="gray").pack(pady=5)

    # 运行 GUI 主循环
    root.mainloop()

    # 清理
    rclpy.shutdown()
    ros_thread.join()

if __name__ == '__main__':
    main()