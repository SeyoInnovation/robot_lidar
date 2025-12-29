# 扫地机器人上位机代码
### 硬件信息
底盘：海尔扫地机器人（白色款）
上位机：树莓派5
Ubuntu版本：24.04 ROS2 版本：jazzy

### 安装相关包
slam-toolbox
rosbridge_server
### 启动
colcon build
source install/setup.bash
ros2 launch ldlidar serial.launch.py
### 其他指令
- 雷达启动
~/robot_lidar$ ros2 launch ldlidar ld14p.launch.py port_name:=/dev/ttyACM0
- base_footprint到base_link的tf变换
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id base_link --child-frame-id base_footprint
- web-socket服务开启
ros2 launch rosbridge_server rosbridge_websocket_launch.xml delay_between_messages:=0.0
- 两轮机器人控制移动
ros2 run teleop_twist_keyboard teleop_twist_keyboard
- 查看建图结果
eog /home/xiluo/robot_lidar/maps/my_room.pgm
- 查看TF树
ros2 run tf2_tools view_frames
### 注意
- 出现版本问题jazzy不支持gazebo，所以做仿真只能用gz-sim
- 测试slam_toolbox需要有/scan和/odom这两个数据 
- 无网页端可以打开teleop_twist_keyboard来控制机器人

### 相关话题
数据接收的话题：wheel_raw_data
里程计话题：odom
接收发送的话题：pub_data

### 相关文件
实时建图launch:slam.launch.py
串口通讯文件：serial.launch.py
导航文件：nav.launch.py