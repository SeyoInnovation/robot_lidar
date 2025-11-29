Ubuntu版本：24.04 ROS2 版本：jazzy

①编译方法：使用catkin编译，执行如下操作
将功能包解压后复制到工作空间的路径下，然后执行指令“colcon build ”进行编译。

②设备别名：端口设备重命名
launch中启动的LD14雷达默认设备名为：/dev/LD14P，别名文件是“ldlidar14_udev.sh”，
如果您使用的ttl电平转换芯片为CP2102，设备号需要改为0001；
如果您使用的ttl电平转换芯片为CH9102设备号则需要改为0003。
具体修改方法请分别查看对应的驱动资料。

③运行方法
source install/setup.bash
ros2 launch ldlidar ld14p.launch.py

④rviz可视化查看点云：
ros2 run rviz2 rviz2
然后选择rivz配置文件即可
rviz的配置在功能包路径下的rviz文件中。
出现版本问题jazzy不支持gazebo
安装cartographer：sudo apt install ros-jazzy-cartographer
sudo apt install ros-jazzy-cartographer-ros
检测安装成功：ros2 pkg list | grep cartographer

启动Gazebo：ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py use_sim_time:=True
安装ros包：sudo apt install ros-jazzy-joint-state-publisher-gui

测试slam_toolbox是否有用
需要/scan和/odom这两个数据 ros2 topic list

里程计的数据从仿真来？

teleop_twist_keyboard可以使用这个来控制机器人

ros2 run tf2_tools view_frames
用来查看tf树
## 测试：代码在ubuntun20.04foxy版本下测试，使用rviz可视化。
