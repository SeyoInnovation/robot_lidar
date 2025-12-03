# 时间戳
std_msgs/Header header

# 左右轮速度 (m/s)
float32 left_velocity
float32 right_velocity

# 左右轮累计距离 (m)  —— 可用于更精确的增量式里程计
float32 left_distance
float32 right_distance

# 陀螺仪角速度 (°/s)
float32 gyro_x
float32 gyro_y
float32 gyro_z