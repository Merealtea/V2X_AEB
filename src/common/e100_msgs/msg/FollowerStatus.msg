Header header
int8 sensor_status # 传感器状态，0为正常，1为没接收到数据，2为异常
int8 tracking_status # 跟车状态，0为正常，1为没接收到数据，2为存在风险，3为丢失前车
int8 relative_pose_status # 相对检测状态，0为正常，1为没接收到数据
geometry_msgs/Pose2D relative_pose # 前车相对位姿
int8 speed_status # 速度状态，0为正常，1为没接收到数据
float64 speed # 速度，单位m/s