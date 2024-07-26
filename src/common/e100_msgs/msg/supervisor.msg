Header header

float64 ego_speed                # 自车的速度，单位 m/s
float64 front_speed              # 前车的速度，单位 m/s 
bool    front_lost               # 是否跟丢前车, 1为跟丢
geometry_msgs/Pose2D front_pose  # 前车相对位姿

int32   brake_cmd                # 刹车控制量[0,30]
int32   speed_cmd                # 速度控制量[0,1000],单位：cm/s
int32   steer_cmd                # 转向控制量[-9172,9172]

