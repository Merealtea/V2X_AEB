from copy import deepcopy
import rosbag  
import rospy
from sensor_msgs.msg import Image
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import NavSatFix, Imu
from message_filters import ApproximateTimeSynchronizer, Subscriber
from pyproj import Transformer
import numpy as np
import math

# From wgs84 to utm
transformer = Transformer.from_crs("epsg:4326", "epsg:32632")
length = 4.6
width = 1.9
bounding_box = np.array([[length/2, width/2],
                            [length/2, -width/2],
                            [-length/2, -width/2],
                            [-length/2, width/2],
                            [length/2, width/2],])
    
bounding_boxes = []
yaws = []
points = []
i = 0
max_frame =400
min_x , min_y = 1e9, 1e9
max_x, max_y = -1e9, -1e9

RADIANS_PER_DEGREE = np.pi / 180
DEGREES_PER_RADIAN = 180 / np.pi
central_meridian = 120

last_gps = None

def compute_convergence_angle(lon, lat, central_meridian):
    # 计算经度偏差
    delta_lon = math.radians(lon - central_meridian)
    # 计算收敛角
    convergence_angle = delta_lon * math.sin(math.radians(lat))
    return math.degrees(convergence_angle)

class KalmanFilter:
    def __init__(self):
        self.x = np.array([[0.0]])  # 初始状态
        self.P = np.array([[100.0]])  # 初始状态协方差
        self.F = np.array([[1.0]])  # 状态转移矩阵
        self.H = np.array([[1.0]])  # 观测矩阵
        self.R = np.array([[0.1]])  # 观测噪声协方差 (根据传感器准确度调整)
        self.Q = np.array([[0.01]]) # 过程噪声协方差

    def predict(self):
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(self.F, np.dot(self.P, self.F.T)) + self.Q

    def update(self, z):
        y = z - np.dot(self.H, self.x)
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(self.P, np.dot(self.H.T, np.linalg.inv(S)))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.H.shape[1])
        self.P = (I - np.dot(K, self.H)) * self.P

kf = KalmanFilter()

def callback(gps, imu):
    global i, min_x, min_y, max_x, max_y, last_gps, kf, central_meridian
    position = transformer.transform(gps.latitude, gps.longitude)

    if last_gps is None:
        last_gps = deepcopy(gps)
        return
    lon1 = last_gps.longitude * RADIANS_PER_DEGREE
    lat1 = last_gps.latitude * RADIANS_PER_DEGREE
    lon2 = gps.longitude * RADIANS_PER_DEGREE
    lat2 = gps.latitude * RADIANS_PER_DEGREE

    ca = compute_convergence_angle(gps.longitude, gps.latitude, central_meridian)
    ca = - np.pi * 50/ 64

    dLon = lon2 - lon1
    y = np.sin(dLon) * np.cos(lat2)
    x = np.cos(lat1) * np.sin(lat2) - np.sin(lat1) * np.cos(lat2) * np.cos(dLon)
    gps_heading = np.arctan2(y, x)

    q0 = imu.orientation.w
    q1 = imu.orientation.x
    q2 = imu.orientation.y
    q3 = imu.orientation.z

    imu_yaw = np.arctan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3))

    
    corrected_heading =  imu_yaw + ca #+ gps_heading + ca - 

    # imu_yaw = kf.x[0, 0]

    yaw = corrected_heading 
    
    last_gps = deepcopy(gps)
    i += 1

    if i < 8500:
        if min_x > position[0]:
            min_x = position[0]
        if min_y > position[1]:
            min_y = position[1]
        if max_x < position[0]:
            max_x = position[0]
        if max_y < position[1]:
            max_y = position[1]
        yaws.append(yaw)
        # yaws[-1] = np.mean(yaws[-50:])
        points.append(position)

        rotation_matrix = np.array([[np.cos(yaw), -np.sin(yaw)],
                            [np.sin(yaw), np.cos(yaw)]])
    
        rotated_box = np.dot(bounding_box, rotation_matrix.T)
        rotated_box += np.array(position).reshape(1, 2)
        bounding_boxes.append(rotated_box)
            # import pdb pdb.set_trace()
    


if __name__ == "__main__":
    rospy.init_node('hycan_bag')
    hycan_bag_path = "/mnt/pool1/2024-07-23-11-11-52.bag"
    
    # 打开两个rosbag文件
    bag = rosbag.Bag(hycan_bag_path, 'r')

    gps_topic = '/Inertial/gps/fix'
    heading_topic = '/Inertial/imu/data'
    vel_topic = '/Inertial/gps/vel'

     # 创建message_filters的Subscriber对象
    subscribers = [Subscriber(gps_topic, NavSatFix),
                   Subscriber(heading_topic, Imu)]
    
    # 创建ApproximateTimeSynchronizer对象
    ts = ApproximateTimeSynchronizer(subscribers, 10, 0.1)
    ts.registerCallback(callback)

    # 读取bag文件并触发callbacks
    for topic, msg, t in bag.read_messages(topics=[gps_topic, heading_topic]):
        for sub in subscribers:
            if topic == sub.name:
                sub.signalMessage(msg)

    bag.close()

    fig, ax = plt.subplots()
    ax.set_aspect('equal', adjustable='box')

    ax.set_ylim(min_y-10, max_y+10)        
    ax.set_xticks(np.arange(min_x-10, max_x+10, 1))
    ax.set_yticks(np.arange(min_y-10, max_y+10, 1))
    
    points= np.array(points)
    # 绘制轨迹
    plt.plot(points[:, 0], points[:, 1], label="Trajectory")

    # # 在每个点上绘制表示朝向的箭头
    # for i in range(len(points)):
    #     x, y = points[i]
    #     angle = yaws[i]
    #     # 使用箭头来表示朝向
    #     dx = np.cos(angle) * 0.5  # 箭头的x分量
    #     dy = np.sin(angle) * 0.5  # 箭头的y分量
    #     plt.arrow(x, y, dx, dy, head_width=0.2, head_length=0.2, fc='r', ec='r')

    for box in bounding_boxes:
        ax.plot(box[:, 0], box[:, 1], 'r')
    plt.savefig('bounding_box.png', dpi = 600)