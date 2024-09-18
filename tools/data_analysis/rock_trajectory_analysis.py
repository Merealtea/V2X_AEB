from copy import deepcopy
import rosbag  
import rospy
from sensor_msgs.msg import Image
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import NavSatFix, Imu
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

length = 0.3
width = 0.3
preson_bounding_box = np.array([[length, width/2],
                            [length/2, -width/2],
                            [-length/2, -width/2],
                            [-length/2, width/2],
                            [length/2, width/2],])
    
hycan_bboxes = []
rock_bboxes = []
detection_boxes = []

hycan_image_process_time = []
hycan_comm_time = []

rock_image_process_time = []
rock_comm_time = []

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

idx = 0
hycan_init_yaw = None

def hycan_callback(msg):
    global hycan_bboxes, detection_boxes, hycan_image_process_time, hycan_comm_time, hycan_init_yaw
    hycan_x, hycan_y = msg.localization.utm_x, msg.localization.utm_y
    hycan_yaw = msg.localization.heading
    hycan_init_yaw = hycan_yaw

    box = np.dot(bounding_box, np.array([[np.cos(hycan_yaw), -np.sin(hycan_yaw)],
                            [np.sin(hycan_yaw), np.cos(hycan_yaw)]]).T)
    box += np.array([hycan_x, hycan_y]).reshape(1, 2)  
    hycan_bboxes.append(box)

    person_bbox = np.dot(preson_bounding_box, np.array([[np.cos(hycan_yaw), -np.sin(hycan_yaw)],
                            [np.sin(hycan_yaw), np.cos(hycan_yaw)]]).T)
    person_bbox += np.array([hycan_x - 1, hycan_y + 2]).reshape(1, 2)
    detection_boxes.append(person_bbox)

    hycan_image_process_time.append(msg.sender.stamp.to_sec() - msg.image_stamp.to_sec())
    hycan_comm_time.append(msg.reciever.stamp.to_sec() - msg.sender.stamp.to_sec())

def rock_callback(msg):
    global rock_bboxes , idx, rock_image_process_time, rock_comm_time
    if idx > 3000 and idx < 3710:
        rock_x, rock_y = msg.localization.utm_x, msg.localization.utm_y
        rock_yaw = msg.localization.heading - np.pi / 2 + np.pi / 32

        box = np.dot(bounding_box, np.array([[np.cos(rock_yaw), -np.sin(rock_yaw)],
                                [np.sin(rock_yaw), np.cos(rock_yaw)]]).T)
        box += np.array([rock_x, rock_y]).reshape(1, 2)  
        rock_bboxes.append(box)
        rock_comm_time.append(msg.reciever.stamp.to_sec() - msg.sender.stamp.to_sec())
    rock_image_process_time.append(msg.sender.stamp.to_sec() - msg.image_stamp.to_sec())
    
    idx += 1


if __name__ == "__main__":
    rospy.init_node('hycan_bag')
    hycan_bag_path = "/mnt/pool1/V2X_AEB/data/2024-09-10_10-21-21.bag"
    
    # 打开两个rosbag文件
    bag = rosbag.Bag(hycan_bag_path, 'r')

    hycan_topic = '/hycan/detection_results'
    rock_topic = '/rock/detection_results'


    # 读取bag文件并触发callbacks
    for topic, msg, t in bag.read_messages(topics=[hycan_topic, rock_topic]):
            if topic == hycan_topic:
                hycan_callback(msg)
            elif topic == rock_topic:
                rock_callback(msg)

    bag.close()

    fig, ax = plt.subplots()
    ax.set_aspect('equal', adjustable='box')

    for box in hycan_bboxes:
        # 绘制车辆包围盒
        ax.plot(box[:, 0], box[:, 1], 'b')

    for box in rock_bboxes:
        # 绘制车辆包围盒
        ax.plot(box[:, 0], box[:, 1], 'r')
    
    for box in detection_boxes:
        # 绘制车辆包围盒
        ax.plot(box[:, 0], box[:, 1], 'g')

    # 将x轴与y轴翻转
    ax.invert_yaxis()
    
    plt.savefig('bounding_box.png', dpi = 600)

    print("hycan image process time mean is {}".format(np.mean(hycan_image_process_time)))
    print("hycan comm time mean is {}".format(np.mean(hycan_comm_time)))

    print("rock image process time mean is {}".format(np.mean(rock_image_process_time)))
    print("rock comm time mean is {}".format(np.mean(rock_comm_time)))