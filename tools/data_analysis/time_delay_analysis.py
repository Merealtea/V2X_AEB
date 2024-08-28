from hycan_msgs.msg import FourImages
from sensor_msgs.msg import  NavSatFix
import rospy
import cv2
import numpy as np
import pickle
import sys
sys.path.append("/mnt/pool1/TigorData/")
import rosbag  
import rospy
from copy import deepcopy
from message_filters import ApproximateTimeSynchronizer, Subscriber
import argparse
from pyproj import Transformer
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(description='arg parser')
parser.add_argument('--date_time', type=str, help='the path to the ros bag file')
parser.add_argument('--with_ground_truth', type=bool, help = 'whether to use ground truth')
parser.add_argument('--vehicle', type=str, help = 'the vehicle where the camera belongs to')
parser.add_argument('--time_diff', type=float, default=0, help = 'the time difference between the lidar and camera, t_lidar = t_cam + time_diff')

distance = []
time_delay = []
x, y = [], []

# From wgs84 to utm
transformer = Transformer.from_crs("epsg:4326", "epsg:32632")

def merge_callback(hycan_msg : FourImages, 
                   localisation_msg : NavSatFix):
    # hycan_msg : the timestamp of the hycan sending msg is in header
    # localisation_msg : the timestamp of the rock sending msg is in header
    global distance, time_delay
    hycan_gps = hycan_msg.gps
    rock_gps = localisation_msg

    hycan_utm = transformer.transform(hycan_gps.latitude, hycan_gps.longitude)
    rock_utm = transformer.transform(rock_gps.latitude, rock_gps.longitude)
    print("rock timestamp is ", rock_gps.header.stamp)
    print("hycan timestamp is ", hycan_msg.header.stamp)
    print("timediff is ", (rock_gps.header.stamp - hycan_msg.header.stamp).to_sec())

    delay = (hycan_msg.image_front.header.stamp - hycan_msg.header.stamp ).to_sec() * 1000
    if delay > 2000:
        return
    time_delay.append(delay)
    distance.append(np.sqrt((hycan_utm[0] - rock_utm[0])**2 \
                            + (hycan_utm[1] - rock_utm[1])**2))
    x.append(hycan_utm[0]-rock_utm[0])
    y.append(hycan_utm[1]-rock_utm[1])

if __name__ == "__main__":
    rospy.init_node('merge_bag')
    rock_bag_path = "/mnt/pool1/outside_parking/changan/2024-08-01-09-36-09.bag"
    hycan_bag_path = "/mnt/pool1/outside_parking/hycan/2024-08-01_09-34-15_send.bag"
    # 打开两个rosbag文件
    bag1 = rosbag.Bag(rock_bag_path, 'r')
    bag2 = rosbag.Bag(hycan_bag_path, 'r')

    # 创建两个Subscriber对象
    image_sub = Subscriber('/Hycan/processed_images', FourImages)
    gps_sub = Subscriber('/Inertial/gps/fix', NavSatFix)

    # 创建ApproximateTimeSynchronizer
    sync = ApproximateTimeSynchronizer([image_sub, gps_sub], queue_size=10, slop=0.1)
    sync.registerCallback(merge_callback)

    # 合并两个bag的消息迭代器
    bag1_msgs = bag1.read_messages(topics=['/Inertial/gps/fix'])
    bag2_msgs = bag2.read_messages(topics=['/Hycan/processed_images'])
    merged_msgs = sorted(list(bag1_msgs) + list(bag2_msgs), key=lambda x: x[2])  # 按时间戳排序

    for topic, msg, t in merged_msgs:
        if topic == '/Hycan/processed_images':
            image_sub.signalMessage(msg)
        elif topic == '/Inertial/gps/fix':
            gps_sub.signalMessage(msg)

    bag1.close()
    bag2.close()

    plt.scatter(distance, time_delay)
    plt.xlabel("distance/m")
    plt.ylabel("time delay/ms")
    plt.savefig("distance_time_delay.png")

    # 创建绘图
    plt.figure(figsize=(8, 6))

    # 使用散点图表示轨迹，并根据时延值改变颜色
    time_delay = np.array(time_delay) / 1000
    scatter = plt.scatter(x, y, c=time_delay, cmap='viridis', s=100, edgecolors='black')

    # 添加颜色条
    plt.colorbar(scatter, label= 'time delay (s)')

    # 设置标题和轴标签
    plt.title('delay heatmap')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.savefig("trajdelay.png")