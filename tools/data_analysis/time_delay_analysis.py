from hycan_msgs.msg import DetectionResults
import rospy
import cv2
import numpy as np
import pickle
import sys
sys.path.append("/mnt/pool1/TigorData/")
import rosbag  
import rospy
import argparse
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(description='arg parser')
parser.add_argument('--date_time', type=str, help='the path to the ros bag file')
parser.add_argument('--with_ground_truth', type=bool, help = 'whether to use ground truth')
parser.add_argument('--vehicle', type=str, help = 'the vehicle where the camera belongs to')
parser.add_argument('--time_diff', type=float, default=0, help = 'the time difference between the lidar and camera, t_lidar = t_cam + time_diff')

distance = []
time_delay = []
x, y = [], []



def callback(hycan_msg : DetectionResults):
    # hycan_msg : the timestamp of the hycan sending msg is in header
    # localisation_msg : the timestamp of the rock sending msg is in header
    global x,y, time_delay
    
    
    send_time = hycan_msg.sender.stamp.to_sec()
    recv_time = hycan_msg.reciever.stamp.to_sec()
    if recv_time - send_time > 1:
        return
    time_delay.append((recv_time - send_time) * 1000)
    x.append(hycan_msg.localization.utm_x)
    y.append(hycan_msg.localization.utm_y)

if __name__ == "__main__":
    rospy.init_node('merge_bag')
    communication_topic = "/hycan/detection_results"
    communication_bag = "/mnt/pool1/V2X_AEB/data/2024-09-09_11-16-03.bag"
    
    # 打开两个rosbag文件
    communication_bag = rosbag.Bag(communication_bag, 'r')

    # 合并两个bag的消息迭代器
    communication_msgs = communication_bag.read_messages(topics=[communication_topic])

    for topic, msg, t in communication_msgs:
        if topic == communication_topic:
            callback(msg)

    communication_bag.close()

    distance = np.sqrt(np.array(x)**2 + np.array(y)**2)

    plt.scatter(distance, time_delay)
    plt.xlabel("distance/m")
    plt.ylabel("time delay/ms")
    plt.savefig("distance_time_delay.png")

    # 创建绘图
    plt.figure(figsize=(8, 6))

    # 使用散点图表示轨迹，并根据时延值改变颜色
    time_delay = np.array(time_delay) / 1000
    scatter = plt.scatter(x, y, c=time_delay, cmap='viridis', s=5)

    # 添加颜色条
    plt.colorbar(scatter, label= 'time delay (s)')

    # 设置标题和轴标签
    plt.title('delay heatmap')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.savefig("trajdelay.png")

    print("mean delay is {}".format(np.mean(time_delay)))