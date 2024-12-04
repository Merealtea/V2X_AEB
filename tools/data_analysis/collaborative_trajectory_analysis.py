from copy import deepcopy
import rosbag  
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cyber_msgs.msg import Heading
from hycan_msgs.msg import Box3D
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import NavSatFix, Imu
from message_filters import ApproximateTimeSynchronizer, Subscriber
import tf.transformations
from utils import hycan_localization, rock_localization, hycan_bounding_box, rock_bounding_box, hycan_gps_topic, \
                    hycan_heading_topic, rock_gps_topic, rock_imu_topic, merge_msgs, transform_bbox, \
                    rock_lidar_topic, create_marker, MarkerArray, hycan_length, hycan_width, rock_length, rock_width,\
                    rock_x_offset, rock_y_offset, hycan_x_offset, hycan_y_offset, hycan_lidar_topic
import numpy as np
import scipy
import sys
import tf
sys.path.append("/mnt/pool1/V2X_AEB/src/common")
from Mono3d.configs.FisheyeParam.lidar_model import Lidar_transformation

hycan_bounding_boxes = []
hycan_yaws = []
hycan_points = []

rock_bounding_boxes = []
rock_yaws = []
rock_points = []

i = 0
max_frame = 400
min_x , min_y = 1e9, 1e9
max_x, max_y = -1e9, -1e9


def callback(hycan_gps, hycan_heading, rock_gps, rock_imu, rock_lidar, hycan_lidar):
    global i, min_x, min_y, max_x, max_y, hycan_bounding_boxes, hycan_yaws, \
        hycan_points, rock_bounding_boxes, rock_yaws, rock_points, pc_pub, marker_pub
    print(hycan_lidar.header.stamp.to_sec())
    hycan_position, hycan_yaw = hycan_localization(hycan_gps, hycan_heading)
    hycan_yaw += np.pi + np.pi * 3 / 128

    rock_position, rock_yaw = rock_localization(rock_gps, rock_imu)

    # rock_yaw += np.pi / 128
    
    rock_tf_mat =np.array([[1, 0, rock_position[0]],
                            [0, 1, rock_position[1]],
                            [0, 0, 1]]) @ np.array([[np.cos(rock_yaw), -np.sin(rock_yaw), 0],
                            [np.sin(rock_yaw), np.cos(rock_yaw), 0],
                            [0, 0, 1]]) 
    
    # import pdb; pdb.set_trace() 

    hycan_position[0] = hycan_position[0]  - hycan_y_offset * np.sin(hycan_yaw) + 0.12 * np.cos(hycan_yaw) #
    hycan_position[1] = hycan_position[1]  + hycan_y_offset * np.cos(hycan_yaw) + 0.12 * np.sin(hycan_yaw) #
    hycan_to_rock_position = np.dot(np.linalg.inv(rock_tf_mat), np.array([hycan_position[0], hycan_position[1], 1]))[:2]
    hycan_to_rock_yaw = hycan_yaw - rock_yaw
    rock_position = np.dot(np.linalg.inv(rock_tf_mat), np.array([rock_position[0], rock_position[1], 1]))[:2]
    rock_yaw = 0
    # import pdb; pdb.set_trace()
    # rock_box = transform_bbox(rock_bounding_box, rock_position, rock_yaw)


    marker_arrray = MarkerArray()
    rock_box = Box3D()
    rock_box.length = rock_length
    rock_box.width = rock_width
    rock_box.height = 1.6

    rock_box.heading = rock_yaw
    rock_box.center_x = rock_position[0] + rock_x_offset * np.cos(rock_yaw) - rock_y_offset * np.sin(rock_yaw) #
    rock_box.center_y = rock_position[1] + rock_x_offset * np.sin(rock_yaw) + rock_y_offset * np.cos(rock_yaw) #
    rock_box.center_z = 0.8

    hycan_box = Box3D()
    hycan_box.length = hycan_length
    hycan_box.width = hycan_width
    hycan_box.height = 1.6

    hycan_box.heading = hycan_to_rock_yaw
    hycan_box.center_x = hycan_to_rock_position[0] + hycan_x_offset * np.cos(hycan_to_rock_yaw) #
    hycan_box.center_y = hycan_to_rock_position[1] + hycan_x_offset * np.sin(hycan_to_rock_yaw) #
    hycan_box.center_z = 0.8

    marker_arrray.markers.append(create_marker(0, [1, 0, 0], rock_box))
    marker_arrray.markers.append(create_marker(1, [0, 1, 0], hycan_box))

    rock_lidar.header.frame_id = "map"
    lidar_points = pc2.read_points(rock_lidar, field_names = ("x", "y", "z"), skip_nans=True)
    lidar_points = list(lidar_points)

    hycan_lidar.header.frame_id = "map"
    hycan_lidar_points = pc2.read_points(hycan_lidar, field_names = ("x", "y", "z"), skip_nans=True)
    hycan_lidar_points = np.array(list(hycan_lidar_points)).T
    new_hycan_points = hycan_lidar_model.lidar_to_rear(hycan_lidar_points)
    hycan_to_rock_tf = np.array([[np.cos(hycan_to_rock_yaw), -np.sin(hycan_to_rock_yaw), hycan_to_rock_position[0]],
                                [np.sin(hycan_to_rock_yaw), np.cos(hycan_to_rock_yaw), hycan_to_rock_position[1]],
                                [0, 0, 1]])
    new_hycan_points[:2] = (hycan_to_rock_tf @ np.concatenate([new_hycan_points[:2], np.ones((1, len(new_hycan_points[0])))]))[:2]
    new_hycan_points = new_hycan_points.T
    new_lidar_points =  rock_lidar_model.lidar_to_rear(np.array(lidar_points).T).T
    rock_lidar = pc2.create_cloud_xyz32(rock_lidar.header, new_lidar_points)
    hycan_lidar = pc2.create_cloud_xyz32(hycan_lidar.header, new_hycan_points) 

    pc_pub.publish(rock_lidar)
    marker_pub.publish(marker_arrray)
    hycan_pc_pub.publish(hycan_lidar)
    
    # time.sleep(0.02)


if __name__ == "__main__":
    rospy.init_node('hycan_bag')
    hycan_bag_path = "/mnt/pool1/ground_truth_generator/data/camera_data/2024-12-02-20-47-42_Hycan.bag"
    rock_bag_path = "/mnt/pool1/ground_truth_generator/data/camera_data/2024-12-02-20-47-42_Rock.bag"

    print("Start to read bag files")

    pc_pub = rospy.Publisher('/hycan/rock/pc', PointCloud2, queue_size=10)
    marker_pub = rospy.Publisher('/hycan/rock/marker', MarkerArray, queue_size=10)
    hycan_pc_pub = rospy.Publisher('/hycan/hycan/pc', PointCloud2, queue_size=10)
    
    # 打开两个rosbag文件
    hycan_bag = rosbag.Bag(hycan_bag_path, 'r')
    rock_bag = rosbag.Bag(rock_bag_path, 'r')


    rock_lidar_model = Lidar_transformation("Rock")
    hycan_lidar_model = Lidar_transformation("Hycan")
     # 创建message_filters的Subscriber对象
    hycan_gps_sub = Subscriber(hycan_gps_topic, NavSatFix)
    hycan_heading_sub = Subscriber(hycan_heading_topic, Heading)
    rock_gps_sub = Subscriber(rock_gps_topic, NavSatFix)
    rock_imu_sub = Subscriber(rock_imu_topic, Imu)
    rock_lidar_sub = Subscriber(rock_lidar_topic, PointCloud2)
    hycan_lidar_sub = Subscriber(hycan_lidar_topic, PointCloud2)
    
    # 创建ApproximateTimeSynchronizer对象
    ts = ApproximateTimeSynchronizer([hycan_gps_sub, hycan_heading_sub, rock_gps_sub, rock_imu_sub, rock_lidar_sub, hycan_lidar_sub], 10, 0.1)
    ts.registerCallback(callback)

    hycan_msgs = hycan_bag.read_messages(topics=[hycan_gps_topic, hycan_heading_topic, hycan_lidar_topic])
    rock_msgs = rock_bag.read_messages(topics=[rock_gps_topic, rock_imu_topic, rock_lidar_topic])
    merged_msgs = merge_msgs([hycan_msgs, rock_msgs])

    # 读取bag文件并触发callbacks 
    for topic, msg, t in merged_msgs:
        if topic == hycan_gps_topic:
            hycan_gps_sub.signalMessage(msg)
        elif topic == hycan_heading_topic:
            hycan_heading_sub.signalMessage(msg)
        elif topic == rock_gps_topic:
            rock_gps_sub.signalMessage(msg)
        elif topic == rock_imu_topic:
            rock_imu_sub.signalMessage(msg)
        elif topic == rock_lidar_topic:
            rock_lidar_sub.signalMessage(msg)
        elif topic == hycan_lidar_topic:
            hycan_lidar_sub.signalMessage(msg)

    hycan_bag.close()
    rock_bag.close()

    fig, ax = plt.subplots()
    ax.set_aspect('equal', adjustable='box')

    ax.set_ylim(min_y-10, max_y+10)        
    ax.set_xticks(np.arange(min_x-10, max_x+10, 1))
    ax.set_yticks(np.arange(min_y-10, max_y+10, 1))
    
    # points= np.array(points)
    # 绘制轨迹
    # plt.plot(points[:, 0], points[:, 1], label="Trajectory")

    # # 在每个点上绘制表示朝向的箭头
    # for i in range(len(points)):
    #     x, y = points[i]
    #     angle = yaws[i]
    #     # 使用箭头来表示朝向
    #     dx = np.cos(angle) * 0.5  # 箭头的x分量
    #     dy = np.sin(angle) * 0.5  # 箭头的y分量
    #     plt.arrow(x, y, dx, dy, head_width=0.2, head_length=0.2, fc='r', ec='r')

    # for box in hycan_bounding_boxes:
    #     ax.plot(box[:, 0], box[:, 1], 'r')

    # for box in rock_bounding_boxes:
    #     ax.plot(box[:, 0], box[:, 1], 'b')
    # plt.savefig('collaborative_bounding_box.png', dpi = 600)