import numpy as np
import sys
sys.path.append("/mnt/pool1/V2X_AEB/src/common")
from Mono3d.configs.FisheyeParam.cam_model import CamModel
import cv2
import rosbag
from sensor_msgs.msg import CompressedImage, PointCloud2
from sensor_msgs import point_cloud2 as pc2
from message_filters import ApproximateTimeSynchronizer, Subscriber
import cv_bridge
import os
import rospy    
from scipy.spatial.transform import Rotation as R

rospy.init_node('merge_bag')

# lidar bag path
lidar_bag_path = "/home/cxy/Desktop/lidar_calib/left_lidar.bag"
# camera_bag_path
camera_bag_path = "/home/cxy/Desktop/lidar_calib/left.bag"

front_camera_subsriber = Subscriber('/miivii_gmsl_ros/camera1/compressed', CompressedImage)
back_camera_subsriber = Subscriber('/miivii_gmsl_ros/camera2/compressed', CompressedImage)
right_camera_subsriber = Subscriber('/miivii_gmsl_ros/camera3/compressed', CompressedImage)
left_camera_subsriber = Subscriber('/miivii_gmsl_ros/camera4/compressed', CompressedImage)

lidar_subscriber = Subscriber('/livox/lidar', PointCloud2)


vehicle_center = np.array([16.3269, 14.4592, 0]) # hycan
center_to_rear_center = np.array([1.426, 0.15, 0.0])
vehicle_rot = np.array([0, 0, -0.999185, 0.040353])

vehicle_center = np.array([17.4442, 14.4884, 0]) # rock
vehicle_rot = np.array([0, 0, -0.999677, 0.02543])
center_to_rear_center = np.array([1.2975, 0, 0])

rock_lidar_calib = np.array([[1.0, 0.0, 0.0, 0],
                            [0.0, 1.0, 0.0, 0],
                            [0.0, 0.0, 1.0, 1.7],
                            [0.0, 0.0, 0.0, 1.0]])
np.savetxt("rock_lidar_calib.csv", rock_lidar_calib, delimiter=",")

rock_rear = np.array([[1.0, 0.0, 0.0, 1.2975],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0]])
                    

np.savetxt("rock_rear.csv", rock_rear, delimiter=",")

hycan_rear = np.array([[1.0, 0.0, 0.0, 1.426],
                    [0.0, 1.0, 0.0, 0.15],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0]])

np.savetxt("hycan_rear.csv", hycan_rear, delimiter=",")
                       

lidar_calib = np.array([[0.0, 1.0, 0.0, -0.1],
                        [1.0, 0.0, 0.0, 0.35],
                        [0.0, 0.0, 1.0, 1.83],
                        [0.0, 0.0, 0.0, 1.0]])
euler = np.array([89.3, 0.7, 179.5])
rotation = R.from_euler('zyx', euler, degrees=True)
lidar_calib[:3, :3] = rotation.as_matrix()
# Saveth the calibration matrix as csv file
np.savetxt("lidar_calib.csv", lidar_calib, delimiter=",")
# import pdb; pdb.set_trace()

lidar_bag = rosbag.Bag(lidar_bag_path, 'r')
camera_bag = rosbag.Bag(camera_bag_path, 'r')

lidar_msgs = lidar_bag.read_messages(['/livox/lidar'])
camera_msgs = camera_bag.read_messages(['/miivii_gmsl_ros/camera1/compressed',
                                        '/miivii_gmsl_ros/camera2/compressed',
                                        '/miivii_gmsl_ros/camera3/compressed',
                                        '/miivii_gmsl_ros/camera4/compressed'])

merged_msgs = sorted(list(lidar_msgs) + list(camera_msgs), key=lambda x: x[2])  # 按时间戳排序
# lidar points to camera coordinate system

def callback(lidar_msg, front_camera_msg, back_camera_msg, left_camera_msg, right_camera_msg):
    # get the lidar points
    points = []
    # 使用 read_points 函数提取点云中的 x, y, z 数据
    for point in pc2.read_points(lidar_msg, skip_nans=True, field_names=("x", "y", "z")):
        points.append([point[0], point[1], point[2]])

    # 转换为 NumPy 数组
    lidar_points = np.array(points, dtype=np.float32)

    # turn lidar carlib into a vehicle rear center coordinate system
    lidar_to_rear = np.linalg.inv(lidar_calib)
    lidar_points = np.concatenate((lidar_points, np.ones((lidar_points.shape[0], 1))), axis = 1)
    lidar_points = np.dot(lidar_to_rear, lidar_points.T)[:3, :]
    
    msgs = {
        "front": front_camera_msg,
        "back": back_camera_msg,
        "left": left_camera_msg,
        "right": right_camera_msg
    }

    for key in cam_models:
        cam_points = cam_models[key].world2cam(lidar_points)
        depth = cam_points[0, :]
        cam_points = cam_points[:, depth < 10]
        depth = depth[depth < 10]
        depth = depth[depth > 0]
        image_points = cam_models[key].cam2image(cam_points)


        # 将深度值归一化到 [0, 255]
        normalized_depth = np.clip(depth / 8, 0, 1) * 255
        normalized_depth = np.uint8(normalized_depth)

        # 使用 OpenCV 的 applyColorMap 将深度值映射到彩虹色
        color = cv2.applyColorMap(normalized_depth, cv2.COLORMAP_JET)

        # draw points to image
        image = cv_bridge.CvBridge().compressed_imgmsg_to_cv2(msgs[key])
        for point, c in zip(image_points.T, color):
            # import pdb; pdb.set_trace()
            cv2.circle(image, (int(point[0]), int(point[1])), 2, tuple(int(x) for x in c[0]), -1)
        cv2.imwrite("./camera/{}/{}.jpg".format(key, msgs[key].header.stamp.to_sec()), image)


vehicle = "Hycan"
cam_models = {"left": CamModel("left", vehicle), 
            "right": CamModel("right", vehicle),
            "front": CamModel("front", vehicle), 
            "back": CamModel("back", vehicle)}

for dir in cam_models:
    os.makedirs("./camera/{}".format(dir), exist_ok=True)

sync = ApproximateTimeSynchronizer([lidar_subscriber, front_camera_subsriber, back_camera_subsriber, left_camera_subsriber, right_camera_subsriber], queue_size=10, slop=0.1)
sync.registerCallback(callback)

for topic, msg, t in merged_msgs:
    if topic == '/livox/lidar':
        lidar_subscriber.signalMessage(msg)
    elif topic == '/miivii_gmsl_ros/camera1/compressed':
        front_camera_subsriber.signalMessage(msg)
    elif topic == '/miivii_gmsl_ros/camera2/compressed':
        back_camera_subsriber.signalMessage(msg)
    elif topic == '/miivii_gmsl_ros/camera3/compressed':
        right_camera_subsriber.signalMessage(msg)
    elif topic == '/miivii_gmsl_ros/camera4/compressed':
        left_camera_subsriber.signalMessage(msg)





