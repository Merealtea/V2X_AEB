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

# camera_bag_path
camera_bag_path = "/mnt/pool1/ground_truth_generator/data/camera_data/2024-10-29-14-14-28_Rock.bag"
lidar_bag_path = "/mnt/pool1/ground_truth_generator/data/lidar_data/2024-10-29-14-14-28_filter.bag"

front_camera_subsriber = Subscriber('/driver/fisheye/front/compressed', CompressedImage)
back_camera_subsriber = Subscriber('/driver/fisheye/back/compressed', CompressedImage)   
right_camera_subsriber = Subscriber('/driver/fisheye/right/compressed', CompressedImage)
left_camera_subsriber = Subscriber('/driver/fisheye/left/compressed', CompressedImage)

lidar_subscriber = Subscriber('/driver/hesai/pandar', PointCloud2)


camera_bag = rosbag.Bag(camera_bag_path, 'r')

camera_msgs = camera_bag.read_messages(['/driver/fisheye/front/compressed', 
                                        '/driver/fisheye/back/compressed', 
                                        '/driver/fisheye/right/compressed', 
                                        '/driver/fisheye/left/compressed'])
lidar_bag = rosbag.Bag(lidar_bag_path, 'r')
lidar_msgs = lidar_bag.read_messages(['/driver/hesai/pandar'])

# merge the camera and lidar messages
merged_msgs = []
for topic, msg, t in lidar_msgs:
    merged_msgs.append((topic, msg, t))

for topic, msg, t in camera_msgs:
    merged_msgs.append((topic, msg, t))

merged_msgs = sorted(merged_msgs, key=lambda x: x[2])

def callback(lidar_msg, front_camera_msg, back_camera_msg, left_camera_msg, right_camera_msg):
    # get the lidar points
    points = []
    # 使用 read_points 函数提取点云中的 x, y, z 数据
    for point in pc2.read_points(lidar_msg, skip_nans=True, field_names=("x", "y", "z")):
        points.append([point[0], point[1], point[2]])

    # 转换为 NumPy 数组
    lidar_points = np.array(points, dtype=np.float32).T

    
    msgs = {
        "front": front_camera_msg,
        "back": back_camera_msg,
        "left": left_camera_msg,
        "right": right_camera_msg
    }
    # import pdb; pdb.set_trace()
    for key in cam_models:
        cam_points = cam_models[key].world2cam(lidar_points)
        depth = cam_points[0, :]
        cam_points = cam_points[:, depth < 10]
        depth = depth[depth < 10]
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


vehicle = "Rock"
cam_models = {"left": CamModel("left", vehicle), 
            "right": CamModel("right", vehicle),
            "front": CamModel("front", vehicle), 
            "back": CamModel("back", vehicle)}

for dir in cam_models:
    os.makedirs("./camera/{}".format(dir), exist_ok=True)

sync = ApproximateTimeSynchronizer([lidar_subscriber, 
                                    front_camera_subsriber, 
                                    back_camera_subsriber, 
                                    left_camera_subsriber, 
                                    right_camera_subsriber], queue_size=50, slop=0.1)
sync.registerCallback(callback)

for topic, msg, t in merged_msgs:
    if topic == '/driver/hesai/pandar':
        lidar_subscriber.signalMessage(msg)
    elif topic == '/driver/fisheye/front/compressed':
        front_camera_subsriber.signalMessage(msg)
    elif topic == '/driver/fisheye/back/compressed':
        back_camera_subsriber.signalMessage(msg)
    elif topic == '/driver/fisheye/right/compressed':
        right_camera_subsriber.signalMessage(msg)
    elif topic == '/driver/fisheye/left/compressed':
        left_camera_subsriber.signalMessage(msg)





