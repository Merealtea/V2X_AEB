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
from copy import deepcopy
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk

rospy.init_node('merge_bag')

# camera_bag_path
camera_bag_path = "/mnt/pool1/ground_truth_generator/data/camera_data/2024-11-13-19-43-57.bag"

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
                       

global_pitch = 89
global_roll = 0.5
global_yaw = 180.5
global_x_offset = 0.1
global_y_offset = 0.5
global_z_offset = 1.8

import pdb; pdb.set_trace()
camera_bag = rosbag.Bag(camera_bag_path, 'r')

history_points = []
his_len = 0

camera_msgs = camera_bag.read_messages(['/livox/lidar',
                                        '/miivii_gmsl_ros/camera1/compressed',
                                        '/miivii_gmsl_ros/camera2/compressed',
                                        '/miivii_gmsl_ros/camera3/compressed',
                                        '/miivii_gmsl_ros/camera4/compressed'])

# lidar points to camera coordinate system

def project_point_cloud(image_label, msgs, lidar_points, pitch, roll, yaw, x_offset, y_offset, z_offset):
        global global_pitch, global_roll, global_yaw, global_x_offset, global_y_offset, global_z_offset

        
        lidar_calib = np.array([[0.0, 1.0, 0.0, x_offset], # 左右平移
                                [1.0, 0.0, 0.0, y_offset], # 前后平移
                                [0.0, 0.0, 1.0, z_offset],
                                [0.0, 0.0, 0.0, 1.0]])

        # 环视方向水平旋转 88.1 度，左右翘 0.5 度，前后翘旋转 180.5 度
        euler = np.array([pitch, roll, yaw])
        rotation = R.from_euler('zyx', euler, degrees=True)
        lidar_calib[:3, :3] = rotation.as_matrix()
        # Saveth the calibration matrix as csv file
        np.savetxt("lidar_calib.csv", lidar_calib, delimiter=",")
        global_pitch = pitch
        global_roll = roll
        global_yaw = yaw
        global_x_offset = x_offset
        global_y_offset = y_offset
        global_z_offset = z_offset


        # turn lidar carlib into a vehicle rear center coordinate system
        lidar_to_rear = np.linalg.inv(lidar_calib)
        lidar_points = np.concatenate((lidar_points, np.ones((lidar_points.shape[0], 1))), axis = 1)
        lidar_points = np.dot(lidar_to_rear, lidar_points.T)[:3, :]

        images = []
            
        for key in cam_models:
            cam_points = cam_models[key].world2cam(lidar_points)
            depth = cam_points[0, :]
            cam_points = cam_points[:, depth < 10]
            depth = depth[depth < 10]
            depth = depth[depth > 0]

            # sort the points by depth from far to near
            # sort_idx = np.argsort(depth, order='desc')
            # cam_points = cam_points[:, sort_idx]
            # depth = depth[sort_idx]
            image_points = cam_models[key].cam2image(cam_points)

            # 将深度值归一化到 [0, 255]
            normalized_depth = np.clip(depth / 10, 0, 1) * 255
            normalized_depth = np.uint8(normalized_depth)

            # 使用 OpenCV 的 applyColorMap 将深度值映射到彩虹色
            color = cv2.applyColorMap(normalized_depth, cv2.COLORMAP_JET)

            # draw points to image
            image = cv_bridge.CvBridge().compressed_imgmsg_to_cv2(msgs[key])
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            for point, c in zip(image_points.T, color):
                # import pdb; pdb.set_trace()
                cv2.circle(image, (int(point[0]), int(point[1])), 1, tuple(int(x) for x in c[0]), -1)
            images.append(image)
        # Concatenate images
        images = [images[:2], images[2:]]
        images = [np.concatenate(images[0], axis=1), np.concatenate(images[1], axis=1)]
        images = np.concatenate(images, axis=0)
        print(images.shape)
        images = cv2.resize(images, (1280, 720))
        # Turn to Image
        images = Image.fromarray(images)
        tk_image = ImageTk.PhotoImage(images)
        image_label.configure(image=tk_image)
        image_label.image = tk_image
        # cv2.imwrite("./camera/{}/{}.jpg".format(key, msgs[key].header.stamp.to_sec()), image)


def callback(lidar_msg, front_camera_msg, back_camera_msg, left_camera_msg, right_camera_msg):
    global history_points, his_len
    # 使用 read_points 函数提取点云中的 x, y, z 数据
    for point in pc2.read_points(lidar_msg, skip_nans=True, field_names=("x", "y", "z")):
        history_points.append([point[0], point[1], point[2]])

    if his_len < 5:
        his_len += 1
        return 
    points = deepcopy(history_points)
    history_points.clear()
    his_len = 0
    # 转换为 NumPy 数组
    lidar_points = np.array(points, dtype=np.float32)

    # lidar_points = lidar_points[:, lidar_points[2] > 0.2]
    
    msgs = {
        "front": front_camera_msg,
        "back": back_camera_msg,
        "left": left_camera_msg,
        "right": right_camera_msg
    }

    window = tk.Tk()
    window.title("LiDAR Projection Adjustment")
    tk_image = ImageTk.PhotoImage(Image.new("RGB", (1280, 720), "white"))

    # Image display
    image_label = ttk.Label(window, image=tk_image)
    image_label.grid(row=0, column=0, columnspan=2)
    
    # Input fields for extrinsic parameters
    params = {
        "Pitch": global_pitch,
        "Roll": global_roll,
        "Yaw": global_yaw,
        "X Offset": global_x_offset,
        "Y Offset": global_y_offset,
        "Z Offset": global_z_offset 
    }

    entries = {}
    
    for idx, (param, value) in enumerate(params.items()):
        label = ttk.Label(window, text=param)
        label.grid(row=idx + 1, column=0, sticky='e')
        
        entry = ttk.Entry(window)
        entry.grid(row=idx + 1, column=1)
        entry.insert(0, str(value))
        
        entries[param] = entry

    # Button to update the projection
    def update_projection():
        # Retrieve all updated parameters
        pitch = float(entries["Pitch"].get())
        roll = float(entries["Roll"].get())
        yaw = float(entries["Yaw"].get())
        x_offset = float(entries["X Offset"].get())
        y_offset = float(entries["Y Offset"].get())
        z_offset = float(entries["Z Offset"].get())
        
        # Call the projection function
        project_point_cloud(image_label, msgs, lidar_points, pitch, roll, yaw, x_offset, y_offset, z_offset)
        
    update_button = ttk.Button(window, text="Update Projection", command=update_projection)
    update_button.grid(row=len(params) + 1, column=0, columnspan=2)
    
    window.mainloop()


vehicle = "Hycan"
cam_models = {"left": CamModel("left", vehicle),
              "front": CamModel("front", vehicle),  
            "right": CamModel("right", vehicle),
            "back": CamModel("back", vehicle)}

for dir in cam_models:
    os.makedirs("./camera/{}".format(dir), exist_ok=True)

sync = ApproximateTimeSynchronizer([lidar_subscriber, front_camera_subsriber, back_camera_subsriber, left_camera_subsriber, right_camera_subsriber], queue_size=10, slop=0.1)
sync.registerCallback(callback)

for topic, msg, t in camera_msgs:
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





