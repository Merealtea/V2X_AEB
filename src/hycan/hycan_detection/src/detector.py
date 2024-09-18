#!/usr/bin/env python3
# -*- encoding: utf-8 -*-

import rospy
import numpy as np
import yaml
from hycan_msgs.msg import FourImages, Box3D, DetectionResults
import sys
import os
# Add the path to the 'src' directory (parent of common and centerserver)
src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path.append(src_path)
sys.path.append(os.path.join(src_path, 'common', 'Mono3d'))
from common.Mono3d.models.builder import build_detector
from core.bbox.structures.lidar_box3d import LiDARInstance3DBoxes
import torch
import rospkg
from time import time
import cv2

class Detector:
    def __init__(self, ckpt_path):
        # Get the vehicle name from ROS parameter server
        rospy.init_node('hycan_detector', anonymous=True)
        self.vehicle = rospy.get_param('~vehicle')
        
        # load the detector
        config = os.path.join(ckpt_path, 'mv_dfm_{}.yaml'.format(self.vehicle))
        with open(config, 'r') as f:
            config = yaml.safe_load(f)
        self.detector = build_detector(config)
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        ckpt_file = "/mnt/pool1/V2X_AEB/data/model_parameter/hycan_mv_focs3d.pth"
        self.detector.load_state_dict(torch.load(ckpt_file))
        self.detector.to(self.device)
        self.detector.eval()

        self.height = 480
        self.width = 640

        # initialze the subscriber
        rospy.Subscriber('{}_processed_images'.format(self.vehicle), FourImages, self.detect)
        self.pub = rospy.Publisher('{}_detection_results'.format(self.vehicle), DetectionResults, queue_size=10)

    def to_tensor(self, img_msg, device):
        return torch.FloatTensor(
            np.array(img_msg.data, dtype=np.float32)
                .reshape((3, self.height, self.width))).to(device)

    def detect(self, msg):
        # start time
        st = time()

        # get the images
        image_front = self.to_tensor(msg.image_front, self.device)
        image_back = self.to_tensor(msg.image_back, self.device)
        image_left = self.to_tensor(msg.image_left, self.device)
        image_right = self.to_tensor(msg.image_right, self.device)

        # turn the images into tensor
        images = torch.stack([image_front, image_back, image_left, image_right]).unsqueeze(0)
        rospy.loginfo("Images shape: {}".format(images.shape))
        img_metas = [
            dict(
                img_shape=[(self.height, self.width, 3)] * 4,
                ori_shape=[(msg.height, msg.width, 3)] *4,
                pad_shape=[(self.height, self.width, 3)] * 4,
                scale_factor=np.array([0.5, 0.5, 0.5, 0.5], dtype=np.float32),
                flip=False,
                keep_ratio=True,
                num_views = 4,
                num_ref_frames = 0,
                direction = ['front', 'back', 'left', 'right'],
                pad_size_divisor = 16,
                box_type_3d = LiDARInstance3DBoxes,
            )
        ]
        
        # detect the objects
        result = self.detector(images, img_metas, return_loss=False)[0]
        bbox = result['boxes_3d'].tensor.cpu().numpy()[:, :7]

        # Demo code
        # bbox = [[1, -2, 0, 0.3, 0.3, 1.75, 0.5]]
        # rospy.sleep(0.2) # simulate the inference time

        results = DetectionResults()
        for box in bbox:
            box_msg = Box3D()
            box_msg.center_x = box[0]
            box_msg.center_y = box[1]
            box_msg.center_z = box[2]
            box_msg.width = box[3]
            box_msg.length = box[4]
            box_msg.height = box[5]
            box_msg.heading = box[6]

            results.box3d_array.append(box_msg)
        results.num_boxes = len(bbox)
        results.localization = msg.localization
        results.image_stamp = msg.image_front.header.stamp

        rospy.loginfo("Inference time : {}".format(time() - st))
        # localization and time delay
        localization = (msg.localization.utm_x, msg.localization.utm_y,msg.localization.heading)
        rospy.loginfo("Localization: {}".format(localization))
        self.pub.publish(results)
        


if __name__ == '__main__':
    # 创建rospkg对象
    rospack = rospkg.RosPack()

    # 获取当前包的路径
    package_path = rospack.get_path('hycan_detection')

    # 获取包的上一级目录
    ws_path = package_path.split('hycan')[0]

    model_config_path = ws_path + 'common/Mono3d/configs/models'
    Detector(model_config_path)
    rospy.spin()