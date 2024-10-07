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
from common.Mono3d.tools.inference_test import TRTModel
from pycuda import driver as cuda
import pycuda.autoinit

class Detector:
    def __init__(self, config_path, ckpt_path):
        # Get the vehicle name from ROS parameter server
        rospy.init_node('hycan_detector', anonymous=True)
        self.vehicle = rospy.get_param('~vehicle')
        
        # load the detector
        config = os.path.join(config_path, 'mv_dfm_{}.yaml'.format(self.vehicle))
        with open(config, 'r') as f:
            config = yaml.safe_load(f)

        self.height = 480
        self.width = 640
        
        self.use_trt = True
        if not self.use_trt:
            self.detector = build_detector(config)
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
            self.detector.load_state_dict(torch.load(ckpt_path))
            self.detector.to(self.device)
            self.detector.eval()
            rospy.loginfo("Pytorch model is loaded")
        else:
            trt_path = ckpt_path.replace('.pth', '.engine')
            self.cfx = cuda.Device(0).make_context()
            self.detector = TRTModel(trt_path)
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
            rospy.loginfo("TensorRT model is loaded")

        # initialze the subscriber
        rospy.Subscriber('{}_processed_images'.format(self.vehicle), FourImages, self.detect)
        self.pub = rospy.Publisher('{}_detection_results'.format(self.vehicle), DetectionResults, queue_size=10)
        rospy.loginfo("Detector is ready")

    def __del__(self):
        if self.use_trt:
            self.cfx.pop()

    def to_tensor(self, img_msg):
        if self.use_trt:
            return torch.FloatTensor(
                np.array(img_msg.data, dtype=np.float32)
                    .reshape((3, img_msg.height, img_msg.width)))
        return torch.FloatTensor(
            np.array(img_msg.data, dtype=np.float32)
                .reshape((3, self.height, self.width))).to(self.device)

    def detect(self, msg):
        # start time
        st = time()
        rospy.loginfo("Received hycan images")

        # get the images
        image_front = self.to_tensor(msg.image_front)
        image_back = self.to_tensor(msg.image_back)
        image_left = self.to_tensor(msg.image_left)
        image_right = self.to_tensor(msg.image_right)

        # turn the images into tensor
        images = torch.stack([image_front, image_back, image_left, image_right]).unsqueeze(0)
        rospy.loginfo("Images shape: {}".format(images.shape))
        img_metas = [
            dict(
                img_shape=[(self.height, self.width, 3)] * 4,
                ori_shape=[(msg.height, msg.width, 3)] *4,
                pad_shape=[(self.height, self.width, 3)] * 4,
                scale_factor=torch.FloatTensor([0.5, 0.5, 0.5, 0.5]),
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
        if not self.use_trt:
            with torch.no_grad():
                result = self.detector(images, img_metas, return_loss=False)[0]
        else:
            self.cfx.push()
            result = self.detector(images, img_metas, return_loss=False)[0]
            self.cfx.pop()

        if hasattr(result['boxes_3d'], 'tensor'):
            bbox = result['boxes_3d'].tensor.cpu().numpy()[:, :7]
        else:
            bbox = result['boxes_3d'].numpy()[:, :7]

        # Demo code
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
    model_ckpt_path =  ws_path + '../model_ckpt/hycan_mv_fcos3d.pth'
    Detector(model_config_path, model_ckpt_path)
    rospy.spin()