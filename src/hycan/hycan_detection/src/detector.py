#!/usr/bin/env python3
# -*- encoding: utf-8 -*-

import rospy
import numpy as np
import yaml
from hycan_msgs.msg import Localization, Box3D, DetectionResults
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
from hycan.hycan_detection.src.shm import Images_Shared_Memory

class Detector:
    def __init__(self, config_path, ckpt_path):
        # Get the vehicle name from ROS parameter server
        rospy.init_node('hycan_detector', anonymous=True)
        self.vehicle = rospy.get_param('~vehicle')
        
        # load the detector
        config = os.path.join(config_path, 'mv_dfm_{}.yaml'.format(self.vehicle))
        with open(config, 'r') as f:
            config = yaml.safe_load(f)

        self.img_shm = Images_Shared_Memory('hycan_image_shm', 4)
        self.mean = np.ascontiguousarray(np.broadcast_to(np.array([123.675, 116.28, 103.53]).reshape(1, 3, 1, 1), (4, 3, 368, 640)))
        self.std = np.ascontiguousarray(np.broadcast_to(np.array([58.395, 57.12, 57.375]).reshape(1, 3, 1, 1), (4, 3, 368, 640)))
        
        self.use_trt = False
        if not self.use_trt:
            self.detector = build_detector(config)
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
            self.detector.load_state_dict(torch.load(ckpt_path, map_location=self.device))
            self.detector.to(self.device)
            self.detector.eval()
            self.detector.bbox_head_3d.test_cfg = {
                "use_rotate_nms" : True,
                "nms_across_levels" : False,
                "nms_thr" : 0.02,
                "score_thr" : 0.2,
                "min_bbox_size" : 0,
                "nms_pre" : 500,
                "max_num" : 100
            }
            rospy.loginfo("Pytorch model is loaded")
        else:
            from common.Mono3d.tools.inference_test import TRTModel
            from pycuda import driver as cuda
            import pycuda.autoinit
            trt_path = ckpt_path.replace('.pth', '.engine')
            self.cfx = cuda.Device(0).make_context()
            self.detector = TRTModel(trt_path, 0.3, 0.05)
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
            rospy.loginfo("TensorRT model is loaded")
        
        # Initialize the GPU Memory
        if self.use_trt:
            dummy_input = np.random.randn(1, 4, 3, 368, 640).astype(np.float32)
            self.detector(dummy_input, None, None)

        # initialze the subscriber
        rospy.Subscriber('{}_processed_images'.format(self.vehicle), Localization, self.detect)
        self.pub = rospy.Publisher('{}_detection_results'.format(self.vehicle), DetectionResults, queue_size=10)
        rospy.loginfo("Detector is ready")

        ### FOR DEBUGGING
        self.prev_timestamp = None
        self.prev_frame_idx = None

    def __del__(self):
        if self.use_trt:
            self.cfx.pop()

    def get_shm_images(self):
        frame_idxs, imgs, camera_ids, \
            width_no_pad, height_no_pad, \
                original_width, original_height, \
                    ratio, timestamp = self.img_shm.read_imgs_from_shm()


        imgs = (imgs.transpose(0,3,1,2) - self.mean) / self.std
        
        if not self.use_trt:
            imgs = torch.FloatTensor(imgs).to(self.device).unsqueeze(0)
        else:
            imgs = np.expand_dims(imgs, axis=0)

        return frame_idxs, imgs, camera_ids, width_no_pad, height_no_pad, original_width, original_height, ratio, timestamp


    def detect(self, msg):
        # start time
        st = time()
        rospy.loginfo("Received hycan images")
        frame_idxs, imgs, camera_ids, \
            width_no_pad, height_no_pad, \
                original_width, original_height, \
                    ratio, timestamp = self.get_shm_images()
        if self.prev_frame_idx and frame_idxs[0] == self.prev_frame_idx:
            rospy.loginfo("The new frame is the same as the previous frame for {}".format(frame_idxs[0]))
            return 
        self.prev_frame_idx = frame_idxs[0]
        
        rospy.loginfo("Processing image {} with timestamp: {}".format(frame_idxs[0], time() - st))

        
        if self.prev_timestamp is not None:
            rospy.loginfo("FPS in detector is : {:6f}".format( 1/ (timestamp - self.prev_timestamp)))
        self.prev_timestamp = timestamp
        height, width = imgs.shape[3], imgs.shape[4]

        # rospy.loginfo("Images shape: {}, shape without pad is {}".format((height, width), 
        #                                                                 (height_no_pad, width_no_pad, 3)))
        img_metas = [
            dict(
                img_shape=[(height_no_pad, width_no_pad, 3)] * 4,
                ori_shape=[(original_height, original_width, 3)] *4,
                pad_shape=[(height, width, 3)] * 4,
                scale_factor=torch.FloatTensor([ratio, ratio]),
                flip=False,
                keep_ratio=True,
                num_views = 4,
                num_ref_frames = 0,
                direction = ['left', 'right','front', 'back'],
                pad_size_divisor = 16,
                box_type_3d = LiDARInstance3DBoxes,
            )
        ]
        
        # detect the objects
        if not self.use_trt:
            with torch.no_grad():
                result = self.detector(imgs, img_metas, return_loss=False)[0]
        else:
            self.cfx.push()
            result = self.detector(imgs, img_metas, return_loss=False)[0]
            self.cfx.pop()

        if hasattr(result['boxes_3d'], 'tensor'):
            bbox = result['boxes_3d'].tensor.cpu().numpy()[:, :7]
            scores = result['scores_3d'].cpu().numpy()
        else:
            bbox = result['boxes_3d'].numpy()[:, :7]
            scores = result['scores_3d'].numpy()

        # Demo code
        results = DetectionResults()
        for box, score in zip(bbox, scores):
            box_msg = Box3D()
            box_msg.center_x = box[0]
            box_msg.center_y = box[1]
            box_msg.center_z = box[2]
            box_msg.width = box[3]
            box_msg.length = box[4]
            box_msg.height = box[5]
            box_msg.heading = box[6]
            box_msg.score = score

            results.box3d_array.append(box_msg)

        results.num_boxes = len(bbox)
        results.localization = msg
        results.image_stamp = rospy.Time.from_sec(timestamp)
        results.vehicle_id = self.vehicle
        results.frame_idx = frame_idxs[0]

        rospy.loginfo("Inference time : {}, detection results number is {}".format(time() - st, results.num_boxes))
        # localization and time delay
        rospy.loginfo("Localization: {}".format((msg.utm_x, msg.utm_y,msg.heading)))
        self.pub.publish(results)

        rospy.loginfo("Total time delay is : {}".format(rospy.Time.now().to_sec() - timestamp))
        


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