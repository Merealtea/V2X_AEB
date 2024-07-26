#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
# AUTHOR: Haoran Wu
# FILE: vehicle_detection.py
# DATE: 2022/10/29 周六
# TIME: 20:31:26
'''
# fmt: off
import sys
import os
sys.path.insert(0, './YOLOX')
MAIN_DIR = os.path.abspath(os.path.dirname(__file__))
os.chdir(MAIN_DIR)



import imutils
import time
import cv2
import torch
import numpy as np
from cyber_msgs.msg import Object, ObjectArray, Box2D, Box2DArray
import std_msgs
import sensor_msgs
import rospy
from YOLOX.yolox.data.datasets.coco_classes import COCO_CLASSES
from detector import Detector
# fmt: on


class_names = COCO_CLASSES


class VehicleDetector:
    def __init__(self):
        self.img_width = 1920
        self.img_height = 1080
        YOLOX_CKPT_DIR = os.path.join(os.path.abspath(
            os.path.dirname(__file__)), 'weights')
        ckpt_path = os.path.join(YOLOX_CKPT_DIR, 'yolox_s.pth.tar')
        self.detector = Detector(model='yolox-s', ckpt=ckpt_path, trt=True)
        self.det_thresh = 0.15

        self.object_id = None
        self.click_pos = None

        self.curr_img = None
        self.curr_img_header = None

        self.det_flag = True

        rospy.init_node('visual_tracking', anonymous=True)
        rospy.Subscriber('/driver/fisheye/front/compressed',
                         sensor_msgs.msg.CompressedImage, self.sub_callback, queue_size=1)
        rospy.Subscriber('/xboxone/rematch', std_msgs.msg.Bool,
                         self.rematch_callback, queue_size=1)
        rospy.Subscriber('/tracking/visual_detection/flag',
                         std_msgs.msg.Bool, self.det_flag_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.detection_callback)
        self.pub = rospy.Publisher(
            '/tracking/visual_detection', Box2DArray, queue_size=2)
        self.roi_pub = rospy.Publisher(
            '/tracking/detection/roi2d', Box2D, queue_size=1)
        rospy.loginfo('visual detector initilized')

    def sub_callback(self, data):
        # image = np.frombuffer(data.data, dtype=np.uint8).reshape(self.img_height, self.img_width, -1)
        # self.curr_img = image
        self.curr_img_header = data.header
        np_arr = np.fromstring(data.data, dtype=np.uint8)
        self.curr_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def rematch_callback(self, data):
        if data.data:
            rospy.logwarn("-------------Visual Tracking Rematch-------------")
            self.object_id = None
            self.click_pos = None

    def det_flag_callback(self, data):
        self.det_flag = data.data

    def detection_callback(self, timer_event):
        if not self.det_flag:
            return
        curr_header = self.curr_img_header
        s = time.time()
        if self.curr_img is None:
            return
        resized = imutils.resize(self.curr_img, height=500)
        info = self.detector.detect(resized, visual=False)
        if info['box_nums'] == 0:
            return

        # bbox_xywh = []
        bbox_xyxy = []
        scores = []
        for (x1, y1, x2, y2), class_id, score in zip(info['boxes'], info['class_ids'], info['scores']):
            if class_names[int(class_id)] not in ['car', 'truck']:
                continue
            if score < self.det_thresh:
                continue
            # bbox_xywh.append(
            #     [int((x1 + x2) / 2), int((y1 + y2) / 2), x2 - x1, y2 - y1])
            bbox_xyxy.append([int(x1), int(y1), int(x2), int(y2)])
            scores.append(score)
        outputs = list(zip(bbox_xyxy, scores))
        outputs = self._nms(outputs)

        deted = imutils.resize(self.curr_img, height=500)
        for box in outputs:
            cv2.rectangle(deted, (int(box[0][0]), int(box[0][1])), (int(
                box[0][2]), int(box[0][3])), (0, 255, 0), 2)
        cv2.imshow('detection', deted)
        cv2.waitKey(1)

        deted = self.curr_img

        object_array = Box2DArray()
        object_array.header.stamp = curr_header.stamp
        for res in outputs:
            box = np.array(res[0])  # xyxy
            score = res[1]
            # resize coord back
            box = box.astype(np.float)
            box *= (self.img_height / 500)
            box = box.astype(np.int)

            obj = Box2D()
            obj.header.stamp = curr_header.stamp
            center_x = (box[0] + box[2]) / 2
            center_y = (box[1] + box[3]) / 2
            obj.center_x = int(center_y)
            obj.center_y = int(center_x)
            obj.height = int(box[3] - box[1])
            obj.width = int(box[2] - box[0])
            obj.score = score

            # cv2.rectangle(deted, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)

            # # ROI
            # roi = Box2D()
            # roi.center_x = int(center_y)
            # roi.center_y = int(center_x)
            # roi.height = int(np.abs(res[1] - res[3]))
            # roi.width = int(np.abs(res[0] - res[2]))

            # self.roi_pub.publish(roi)

            object_array.boxes.append(obj)
        self.pub.publish(object_array)

        # cv2.imshow('detection', deted)
        # cv2.waitKey(1)

        e = time.time()
        rospy.loginfo(
            'Visual detection | costs {} secs'.format(e - s))

    @staticmethod
    def _nms(bboxes: list):
        bboxes.sort(key=lambda box: box[1], reverse=True)
        bbox_xyxy = [box[0] for box in bboxes]
        # scores = [box[1] for box in bboxes]
        keep = [True] * len(bbox_xyxy)

        for i in range(len(bbox_xyxy)):
            if not keep[i]:
                continue
            area_1 = abs(bbox_xyxy[i][0] - bbox_xyxy[i][2]) * \
                abs(bbox_xyxy[i][1] - bbox_xyxy[i][3])
            for j in range(i + 1, len(bbox_xyxy)):
                if not keep[j]:
                    continue
                area_2 = abs(bbox_xyxy[j][0] - bbox_xyxy[j][2]) * \
                    abs(bbox_xyxy[j][1] - bbox_xyxy[j][3])
                w_intersec = min(
                    bbox_xyxy[i][2], bbox_xyxy[j][2]) - max(bbox_xyxy[i][0], bbox_xyxy[j][0])
                h_intersec = min(
                    bbox_xyxy[i][3], bbox_xyxy[j][3]) - max(bbox_xyxy[i][1], bbox_xyxy[j][1])
                area_intersec = w_intersec * h_intersec
                try:
                    iou = area_intersec / (area_1 + area_2 - area_intersec)
                    if iou > 0.5:
                        keep[j] = False
                except ZeroDivisionError:
                    continue

        return [bboxes[i] for i in range(len(bboxes)) if keep[i]]


if __name__ == '__main__':
    tracker = VehicleDetector()
    rospy.spin()
