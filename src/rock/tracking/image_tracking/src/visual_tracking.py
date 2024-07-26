#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
# AUTHOR: Haoran Wu
# FILE: visual_tracking.py
# DATE: 2021/09/22 周三
# TIME: 16:47:24
'''

import sys
import os

from pytools import F

MAIN_DIR = os.path.abspath(os.path.dirname(__file__))
os.chdir(MAIN_DIR)

from imutils.convenience import resize
import rospy
import sensor_msgs
import std_msgs
import numpy as np
import cv2
import imutils

from tracker import Tracker

from cyber_msgs.msg import Object, ObjectArray, Box2D

import time


class VisualTracker:
    def __init__(self) -> None:
        self.img_width = 1920
        self.img_height = 1200
        YOLOX_CKPT_DIR = os.path.join(os.path.abspath(os.path.dirname(__file__)), 'weights')
        ckpt_path = os.path.join(YOLOX_CKPT_DIR, 'yolox_s.pth.tar')
        self.tracker = Tracker(model='yolox-s', ckpt=ckpt_path, filter_class=['car', 'trunk'], trt=True)
        self.object_id = None
        self.click_pos = None
        # cv2.namedWindow('tracking', cv2.WINDOW_AUTOSIZE)
        # cv2.startWindowThread()
        # cv2.setMouseCallback('tracking', self.mouse_callback)
        self.curr_img = None

        rospy.init_node('visual_tracking', anonymous=True)
        rospy.Subscriber('/driver/fisheye/front/compressed', sensor_msgs.msg.CompressedImage , self.sub_callback, queue_size=1)
        rospy.Subscriber('/xboxone/rematch', std_msgs.msg.Bool, self.rematch_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.tracking_callback)
        self.pub = rospy.Publisher('/visual_track', ObjectArray, queue_size=1)
        self.roi_pub = rospy.Publisher('/tracking/detection/roi2d', Box2D, queue_size=1)
        
        # self.tracked_img_pub = rospy.Publisher('/visual_tracking/tracked_image', sensor_msgs.msg.Image, queue_size=1)
        rospy.spin()

    def sub_callback(self, data):
        # image = np.frombuffer(data.data, dtype=np.uint8).reshape(self.img_height, self.img_width, -1)
        # self.curr_img = image
        np_arr = np.fromstring(data.data, dtype=np.uint8)
        self.curr_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
    def rematch_callback(self, data):
        if data.data:
            rospy.logwarn("-------------Visual Tracking Rematch-------------")
            self.object_id = None
            self.click_pos = None

    def tracking_callback(self, timer_event):
        s = time.time()
        if self.curr_img is None:
            return
        resized = imutils.resize(self.curr_img, height=500)
        tracked, result = self.tracker.update(resized, filter_id=self.object_id)
        raw_tracked = tracked
        # tracked_image_msg = sensor_msgs.msg.Image()
        # tracked_image_msg_header = std_msgs.msg.Header(stamp=rospy.Time.now())
        # tracked_image_msg_header.frame_id = 'image'
        # tracked_image_msg.height = self.img_height
        # tracked_image_msg.width = self.img_width
        # tracked_image_msg.encoding = 'rgb8'
        # tracked_image_msg.step = 1920 * 3
        tracked = imutils.resize(self.curr_img, height=self.img_height)
        tracked = cv2.cvtColor(tracked, cv2.COLOR_BGR2RGB)
        # tracked_image_msg.header = tracked_image_msg_header
        cv2.imshow('tracking', raw_tracked)
        cv2.setMouseCallback('tracking', self.mouse_callback)
        cv2.waitKey(1)

        object_array = ObjectArray()
        for res in result:
            if self.object_id is None and self.click_pos is not None:  # decide new object id
                if res[1] < self.click_pos[0] < res[3] and res[0] < self.click_pos[1] < res[2]:
                    self.object_id = res[4]
            # resize coord back
            res = res.astype(np.float)
            res[:4] *= (self.img_height / 500)
            res = res.astype(np.int)
            
            if self.object_id != res[4]:
                continue

            obj = Object()
            obj.sensor_type = 1
            obj.object_id = res[4]
            center_x = (res[0] + res[2]) / 2
            center_y = (res[1] + res[3]) / 2
            obj.pose.position.x = center_x  # column index
            obj.pose.position.y = center_y  # row index

            # ROI
            roi = Box2D()
            roi.center_x = int(center_y)
            roi.center_y = int(center_x)
            roi.height = int(np.abs(res[1] - res[3]))
            roi.width = int(np.abs(res[0] - res[2]))

            self.roi_pub.publish(roi)

            object_array.objects.append(obj)
        obj_id = object_array.objects[0].object_id if len(object_array.objects) > 0 else -1
        # tracked_image_msg.data = tracked.tostring()
        # self.tracked_img_pub.publish(tracked_image_msg)
        self.pub.publish(object_array)
        

        e = time.time()
        rospy.loginfo('Visual tracking target {} | costs {} secs'.format(obj_id, e - s))
        
    def mouse_callback(self, event, x, y, flags, param):
        # x -- column index, y -- row index
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click_pos = (y, x)
            self.object_id = None
            rospy.loginfo('click pos ({}, {})'.format(y, x))


if __name__ == '__main__':
    tracker = VisualTracker()

