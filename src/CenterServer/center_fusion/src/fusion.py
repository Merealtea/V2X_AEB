#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
'''
# AUTHOR: Haoran Wu, Hongxin Chen
# FILE: decoder.py
# DATE: 2022/01/04 周二
# TIME: 20:36:14
'''

import rospy
import numpy as np
import threading
import time
from hycan_msgs.msg import DetectionResults, Box3D
import sys
import os
abs_path = os.path.dirname(__file__).split("fusion.py")[0]
sys.path.append(abs_path)
from kalman_filter import Sort
from sklearn.cluster import DBSCAN

class DetectionFusion:
    def __init__(self ):
        rospy.init_node('CenterFusion', anonymous=True)
        self.single_res_sub = rospy.Subscriber('detection_results', DetectionResults, self.track_callback)
        self.fusion_results = []
        self.fusion_result_pub = rospy.Publisher('fusion_results', DetectionResults, queue_size=10)
        self.vehicle_sort = {}
        self.original_res_pub = rospy.Publisher('original_results', DetectionResults, queue_size=10)
        self.vehicle_res_dict = {}
        thread = threading.Thread(target=self.send_fusion)
        thread.start()
        thread.join()

    def track_callback(self, msg):
        vehicle_id = msg.vehicle_id
        localization = msg.localization
        num_bboxes = msg.num_boxes

        rospy.loginfo(f"Receive {num_bboxes} bboxes from {vehicle_id}")
        if num_bboxes > 0:
            bbox_array = []
            for i in range(num_bboxes):
                bbox = msg.box3d_array[i]
                bbox_array.append([bbox.center_x, bbox.center_y, bbox.center_z, bbox.width, bbox.length, bbox.height, bbox.heading])
            bbox_array = np.ascontiguousarray(
                            np.array(bbox_array,
                                    dtype=np.float32)).reshape(num_bboxes, -1)

            if vehicle_id not in self.vehicle_sort:
                self.vehicle_sort[vehicle_id] = Sort(vehicle_id, (localization.utm_x, localization.utm_y, localization.heading))
            
            self.vehicle_sort[vehicle_id].update(localization,
                                                    bbox_array, 
                                                    msg.image_stamp.to_sec())
        self.vehicle_res_dict[vehicle_id] = msg
        st = time.time()
        new_msg = DetectionResults()
        for vehicle_id, msg in self.vehicle_res_dict.items():
            vehicle_box = Box3D()
            vehicle_box.center_x = msg.localization.utm_x
            vehicle_box.center_y = msg.localization.utm_y
            vehicle_box.center_z = 0.8
            vehicle_box.width = 4.6
            vehicle_box.length = 1.9
            vehicle_box.height = 1.6
            vehicle_box.heading = msg.localization.heading
            new_msg.box3d_array.append(vehicle_box)
            for box in msg.box3d_array:
                new_msg.box3d_array.append(box)
        new_msg.localization = msg.localization
        self.original_res_pub.publish(new_msg)
        rospy.loginfo("Publish original results with time {:4f}".format(time.time() - st))

    def send_fusion(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            # TODO : Get prediction results from kalman filter
            if len(self.vehicle_sort) == 0:
                cur_time = time.time()
                
                points_2d = []
                prediction_results = {}
                # Get asynchronized results from kalman filter
                for vehicle_id in self.vehicle_sort:
                    prediction_results[vehicle_id] =\
                        self.vehicle_sort[vehicle_id].predict(cur_time)
                    rospy.loginfo("Predicted result is {}".format(prediction_results[vehicle_id]))
                    for box in prediction_results[vehicle_id]:
                        points_2d.append(box[:2])
                points_2d = np.array(points_2d)
                if len(points_2d) == 0:
                    rate.sleep()
                    continue
                db = DBSCAN(eps=0.25, min_samples=1).fit(points_2d)
                labels = db.labels_

                # Fuse the results 
                fusion_results = DetectionResults()

                for i in range(len(labels)):
                    mask = labels == i
                    box = Box3D()
                    box.center_x = points_2d[mask, 0].mean()
                    box.center_y = points_2d[mask, 1].mean()
                    box.center_z = 1.75 / 2
                    box.width = 0.3
                    box.length = 0.3
                    box.height = 1.75
                    # TODO: yaw should be calculated by the direction of the vehicle
                    box.heading = 0
                    fusion_results.box3d_array.append(box) 

                fusion_results.num_boxes = len(labels)
                fusion_results.sender.stamp = rospy.Time.from_sec(cur_time)

                self.fusion_result_pub.publish(fusion_results)
                self.fusion_results = []

                rospy.loginfo("Send fusion results")

            rate.sleep()


if __name__ == '__main__':
    # 创建rospkg对象
    fusion_node = DetectionFusion()
    rospy.spin()