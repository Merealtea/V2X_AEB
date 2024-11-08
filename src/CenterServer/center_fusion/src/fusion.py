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
from copy import deepcopy
from sklearn.cluster import DBSCAN
import cv2
from utils import nms_depth

class DetectionFusion:
    def __init__(self ):
        rospy.init_node('CenterFusion', anonymous=True)
        self.single_res_sub = rospy.Subscriber('detection_results', DetectionResults, self.track_callback)
        self.fusion_results = []
        self.fusion_result_pub = rospy.Publisher('fusion_results', DetectionResults, queue_size=10)
        self.original_res_pub = rospy.Publisher('original_results', DetectionResults, queue_size=10)
        self.vehicle_res_dict = {}
        self.vehicle_track_dict = {}
        self.prev_time = {}
        self.max_age = 0.5
        self.search_range = 1
        self.angle_diff_threshold = 10 / 180 * np.pi
        self.score_diff_threshold = 0.1

        ######### DEBUG #########
        self.debug_path = os.path.abspath(__file__).split('src')[0] + '/debug'
        if not os.path.exists(self.debug_path):
            os.makedirs(self.debug_path)
            rospy.loginfo("Created {} directory".format(self.debug_path))
        ######### DEBUG #########
        
        thread = threading.Thread(target=self.send_fusion)
        thread.start()
        thread.join()

    def track_callback(self, msg):
        vehicle_id = msg.vehicle_id
        localization = msg.localization
        num_bboxes = msg.num_boxes
        frame_idx = msg.frame_idx
        
        msg.sender.stamp = rospy.Time.now() - (msg.reciever.stamp - msg.sender.stamp)
        msg.image_stamp = rospy.Time.now() - (msg.reciever.stamp - msg.image_stamp)
        msg.reciever.stamp = rospy.Time.now()

        if vehicle_id not in self.prev_time:
            self.prev_time[vehicle_id] = msg.image_stamp.to_sec()
        else:
            rospy.loginfo(f"vehicle {vehicle_id} Time delay between two frames is {(msg.image_stamp.to_sec() - self.prev_time[vehicle_id])}")
            self.prev_time[vehicle_id] = msg.image_stamp.to_sec()
        
        update_res = {}
        bbox_array = []
        rospy.loginfo(f"Receive {num_bboxes} bboxes in {frame_idx} from {vehicle_id}, time delay is {(msg.reciever.stamp - msg.image_stamp).to_sec()}" )
        if num_bboxes > 0:
            
            for i in range(num_bboxes):
                bbox = msg.box3d_array[i]
                bbox_array.append([bbox.center_x, bbox.center_y, bbox.center_z, 
                                   bbox.width, bbox.length, bbox.height, 
                                   bbox.heading, bbox.score, bbox.id, bbox.speed_x, bbox.speed_y, bbox.speed_angle])
            bbox_array = np.ascontiguousarray(
                            np.array(bbox_array,
                                    dtype=np.float32)).reshape(num_bboxes, -1)
            
            keep = nms_depth(bbox_array, (localization.utm_x, localization.utm_y), self.angle_diff_threshold, self.score_diff_threshold)
            bbox_array = bbox_array[keep]
            
            if len(update_res) != len(bbox_array):
                raise ValueError("The number of track results is not equal to the number of bboxes")

        # The true range of the map is 25m * 25m 
        res = 0.05
        height, width = 500, 500
        bev_figure = np.zeros((height, height, 3), dtype=np.uint8)

        track_res = dict(zip(
            [int(box[8]) for box in bbox_array],
            bbox_array
        ))

        self.track_res[vehicle_id] = track_res

        for person_id in track_res:
            x, y, z, w, l, h, yaw, score = track_res[person_id][:8]
            x = x - localization.utm_x
            y = y - localization.utm_y
            x, y = int(x / res + height // 2), int(y / res + width // 2)
            cv2.circle(bev_figure, (x, y), 5, (255, 0, 0), -1)
            cv2.putText(bev_figure, "{}_{:.2f}".format(person_id,score), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # draw line fron origin point to the bbox
            cv2.line(bev_figure, (height // 2, width // 2), (x, y), (25, 200, 255), 1)

        cv2.circle(bev_figure, (height // 2, width // 2), 5, (0, 0, 255), -1)
        cv2.imwrite(os.path.join(self.debug_path, f"{frame_idx}_{vehicle_id}.png"), bev_figure)
            
        self.vehicle_res_dict[vehicle_id] = msg
        st = time.time()
        new_msg = DetectionResults()

        msg.box3d_array = []
        for id in update_res:
            box = Box3D()
            box.center_x = update_res[id][0]
            box.center_y = update_res[id][1]
            box.center_z = update_res[id][2]
            box.width = update_res[id][3]
            box.length = update_res[id][4]
            box.height = update_res[id][5]
            # TODO: yaw should be calculated by the direction of the vehicle
            box.heading = update_res[id][6]
            box.id = id
            msg.box3d_array.append(box) 

        for vehicle_id, msg in self.vehicle_res_dict.items():
            if vehicle_id == "rock":
                vehicle_id = -1
            else:
                vehicle_id = -2
            vehicle_box = Box3D()
            vehicle_box.center_x = msg.localization.utm_x
            vehicle_box.center_y = msg.localization.utm_y
            vehicle_box.center_z = 0.8
            vehicle_box.width = 4.6
            vehicle_box.length = 1.9  
            vehicle_box.height = 1.6
            vehicle_box.heading = msg.localization.heading
            vehicle_box.id = vehicle_id
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
            rospy.loginfo("Start to send fusion results, vehicle_sort is {}".format(len(self.vehicle_sort)))
            if len(self.vehicle_track_dict) > 0:
                cur_time = rospy.Time.now().to_sec()
                
                bbox_array = []
                prediction_results = {}

                # predict the current state
                for vehicle_id in self.vehicle_track_dict:
                    time_diff = cur_time - self.prev_time[vehicle_id]
                    new_boxes = deepcopy(self.vehicle_track_dict[vehicle_id])
                    new_boxes[:, [0, 1, 6]] = new_boxes[:, [0, 1, 6]] + time_diff * new_boxes[:, [9, 10, 11]]
                    prediction_results[vehicle_id] = new_boxes

                # TODO: Find better algorithm
                for vehicle_id in self.vehicle_track_dict:
                    bbox_array.append(prediction_results[vehicle_id])
                bbox_array = np.concatenate(bbox_array, axis=0)

                # clustering the bbox_array
                clustering = DBSCAN(eps=self.search_range, min_samples=1).fit(bbox_array[:, :2])
                labels = clustering.labels_
                unique_labels = np.unique(labels)
                fusion_results = DetectionResults()
                for label in unique_labels:
                    if label == -1:
                        continue
                    mask = labels == label
                    box = Box3D()
                    box.center_x = np.mean(bbox_array[mask, 0])
                    box.center_y = np.mean(bbox_array[mask, 1])
                    box.center_z = np.mean(bbox_array[mask, 2])
                    box.width = np.mean(bbox_array[mask, 3])
                    box.length = np.mean(bbox_array[mask, 4])
                    box.height = np.mean(bbox_array[mask, 5])
                    box.heading = np.mean(bbox_array[mask, 6])
                    box.id = label
                    fusion_results.box3d_array.append(box)

                fusion_results.num_boxes = len(fusion_results.box3d_array)
                fusion_results.sender.stamp = rospy.Time.from_sec(cur_time)

                self.fusion_result_pub.publish(fusion_results)
                self.fusion_results = []

                rospy.loginfo("Send fusion results")

            rate.sleep()


if __name__ == '__main__':
    # 创建rospkg对象
    fusion_node = DetectionFusion()
    rospy.spin()