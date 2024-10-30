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
import cv2

class DetectionFusion:
    def __init__(self ):
        rospy.init_node('CenterFusion', anonymous=True)
        self.single_res_sub = rospy.Subscriber('detection_results', DetectionResults, self.track_callback)
        self.fusion_results = []
        self.fusion_result_pub = rospy.Publisher('fusion_results', DetectionResults, queue_size=10)
        self.vehicle_sort = {}
        self.original_res_pub = rospy.Publisher('original_results', DetectionResults, queue_size=10)
        self.vehicle_res_dict = {}
        self.prev_time = {}

        ######### DEBUG #########
        self.debug_path = os.path.abspath(__file__).split('src')[0] + '../debug'
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
        tracker_res = {}
        bbox_array = []
        rospy.loginfo(f"Receive {num_bboxes} bboxes in {frame_idx} from {vehicle_id}, time delay is {(msg.reciever.stamp - msg.image_stamp).to_sec()}" )
        if num_bboxes > 0:
            
            for i in range(num_bboxes):
                bbox = msg.box3d_array[i]
                bbox_array.append([bbox.center_x, bbox.center_y, bbox.center_z, 
                                   bbox.width, bbox.length, bbox.height, 
                                   bbox.heading, bbox.score])
            bbox_array = np.ascontiguousarray(
                            np.array(bbox_array,
                                    dtype=np.float32)).reshape(num_bboxes, -1)
            if vehicle_id not in self.vehicle_sort:
                self.vehicle_sort[vehicle_id] = Sort(vehicle_id, (localization.utm_x, localization.utm_y, localization.heading))
            
            update_res, tracker_res = self.vehicle_sort[vehicle_id].update(localization,
                                                    bbox_array, 
                                                    msg.image_stamp.to_sec())
            
            if len(update_res) != len(bbox_array):
                raise ValueError("The number of track results is not equal to the number of bboxes")

        # The true range of the map is 25m * 25m 
        bev_figure = np.zeros((1000, 1000, 3), dtype=np.uint8)

        for person_id in tracker_res:
            x, y, z, w, l, h, yaw = tracker_res[person_id]
            x = x - localization.utm_x
            y = y - localization.utm_y
            x, y = int(x / 0.025 + 500), int(y / 0.025 + 500)

            cv2.circle(bev_figure, (x, y), 5, (0, 255, 255), -1)

            search_range = 0.5
            cv2.cirecle(bev_figure, (x, y), int(search_range / 0.025), (0, 255, 0), 1)

        for box in bbox_array:
            x, y, z, w, l, h, yaw, score = box
            x = x - localization.utm_x
            y = y - localization.utm_y
            x, y = int(x / 0.025 + 500), int(y / 0.025 + 500)
            cv2.circle(bev_figure, (x, y), 5, (255, 0, 0), -1)
            cv2.putText(bev_figure, "{:3f}".format(score), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

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

        ####################################################
        # DEBUG
        # tracker prediction
        # track_msg= DetectionResults()
        # for id in tracker_res:
        #     box = Box3D()
        #     box.center_x = tracker_res[id][0]
        #     box.center_y = tracker_res[id][1]
        #     box.center_z = tracker_res[id][2]
        #     box.width = tracker_res[id][3]
        #     box.length = tracker_res[id][4]
        #     box.height = tracker_res[id][5]
        #     box.heading = tracker_res[id][6]
        #     box.id = id
        #     track_msg.box3d_array.append(box)
        # self.fusion_result_pub.publish(track_msg)
        ####################################################

    def send_fusion(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            # TODO : Get prediction results from kalman filter
            rospy.loginfo("Start to send fusion results, vehicle_sort is {}".format(len(self.vehicle_sort)))
            if len(self.vehicle_sort) > 0:
                cur_time = rospy.Time.now().to_sec()
                
                points_2d = []
                bbox_array = []
                prediction_results = {}
                # Get asynchronized results from kalman filter
                for vehicle_id in self.vehicle_sort:
                    prediction_results[vehicle_id] =\
                        self.vehicle_sort[vehicle_id].predict(cur_time)
                    for box in prediction_results[vehicle_id].values():
                        points_2d.append(box[:2].T)
                        bbox_array.append(box.T)
                if len(points_2d) == 0:
                    rate.sleep()
                    continue
                points_2d = np.concatenate(points_2d, axis=0)
                bbox_array = np.concatenate(bbox_array, axis=0)
                rospy.loginfo("Predicted result is {}".format(bbox_array))

                db = DBSCAN(eps=0.25, min_samples=1).fit(points_2d)
                labels = db.labels_

                # Fuse the results 
                fusion_results = DetectionResults()

                for i in range(len(labels)):
                    mask = labels == i
                    if sum(mask) < 1:
                        continue
                    box = Box3D()
                    box.center_x = bbox_array[mask, 0].mean()
                    box.center_y = bbox_array[mask, 1].mean()
                    box.center_z = bbox_array[mask, 2].mean()
                    box.width = bbox_array[mask, 3].mean()
                    box.length = bbox_array[mask, 4].mean()
                    box.height = 1.75
                    box.id = i
                    # TODO: yaw should be calculated by the direction of the vehicle
                    box.heading = 0
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