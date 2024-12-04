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
from utils import nms_depth
from collections import deque

def linear_assignment(cost_matrix):
  try:
    import lap
    _, x, y = lap.lapjv(cost_matrix, extend_cost=True)
    return np.array([[y[i],i] for i in x if i >= 0]) #
  except ImportError:
    from scipy.optimize import linear_sum_assignment
    x, y = linear_sum_assignment(cost_matrix)
    return np.array(list(zip(x, y)))

def pairwise_assignment(track_array1, track_array2, cost_threshold):
    """
        track_array is [x, y, z, l, w, h, yaw, global_id, score]
    """
    
    # calculate the cost matrix
    track1_position = np.expand_dims(track_array1[:, :2], axis=1)
    track2_position = np.expand_dims(track_array2[:, :2], axis=0)
    cost_matrix = np.linalg.norm(track1_position - track2_position, axis=2)

    # assign the track
    if min(cost_matrix.shape) > 0:
        a = (cost_matrix < cost_threshold).astype(np.int32)
        if a.sum(1).max() == 1 and a.sum(0).max() == 1:
            matched_indices = np.stack(np.where(a), axis=1)
        else:
            matched_indices = linear_assignment(cost_matrix)
    else:
        matched_indices = np.empty(shape=(0,2))

    # assign the unmatched track
    unmatched_track1 = []
    unmatched_track2 = []
    for t, trk in enumerate(track_array1):
        if(t not in matched_indices[:,0]):
            unmatched_track1.append(t)
    
    for t, trk in enumerate(track_array2):
        if(t not in matched_indices[:,1]):
            unmatched_track2.append(t)  

    #filter out matched with low IOU
    matches = []
    for m in matched_indices:
        if(cost_matrix[m[0], m[1]] > cost_threshold):
            unmatched_track1.append(m[0])
            unmatched_track2.append(m[1])
        else:
            matches.append(m.reshape(1,2))

    if(len(matches)==0):
        matches = np.empty((0,2),dtype=int)
    else:
        matches = np.concatenate(matches,axis=0)

    return matches, np.array(unmatched_track1), np.array(unmatched_track2)


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
        self.max_age = 2
        self.max_history_len = 10
        self.fusion_distance = 20

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
        num_bboxes = msg.num_boxes
        frame_idx = msg.frame_idx
        
        msg.sender.stamp = rospy.Time.now() - (msg.reciever.stamp - msg.sender.stamp)
        msg.image_stamp = rospy.Time.now() - (msg.reciever.stamp - msg.image_stamp)
        msg.reciever.stamp = rospy.Time.now()

        if vehicle_id not in self.prev_time:
            self.prev_time[vehicle_id] = msg.image_stamp.to_sec()
            self.vehicle_res_dict[vehicle_id] = deque(maxlen=self.max_history_len)
        else:
            rospy.loginfo(f"vehicle {vehicle_id} Time delay between two frames is {(msg.image_stamp.to_sec() - self.prev_time[vehicle_id])}")
            self.prev_time[vehicle_id] = msg.image_stamp.to_sec()

        rospy.loginfo(f"Receive {num_bboxes} bboxes in {frame_idx} from {vehicle_id}, time delay is {(msg.reciever.stamp - msg.image_stamp).to_sec()}" )
        # for person_id in track_res:
        #     x, y, z, w, l, h, yaw, score = track_res[person_id][:8]
        #     x = x - localization.utm_x
        #     y = y - localization.utm_y
        #     x, y = int(x / res + height // 2), int(y / res + width // 2)
        #     cv2.circle(bev_figure, (x, y), 5, (255, 0, 0), -1)
        #     cv2.putText(bev_figure, "{}_{:.2f}".format(person_id,score), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        #     # draw line fron origin point to the bbox
        #     cv2.line(bev_figure, (height // 2, width // 2), (x, y), (25, 200, 255), 1)

        # cv2.circle(bev_figure, (height // 2, width // 2), 5, (0, 0, 255), -1)
        # cv2.imwrite(os.path.join(self.debug_path, f"{frame_idx}_{vehicle_id}.png"), bev_figure)
            
        self.vehicle_res_dict[vehicle_id].append(msg)
        st = time.time()
        new_msg = DetectionResults()

        for vehicle_id in self.vehicle_res_dict.keys():
            if vehicle_id == "rock":
                vehicle_name = -1
            else:
                vehicle_name = -2
       
            msg = self.vehicle_res_dict[vehicle_id][-1]
            vehicle_box = Box3D()
            vehicle_box.center_x = msg.localization.utm_x
            vehicle_box.center_y = msg.localization.utm_y
            vehicle_box.center_z = 0.8
            vehicle_box.width = 4.6
            vehicle_box.length = 1.9  
            vehicle_box.height = 1.6
            vehicle_box.heading = msg.localization.heading
            vehicle_box.id = vehicle_name
            new_msg.box3d_array.append(vehicle_box)
            for box in msg.box3d_array:
                new_msg.box3d_array.append(box)

        new_msg.localization = msg.localization
        new_msg.num_boxes = len(new_msg.box3d_array)
        self.original_res_pub.publish(new_msg)
        rospy.loginfo("Publish original results with time {:4f}".format(time.time() - st))

    def send_fusion(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            cur_time = rospy.Time.now().to_sec()
            fusion_results = DetectionResults()
            prediction_results = {}
            vehicle_localization = []
            vehicle_align_result = {}

            for vehicle_id in self.vehicle_res_dict:
                time_diff = cur_time - self.prev_time[vehicle_id]
                
                if time_diff > self.max_age:
                    continue
                msg = self.vehicle_res_dict[vehicle_id][-1]
                bbox_array = []
                localization = msg.localization
                vehicle_localization.append([localization.utm_x, localization.utm_y, localization.heading])

                
                for i in range(msg.num_boxes):
                    box = msg.box3d_array[i]
                    bbox_array.append([box.center_x + time_diff * box.speed_x, 
                                       box.center_y + time_diff * box.speed_y, 
                                       box.center_z,
                                       box.width, box.length, box.height,
                                       box.heading, box.score, -1])
                bbox_array = np.array(bbox_array, dtype=np.float32)
                bbox_array = bbox_array.reshape(msg.num_boxes, -1)

                vehicle_align_result[vehicle_id] = bbox_array

            vehicle_localization = np.array(vehicle_localization).reshape(-1, 3)
            distance = np.linalg.norm(np.expand_dims(vehicle_localization[:, :2], axis = 0) - 
                                      np.expand_dims(vehicle_localization[:, :2], axis = 1), axis=2)
            connection_graph = np.triu(distance < self.fusion_distance, 1)
            edges = np.where(connection_graph)
            vehicle_ids = list(self.vehicle_res_dict.keys())
            
            for vehicle_id in vehicle_align_result:
                for idx in range(len(vehicle_align_result[vehicle_id])):
                    prediction_results[(vehicle_id, idx)] = {(vehicle_id, idx)}

            for i, j in zip(edges[0], edges[1]):
                first_vehicle_id = vehicle_ids[i]
                second_vehicle_id = vehicle_ids[j]

                first_vehicle_bboxes = vehicle_align_result[first_vehicle_id]
                second_vehicle_bboxes = vehicle_align_result[second_vehicle_id]

                matched, unmatched_first, unmatched_second =\
                      pairwise_assignment(deepcopy(first_vehicle_bboxes),
                                           deepcopy(second_vehicle_bboxes), 1)
                
                #  将所有相近的检测结果进行融合
                for m in matched:
                    box_1 = (first_vehicle_id, m[0])
                    box_2 = (second_vehicle_id, m[1])
                    rospy.loginfo("Matched {} and {}".format(box_1, box_2))
                    union_set = prediction_results[box_1] | prediction_results[box_2] 
                    prediction_results[box_1] = union_set
                    prediction_results[box_2] = union_set

            visited = dict(zip(prediction_results.keys(), [False] * len(prediction_results)))
            # 所有的匹配结果
            cluster_idx = 0
            cluster_res = {}
            
            for box_idx in prediction_results:
                if visited[box_idx]:
                    continue
                relevant_box_idxes = prediction_results[box_idx]
                cluster_res[cluster_idx] = []
                for vehicle_id, box_id in relevant_box_idxes:
                    cluster_res[cluster_idx].append(vehicle_align_result[vehicle_id][box_id])
                    visited[(vehicle_id, box_id)] = True
                cluster_idx += 1

            # 融合结果
            rospy.loginfo("Cluster {} results".format(len(cluster_res)))
            for cluster_idx in cluster_res:
                box_array = np.array(cluster_res[cluster_idx])

                cluster_center = np.mean(box_array, axis=0)
                distance = np.linalg.norm(box_array[:, :2] - cluster_center[:2], axis=1) + 1e-6
                fusion_weight = distance / np.sum(distance)
                fusion_box = np.sum(box_array * np.expand_dims(fusion_weight, axis=1), axis=0)

                fusion_res = Box3D()
                fusion_res.center_x = fusion_box[0]
                fusion_res.center_y = fusion_box[1]
                fusion_res.center_z = fusion_box[2]
                fusion_res.width = fusion_box[3]
                fusion_res.length = fusion_box[4]
                fusion_res.height = fusion_box[5]
                fusion_res.heading = fusion_box[6]
                fusion_res.id = cluster_idx
                fusion_results.box3d_array.append(fusion_res)
            
            fusion_results.num_boxes = len(fusion_results.box3d_array)
            fusion_results.sender.stamp = rospy.Time.from_sec(cur_time)
            self.fusion_result_pub.publish(fusion_results)
            self.fusion_results = []

            rospy.loginfo("Send fusion results with {} boxes ".format(fusion_results.num_boxes))

            rate.sleep()


if __name__ == '__main__':
    # 创建rospkg对象
    fusion_node = DetectionFusion()
    rospy.spin()