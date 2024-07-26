#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
import std_msgs
from sensor_msgs.msg import Image, CompressedImage
from cyber_msgs.msg import Box2D, Box2DArray

import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
import json
import time

# In OpenCV, dim 0: horizontal, dim 1: vertical
# In NumPy and Box2D, dim 0: vertical, dim 1: horizontal


class RearLightDetector:
    def __init__(self, log_path: str) -> None:
        # ros
        # rospy.Subscriber('/driver/camera/image', Image, self._image_callback)
        rospy.Subscriber('/driver/fisheye/front/compressed',
                         CompressedImage, self._image_callback, queue_size=1)
        rospy.Subscriber('/tracking/components_tracking/vh', Box2D,
                         self._roi_callback, queue_size=1)
        self._detection_timer = rospy.Timer(
            rospy.Duration(0.1), self._detect_callback)
        self._pub_rearlights = rospy.Publisher(
            '/tracking/components_tracking/rearlight', Box2DArray, queue_size=1)

        self._log_path = log_path

        # parameters
        self._img_size = (1080, 1920)
        self._brightness_threshold = 80.0
        self._connected_component_area_threshold = (50, 2e5)
        self._match_gate = 20.0

        # buffer
        self._curr_img = None
        self._curr_img_stamp = None
        self._curr_roi = None
        self._last_det = [None, None]

    def _image_callback(self, data):
        np_arr = np.frombuffer(data.data, dtype=np.uint8)
        self._curr_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self._curr_img_stamp = data.header.stamp

    def _roi_callback(self, data):
        self._curr_roi = data

    def _detect_callback(self, timer_event):
        ros_time = rospy.Time.now()
        s = time.time()
        # read image
        if self._curr_img is None or self._curr_roi is None:
            return
        image = self._curr_img
        roi = self._curr_roi

        # # ROI expasion
        # roi.width += 50
        # roi.height += 20

        # detect rear lights
        img_HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        avg_brightness = np.sum(
            img_HSV[:, :, 2]) / (self._img_size[0] * self._img_size[1])
        is_night = avg_brightness < self._brightness_threshold
        # segment red area
        if is_night:
            red_mask_1 = cv2.inRange(
                img_HSV, (0, 70, 55), (10, 255, 255))  # nighttime
            red_mask_2 = cv2.inRange(img_HSV, (170, 70, 55), (180, 255, 255))
            # white_mask = cv2.inRange(img_HSV, (0, 0, 220), (180, 30, 255))
            # red_mask_1 = cv2.bitwise_or(red_mask_1, white_mask)
            red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)
        else:
            red_mask_1 = cv2.inRange(img_HSV, (0, 70, 30), (10, 255, 255))
            red_mask_2 = cv2.inRange(img_HSV, (170, 70, 30), (180, 255, 255))
            red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)
        # morphology
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(
            red_mask, cv2.MORPH_CLOSE, kernel, iterations=5)
        mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel, iterations=1)

        # ROI mask
        ROI_left_top = np.array([np.max([roi.center_x - roi.height / 2.0, 0.0]),
                                np.max([roi.center_y - roi.width / 2.0, 0.0])], dtype=np.int)
        ROI_right_down = np.array([np.min([roi.center_x + roi.height / 2.0, self._img_size[0]]),
                                  np.min([roi.center_y + roi.width / 2.0, self._img_size[1]])], dtype=np.int)
        ROI_mask = np.zeros(self._img_size, dtype=np.uint8)
        ROI_mask[ROI_left_top[0]:ROI_right_down[0],
                 ROI_left_top[1]:ROI_right_down[1]] = 255

        # rearlights_mask = cv2.bitwise_and(mask, ROI_mask)

        # connected component extration
        connected_components = cv2.connectedComponentsWithStats(
            mask, connectivity=8, ltype=cv2.CV_32S)
        num_labels = connected_components[0]
        labels = connected_components[1]
        stats = connected_components[2]
        centroids = connected_components[3].astype(
            np.int)  # np.ndarray (num_labels, 2)
        left_idx, right_idx = self._select_best_match(
            connected_components, roi, ROI_mask)

        # record current frame detection result
        for i, idx in enumerate([left_idx, right_idx]):
            if idx is None:
                continue
            det = Box2D()
            det.center_x = int(centroids[idx, 1])
            det.center_y = int(centroids[idx, 0])
            det.height = int(stats[idx, 3])
            det.width = int(stats[idx, 2])

            # check
            if self._last_det[i] is not None:
                x_diff = np.abs(det.center_x - self._last_det[i].center_x)
                y_diff = np.abs(det.center_y - self._last_det[i].center_y)
                # if np.linalg.norm([x_diff, y_diff]) > 80.0:
                #     det = self._last_det[i]

            self._last_det[i] = det

        # # publish
        # rearlight_array = Box2DArray()
        # rearlight_array.header.stamp = self._curr_img_stamp
        # rearlight_array.header.frame_id = 'camera'
        # rearlight_array.boxes.append(self._last_det[0])
        # rearlight_array.boxes.append(self._last_det[1])
        # self._pub_rearlights.publish(rearlight_array)
        # log_data = {}
        # log_data['vehicle'] = None
        # left_rearlight_data = {'x': float(self._last_det[0].center_y - self._last_det[0].width / 2),
        #                        'y': float(self._last_det[0].center_x - self._last_det[0].height / 2),
        #                        'w': float(self._last_det[0].width),
        #                        'h': float(self._last_det[0].height)} if self._last_det[0] is not None else None
        # log_data['left_rearlight'] = left_rearlight_data
        # right_rearlight_data = {'x': float(self._last_det[1].center_y - self._last_det[1].width / 2),
        #                        'y': float(self._last_det[1].center_x - self._last_det[1].height / 2),
        #                        'w': float(self._last_det[1].width),
        #                        'h': float(self._last_det[1].height)} if self._last_det[1] is not None else None
        # log_data['right_rearlight'] = right_rearlight_data
        # with open(os.path.join(self._log_path, 'inference_result', '{}.json'.format(self._curr_img_stamp.to_sec())), 'w') as f:
        #     json.dump(log_data, f)

        e = time.time()
        rospy.loginfo('cost {} sec'.format(e - s))

        # vis
        vis = image
        # vis = cv2.bitwise_and(vis, vis, mask=mask)
        # if left_idx is not None:
        #     vis = cv2.rectangle(vis, (int(self._last_det[0].center_y - 0.5 * self._last_det[0].width),
        #                               int(self._last_det[0].center_x - 0.5 * self._last_det[0].height)),
        #                         (int(self._last_det[0].center_y + 0.5 * self._last_det[0].width),
        #                          int(self._last_det[0].center_x + 0.5 * self._last_det[0].height)),
        #                         (0, 255, 0), 2)
        # if right_idx is not None:
        #     vis = cv2.rectangle(vis, (int(self._last_det[1].center_y - 0.5 * self._last_det[1].width),
        #                               int(self._last_det[1].center_x - 0.5 * self._last_det[1].height)),
        #                         (int(self._last_det[1].center_y + 0.5 * self._last_det[1].width),
        #                          int(self._last_det[1].center_x + 0.5 * self._last_det[1].height)),
        #                         (0, 255, 0), 2)
        if self._last_det[0] is not None:
            vis = cv2.rectangle(vis, (int(self._last_det[0].center_y - 0.5 * self._last_det[0].width),
                                        int(self._last_det[0].center_x - 0.5 * self._last_det[0].height)),
                                    (int(self._last_det[0].center_y + 0.5 * self._last_det[0].width),
                                    int(self._last_det[0].center_x + 0.5 * self._last_det[0].height)),
                                    (0, 255, 0), 2)
        if self._last_det[1] is not None:
            vis = cv2.rectangle(vis, (int(self._last_det[1].center_y - 0.5 * self._last_det[1].width),
                                        int(self._last_det[1].center_x - 0.5 * self._last_det[1].height)),
                                    (int(self._last_det[1].center_y + 0.5 * self._last_det[1].width),
                                    int(self._last_det[1].center_x + 0.5 * self._last_det[1].height)),
                                    (0, 255, 0), 2)
        vis = cv2.resize(
            vis, (int(self._img_size[1] / 2), int(self._img_size[0] / 2)), cv2.INTER_CUBIC)
        cv2.imwrite(os.path.join(self._log_path, 'vis', '{}.jpg'.format(self._curr_img_stamp.to_sec())), vis)
        cv2.imshow('rearlights', vis)
        cv2.waitKey(1)

    def _select_best_match(self, connected_components, roi, roi_mask):
        num_labels = connected_components[0]    # int
        labels = connected_components[1]    # np.ndarray (H, W)
        # np.ndarray (num_labels, 5), (leftmost x, topmost y, width, height, area)
        stats = connected_components[2]
        centroids = connected_components[3].astype(
            np.int)  # np.ndarray (num_labels, 2)

        # filter connected components with area and location
        keep_area = np.where((stats[:, 4] > self._connected_component_area_threshold[0]) & (
            stats[:, 4] < self._connected_component_area_threshold[1]))
        # keep_loc_1 = np.where((centroids[:, 0] >= int(roi.center_y - 0.5 * roi.width)) & \
        #     (centroids[:, 0] <= int(roi.center_y + 0.5 * roi.width)))
        # keep_loc_2 = np.where((centroids[:, 1] >= int(roi.center_y - 0.5 * roi.height)) & \
        #     (centroids[:, 1] <= int(roi.center_y + 0.5 * roi.height)))
        keep_area = set(keep_area[0])
        # keep_loc_1 = set(keep_loc_1[0])
        # keep_loc_2 = set(keep_loc_2[0])
        keep = list(keep_area)

        # symmetry verification
        symmetry_verified = []
        for i, master in enumerate(keep):
            master_x = centroids[master, 0]
            master_y = centroids[master, 1]
            if roi_mask[master_y, master_x] == 0:
                continue
            master_height = stats[master, 3]
            master_area = stats[master, 4]
            for slave in keep[i + 1:]:
                slave_x = centroids[slave, 0]
                slave_y = centroids[slave, 1]
                if roi_mask[slave_y, slave_x] == 0:
                    continue
                slave_height = stats[slave, 3]
                slave_area = stats[slave, 4]
                max_height = np.max([master_height, slave_height])
                if (np.abs(master_y - slave_y) <= max_height):  # pass symmetry verification
                    horizontal_dis = np.abs(master_x - slave_x)
                    avg_area = (master_area + slave_area) / 2
                    symmetry_verified.append(
                        (master, slave, horizontal_dis, avg_area))
        if len(symmetry_verified) == 0:
            return (None, None)

        # find best match
        symmetry_verified.sort(key=lambda x: x[2], reverse=True)
        k = np.min([len(symmetry_verified), 5])
        dis_topk = symmetry_verified[:k]
        dis_topk.sort(key=lambda x: x[3], reverse=True)

        best_match = dis_topk[0]
        # check according last frame
        if self._last_det[0] is not None and self._last_det[1] is not None:
            for match in dis_topk:
                (left_idx, right_idx) = (
                    match[0], match[1]) if centroids[match[0], 0] < centroids[match[1], 0] else (match[1], match[0])
                left_in_gate = np.linalg.norm([centroids[left_idx, 0] - self._last_det[0].center_y, centroids[left_idx, 1] - self._last_det[0].center_x]) < \
                    self._match_gate
                right_in_gate = np.linalg.norm([centroids[right_idx, 0] - self._last_det[1].center_y, centroids[right_idx, 1] - self._last_det[1].center_x]) < \
                    self._match_gate
                if left_in_gate and right_in_gate:
                    best_match = match
                    break
        if (centroids[best_match[0], 0] < centroids[best_match[1], 0]):
            best_match = (best_match[0], best_match[1])
        else:
            best_match = (int(best_match[1]), int(best_match[0]))

        return best_match   # (left idx, right idx)


if __name__ == '__main__':
    rospy.init_node('rearlight_detector', anonymous=True)
    detector = RearLightDetector(
        '/media/wuhr/data/platoon_dataset/2022_10_20/evaluation/2d/compare')
    rospy.spin()
