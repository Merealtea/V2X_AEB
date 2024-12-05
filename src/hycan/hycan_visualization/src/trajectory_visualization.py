#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from hycan_msgs.msg import Localization, DetectionResults
import numpy as np
import os
import scipy

class Visualization:
    def __init__(self):
        rospy.init_node('hycan_vis', anonymous=True)
        self.veh_pub = rospy.Publisher('/hycan/vehicle', MarkerArray, queue_size=10)
        self.det_pub = rospy.Publisher('/hycan/detection', MarkerArray, queue_size=10)
        self.track_pub = rospy.Publisher('/hycan/track', MarkerArray, queue_size=10)
        self.fusion_pub = rospy.Publisher('/hycan/fusion', MarkerArray, queue_size=10)

        rospy.Subscriber('hycan_detection_results', DetectionResults, self.detection_callback)
        rospy.Subscriber('hycan_track_results', DetectionResults, self.track_callback)
        rospy.Subscriber('hycan_fusion_results', DetectionResults, self.fusion_callback)
        rospy.Subscriber('hycan_utm_localization', Localization, self.callback)
        self.init_x = None
        self.init_y = None
        self.yaw = None

        self.x_offset = 1.425
        self.y_offset = 0

        config_path = os.path.abspath(__file__).split('hycan')[0] + '/common/config/hycan_vis_config.rviz'
        os.system(f'rosrun rviz rviz -d {config_path}')

    def detection_callback(self, msg : DetectionResults):
        if self.init_x is None:
            return
        marker_array = MarkerArray()
        for i in range(msg.num_boxes):
            box = msg.box3d_array[i]
            tran_mat = np.array([[np.cos(self.yaw), -np.sin(self.yaw), self.init_x],
                                 [np.sin(self.yaw), np.cos(self.yaw), self.init_y],
                                 [0, 0, 1]])
            x, y, _ = tran_mat @ (np.array([[box.center_x, box.center_y, 1]])).T
            box.center_x = x
            box.center_y = y
            box.heading += self.yaw
            person_marker = self.person_marker(i, [1, 0, 0], box)
            marker_array.markers.append(person_marker)
        if len(marker_array.markers) > 0:
            self.det_pub.publish(marker_array)

    def track_callback(self, msg : DetectionResults):
        rospy.loginfo("Received {} boxes for visualization".format(msg.num_boxes))
        if self.init_x is None:
            return
        marker_array = MarkerArray()
        for i in range(msg.num_boxes):
            box = msg.box3d_array[i]
            person_marker = self.person_marker(i, [0, 1, 0], box)
            text_marker = self.text_marker(i + msg.num_boxes, f"{box.id}", box, [0, 1, 0])
            marker_array.markers.append(person_marker)
            marker_array.markers.append(text_marker)
        if len(marker_array.markers) > 0:
            self.track_pub.publish(marker_array)

    def fusion_callback(self, msg : DetectionResults):
        if self.init_x is None:
            return
        marker_array = MarkerArray()
        for i in range(msg.num_boxes):
            box = msg.box3d_array[i]
            person_marker = self.person_marker(i, [0, 0, 1], box)
            text_marker = self.text_marker(i + msg.num_boxes, f"{box.id}", box, [0, 0, 1])
            marker_array.markers.append(person_marker)
            marker_array.markers.append(text_marker)
        if len(marker_array.markers) > 0:
            self.fusion_pub.publish(marker_array)

    def person_marker(self, id, color, box):
        marker = Marker()
        marker.id = id
        marker.header.frame_id = "map"
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = box.length  # 长度
        marker.scale.y = box.width  # 宽度
        marker.scale.z = box.height  # 高度，设为一个较小的值
        marker.lifetime = rospy.Duration(0.2)

        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = box.center_x - self.init_x
        marker.pose.position.y = box.center_y - self.init_y
        marker.pose.position.z = box.center_z

        # 设置方向
        # 使用scipy将欧拉角转换为四元数
        orientation = scipy.spatial.transform.Rotation.from_euler('zyx', [box.heading, 0, 0]).as_quat()
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]

        # 设置颜色
        marker.color.a = 1.0  # Alpha, 1表示完全不透明
        marker.color.r = color[0]  # 红色
        marker.color.g = color[1]  # 绿色
        marker.color.b = color[2]  # 蓝色
        return marker

    def text_marker(self, id, text, box, color):
        marker = Marker()
        marker.id = id
        marker.header.frame_id = "map"
        marker.type = marker.TEXT_VIEW_FACING
        marker.action = marker.ADD
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.lifetime = rospy.Duration(0.2)

        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = box.center_x - self.init_x
        marker.pose.position.y = box.center_y - self.init_y
        marker.pose.position.z = box.center_z + 0.25 + box.height / 2

        # 设置颜色
        marker.color.a = 1.0  # Alpha, 1表示完全不透明
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]

        marker.text = text
        return marker

    def vehicle_arrow(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = 100
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.scale.x = 1
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        return marker

    def vehicle_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = 0
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = 4.6  # 长度
        marker.scale.y = 1.9  # 宽度
        marker.scale.z = 1.8  # 高度，设为一个较小的值
        marker.color.a = 1.0  # Alpha, 1表示完全不透明
        marker.color.r = 1.0  # 红色
        marker.color.g = 1.0  # 绿色
        marker.color.b = 0.0  # 蓝色
        return marker

    def callback(self, msg : Localization):
        marker_array = MarkerArray()
        x = msg.utm_x; y = msg.utm_y; yaw = msg.heading #- np.pi / 2

        self.init_x = x
        self.init_y = y
        self.yaw = yaw

        marker = self.vehicle_marker()
        arrow = self.vehicle_arrow()

        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = 1.4259 * np.cos(yaw) # center to rear axix
        marker.pose.position.y = 1.4259 * np.sin(yaw) # center to rear axix
        marker.pose.position.z = 0.9  # 高度，设为一个较小的值

        # 设置方向
        orientation = scipy.spatial.transform.Rotation.from_euler('zyx', [yaw, 0, 0]).as_quat()
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]
        
        arrow.header.stamp = rospy.Time.now()
        arrow.pose.position.x = self.x_offset * np.cos(yaw) - self.y_offset * np.sin(yaw)
        arrow.pose.position.y = self.x_offset * np.sin(yaw) + self.y_offset * np.cos(yaw)
        arrow.pose.position.z = 1.8
        arrow.pose.orientation.x = orientation[0]
        arrow.pose.orientation.y = orientation[1]
        arrow.pose.orientation.z = orientation[2]
        arrow.pose.orientation.w = orientation[3]

        marker_array.markers.append(marker)
        marker_array.markers.append(arrow)
        self.veh_pub.publish(marker_array)

if __name__ == '__main__':
    node = Visualization()
    rospy.spin()
