#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
from hycan_msgs.msg import Localization
import math

class Visualization:
    def __init__(self):
        rospy.init_node('visualization', anonymous=True)
        self.pub = rospy.Publisher('rock_vehicle', Marker, queue_size=10)
        rospy.Subscriber('rock_utm_localization', Localization, self.callback)
        self.init_x = 1131.573701
        self.init_y = 1408.584124

    def initialize_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.scale.x = 1.0  # 长度
        marker.scale.y = 0.2  # 箭头的宽度
        marker.scale.z = 0.2  # 箭头的高度
        marker.color.a = 1.0  # Alpha
        marker.color.r = 1.0  # 红色
        marker.color.g = 0.0  # 绿色
        marker.color.b = 0.0  # 蓝色
        return marker

    def callback(self, msg : Localization):
        x = msg.utm_x; y = msg.utm_y; yaw = msg.heading
        
        if self.init_x is None:
            self.init_x = x
            self.init_y = y
            return
        x = x - self.init_x
        y = y - self.init_y

        print(f"x: {x}, y: {y}, yaw: {yaw}")
        marker = self.initialize_marker()

        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0

        # 设置方向
        orientation = self.quaternion_from_euler(0, 0, yaw)  # 将欧拉角转换为四元数
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]
        
        self.pub.publish(marker)

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]


if __name__ == '__main__':
    node = Visualization()
    rospy.spin()
