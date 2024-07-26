#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cyber_msgs.msg import Object
import numpy as np
import cv2


class Radar2Image:
    def __init__(self) -> None:
        rospy.init_node('radar2image', anonymous=True)
        rospy.Subscriber('/driver/camera/image', Image, self.image_callback, queue_size=10)
        rospy.Subscriber('/fusion_tracking/object', Object, self.object_callback, queue_size=10)

        self.match = Object()

        self.radar2cam_z = 2.4 # front
        self.radar2cam_x = 0.2  # right
        self.radar2cam_y = 1.90 # down
        
        self.rotation_mat = np.zeros((3, 1))
        self.translation_mat = np.zeros((3, 1))
        self.camera_mat = np.array([[1201.8, -0.935, 953.9414], \
            [0.0, 1211.8, 636.9038], 
            [0.0, 0.0, 1.0]])
        self.dist_mat = np.array([-0.1430, 0.0383, 0.0027, -0.0012, 0.0]).reshape(-1, 1)

    def image_callback(self, msg):
        rospy.loginfo('reveive image')
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(1200, 1920, -1)
        
        x_radar = self.match.pose.position.x
        y_radar = self.match.pose.position.y
        z_radar = 0.5

        x_cam = -y_radar + self.radar2cam_x
        y_cam = -z_radar + self.radar2cam_y
        z_cam = x_radar + self.radar2cam_z

        _3d_pt = np.array([x_cam, y_cam, z_cam]).reshape(-1, 3)
        
        img_pt = cv2.projectPoints(_3d_pt, self.rotation_mat,\
            self.translation_mat, self.camera_mat, self.dist_mat)[0]
        # img_pt: shape (1, 1, 2), (u, v)
        center = tuple([int(img_pt[0, 0, 0]), int(img_pt[0, 0, 1])])
        cv2.circle(image, center, 10, (0, 0, 255), 4)
        resized = cv2.resize(image, (int(1920 / 2), int(1200 / 2)), interpolation=cv2.INTER_CUBIC)
        cv2.imshow('image', resized)
        cv2.waitKey(10)



    def object_callback(self, msg):
        rospy.loginfo('reveive obj')
        self.match = msg


if __name__ == '__main__':
    trans = Radar2Image()
    rospy.spin()