#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
'''
# AUTHOR: Haoran Wu, Hongxin Chen
# FILE: decoder.py
# DATE: 2022/01/04 周二
# TIME: 20:36:14
'''

# UDP

import rospy
import numpy as np
import threading
import time
from hycan_msgs.msg import DetectionResults, Box3D


class DetectionFusion:
    def __init__(self ):
        rospy.init_node('CenterFusion', anonymous=True)
        self.single_res_sub = rospy.Subscriber('detection_results', DetectionResults, self.fusion)
        self.fusion_results = []
        self.fusion_result_pub = rospy.Publisher('fusion_results', DetectionResults, queue_size=10)

        thread = threading.Thread(target=self.send_fusion)
        thread.start()
        thread.join()

    def fusion(self, msg):
        single_results = []
        for i in range(msg.num_boxes):
            single_results.append(msg.box3d_array[i])

        detection_timestamp = msg.image_stamp.to_sec()
        # TODO : Add kalman filter to fusion the detection results

        self.fusion_results += single_results

    def send_fusion(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            # TODO : Get prediction results from kalman filter
            cur_time = time.time()
            fusion_results = DetectionResults()

            for i in range(len(self.fusion_results)):
                fusion_results.box3d_array.append(self.fusion_results[i])
            fusion_results.num_boxes = len(self.fusion_results)
            fusion_results.sender.stamp = rospy.Time.from_sec(cur_time)

            self.fusion_result_pub.publish(fusion_results)
            self.fusion_results = []

            rospy.loginfo("Send fusion results")

            rate.sleep()


if __name__ == '__main__':
    # 创建rospkg对象
    fusion_node = DetectionFusion()
    rospy.spin()