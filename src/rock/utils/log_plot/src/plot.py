#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
# AUTHOR: Haoran Wu
# FILE: plot.py
# DATE: 2022/02/23 周三
# TIME: 16:33:56
'''


import rospy
import std_msgs
from geometry_msgs.msg import Pose2D
from cyber_msgs.msg import V2VPacket

import numpy as np


def local_pose_callback(msg):
    dis_msg = std_msgs.msg.Float32()
    dis_msg.data = np.linalg.norm([msg.x, msg.y]) - 3.55
    log_dis_pub.publish(dis_msg)
    print('publish dis')

def estimated_speed_callback(msg):
    pass

def v2v_speed_callback(msg):
    speed_msg = std_msgs.msg.Float32()
    speed_msg.data = 0.01 * msg.speed_cmps
    log_speed_pub.publish(speed_msg)
    print('publish speed')

log_dis_pub = rospy.Publisher('/log/dis', std_msgs.msg.Float32, queue_size=1)
log_speed_pub = rospy.Publisher('/log/speed', std_msgs.msg.Float32, queue_size=1)

if __name__ == '__main__':
    rospy.init_node('log_plot', anonymous=True)
    rospy.Subscriber('/tracking/front_vehicle/local_pose', Pose2D, local_pose_callback)
    rospy.Subscriber('/tracking/front_vehicle/speed', std_msgs.msg.Float32, estimated_speed_callback)
    rospy.Subscriber('/V2V/leader', V2VPacket, v2v_speed_callback)

    rospy.spin()
