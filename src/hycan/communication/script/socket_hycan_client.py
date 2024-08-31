#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
# AUTHOR: Haoran Wu, Hongxin Chen
# FILE: transmitter.py
# DATE: 2022/01/06 周四
# TIME: 17:37:19
'''

import rospy
from hycan_msgs.msg import DetectionResults
import socket, sys, struct
import json
import time
import numpy as np
import cv2
import rospkg

class SocketClient:
    def __init__(self, local_host, local_port, remote_host, remote_port):
        rospy.init_node('Hycan_client', anonymous=True)
        rospy.Subscriber('hycan_detection_results', DetectionResults, self._transimit_results)
        self._connected = False
        self.target_host = remote_host
        self.target_port = remote_port

        self.local_host = local_host
        self.local_port = local_port
        self.idx = 0


    def __del__(self):
        if hasattr(self, '_socket'):
            self._socket.close()

    def connection(self):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            # self._socket.bind((self.local_host, self.local_port))
            self._socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4194304)

            self._socket.setblocking(False)
            self._socket.settimeout(10)
        except Exception as e:
            rospy.logerr('Failed to create socket. Error: {}'.format(e))
            sys.exit()

        while not self._connected:
            try:
                rospy.loginfo("Connecting to {}...".format((self.target_host, self.target_port)))
                self._socket.connect((self.target_host, self.target_port))
                self._connected = True
            except Exception as e:
                rospy.logwarn("Failed to connect to the server because : {}".format(e))
                rospy.sleep(1)
        rospy.loginfo("Connection from {} has been established.".format((self.target_host, self.target_port)))
        rospy.loginfo('Socket Bind Success!')
        self._connected = True

    def pack_data(self, image_timestamp, send_timestamp, num_bboxes, idx, count, x, y, yaw):
        # 假设我们只关心IMU的orientation和angular_velocity以及GPS的latitude和longitude
        fmt = "ddiIIddd"
        try:
            packed_data = struct.pack(fmt, image_timestamp, send_timestamp, num_bboxes, idx, count, x, y, yaw)
        except Exception as e:
            packed_data = struct.pack(fmt, image_timestamp, send_timestamp, num_bboxes, idx, count, 0.0, 0.0, 0.0)
        return packed_data

    def _transimit_results(self, msg):
        # FourImages contrains four sensor_msgs/Image msg
        # 使用socket发送数据
        try:
            if self._connected:
                boxes_array = np.ascontiguousarray(msg.box3d_array).tobytes()
                count = len(boxes_array)
                image_stamp = msg.image_stamp.to_sec() 
                cur_stamp = time.time()
                # 将数据打包
                header = self.pack_data(image_stamp, cur_stamp, msg.num_boxes, self.idx, count, msg.localization.utm_x, msg.localization.utm_y, msg.localization.heading)
                data = header + boxes_array
                self._socket.sendall(data)
                rospy.loginfo("Sending image data...")
            else:
                rospy.logwarn('Not connected to any server!')

        except Exception as e:
            # disconnect and reconnect
            rospy.logerr("Socket error: {}".format(e))
            self._socket.close()
            self._connected = False
            self.connection()
        self.idx += 1


if __name__ ==  '__main__':
    # 读取配置文件
    rospack = rospkg.RosPack()
    ws_path = rospack.get_path('hycan_comm').split('hycan')[0]

    with open(ws_path + 'common/config/comm.json', 'r') as f:
        net_config = json.load(f)
    local_host = "" #net_config['Host']['ip']
    local_port = net_config['hycan']['port']
    target_host = net_config['CenterServer']['ip']
    target_port = net_config['CenterServer']['port']
    trans = SocketClient(local_host, local_port, target_host, target_port)
    trans.connection()
    rospy.spin()
