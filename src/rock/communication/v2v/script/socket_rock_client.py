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
import socket, sys,  struct
import json
import time
import numpy as np
import rospkg

class SocketClient:
    def __init__(self, local_host, local_port, remote_host, remote_port):
        rospy.init_node('Rock_client', anonymous=True)
        rospy.Subscriber('/rock_detection_results', DetectionResults, self._transimit_results)
        self._connected = False
        self.target_host = remote_host
        self.target_port = remote_port

        self.local_host = local_host
        self.local_port = local_port

        self.idx = 0

        self.fmt = "ddiIIddd"
        self.get_fmt_length()

        self.fusion_pub = rospy.Publisher('fusion_results', DetectionResults, queue_size=10)


    def __del__(self):
        if hasattr(self, '_socket'):
            self._socket.close()

    def get_fmt_length(self):
        self.fmt_length = 0
        for ch in self.fmt:
            if ch in ['d', "D", 'I']:
                self.fmt_length += 8
            elif ch in ['i', 'f', 'F']:
                self.fmt_length += 4

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
        try:
            packed_data = struct.pack(self.fmt, image_timestamp, send_timestamp, num_bboxes, idx, count, x, y, yaw)
        except Exception as e:
            packed_data = struct.pack(self.fmt, image_timestamp, send_timestamp, num_bboxes, idx, count, 0.0, 0.0, 0.0)
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

    def _recieve_results(self):

        while not rospy.is_shutdown():
            try:
                if self._connected:
                    fusion_timestamp, send_timestamp, num_bboxes, _, count, _, _, _ = self._socket.recv(self.fmt_length)
                    fusion_data = ''
                    while len(fusion_data) < count:
                        fusion_data += self._socket.recv(count - len(fusion_data))
                    fusion_data = np.frombuffer(fusion_data, dtype=np.float32).reshape(num_bboxes, -1)

                    fusion_results = DetectionResults()
                    for i in range(num_bboxes):
                        box = Box3D()
                        box.center_x = fusion_data[i][0]
                        box.center_y = fusion_data[i][1]
                        box.center_z = fusion_data[i][2]
                        box.width = fusion_data[i][3]
                        box.length = fusion_data[i][4]
                        box.height = fusion_data[i][5]
                        box.heading = fusion_data[i][6]
                        fusion_results.box3d_array.append(box)
                    fusion_results.num_boxes = num_bboxes
                    fusion_results.sender.stamp = rospy.Time.from_sec(send_timestamp)
                    fusion_results.reciever.stamp = rospy.Time.now()
                    fusion_results.image_stamp = rospy.Time.from_sec(fusion_timestamp)

                else:
                    rospy.logwarn('Not connected to any server!')
                    rospy.sleep(1)
            except Exception as e:
                rospy.logerr("Failed to receive fusion results {}.".format(e))
                rospy.sleep(1)

if __name__ ==  '__main__':
    # 读取配置文件
    rospack = rospkg.RosPack()
    ws_path = rospack.get_path('rock_comm').split('rock')[0]

    with open(ws_path + 'common/config/comm.json', 'r') as f:
        net_config = json.load(f)
    local_host = "" #net_config['Rock']['ip']
    local_port = net_config['rock']['port']
    target_host = net_config['CenterServer']['ip']
    target_port = net_config['CenterServer']['port']
    trans = SocketClient(local_host, local_port, target_host, target_port)
    trans.connection()
    rospy.spin()
