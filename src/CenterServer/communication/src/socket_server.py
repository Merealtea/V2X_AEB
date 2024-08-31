#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
'''
# AUTHOR: Haoran Wu, Hongxin Chen
# FILE: decoder.py
# DATE: 2022/01/04 周二
# TIME: 20:36:14
'''

# UDP

import socket, sys, struct
import rospy
import numpy as np
import json
import threading
import time
import cv2
from hycan_msgs.msg import DetectionResults, Box3D
import rosbag
import rospkg
from datetime import datetime
import os

class SocketServer:
    def __init__(self, local_host, local_port, addr_to_vehicle, bag_path):
        rospy.init_node('CenterServer', anonymous=True)
        self.max_client = 5
        self.connected_client = {}
        self.addr_to_vehicle = addr_to_vehicle
        self.fmt = "ddiIIddd"
        self.get_fmt_length()

        try:
            # we will first regard it as a receiver
            self.server = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
            self.server.bind((local_host,local_port))
            self.server.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4194304)
            self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server.settimeout(10)
            self.server.listen(self.max_client)
            rospy.loginfo("Socket bind for center server success!")
        except Exception as e:
            rospy.logerr('Failed to create socket. Error: {}'.format(e))
            sys.exit()

         # 获取当前时间并格式化到秒
        formatted_datetime = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        file_name = "{}.bag".format(formatted_datetime)
        self.new_bag = rosbag.Bag(os.path.join(bag_path, file_name.format(datetime.now())), 'w')

    def get_fmt_length(self):
        self.fmt_length = 0
        for ch in self.fmt:
            if ch in ['d', "D", 'I']:
                self.fmt_length += 8
            elif ch in ['i', 'f', 'F']:
                self.fmt_length += 4

    def wait_for_connection(self):
        while not rospy.is_shutdown() and len(self.connected_client) < self.max_client:
            try:
                client, addr = self.server.accept()
                if addr in self.connected_client:
                    continue
                rospy.loginfo(f"Connected to {addr}")
                self.connected_client[addr] = client
                thread = threading.Thread(target=self.receive_from, args=(client, addr))
                thread.start()

            except Exception as e:
                rospy.logwarn(e)
                continue

    def __del__(self):
        if hasattr(self, 'new_bag'):
            self.new_bag.close()

        for addr in self.connected_client:
            self.connected_client[addr].close()

        if hasattr(self, 'server'):
            self.server.close()

    def receive_from(self, client, addr):
        """ 
            this is the test recieve function 
        """
        rospy.loginfo("Start to receive image data.")
        vehicle = self.addr_to_vehicle[addr[0]]
        topic = f'/{vehicle}/detection_results'
        max_retry = 5
        # pub = rospy.Publisher(f'/{vehicle}/processed_images', FourImages, queue_size=1)
        while not rospy.is_shutdown():
            try:
                # 读取图像数据
                header = client.recv(self.fmt_length)
                if not header and max_retry > 0:
                    max_retry -= 1
                    continue
                if max_retry == 0:
                    rospy.logwarn(f"Connection from {addr} has been lost.")
                    self.delete_client(addr)
                    return

                image_timestamp, send_timestamp, num_bboxes, idx, count, x, y, yaw =\
                        struct.unpack(self.fmt, header) 

                rospy.loginfo(f"Receive image data from {vehicle}: {send_timestamp},  {count}")
                data = b''
                while len(data) < count:
                    data += client.recv(count - len(data))
                st = time.time()
            
                rospy.loginfo(f"Receive data length: {len(data)}")
                rospy.loginfo(f"Time delay is {time.time()-send_timestamp}")

                # Package the images and imu data into a ROS message
                detection_results = DetectionResults()
                data = np.ascotiguousarray(data).reshape(num_bboxes, -1)
                for i in range(num_bboxes):
                    box = Box3D()
                    box.center_x = data[i][0]
                    box.center_y = data[i][1]
                    box.center_z = data[i][2]
                    box.width = data[i][3]
                    box.length = data[i][4]
                    box.height = data[i][5]
                    box.heading = data[i][6]
                    detection_results.box3d_array.append(box)
                detection_results.num_boxes = num_bboxes
                detection_results.localization.utm_x = x   
                detection_results.localization.utm_y = y
                detection_results.localization.heading = yaw
                
                # 记录接收到消息的时间
                detection_results.sender.stamp = rospy.Time.from_sec(send_timestamp)
                detection_results.sender.idx = idx
                detection_results.reciever.stamp = rospy.Time.now()
                detection_results.image_stamp = rospy.Time.from_sec(image_timestamp)
                self.new_bag.write(topic, detection_results)

            except Exception as e:
                self.delete_client(addr)
                rospy.logwarn(e)
                return

    def delete_client(self, addr):
        if addr in self.connected_client:
            self.connected_client[addr].close()
            del self.connected_client[addr]

if __name__ == '__main__':
    # 创建rospkg对象
    rospack = rospkg.RosPack()

    # 获取当前包的路径
    package_path = rospack.get_path('center_comm')

    # 获取包的上一级目录
    ws_path = package_path.split('CenterServer')[0]

    bag_path = ws_path.split('src')[0] + '/data'
    if not os.path.exists(bag_path):
        os.makedirs(bag_path)

    with open(ws_path + 'common/config/comm.json', 'r') as f:
        config = json.load(f)
        local_host = "" #config['CenterServer']['ip']
        local_port = config['CenterServer']['port']
        addr_to_vehicle = {}
        for key in config:
            addr = config[key]['ip']
            addr_to_vehicle[addr] = key

    server = SocketServer(local_host, local_port, addr_to_vehicle, bag_path)
    server.wait_for_connection()
    rospy.spin()