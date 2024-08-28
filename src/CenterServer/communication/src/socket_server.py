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
from hycan_msgs.msg import FourImages
from sensor_msgs.msg import Imu, NavSatFix
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
        self.fmt = "diiiiIddd"
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
        topic = f'/{vehicle}/processed_images'
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

                timestamp, width, height, original_width, original_height, count, vhx, vhy, yaw =\
                        struct.unpack(self.fmt, header) 

                rospy.loginfo(f"Receive image data from {vehicle}: {timestamp}, {width}, {height}, {count}")
                compressed_data = b''
                while len(compressed_data) < count:
                    compressed_data += client.recv(count - len(compressed_data))
                st = time.time()
                images = np.uint8(cv2.imdecode(np.frombuffer(compressed_data, dtype=np.uint8), cv2.IMREAD_COLOR))
                rospy.loginfo(f"Decompress rate: {1-len(compressed_data)/(width*height*3*4)}")
                rospy.loginfo(f"Decompress time: {time.time()-st}")
                rospy.loginfo(f"Receive image length: {len(images)}")
                rospy.loginfo(f"Time delay is {time.time()-timestamp}")

                # Package the images and imu data into a ROS message
                images = np.reshape(images, (height, -1, 3))
                images_msg = FourImages()
                images_msg.image_front.data = images[:, :width].tobytes()
                images_msg.image_back.data = images[:, width:2*width].tobytes()
                images_msg.image_left.data = images[:, 2*width:3*width].tobytes()
                images_msg.image_right.data = images[:, 3*width:].tobytes()
                images_msg.localization.utm_x = vhx
                images_msg.localization.utm_y = vhy
                images_msg.localization.heading = yaw
                secs = int(timestamp)  # 秒数部分
                nsecs = int((timestamp - secs) * 1e9)  # 将小数部分转换为纳秒
                rospy_time = rospy.Time(secs, nsecs)  # 创建 rospy.Time 对象

                # 记录客户端发送消息的时间
                images_msg.image_front.header.stamp = rospy_time
                images_msg.image_front.height = height
                images_msg.image_front.width = width
                images_msg.image_back.height = original_height
                images_msg.image_back.width = original_width
                
                # 记录接收到消息的时间
                images_msg.header.stamp = rospy.Time.now()
                self.new_bag.write(topic, images_msg)

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