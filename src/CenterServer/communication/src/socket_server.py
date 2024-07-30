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
# from cyber_msgs.msg import V2VPacket
from PIL import Image
import io
import json
import threading
import time
import cv2
from hycan_msgs.msg import FourImages
import rosbag
import cv_bridge

class SocketServer:
    def __init__(self, local_host, local_port, addr_to_vehicle):
        rospy.init_node('CenterServer', anonymous=True)
        self.max_client = 5
        self.connected_client = {}
        self.addr_to_vehicle = addr_to_vehicle
        self.new_bag = rosbag.Bag(f"./multi_vehicle.bag", 'w')
        # self._leader_pub = rospy.Publisher('/V2V/leader', V2VPacket, queue_size=1)
        try:
            # we will first regard it as a receiver
            self.server = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
            self.server.bind((local_host,local_port))
            self.server.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4194304)
            self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server.settimeout(10)
            self.server.listen(self.max_client)
            rospy.loginfo("Socket bind for center server success!")
        except:
            rospy.logerr('Failed to create socket.')
            sys.exit()

    def wait_for_connection(self):
        while not rospy.is_shutdown() and len(self.connected_client) < self.max_client:
            try:
                client, addr = self.server.accept()
                if addr in self.connected_client:
                    continue
                rospy.loginfo(f"Connected to {addr}")
                self.connected_client[addr] = client
                thread = threading.Thread(target=self.receive_from, args=(client, addr)).start()
                self.alive_thread[addr] = thread
                thread.start()
                # thread.detach()

            except Exception as e:
                rospy.logwarn(e)
                continue

    def __del__(self):
        if hasattr(self, 'server'):
            self.server.close()

    def receive_from(self, client, addr):
        """ 
            this is the test recieve function 
        """
        rospy.loginfo("Start to receive image data.")
        vehicle = self.addr_to_vehicle[addr]
        topic = f'/{vehicle}/processed_images'
        max_retry = 5
        # pub = rospy.Publisher(f'/{vehicle}/processed_images', FourImages, queue_size=1)
        while not rospy.is_shutdown():
            try:
                # 读取图像数据
                header = client.recv(20)
                if not header and max_retry > 0:
                    continue
                if max_retry == 0:
                    rospy.logwarn(f"Connection from {addr} has been lost.")
                    self.delete_client(addr)
                    return

                timestamp, width, height, count = struct.unpack('dIIi', header)
                rospy.loginfo(f"Receive image data from {vehicle}: {timestamp}, {width}, {height}, {count}")
                compressed_data = b''
                while len(compressed_data) < count:
                    compressed_data += client.recv(count - len(compressed_data))
                st = time.time()
                images = np.uint8(cv2.imdecode(np.frombuffer(compressed_data, dtype=np.uint8), cv2.IMREAD_COLOR))
                # images = zlib.decompress(compressed_data)
                rospy.loginfo(f"Decompress rate: {1-len(compressed_data)/(width*height*3*4)}")
                rospy.loginfo(f"Decompress time: {time.time()-st}")
                rospy.loginfo(f"Receive image length: {len(images)}")
                images = np.reshape(images, (-1,height, width, 3))
                hycan_msg = FourImages()
                hycan_msg.image_front = cv_bridge.cv2_to_imgmsg(images[0], encoding='bgr8')
                hycan_msg.image_back = cv_bridge.cv2_to_imgmsg(images[1], encoding='bgr8')
                hycan_msg.image_left = cv_bridge.cv2_to_imgmsg(images[2], encoding='bgr8')
                hycan_msg.image_right = cv_bridge.cv2_to_imgmsg(images[3], encoding='bgr8')
                secs = int(timestamp)  # 秒数部分
                nsecs = int((timestamp - secs) * 1e9)  # 将小数部分转换为纳秒
                rospy_time = rospy.Time(secs, nsecs)  # 创建 rospy.Time 对象
                hycan_msg.header.stamp = rospy_time
                self.new_bag.write(topic, hycan_msg)

            except Exception as e:
                self.delete_client(addr)
                rospy.logwarn(e)
                return

    def delete_client(self, addr):
        if addr in self.connected_client:
            self.connected_client[addr].close()
            del self.connected_client[addr]
        if len(self.connected_client) == 0:
            rospy.logwarn("No client connected.")
            self.new_bag.close()

if __name__ == '__main__':
    with open('/home/pi/V2X_AEB/src/common/config/comm.json', 'r') as f:
        config = json.load(f)
        local_host = "" #config['CenterServer']['ip']
        local_port = config['CenterServer']['port']
        addr_to_vehicle = {}
        for key in config:
            addr = config[key]['ip']
            addr_to_vehicle[addr] = key
    
    server = SocketServer(local_host, local_port, addr_to_vehicle)
    server.wait_for_connection()
    rospy.spin()
    server.delete_client()
