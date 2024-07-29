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

class SocketServer:
    def __init__(self, local_host, local_port):
        rospy.init_node('CenterServer', anonymous=True)
        self.max_client = 5
        self.connected_client = {}
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
        while not rospy.is_shutdown():
            try:
                # 读取图像数据
                header = client.recv(20)
                if not header:
                    continue
                timestamp, width, height, count = struct.unpack('dIIi', header)
                rospy.loginfo(f"Receive image data: {timestamp}, {width}, {height}, {count}")
                for i in range(count):  # 预期接收四个图像
                    target_length = width * height * 3
                    image_data = b''
                    while len(image_data) < target_length:
                        image_data += client.recv(target_length - len(image_data))
                    rospy.loginfo(f"Receive image length: {len(image_data)}")
                    # 将接收到的数据转为图像
                    image = np.frombuffer(image_data, dtype=np.uint8).reshape((height, width, 3))
                    # cv2.imwrite(f'/home/pi/V2X_AEB/src/common/{i}.jpg', image)
                rospy.loginfo(f"Time delay for {addr} is {time.time()-timestamp}.")
            except Exception as e:
                self.delete_client(addr)
                rospy.logwarn(e)
                return

    def delete_client(self, addr):
        if addr in self.connected_client:
            self.connected_client[addr].close()
            del self.connected_client[addr]

if __name__ == '__main__':
    with open('/home/pi/V2X_AEB/src/common/config/comm.json', 'r') as f:
        config = json.load(f)
        local_host = "" #config['CenterServer']['ip']
        local_port = config['CenterServer']['port']
    server = SocketServer(local_host, local_port)
    server.wait_for_connection()
    rospy.spin()
