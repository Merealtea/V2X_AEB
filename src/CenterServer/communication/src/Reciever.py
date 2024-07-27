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

SIZE = 1024

local_host = ''
local_port = 8901

class Reciever:
    def __init__(self, remote_host, remote_port):
        rospy.init_node('CenterReceiver')
        # self._leader_pub = rospy.Publisher('/V2V/leader', V2VPacket, queue_size=1)
        try:
            self.client = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            self.client.bind((local_host,local_port))
            self.client.settimeout(10)
            rospy.loginfo("Socket bind for center server success!")
        except:
            rospy.logerr('Failed to create socket.')
            sys.exit()
        is_connect = False
        while(not is_connect):
            try:
                self.client.sendto("V2V request\n".encode(),(remote_host,remote_port))
                _, server = self.client.recvfrom(SIZE)
                rospy.loginfo('----------------------connected to {}----------------------'.format(server))
                is_connect = True
            except socket.timeout:
                rospy.logwarn('V2V request timed out, retrying')
                continue

    def __del__(self):
        if hasattr(self, 'client'):
            self.client.close()

    def receive(self):
        """ 
            this is the test recieve function 
        """
        while not rospy.is_shutdown():
            try:
                 # 读取数据长度
                length = struct.unpack('>I', self.client.recv(4))[0]
                # 读取图像数据
                image_data = b''

                for _ in range(4):  # 预期接收四个图像
                    length = struct.unpack('>I', self.client.recv(4))[0]
                    image_data = self.client.recv(length)  # 假设图像数据小于socket缓冲区大小

                    # 将接收到的数据转为图像
                    image = Image.open(io.BytesIO(image_data))

            except Exception as e:
                rospy.logwarn(e)
                pass

if __name__ == '__main__':
    with open('/mnt/pool1/cyberc3_platooning-main/src/common/config/comm.json', 'r') as f:
        config = json.load(f)
        remote_host = config['Hycan']['ip']
        remote_port = config['Hycan']['port']
    encoder = Reciever(remote_host, remote_port)
    encoder.receive()
