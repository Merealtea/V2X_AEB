#!/usr/bin/env python
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
from cyber_msgs.msg import V2VPacket

SIZE = 1024

local_host = ''
local_port = 8901

#remote_host = '10.42.0.1'
remote_host = '192.168.10.100'
remote_port = 8900

class V2VEncoder:
    def __init__(self, remote_host, remote_port):
        rospy.init_node('V2V_encoder')
        self._leader_pub = rospy.Publisher('/V2V/leader', V2VPacket, queue_size=1)
        try:
            self.client = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            self.client.bind((local_host,local_port))
            self.client.settimeout(10)
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
        """ original recieve function for platooning
        """
        msg_struct = struct.Struct('!2i1?6f2d')
        while not rospy.is_shutdown():
            try:
                packet = self.client.recv(SIZE)
                raw_msg = msg_struct.unpack(packet)
                msg = V2VPacket()
                msg.header.stamp = rospy.Time(raw_msg[0],raw_msg[1])
                msg.transmit_delay_ms = (rospy.Time.now() - msg.header.stamp).to_sec()*1000
                msg.is_updated = raw_msg[2]
                msg.speed = raw_msg[3]
                msg.steer = raw_msg[4]
                msg.angular_velo_z = raw_msg[5]
                msg.accel_x = raw_msg[6]
                msg.accel_y = raw_msg[7]
                msg.yaw = raw_msg[8]
                msg.latitude = raw_msg[9]
                msg.longitude = raw_msg[10]
                self._leader_pub.publish(msg)
                rospy.loginfo('Msg delay: {:.2f} ms'.format(msg.transmit_delay_ms))
                # rospy.loginfo('Raw msg: {}'.format(raw_msg))

            except Exception as e:
                rospy.logwarn(e)
                pass

    def receive(self):
        """ 
            this is the test recieve function 
        """
        msg_struct = struct.Struct('!2i1?6f2d')
        while not rospy.is_shutdown():
            try:
                 # 接收 header
                header_size = struct.calcsize('iiI3i')
                header = self.client.recv(header_size)
                
                # 解析 header
                secs, nsecs, dtype_len = struct.unpack('iiI', header[:struct.calcsize('iiI')])
                dtype_str = self.client.recv(dtype_len).decode('utf-8')
                dtype = np.dtype(dtype_str)
                data_len = struct.unpack('I', header[struct.calcsize('iiI'):struct.calcsize('iiI') + 4])[0]
                
                # 解析数据的形状
                shape_format = '3i'
                shape_size = struct.calcsize(shape_format)
                shape_bytes = header[struct.calcsize('iiI') + 4:]
                shape = struct.unpack(shape_format, shape_bytes)

                # 接收数据
                data_bytes = self.client.recv(data_len)

                # 将数据转换为 NumPy 数组
                array = np.frombuffer(data_bytes, dtype=dtype).reshape(shape)

                rospy.loginfo('Received array: {}'.format(array))
                
                stamp = rospy.Time(secs,nsecs)
                transmit_delay_ms = (rospy.Time.now() - stamp).to_sec()*1000

                rospy.loginfo('Msg delay: {:.2f} ms'.format(transmit_delay_ms))
                # rospy.loginfo('Raw msg: {}'.format(raw_msg))

            except Exception as e:
                rospy.logwarn(e)
                pass

if __name__ == '__main__':
    encoder = V2VEncoder(remote_host, remote_port)
    encoder.receive()
