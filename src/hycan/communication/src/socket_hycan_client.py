#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
# AUTHOR: Haoran Wu, Hongxin Chen
# FILE: transmitter.py
# DATE: 2022/01/06 周四
# TIME: 17:37:19
'''

import rospy
from hycan_msgs.msg import FourImages
import socket, sys, struct
import json
import time
import numpy as np
import cv2
import rospkg

class SocketClient:
    def __init__(self, local_host, local_port, remote_host, remote_port):
        rospy.init_node('Hycan_client', anonymous=True)
        rospy.Subscriber('hycan_processed_images', FourImages, self._transimit_images)
        self._connected = False
        self.target_host = remote_host
        self.target_port = remote_port

        try:
            self._socket.bind((local_host, local_port))
            self._socket.setblocking(False)
            self._socket.settimeout(10)
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4194304)

        except Exception as e:
            rospy.logwarn('Socket Bind Failed! {}'.format(e))
            sys.exit()
        

    def connection(self):
        while not self._connected:
            try:
                rospy.loginfo("Connecting to {}...".format((self.target_host, self.target_port)))
                self._socket.connect((self.target_host, self.target_port))
                self._connected = True
            except Exception as e:
                rospy.logwarn("Failed to connect to the server because of: {}".format(e))
                rospy.sleep(0.5)
        rospy.loginfo("Connection from {} has been established.".format((self.target_host, self.target_port)))
        rospy.loginfo('Socket Bind Success!')
        self._connected = True

    def pack_data(self, timestamp, width, height, count, imu_data, gps_data):
        # 假设我们只关心IMU的orientation和angular_velocity以及GPS的latitude和longitude
        fmt = "diiifffffffffffddd"
        packed_data = struct.pack(fmt, timestamp, width, height, count,
                              imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w,
                              imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z,
                              gps_data.latitude, gps_data.longitude, gps_data.altitude)
        return packed_data

    def _transimit_images(self, img_msg : FourImages):
        # FourImages contrains four sensor_msgs/Image msg
        # 使用socket发送数据
        try:
            if self._connected:
                serialized_data = b''
                # 对于每个图像，序列化并发送
                width, height = img_msg.image_front.width, img_msg.image_front.height
                concat_image = np.zeros((height, width*4, 3), dtype=np.uint8)
                # 打包发送数据
                timestamp = time.time()  # 获取当前时间戳
                rospy.loginfo("image_front shape: {}x{} timestamp: {}".format(width, height, timestamp))
                for i, image in enumerate([img_msg.image_front,
                                img_msg.image_back, 
                                    img_msg.image_left, 
                                        img_msg.image_right]):
                    data = np.frombuffer(image.data, dtype=np.uint8).reshape((height, width, 3))
                    # 将图像数据拼接到一起
                    concat_image[:, width*i:width*(i+1)] = data
   
                st = time.time()
                # encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 20]        # 设置JPEG图像的质量参数为20,参数可以自定义调整
                _, encimg = cv2.imencode('.jpg', concat_image)
                compressed_data = encimg.tobytes()# zlib.compress(serialized_data)
                rospy.loginfo("Compressed time: {}".format(time.time()-st))
                rospy.loginfo("Compressed data length: {}".format(len(compressed_data)))

                imu = img_msg.imu
                gps = img_msg.gps
                # 将数据打包
                header = self.pack_data(timestamp, width, height, len(compressed_data), imu, gps)
                compressed_data = header + compressed_data
                self._socket.sendall(compressed_data)
                rospy.loginfo("Sending image data...")
            else:
                rospy.logwarn('Not connected to any server!')

        except Exception as e:
            # disconnect and reconnect
            self._socket.close()
            self._connected = False
            self.connection()
            rospy.logerr("Socket error: {}".format(e))

if __name__ ==  '__main__':
    # 读取配置文件
    rospack = rospkg.RosPack()
    ws_path = rospack.get_path('hycan_comm').split('hycan')[0]

    with open(ws_path + 'common/config/comm.json', 'r') as f:
        net_config = json.load(f)
    local_host = "" #net_config['Host']['ip']
    local_port = net_config['Hycan']['port']
    target_host = net_config['CenterServer']['ip']
    target_port = net_config['CenterServer']['port']
    trans = SocketClient(local_host, local_port, target_host, target_port)
    trans.connection()
    rospy.spin()
