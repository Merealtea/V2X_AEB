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
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
import tf
from tf.transformations import *
import socket, sys, threading, struct
import json
import time

SIZE = 1024
FREQ = 50

class SocketClient:
    def __init__(self, local_host, local_port, remote_host, remote_port):
        rospy.init_node('Hycan_client', anonymous=True)
        # rospy.Subscriber('/diankong/raw_vehicle_feedback', VehicleFeedback, self._vehicle_feedback_callback)
        # rospy.Subscriber('/wit/imu', Imu, self._imu_callback)
        # rospy.Subscriber('/wit/gps_fix', NavSatFix, self._gps_callback)
        rospy.Subscriber('hycan_processed_images', FourImages, self._transimit_images)
        # rospy.loginfo('Target host: {}, port: {}'.format(host, port))
        # self._curr_speed = 0.0
        # self._curr_steer = 0.0
        # self._curr_imu_angular_velo = Vector3()
        # self._curr_imu_accel = Vector3()
        # self._curr_gps_fix = NavSatFix()
        # self._curr_yaw = 0.0
        # self._checklist = [FREQ]*3
        # self._is_updated = False
        self._connected = False
        self.max_retry = 5

        # t_check = threading.Thread(target=self._valid_check)
        # t_check.start()
        while not self._connected and self.max_retry > 0:
            try:
                self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
                self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4194304)
                self._socket.bind((local_host, local_port))
                self._socket.setblocking(False)
                self._socket.settimeout(10)
                self._socket.connect((remote_host, remote_port))

                rospy.loginfo(f"Connection from {(remote_host, remote_port)} has been established.")
                rospy.loginfo('Socket Bind Success!')
                self._connected = True
            except Exception as e:
                self.max_retry -= 1
                rospy.logwarn('Socket Bind Failed! {}'.format(e))
        
        if self.max_retry == 0:
            rospy.logerr('Failed to connect to the server after 5 retries.')
            sys.exit()

    def _vehicle_feedback_callback(self, msg):
        self._curr_speed = msg.speed/3.6
        self._curr_steer = msg.steer
        self._checklist[0] = 0

    def _imu_callback(self, msg):
        self._curr_imu_accel = msg.linear_acceleration
        self._curr_imu_angular_velo = msg.angular_velocity
        (_,_,self._curr_yaw) = tf.transformations.euler_from_quaternion((msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w))
        self._checklist[1] = 0

    def _gps_callback(self, msg):
        self._curr_gps_fix.latitude = msg.latitude
        self._curr_gps_fix.longitude = msg.longitude
        self._checklist[2] = 0

    def _valid_check(self):
        while not rospy.is_shutdown():
            self._checklist = [x+1 for x in self._checklist]
            if max(self._checklist) < FREQ:
                self._is_updated = True
            else:
                self._is_updated = False
            rospy.sleep(1.0/FREQ)

    def _transimit_images(self, img_msg : FourImages):
        # FourImages contrains four sensor_msgs/Image msg
        # 使用socket发送数据
        try:
            if self._connected:
                serialized_data = b''
                # 对于每个图像，序列化并发送
                width, height = img_msg.image_front.width, img_msg.image_front.height
                # 打包发送数据
                timestamp = time.time()  # 获取当前时间戳
                header = struct.pack('dIIi', timestamp, width, height, 4)
                rospy.loginfo(f"Header length: {len(header)}")
                rospy.loginfo(f"image_front shape: {width}x{height} timestamp: {timestamp}")
                self._socket.sendall(header)
                for image in [img_msg.image_front,
                                img_msg.image_back, 
                                    img_msg.image_left, 
                                        img_msg.image_right]:
                    # 将图像数据转换为二进制格式
                    data = image.data
                    # 循环发送数据
                    rospy.loginfo("Sending image data with length: {}".format(len(data)))
                    self._socket.sendall(data)
                
                # rospy.loginfo("Data length: {}".format(len(serialized_data)))
                # 循环发送数据
                
                self._socket.sendall(serialized_data)
                rospy.loginfo("Sending image data...")
            else:
                rospy.logwarn('Not connected to any client!')

        except Exception as e:
            # self._socket.close()
            rospy.logerr(f"Socket error: {e}")

if __name__ ==  '__main__':
    with open('/mnt/pool1/cyberc3_platooning-main/src/common/config/comm.json', 'r') as f:
        net_config = json.load(f)
    target_host = net_config['CenterServer']['ip']
    target_port = net_config['CenterServer']['port']
    trans = SocketClient(target_host, target_port)
    # trans.connection()
    rospy.spin()
