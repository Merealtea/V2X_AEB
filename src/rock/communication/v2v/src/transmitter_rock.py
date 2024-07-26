#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
# AUTHOR: Haoran Wu, Hongxin Chen
# FILE: transmitter.py
# DATE: 2022/01/06 周四
# TIME: 17:37:19
'''

import rospy
from hycan_msgs.msg import VehicleFeedback
from hycan_msgs.msg import FourImages
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
import tf
from tf.transformations import *
import socket, sys, threading, struct

SIZE = 1024
FREQ = 50

class RockTransmitter:
    def __init__(self, host, port):
        rospy.init_node('Rock_transmitter', anonymous=True)
        # rospy.Subscriber('/diankong/raw_vehicle_feedback', VehicleFeedback, self._vehicle_feedback_callback)
        # rospy.Subscriber('/wit/imu', Imu, self._imu_callback)
        # rospy.Subscriber('/wit/gps_fix', NavSatFix, self._gps_callback)
        rospy.Subscriber('rock_processed_images', FourImages, self._transimit_images)
        # self._curr_speed = 0.0
        # self._curr_steer = 0.0
        # self._curr_imu_angular_velo = Vector3()
        # self._curr_imu_accel = Vector3()
        # self._curr_gps_fix = NavSatFix()
        # self._curr_yaw = 0.0
        # self._checklist = [FREQ]*3
        # self._is_updated = False
        self._connected = False

        # t_check = threading.Thread(target=self._valid_check)
        # t_check.start()

        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._socket.bind((host, port))
            self._socket.settimeout(None)
            rospy.loginfo('Socket Bind Success!')
        except:
            rospy.logwarn('Socket Bind Failed!')
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
                for image in [img_msg.image_front,
                                img_msg.image_back, 
                                    img_msg.image_left, 
                                        img_msg.image_right]:
                    # 将图像数据转换为二进制格式
                    data_length = struct.pack('>I', len(image.data))
                    serialized_data += data_length + image.data
                    # 发送数据长度
                self._socket.sendall(serialized_data)
            else:
                rospy.logwarn('Not connected to any client!')

        except Exception as e:
            print(f"Socket error: {e}")



    def transmit(self):
        addr_list=[]
        while not rospy.is_shutdown():
            try:
                conn, addr = self._socket.recvfrom(SIZE)
                if(conn.decode('UTF-8') == "V2V request\n"\
                    and addr not in addr_list):
                    rospy.loginfo('V2V request from {}'.format(addr))
                    self._connected = True
                    t = threading.Thread(target=self._trans, args=(addr))
                    t.start()
                    addr_list.append(addr)
            except Exception as e:
                rospy.logwarn(e)

    def _trans(self,host,port):
        rospy.loginfo('Connected to {}'.format((host,port)))
        msg_struct = struct.Struct('!2i1?6f2d')
        while not rospy.is_shutdown():
            try:
                msg = msg_struct.pack(rospy.Time.now().secs,\
                    rospy.Time.now().nsecs,\
                    self._is_updated,\
                    self._curr_speed,\
                    self._curr_steer,\
                    self._curr_imu_angular_velo.z,\
                    self._curr_imu_accel.x,\
                    self._curr_imu_accel.y,\
                    self._curr_yaw,\
                    self._curr_gps_fix.latitude,\
                    self._curr_gps_fix.longitude)
                self._socket.sendto(msg,(host,port))
            except Exception as e:
                rospy.logwarn(e)
            rospy.sleep(1. / FREQ)

if __name__ ==  '__main__':
    trans = RockTransmitter('',8900)
    trans.transmit()
