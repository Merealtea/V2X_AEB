#!/usr/bin/python2
# -*- coding:utf-8 -*-
import serial
import rospy
import math
import platform
import serial.tools.list_ports
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from tf.transformations import quaternion_from_euler
import time
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu

# 查找 ttyUSB* 设备
def find_ttyUSB():
    print('imu 默认串口为 /dev/ttyUSB0, 若识别多个串口设备, 请在 launch 文件中修改 imu 对应的串口')
    posts = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device]
    print('当前电脑所连接的 {} 串口设备共 {} 个: {}'.format('USB', len(posts), posts))





angularVelocity = [0, 0, 0]
acceleration = [0, 0, 0]
angle_degree = [0, 0, 0]


if __name__ == "__main__":
    python_version = platform.python_version()[0]

    find_ttyUSB()
    rospy.init_node("imu")
    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baudrate = rospy.get_param("~baud", 115200)
    print("IMU Type: Modbus Port:%s baud:%d" %(port,baudrate))
    imu_msg = Imu()
    gps_msg = NavSatFix()
    try:
        wt_imu = serial.Serial(port=port, baudrate=baudrate, timeout=0.5)
        if wt_imu.isOpen():
            rospy.loginfo("\033[32mport open success...\033[0m")
        else:
            wt_imu.open()
            rospy.loginfo("\033[32mport open success...\033[0m")
    except Exception as e:
        print(e)
        rospy.loginfo("\033[31mport open failed\033[0m")
        exit(0)
    else:
        imu_pub = rospy.Publisher("/wit/imu", Imu, queue_size=10)
        gps_pub = rospy.Publisher("/wit/gps_fix", NavSatFix, queue_size=10)

        master = modbus_rtu.RtuMaster(wt_imu)
        master.set_timeout(1)
        master.set_verbose(True)
        while not rospy.is_shutdown():

            try:
                reg = master.execute(0x50,cst.READ_HOLDING_REGISTERS,0x14,44)
            except Exception as e:
                print(e)
                rospy.loginfo("\033[31mread register time out, please check connection or baundrate set!\033[0m")
                time.sleep(0.1)
            else:
                v=list(reg[32:44])
                for i in range(0,12):
                    if (v[i]>32767):
                        v[i]=v[i]-65536

                acceleration = [v[i] / 32768.0 * 16 * 9.8 for i in range(0, 3)]
                angularVelocity = [v[i] / 32768.0 * 2000 * math.pi / 180 for i in range(3, 6)]
                angle_degree = [v[i] / 32768.0 * 180 for i in range(9, 12)]

                stamp = rospy.get_rostime()

                imu_msg.header.stamp = stamp
                imu_msg.header.frame_id = "imu"


                angle_radian = [angle_degree[i] * math.pi / 180 for i in range(3)]
                qua = quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])

                imu_msg.orientation.x = qua[0]
                imu_msg.orientation.y = qua[1]
                imu_msg.orientation.z = qua[2]
                imu_msg.orientation.w = qua[3]

                imu_msg.angular_velocity.x = angularVelocity[0]
                imu_msg.angular_velocity.y = angularVelocity[1]
                imu_msg.angular_velocity.z = angularVelocity[2]

                imu_msg.linear_acceleration.x = acceleration[0]
                imu_msg.linear_acceleration.y = acceleration[1]
                imu_msg.linear_acceleration.z = acceleration[2]

                imu_pub.publish(imu_msg)

                g = reg[0:12]
                gps_msg.header.stamp = stamp
                gps_msg.header.frame_id = "gps"
                lon_dd = (int)((g[1]<<16|g[0])/10000000)
                lon_mm = ((int)((g[1]<<16|g[0])%10000000))/100000.0
                lat_dd = (int)((g[3]<<16|g[2])/10000000)
                lat_mm = ((int)((g[3]<<16|g[2])%10000000))/100000.0
                gps_msg.longitude = lon_dd + lon_mm / 60.0
                gps_msg.latitude = lat_dd + lat_mm / 60.0
                gps_msg.altitude = g[4]/10.0
                gps_msg.position_covariance[0] = g[10]/100.0
                gps_msg.position_covariance[4] = g[10]/100.0
                gps_msg.position_covariance[8] = g[11]/100.0
                gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
                gps_msg.status.status = g[8]>3
                gps_msg.status.service = g[8]

                gps_pub.publish(gps_msg)
