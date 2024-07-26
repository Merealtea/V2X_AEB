#!/usr/bin/python
# coding:utf-8

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *
import python_qt_binding.QtCore as QtCore

import sys

import rospy

from std_msgs.msg import Float64
from cyber_msgs.msg import SpeedFeedback
from cyber_msgs.msg import SteerFeedback
from cyber_msgs.msg import BrakeFeedback
from cyber_msgs.msg import speedcmd
from cyber_msgs.msg import PlatoonControlTarget


class UI(QWidget):
    def __init__(self):
        QWidget.__init__(self, None, QtCore.Qt.WindowStaysOnTopHint)
        rospy.init_node("ControllerTestUi")
        self.layout = QVBoxLayout()  #

        self.target_speed_send_button = None

        self.target_speed_value_edit = None

        self.state_button = None

        self.state_flag = False

        # ROS接口
        self._pub_targetspeed = rospy.Publisher(
            "/control/control_target", PlatoonControlTarget, queue_size=1)
        self._sub_speedfeedback = rospy.Subscriber(
            "/rock_can/speed_feedback", SpeedFeedback, self.speedback_callback, queue_size=5)
        self._sub_steerfeedback = rospy.Subscriber(
            "/rock_can/steer_feedback", SteerFeedback, self.steerback_callback, queue_size=5)
        self._sub_brakefeedback = rospy.Subscriber(
            "/rock_can/brake_feedback", BrakeFeedback, self.brakeback_callback, queue_size=5)
        self._sub_speedcommand = rospy.Subscriber(
            "/rock_can/speed_command", speedcmd, self.speedcmd_callback, queue_size=5)
        self._init_ui()
        print("Contoller Test UI is ready!!")

    def speedback_callback(self, speed):
        self.speed_label.setText(str(speed.speed_cms/100.0))

    def steerback_callback(self, steer):
        self.angular_label.setText(str(steer.SteerAngle))
        self.angular_speed_label.setText(str(steer.SteerAngulaSpeed))

    def brakeback_callback(self, brake):
        self.brake_label.setText(str(brake.BrakePressure))

    def speedcmd_callback(self, speed_cmd):
        self.label1.setText(str(speed_cmd.speed_cmd))

    def _init_ui(self):
        self._init_speed_module()
        self._init_supervisor_module()
        self._current_speedcmd_labels()
        self.setLayout(self.layout)

    def _init_speed_module(self):
        row = QHBoxLayout()  # 水平排布

        label = QLabel()
        label.setFixedSize(200, 50)
        label.setText("Target_Speed(m/s):")
        row.addWidget(label)  # 将该按钮添加到该行(row)

        self.target_speed_value_edit = QTextEdit()
        self.target_speed_value_edit.setFixedSize(100, 50)
        row.addWidget(self.target_speed_value_edit)

        self.target_speed_send_button = QPushButton()
        self.target_speed_send_button.setText("Send")  # 按钮名称
        self.target_speed_send_button.setFixedSize(50, 30)
        self.target_speed_send_button.clicked.connect(
            self._target_speed_callback)
        row.addWidget(self.target_speed_send_button)  # 将该按钮添加到该行(row)

        # self.state_button=QPushButton()
        # self.state_button.setText("Switch")  # 按钮名称
        # self.state_button.setFixedSize(50, 30)
        # self.state_button.clicked.connect(
        #     self._state_button_callback)
        # row.addWidget(self.state_button)  # 将该按钮添加到该行(row)

        self.layout.addLayout(row)

    def _init_supervisor_module(self):
        row = QHBoxLayout()

        label = QLabel()
        label.setFixedSize(200, 50)
        label.setText("Steer_angle(degree):")
        row.addWidget(label)
        self.angular_label = QLabel()
        self.angular_label.setFixedSize(100, 50)
        row.addWidget(self.angular_label)

        self.layout.addLayout(row)

        row = QHBoxLayout()

        label = QLabel()
        label.setFixedSize(200, 50)
        label.setText("Current_Speed(m/s):")
        row.addWidget(label)
        self.speed_label = QLabel()
        self.speed_label.setFixedSize(100, 50)
        row.addWidget(self.speed_label)

        self.layout.addLayout(row)

        row = QHBoxLayout()

        label = QLabel()
        label.setFixedSize(200, 50)
        label.setText("Angular_speed(degree/s):")
        row.addWidget(label)
        self.angular_speed_label = QLabel()
        self.angular_speed_label.setFixedSize(100, 50)
        row.addWidget(self.angular_speed_label)

        self.layout.addLayout(row)

        row = QHBoxLayout()

        label = QLabel()
        label.setFixedSize(200, 50)
        label.setText("Brake_Pressure(Bar):")
        row.addWidget(label)
        self.brake_label = QLabel()
        self.brake_label.setFixedSize(100, 50)
        row.addWidget(self.brake_label)

        self.layout.addLayout(row)

    def _current_speedcmd_labels(self):  # 初始化一个current_speed的标签
        row = QHBoxLayout()  # 水平排布

        label = QLabel(self)
        label.setFixedSize(200, 50)
        label.setText("Current Speedcmd(Nm):")
        row.addWidget(label)  # 将该lable添加到该行(row)
        self.label1 = QLabel(self)
        self.label1.setFixedSize(100, 50)
        row.addWidget(self.label1)

        self.layout.addLayout(row)

    # def _state_button_callback(self):
    #     if self.state_flag is True:
    #         self.state_flag=False
    #         print("Switch is false!")
    #     else:
    #         self.state_flag=True
    #         print("Switch is true!")

    def _target_speed_callback(self):
        if not self.target_speed_value_edit.toPlainText():
            return
        value = float(self.target_speed_value_edit.toPlainText())

        pub_msg = PlatoonControlTarget()
        pub_msg.header.stamp = rospy.Time.now()
        pub_msg.speed_ref = value
        # if self.state_flag is True:
        print('Target_speed = ', value)
        self._pub_targetspeed.publish(pub_msg)


if __name__ == "__main__":

    app = QApplication(sys.argv)

    controller_test_ui = UI()
    controller_test_ui.resize(500, 100)
    controller_test_ui.show()

    app.exec_()
