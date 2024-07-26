#!/usr/bin/python
# coding:utf-8

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *
import python_qt_binding.QtCore as QtCore

import sys

import rospy

from std_msgs.msg import Int32
from std_msgs.msg import Bool
from cyber_msgs.msg import speedcmd
from cyber_msgs.msg import steercmd
from cyber_msgs.msg import brakecmd

import math


class UI(QWidget):
    def __init__(self):
        QWidget.__init__(self, None, QtCore.Qt.WindowStaysOnTopHint)
        rospy.init_node("EncoderTestUi")
        self.layout = QVBoxLayout()  #

        self.steer_send_button = None
        self.brake_send_button = None
        self.speed_send_button = None

        self.steer_valid_edit = None
        self.brake_valid_edit = None
        self.speed_valid_edit = None

        self.steer_value_edit = None
        self.brake_value_edit = None
        self.speed_value_edit = None

        # ROS接口
        self._pub_speedcmd = rospy.Publisher(
            "/rock_can/speed_command", speedcmd, queue_size=1)
        self._pub_steercmd = rospy.Publisher(
            "/rock_can/steer_command", steercmd, queue_size=1)
        self._pub_brakecmd = rospy.Publisher(
            "/rock_can/brake_command", brakecmd, queue_size=1)

        self._init_ui()
        print("Encoder Test UI is ready!!")

    def _init_ui(self):
        self._init_steer_module()
        self._init_brake_module()
        self._init_speed_module()
        self.setLayout(self.layout)

    def _init_steer_module(self):
        row = QHBoxLayout()  # 水平排布

        label = QLabel()
        label.setFixedSize(70, 50)
        label.setText("Steering: ")
        row.addWidget(label)  # 将该按钮添加到该行(row)

        label = QLabel()
        label.setFixedSize(55, 50)
        label.setText("Validity:")
        row.addWidget(label)

        self.steer_valid_edit = QTextEdit()
        self.steer_valid_edit.setFixedSize(30, 30)
        row.addWidget(self.steer_valid_edit)

        label = QLabel()
        label.setFixedSize(40, 50)
        label.setText("Value:")
        row.addWidget(label)

        self.steer_value_edit = QTextEdit()
        self.steer_value_edit.setFixedSize(80, 30)
        row.addWidget(self.steer_value_edit)

        self.steer_send_button = QPushButton()
        self.steer_send_button.setText("Send")  # 按钮名称
        self.steer_send_button.setFixedSize(50, 50)
        # 按下后触发_steer_callback函数
        self.steer_send_button.clicked.connect(self._steer_callback)
        row.addWidget(self.steer_send_button)  # 将该按钮添加到该行(row)

        self.layout.addLayout(row)

    def _init_brake_module(self):
        row = QHBoxLayout()  # 水平排布

        label = QLabel()
        label.setFixedSize(70, 50)
        label.setText("Brake: ")
        row.addWidget(label)  # 将该按钮添加到该行(row)

        label = QLabel()
        label.setFixedSize(55, 50)
        label.setText("Validity:")
        row.addWidget(label)

        self.brake_valid_edit = QTextEdit()
        self.brake_valid_edit.setFixedSize(30, 30)
        row.addWidget(self.brake_valid_edit)

        label = QLabel()
        label.setFixedSize(40, 50)
        label.setText("Value:")
        row.addWidget(label)

        self.brake_value_edit = QTextEdit()
        self.brake_value_edit.setFixedSize(80, 30)
        row.addWidget(self.brake_value_edit)

        self.brake_send_button = QPushButton()
        self.brake_send_button.setText("Send")  # 按钮名称
        self.brake_send_button.setFixedSize(50, 50)
        self.brake_send_button.clicked.connect(self._brake_callback)
        row.addWidget(self.brake_send_button)  # 将该按钮添加到该行(row)

        self.layout.addLayout(row)

    def _init_speed_module(self):
        row = QHBoxLayout()  # 水平排布

        label = QLabel()
        label.setFixedSize(70, 50)
        label.setText("Speed:")
        row.addWidget(label)  # 将该按钮添加到该行(row)

        label = QLabel()
        label.setFixedSize(55, 50)
        label.setText("Validity:")
        row.addWidget(label)

        self.speed_valid_edit = QTextEdit()
        self.speed_valid_edit.setFixedSize(30, 30)
        row.addWidget(self.speed_valid_edit)

        label = QLabel()
        label.setFixedSize(40, 50)
        label.setText("Value:")
        row.addWidget(label)

        self.speed_value_edit = QTextEdit()
        self.speed_value_edit.setFixedSize(80, 30)
        row.addWidget(self.speed_value_edit)

        self.speed_send_button = QPushButton()
        self.speed_send_button.setText("Send")  # 按钮名称
        self.speed_send_button.setFixedSize(50, 50)
        self.speed_send_button.clicked.connect(self._speed_callback)
        row.addWidget(self.speed_send_button)  # 将该按钮添加到该行(row)

        self.layout.addLayout(row)

    def _steer_callback(self):
        if not self.steer_valid_edit.toPlainText() or not self.steer_value_edit.toPlainText():
            return
        valid = int(self.steer_valid_edit.toPlainText())
        value = float(self.steer_value_edit.toPlainText())
        if valid == 1:
            print("1,valid")
            print(value)
            pub_msg = steercmd()
            pub_msg.is_updated = True
            pub_msg.enable_auto_steer = True
            pub_msg.steer_cmd = value
            self._pub_steercmd.publish(pub_msg)
        else:
            print("1,invalid")
            print(value)
            pub_msg = steercmd()
            pub_msg.is_updated = True
            pub_msg.enable_auto_steer = False
            pub_msg.steer_cmd = value
            self._pub_steercmd.publish(pub_msg)

    def _brake_callback(self):
        if not self.brake_valid_edit.toPlainText() or not self.brake_value_edit.toPlainText():
            return
        valid = int(self.brake_valid_edit.toPlainText())
        value = float(self.brake_value_edit.toPlainText())
        if valid == 1:
            print("2,valid")
            print(value)
            pub_msg = brakecmd()
            pub_msg.enable_auto_brake = True
            pub_msg.deceleration = value
            self._pub_brakecmd.publish(pub_msg)
        else:
            print("2,invalid")
            print(value)
            pub_msg = brakecmd()
            pub_msg.enable_auto_brake = False
            pub_msg.deceleration = value
            self._pub_brakecmd.publish(pub_msg)

    def _speed_callback(self):
        if not self.speed_valid_edit.toPlainText() or not self.speed_value_edit.toPlainText():
            return
        valid = int(self.speed_valid_edit.toPlainText())
        value = float(self.speed_value_edit.toPlainText())
        if valid == 1:
            print("3,valid")
            print(value)
            pub_msg = speedcmd()
            pub_msg.is_updated = True
            pub_msg.enable_auto_speed = True
            pub_msg.acc_cmd = value
            pub_msg.speed_cmd = value
            self._pub_speedcmd.publish(pub_msg)
        else:
            print("3,invalid")
            print(value)
            pub_msg = speedcmd()
            pub_msg.is_updated = True
            pub_msg.enable_auto_speed = False
            pub_msg.acc_cmd = value
            pub_msg.speed_cmd = value
            self._pub_speedcmd.publish(pub_msg)


if __name__ == "__main__":

    app = QApplication(sys.argv)

    encoder_test_ui = UI()
    encoder_test_ui.resize(500, 100)
    encoder_test_ui.show()

    app.exec_()
