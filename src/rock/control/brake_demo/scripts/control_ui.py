#!/usr/bin/python
# coding:utf-8

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *
import python_qt_binding.QtCore as QtCore

import sys
import time

import rospy

from std_msgs.msg import Int32
from std_msgs.msg import Bool
from cyber_msgs.msg import canframe
from cyber_msgs.msg import ObjectArray, Object, LocalizationEstimate, BodyworkControl

from tf.transformations import euler_from_quaternion
import math


class UI(QWidget):
    def __init__(self):
        QWidget.__init__(self, None, QtCore.Qt.WindowStaysOnTopHint)
        rospy.init_node("PlanningUI")
        self.layout = QVBoxLayout()  # 从上到下竖直布局

        # Analog Telecontrol
        self.pub_body_control = rospy.Publisher(
            "/rock_can/bodywork_control", BodyworkControl, queue_size=1)

        self._pub_auto_drive = rospy.Publisher(
            "/rock_can/auto_drive", Bool, queue_size=1)

        self._pub_release_manual_intervention = rospy.Publisher(
            "/rock_can/release_manual_intervention", Bool, queue_size=1)

        self._init_ui()
        print("Task UI is ready!!")

    def _init_ui(self):
        # Analog Telecontrol
        self._init_release_button()
        self._init_auto_drive_button()

        self.setLayout(self.layout)

    # Analog Telecontrol
    def _init_release_button(self):
        row = QHBoxLayout()

        label = QLabel()
        label.setFixedSize(120, 50)
        label.setText("Release Manual:")
        row.addWidget(label)

        set_release_button = QPushButton()
        set_release_button.setText("Set")
        set_release_button.setFixedSize(50, 40)
        set_release_button.clicked.connect(self._release_manual)
        row.addWidget(set_release_button)

        self.layout.addLayout(row)

    def _init_auto_drive_button(self):
        row = QHBoxLayout()

        label = QLabel()
        label.setFixedSize(120, 50)
        label.setText("Auto Drive:")
        row.addWidget(label)

        set_auto_drive_button = QPushButton()
        set_auto_drive_button.setText("Set")
        set_auto_drive_button.setFixedSize(50, 40)
        set_auto_drive_button.clicked.connect(self._set_auto_drive)
        row.addWidget(set_auto_drive_button)

        reset_auto_drive_button = QPushButton()
        reset_auto_drive_button.setText("Reset")
        reset_auto_drive_button.setFixedSize(50, 40)
        reset_auto_drive_button.clicked.connect(self._reset_auto_drive)
        row.addWidget(reset_auto_drive_button)

        self.layout.addLayout(row)

    def _release_manual(self):
        flag = Bool()
        flag.data = True

        self._pub_release_manual_intervention.publish(flag)

    def _set_auto_drive(self):
        flag = Bool()
        flag.data = True

        self._pub_auto_drive.publish(flag)

    def _reset_auto_drive(self):
        flag = Bool()
        flag.data = False
        body_control = BodyworkControl()
        body_control.LCM_TurnLight = 0
        self.pub_body_control.publish(body_control)
        self._pub_auto_drive.publish(flag)

if __name__ == "__main__":
    app = QApplication(sys.argv)

    planning_ui = UI()
    planning_ui.resize(300, 100)
    planning_ui.show()

    app.exec_()
