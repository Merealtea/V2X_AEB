#!/usr/bin/python
# -*- encoding: utf-8 -*-

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *
import python_qt_binding.QtCore as QtCore

import sys

import rospy

from std_msgs.msg import Bool


class UI(QWidget):
    def __init__(self):
        QWidget.__init__(self, None, QtCore.Qt.WindowStaysOnTopHint)
        rospy.init_node("Telecontroller")
        self.layout = QVBoxLayout()

        self._pub_auto_drive = rospy.Publisher(
            "/rock_can/auto_drive", Bool, queue_size=1)

        self._pub_release_manual_intervention = rospy.Publisher(
            "/rock_can/release_manual_intervention", Bool, queue_size=1)
        
        self._pub_rematch = rospy.Publisher("/xboxone/rematch", Bool,queue_size=1)

        self._init_ui()

        print("Telecontroller UI is ready!")

    def _init_ui(self):
        self._init_release_button()
        self._init_auto_drive_button()
        self._init_rematch_button()

        self.setLayout(self.layout)

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
    
    def _init_rematch_button(self):
        row = QHBoxLayout()

        label = QLabel()
        label.setFixedSize(120, 50)
        label.setText("Rematch:")
        row.addWidget(label)

        set_rematch_button = QPushButton()
        set_rematch_button.setText("Rematch")
        set_rematch_button.setFixedSize(65, 40)
        set_rematch_button.clicked.connect(self._rematch)
        row.addWidget(set_rematch_button)

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

        self._pub_auto_drive.publish(flag)
    
    def _rematch(self):
        flag = Bool()
        flag.data = True

        self._pub_rematch.publish(flag)


if __name__ == "__main__":

    app = QApplication(sys.argv)

    telecontroller_ui = UI()
    telecontroller_ui.resize(300, 100)
    telecontroller_ui.show()

    app.exec_()
