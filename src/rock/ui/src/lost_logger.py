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
        rospy.init_node("LostLogger")
        self.layout = QVBoxLayout()

        self._pub_lost = rospy.Publisher(
            "/tracking/debug/lost", Bool, queue_size=1)
        self._init_ui()

        print("Telecontroller UI is ready!")

    def _init_ui(self):
        self._init_lost_button()


        self.setLayout(self.layout)


    def _init_lost_button(self):
        row = QHBoxLayout()

        label = QLabel()
        label.setFixedSize(120, 50)
        label.setText("Lost:")
        row.addWidget(label)

        set_lost_button = QPushButton()
        set_lost_button.setText("Set")
        set_lost_button.setFixedSize(50, 40)
        set_lost_button.clicked.connect(self._set_lost)
        row.addWidget(set_lost_button)

        reset_lost_button = QPushButton()
        reset_lost_button.setText("Reset")
        reset_lost_button.setFixedSize(50, 40)
        reset_lost_button.clicked.connect(self._reset_lost)
        row.addWidget(reset_lost_button)

        self.layout.addLayout(row)

    def _set_lost(self):
        flag = Bool()
        flag.data = True

        self._pub_lost.publish(flag)

    def _reset_lost(self):
        flag = Bool()
        flag.data = False

        self._pub_lost.publish(flag)
    


if __name__ == "__main__":

    app = QApplication(sys.argv)

    telecontroller_ui = UI()
    telecontroller_ui.resize(300, 100)
    telecontroller_ui.show()

    app.exec_()
