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
        rospy.init_node("Ablation")
        self.layout = QVBoxLayout()

        self._pub_ablation_rearlight_all = rospy.Publisher(
            "/tracking/ablation/rearlight_all", Bool, queue_size=1)
        self._pub_ablation_rearlight_left = rospy.Publisher(
            "/tracking/ablation/rearlight_left", Bool, queue_size=1)
        self._pub_ablation_rearlight_right = rospy.Publisher(
            "/tracking/ablation/rearlight_right", Bool, queue_size=1)
        

        self._pub_ablation_radar = rospy.Publisher(
            "/tracking/ablation/radar", Bool, queue_size=1)

        self._pub_ablation_v2v = rospy.Publisher(
            "/tracking/ablation/v2v", Bool, queue_size=1)

        self._init_ui()

        print("Info ablation UI is ready!")

    def _init_ui(self):
        self._init_rearlight_button()
        self._init_radar_button()
        self._init_v2v_button()

        self.setLayout(self.layout)

    def _init_rearlight_button(self):
        row = QHBoxLayout()

        label = QLabel()
        label.setFixedSize(120, 50)
        label.setText("Rearlight:")
        row.addWidget(label)

        block_rearlight_button = QPushButton()
        block_rearlight_button.setText("block all")
        block_rearlight_button.setFixedSize(80, 40)
        block_rearlight_button.clicked.connect(self._ablation_rearlight_all)
        row.addWidget(block_rearlight_button)
        
        block_left_rearlight_button = QPushButton()
        block_left_rearlight_button.setText("block left")
        block_left_rearlight_button.setFixedSize(80, 40)
        block_left_rearlight_button.clicked.connect(self._ablation_rearlight_left)
        row.addWidget(block_left_rearlight_button)
        
        block_right_rearlight_button = QPushButton()
        block_right_rearlight_button.setText("block right")
        block_right_rearlight_button.setFixedSize(80, 40)
        block_right_rearlight_button.clicked.connect(self._ablation_rearlight_right)
        row.addWidget(block_right_rearlight_button)

        self.layout.addLayout(row)

    def _init_radar_button(self):
        row = QHBoxLayout()

        label = QLabel()
        label.setFixedSize(120, 50)
        label.setText("Radar:")
        row.addWidget(label)

        block_radar_button = QPushButton()
        block_radar_button.setText("block")
        block_radar_button.setFixedSize(50, 40)
        block_radar_button.clicked.connect(self._ablation_radar)
        row.addWidget(block_radar_button)

        self.layout.addLayout(row)

    def _init_v2v_button(self):
        row = QHBoxLayout()

        label = QLabel()
        label.setFixedSize(120, 50)
        label.setText("V2V:")
        row.addWidget(label)

        block_v2v_button = QPushButton()
        block_v2v_button.setText("block")
        block_v2v_button.setFixedSize(65, 40)
        block_v2v_button.clicked.connect(self._ablation_v2v)
        row.addWidget(block_v2v_button)

        self.layout.addLayout(row)

    def _ablation_rearlight_all(self):
        flag = Bool()
        flag.data = True

        self._pub_ablation_rearlight_all.publish(flag)
    
    def _ablation_rearlight_left(self):
        flag = Bool()
        flag.data = True

        self._pub_ablation_rearlight_left.publish(flag)
    
    def _ablation_rearlight_right(self):
        flag = Bool()
        flag.data = True

        self._pub_ablation_rearlight_right.publish(flag)
    
    def _ablation_radar(self):
        flag = Bool()
        flag.data = True

        self._pub_ablation_radar.publish(flag)
    
    def _ablation_v2v(self):
        flag = Bool()
        flag.data = True

        self._pub_ablation_v2v.publish(flag)


if __name__ == "__main__":

    app = QApplication(sys.argv)

    ablation_ui = UI()
    ablation_ui.resize(300, 100)
    ablation_ui.show()

    app.exec_()
