#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
# AUTHOR: Haoran Wu
# FILE: vehicle_tracking.py
# DATE: 2023/05/10 周三
# TIME: 15:33:40
'''

# fmt: off
import sys
import os
env_path = os.path.join(os.path.dirname(__file__), 'pytracking')
if env_path not in sys.path:
    sys.path.append(env_path)

from collections import OrderedDict, deque
import numpy as np
import pandas as pd
import cv2
import time
from hmmlearn import hmm

import rospy
import std_msgs
import sensor_msgs
import geometry_msgs
from cyber_msgs.msg import Box2D, Box2DArray

from pytracking.evaluation.multi_object_wrapper import MultiObjectWrapper
from pytracking.evaluation import Tracker
# fmt: on


class OnlineTracker(Tracker):
    def __init__(self, name: str, parameter_name: str, run_id: int = None, display_name: str = None):
        super(OnlineTracker, self).__init__(
            name, parameter_name, run_id, display_name)

        self._tracker = None
        self._display_name = None
        self._init = False

    def _init_track(self, image, debug=None, visdom_info=None, optional_box=None):
        visdom_info = {} if visdom_info is None else visdom_info

        params = self.get_parameters()

        debug_ = debug
        if debug is None:
            debug_ = getattr(params, 'debug', 0)
        params.debug = debug_

        params.tracker_name = self.name
        params.param_name = self.parameter_name

        self._init_visdom(visdom_info, debug_)

        multiobj_mode = getattr(params, 'multiobj_mode', getattr(
            self.tracker_class, 'multiobj_mode', 'default'))

        if multiobj_mode == 'default':
            self._tracker = self.create_tracker(params)
        elif multiobj_mode == 'parallel':
            self._tracker = MultiObjectWrapper(
                self.tracker_class, params, self.visdom, fast_load=True)
        else:
            raise ValueError(
                'Unknown multi object mode {}'.format(multiobj_mode))

        display_name = 'Display: ' + self.name
        self._display_name = display_name
        cv2.namedWindow(display_name, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow(display_name, 960, 720)

        # initialize first frame target
        def _build_init_info(box):
            return {'init_bbox': OrderedDict({1: box}), 'init_object_ids': [1, ], 'object_ids': [1, ],
                    'sequence_object_ids': [1, ]}
        if optional_box is not None:
            assert isinstance(optional_box, (list, tuple))
            assert len(optional_box) == 4, "valid box's foramt is [x,y,w,h]"
            self._tracker.initialize(image, _build_init_info(optional_box))
        else:
            while True:
                # cv2.waitKey()
                image_disp = image.copy()

                cv2.putText(image_disp, 'Select target ROI and press ENTER', (20, 30), cv2.FONT_HERSHEY_COMPLEX_SMALL,
                            1.5, (0, 0, 0), 1)

                x, y, w, h = cv2.selectROI(
                    display_name, image_disp, fromCenter=False)
                init_state = [x, y, w, h]
                self._tracker.initialize(image, _build_init_info(init_state))
                break

    def track(self, image, info: dict = None, debug=None, visdom_info=None, optional_box=None):
        # init
        if not self._init:
            self._init_track(image, debug=debug,
                             visdom_info=visdom_info, optional_box=optional_box)
            self._init = True

        image_disp = image.copy()
        out = self._tracker.track(image, info=info)

        # Draw box
        state = [int(s) for s in out['target_bbox'][1]]
        cv2.rectangle(image_disp, (state[0], state[1]), (state[2] + state[0], state[3] + state[1]),
                      (0, 255, 0), 5)
        font_color = (0, 0, 0)
        cv2.putText(image_disp, 'Tracking!', (20, 30), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1,
                    font_color, 1)
        cv2.putText(image_disp, 'Press r to reset', (20, 55), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1,
                    font_color, 1)
        cv2.putText(image_disp, 'Press q to quit', (20, 80), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1,
                    font_color, 1)

        # Display the resulting frame
        if not self._init:
            cv2.imshow(self._display_name, image_disp)
        # cv2.imshow(self._display_name, image_disp)
        key = cv2.waitKey(1)
        if key == ord('q'):
            pass
        elif key == ord('r'):
            reset_disp = image.copy()
            cv2.putText(reset_disp, 'Select target ROI and press ENTER', (20, 30), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.5,
                        (0, 0, 0), 1)

            cv2.imshow(self._display_name, reset_disp)
            x, y, w, h = cv2.selectROI(
                self._display_name, reset_disp, fromCenter=False)
            init_state = [x, y, w, h]

            def _build_init_info(box):
                return {'init_bbox': OrderedDict({1: box}), 'init_object_ids': [1, ], 'object_ids': [1, ],
                        'sequence_object_ids': [1, ]}
            self._tracker.initialize(image, _build_init_info(init_state))

        return out


class VehicleTracking:
    def __init__(self, tracker_name: str = 'dimp', parameter_name: str = 'dimp18'):
        self._tracker = OnlineTracker(
            name=tracker_name, parameter_name=parameter_name, run_id=0, display_name='rearlight')

        self._box = None

        self._disp_name = 'Display: ' + tracker_name
        # cv2.namedWindow('Display: ' + tracker_name, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        # cv2.resizeWindow('Display: ' + tracker_name, 960, 720)

    def track(self, image: np.ndarray, info: dict, debug=None, visdom_info=None, optional_box=None):
        s = time.time()
        image_disp = image.copy()
        
        # track
        print('track')
        output = self._tracker.track(
            image, info=None, debug=debug, visdom_info=visdom_info, optional_box=optional_box)
        print('inference cost {} sec'.format(output['time'][1]))
        self._box = output['target_bbox'][1]
        
        ##################################Draw box##################################
        if self._box is not None:
            cv2.rectangle(image_disp, (int(self._box[0]), int(self._box[1])), (int(self._box[0] + self._box[2]), int(self._box[1] + self._box[3])),
                          (0, 255, 0), 5)
            cv2.imshow(self._disp_name, image_disp)
            key = cv2.waitKey(1)

        # # Display the resulting frame
        # t_s = 0
        # t_e = 999999999999999999
        # if t_s <= info['header'].stamp.to_sec() <= t_e:
        #     cv2.imshow(self._disp_name, image_disp)
        #     key = cv2.waitKey(1)
        # elif info['header'].stamp.to_sec() > t_e:
        #     try:
        #         cv2.destroyWindow(self._disp_name)
        #     except:
        #         pass

        e = time.time()
        print('total: {} sec'.format(e - s))
        return output


class VehicleTrackingWrapper:
    def __init__(self, tracker: VehicleTracking):
        rospy.init_node('vehicle_tracking', anonymous=True)
        rospy.Subscriber('/driver/fisheye/front/compressed',
                         sensor_msgs.msg.CompressedImage, self._image_callback, queue_size=1, buff_size=2**24)
        rospy.Subscriber('/tracking/visual_detection',
                         Box2DArray, self._vis_det_callback, queue_size=1)
        # rospy.Subscriber('/tracking/components_tracking/rearlight/left',
        #                  Box2D, self._left_rearlight_callback, queue_size=1)
        # rospy.Subscriber('/tracking/components_tracking/rearlight/right',
        #                  Box2D, self._right_rearlight_callback, queue_size=1)
        self._pub_vh = rospy.Publisher(
            '/tracking/components_tracking/rearlight', Box2DArray, queue_size=1)
        self._timer = rospy.Timer(
            rospy.Duration(0.05), self._tracking_callback)

        self._vh_box_thresh = rospy.get_param('/vh_box_thresh', 0.2)

        self._curr_image = None
        self._curr_image_header = None
        self._curr_vis_det = []
        self._left_box = None
        self._right_box = None
        
        self._curr_box = None
        self._curr_score = None

        self._tracker = tracker
        self._init = False

    def _image_callback(self, data):
        self._curr_image_header = data.header

        np_arr = np.fromstring(data.data, dtype=np.uint8)
        self._curr_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def _vis_det_callback(self, msg):
        self._curr_vis_det.clear()
        for box in msg.boxes:
            if box.score > self._vh_box_thresh:
                self._curr_vis_det.append(
                    (box.center_y - box.width / 2, box.center_x - box.height / 2, box.width, box.height))

    @staticmethod
    def _is_in_box(outer, inner):
        # outer / inner: (x, y, w, h)
        return inner[0] >= outer[0] and inner[0] + inner[2] <= outer[0] + outer[2] and \
            inner[1] >= outer[1] and inner[1] + inner[3] <= outer[1] + outer[3]
    
    def _tracking_callback(self, timer_event):
        rospy.loginfo('tracking callback')
        if self._curr_image is None:
            rospy.logwarn('no image for tracking')
            return
        
        s = time.time()
        track_res = Box2DArray()
        track_res.header = self._curr_image_header
        
        output = self._tracker.track(self._curr_image, info=None)
        self._vh_box = output['target_bbox'][1]
        self._curr_score = output['score'][1]
        
        left_box = Box2D()
        left_box.header = track_res.header
        left_box.score = self._curr_score
        left_box.center_x = int(self._vh_box[1] + self._vh_box[3] / 2)
        left_box.center_y = int(self._vh_box[0])
        left_box.width = int(2)
        left_box.width = int(2)
        track_res.boxes.append(left_box)
        
        right_box = Box2D()
        right_box.header = track_res.header
        right_box.score = self._curr_score
        right_box.center_x = int(self._vh_box[1] + self._vh_box[3] / 2)
        right_box.center_y = int(self._vh_box[0] + self._vh_box[2])
        right_box.width = int(2)
        right_box.width = int(2)
        track_res.boxes.append(right_box)
        
        self._pub_vh.publish(track_res)
        
        e = time.time()
        rospy.loginfo('vehicle track once cost {} sec'.format(e - s))


if __name__ == '__main__':
    tracker = VehicleTracking(tracker_name='dimplt', parameter_name='new')
    tracking_wrapper = VehicleTrackingWrapper(tracker)
    rospy.loginfo('rearlight tracking ready')
    rospy.spin()