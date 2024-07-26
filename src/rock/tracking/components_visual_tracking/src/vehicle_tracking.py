#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
# AUTHOR: Haoran Wu
# FILE: vehicle_tracking.py
# DATE: 2023/06/06 周二
# TIME: 17:45:22
'''

# fmt: off
import sys
import os
env_path = os.path.join(os.path.dirname(__file__), 'pytracking')
if env_path not in sys.path:
    sys.path.append(env_path)

from hmmlearn import hmm
from collections import OrderedDict, deque
from pytracking.evaluation.multi_object_wrapper import MultiObjectWrapper
from pytracking.evaluation import Tracker
import numpy as np
import pandas as pd
import cv2
import time
import rospy
import std_msgs
import sensor_msgs
import geometry_msgs
from cyber_msgs.msg import Box2D, Box2DArray
# fmt: on


class OnlineTracker(Tracker):
    def __init__(self, name: str, parameter_name: str, run_id: int = None, display_name: str = None):
        super(OnlineTracker, self).__init__(
            name, parameter_name, run_id, display_name)

        self._tracker = None
        self._display_name = display_name
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

        # display_name = 'Display: ' + self.name
        # self._display_name = display_name
        cv2.namedWindow(self._display_name, cv2.WINDOW_NORMAL |
                        cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow(self._display_name, 960, 720)

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
                    self._display_name, image_disp, fromCenter=False)
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
    # TODO: add tracking lost judge
    # TODO: introduce 3d tracking feedback into components tracking
    def __init__(self, tracker_name: str = 'dimp', parameter_name: str = 'dimp18'):
        self._tracker = OnlineTracker(
            name=tracker_name, parameter_name=parameter_name, run_id=0, display_name='vehicle')

        self._box = None

        self._disp_name = 'Display: ' + tracker_name
        # cv2.namedWindow('Display: ' + tracker_name, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        # cv2.resizeWindow('Display: ' + tracker_name, 960, 720)

    def track(self, image: np.ndarray, info: dict, debug=None, visdom_info=None, optional_box=None):
        tic = time.time()
        image_disp = image.copy()
        init_box = None
        if optional_box is not None:
            init_box = optional_box

        # get auxiliary information ready
        aux_info = {}
        toc = time.time()
        print('pre cost {} sec'.format(toc - tic))

        # track
        tic = time.time()
        print('track')
        one_output = self._tracker.track(
            image, info=aux_info, debug=debug, visdom_info=visdom_info, optional_box=init_box)
        print('inference cost {} sec'.format(one_output['time'][1]))

        self._box = one_output['target_bbox'][1]
        toc = time.time()
        print('inference cost {} sec'.format(toc - tic))

        ##################################Draw box##################################
        if self._box is not None:
            cv2.rectangle(image_disp, (int(self._box[0]), int(self._box[1])), (int(self._box[0] + self._box[2]), int(self._box[1] + self._box[3])),
                          (0, 255, 0), 5)

        # Display the resulting frame
        t_s = 0
        t_e = 999999999999999999
        if t_s <= info['header'].stamp.to_sec() <= t_e:
            cv2.imshow(self._disp_name, image_disp)
            key = cv2.waitKey(1)
        elif info['header'].stamp.to_sec() > t_e:
            try:
                cv2.destroyWindow(self._disp_name)
            except:
                pass

        return one_output


class VehicleTrackingWrapper:
    def __init__(self, tracker: VehicleTracking, vis_save_path: str = None):
        rospy.init_node('vehicle_visual_tracking', anonymous=True)
        ns = rospy.get_namespace()
        rospy.Subscriber('/driver/sensing/image/compressed',
                         sensor_msgs.msg.CompressedImage, self._image_callback, queue_size=1, buff_size=2**24)
        rospy.Subscriber('/tracking/visual_detection',
                         Box2DArray, self._vis_det_callback, queue_size=1)
        rospy.Subscriber('/tracking/components_tracking/rearlight/left',
                         Box2D, self._left_rearlight_callback, queue_size=1)
        rospy.Subscriber('/tracking/components_tracking/rearlight/right',
                         Box2D, self._right_rearlight_callback, queue_size=1)
        self._pub_vh = rospy.Publisher(
            '/tracking/visual_tracking', Box2D, queue_size=1)
        self._pub_det_flag = rospy.Publisher(
            '/tracking/visual_detection/flag', std_msgs.msg.Bool, queue_size=1)
        self._timer = rospy.Timer(
            rospy.Duration(0.04), self._tracking_callback)

        self._vh_box_thresh = rospy.get_param(ns + '/vh_box_thresh', 0.2)

        self._curr_image = None
        self._curr_image_header = None
        self._curr_vis_det = []
        self._curr_left_box = None
        self._curr_right_box = None

        self._tracker = tracker
        self._init = False

    def _image_callback(self, data):
        self._curr_image_header = data.header
        np_arr = np.fromstring(data.data, dtype=np.uint8)
        self._curr_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def _vis_det_callback(self, data):
        self._curr_vis_det.clear()
        for box in data.boxes:
            if box.score > self._vh_box_thresh:
                self._curr_vis_det.append(
                    ((box.center_y - box.width / 2, box.center_x - box.height / 2, box.width, box.height), box.score))

    def _left_rearlight_callback(self, data):
        self._curr_left_box = (data.center_y - data.width / 2,
                               data.center_x - data.height / 2, data.width, data.height)

    def _right_rearlight_callback(self, data):
        self._curr_right_box = (data.center_y - data.width / 2,
                                data.center_x - data.height / 2, data.width, data.height)

    @staticmethod
    def _is_in_box(outer_box, inner_box):
        inner_min_x = inner_box[0]
        inner_min_y = inner_box[1]
        inner_max_x = inner_box[0] + inner_box[2]
        inner_max_y = inner_box[1] + inner_box[3]
        outer_min_x = outer_box[0]
        outer_min_y = outer_box[1]
        outer_max_x = outer_box[0] + outer_box[2]
        outer_max_y = outer_box[1] + outer_box[3]
        return inner_min_x >= outer_min_x and inner_max_x <= outer_max_x and \
            inner_min_y >= outer_min_y and inner_max_y <= outer_max_y

    def __find_optimal_init_box(self):
        if len(self._curr_vis_det) == 0:
            return None
        if self._curr_left_box is None or self._curr_right_box is None:
            return None
        candidates = []
        for box in self._curr_vis_det:
            if self._is_in_box(box[0], self._curr_left_box) and self._is_in_box(box[0], self._curr_right_box):
                candidates.append(box)
        if len(candidates) == 0:
            return None
        candidates.sort(reverse=True, key=lambda x: x[1])

        return candidates[0][0]

    def _tracking_callback(self, timer_event):
        if self._curr_image is None:
            return

        track_res = Box2D()
        track_res.header.stamp = self._curr_image_header.stamp
        if not self._init:
            init_box = self.__find_optimal_init_box()
            if init_box is not None:
                self._init = True
                _ = self._tracker.track(
                    self._curr_image,
                    {
                        'header': track_res.header
                    },
                    optional_box=init_box
                )
                det_flag = std_msgs.msg.Bool()
                det_flag.data = False
                self._pub_det_flag.publish(det_flag)
            return

        output = self._tracker.track(
            self._curr_image,
            {
                'header': track_res.header
            }
        )
        target_box = output['target_bbox'][1]
        target_score = output['score'][1]

        track_res.center_x = int(target_box[1] + target_box[3] / 2)
        track_res.center_y = int(target_box[0] + target_box[2] / 2)
        track_res.width = int(target_box[2])
        track_res.height = int(target_box[3])
        self._pub_vh.publish(track_res)


if __name__ == '__main__':
    tracker = VehicleTracking(tracker_name='dimplt', parameter_name='new')
    tracking_wrapper = VehicleTrackingWrapper(tracker=tracker)
    rospy.spin()
