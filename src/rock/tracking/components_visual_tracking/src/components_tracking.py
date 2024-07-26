#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
# AUTHOR: Haoran Wu
# FILE: components_tracking.py
# DATE: 2023/02/06 周一
# TIME: 17:18:47
'''
# fmt: off
import sys
import os
env_path = os.path.join(os.path.dirname(__file__), 'pytracking')
if env_path not in sys.path:
    sys.path.append(env_path)

from cyber_msgs.msg import Box2D, Box2DArray
import geometry_msgs
import sensor_msgs
import std_msgs
import rospy
import time
import cv2
import pandas as pd
import numpy as np
from pytracking.evaluation import Tracker
from pytracking.evaluation.multi_object_wrapper import MultiObjectWrapper
from collections import OrderedDict, deque
from hmmlearn import hmm
# fmt: on


_tracker_disp_colors = {1: (0, 255, 0), 2: (0, 0, 255), 3: (255, 0, 0),
                        4: (255, 255, 255), 5: (0, 0, 0), 6: (0, 255, 128),
                        7: (123, 123, 123), 8: (255, 128, 0), 9: (128, 0, 255)}


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


class ComponentsTracking:
    # TODO: add tracking lost judge
    # TODO: introduce 3d tracking feedback into components tracking
    def __init__(self, tracker_name: str = 'dimp', parameter_name: str = 'dimp18'):
        self._left_tracker = OnlineTracker(
            name=tracker_name, parameter_name=parameter_name, run_id=0, display_name='left')
        self._right_tracker = OnlineTracker(
            name=tracker_name, parameter_name=parameter_name, run_id=1, display_name='right')

        # (frame_num, score or err)
        self._lost_obs_len = 10
        self._left_buffer = deque(maxlen=50)
        self._right_buffer = deque(maxlen=50)
        self._left_lost = False
        self._right_lost = False
        self._left_seq_dist = None
        self._right_seq_dist = None

        # memorize tail lights' relative pos in target box
        self._left_relative_pos = None
        self._right_relative_pos = None

        self._left_box = None
        self._right_box = None

        # lost decider
        # state: (normal, lost)
        # observation: (err, score)
        self._left_lost_decider = hmm.GaussianHMM(
            n_components=2, covariance_type='diag')
        self._left_lost_decider.startprob_ = np.array([1.0, 0.0])
        self._left_lost_decider.transmat_ = np.array(
            [[0.5, 0.5], [0.2, 0.8]])
        self._left_lost_decider.means_ = np.array(
            [[0.001514, 1.0179], [0.020159, 0.526696]])
        self._left_lost_decider.covars_ = np.array(
            [[0.046006, 0.140221], [0.436015, 0.42602]])

        self._right_lost_decider = hmm.GaussianHMM(
            n_components=2, covariance_type='diag')
        self._right_lost_decider.startprob_ = np.array([1.0, 0.0])
        self._right_lost_decider.transmat_ = np.array(
            [[0.5, 0.5], [0.2, 0.8]])
        self._right_lost_decider.means_ = np.array(
            [[0.001514, 1.0179], [0.020159, 0.526696]])
        self._right_lost_decider.covars_ = np.array(
            [[0.046006, 0.140221], [0.436015, 0.42602]])

        # camera parameters
        self._cam_K = None
        self._cam_D = None

        self._disp_name = 'Display: ' + tracker_name
        # cv2.namedWindow('Display: ' + tracker_name, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        # cv2.resizeWindow('Display: ' + tracker_name, 960, 720)

    def set_cam_params(self, K: np.array, D: np.array):
        self._cam_K = K
        self._cam_D = D

    def track(self, image: np.ndarray, info: dict, pub_left, pub_right, pub_pseudo_left, pub_pseudo_right, frame: int, debug=None, visdom_info=None, optional_boxes=None):

        image_disp = image.copy()
        left_init_box = None if optional_boxes is None else optional_boxes[0]
        right_init_box = None if optional_boxes is None else optional_boxes[1]

        visual_track = None
        visual_track_header = None
        if 'visual_track' in info and info['visual_track'] is not None:
            visual_track = info['visual_track']
            visual_track_header = info['visual_track_header']

        # left
        if frame < 50 or frame % 4 == 0 or frame % 4 == 3:
            s = time.time()
            # get auxiliary information ready
            tic = time.time()
            left_info = {}
            left_info['lost'] = self._left_lost
            left_info['motion_feedback'] = info['left_motion_feedback'] if 'left_motion_feedback' in info else None
            left_info['cam_params'] = {'K': self._cam_K, 'D': self._cam_D}
            left_info['vh_boxes'] = info['vh_boxes'] if 'vh_boxes' in info else None
            left_info['is_night'] = info['is_night'] if 'is_night' in info else False
            toc = time.time()
            print('left pre cost {} sec'.format(toc - tic))

            # track
            print('left track')
            left_output = self._left_tracker.track(
                image, info=left_info, debug=debug, visdom_info=visdom_info, optional_box=left_init_box)
            print('left inference cost {} sec'.format(left_output['time'][1]))
            self._left_box = left_output['target_bbox'][1]

            # lost decision-making
            tic = time.time()
            left_score = left_output['score'][1]

            if 'left_err' in info and info['left_err'] is not None:
                self._left_buffer.append([info['left_err'], left_score])
            else:
                self._left_buffer.append([0.0, left_score])
            left_lost = self._side_lost_decide(
                side='left', vh_boxes=info['vh_boxes'] if 'vh_boxes' in info else None)
            toc = time.time()
            print('left lost decider cost {} sec'.format(toc - tic))
            self._left_lost = left_lost
            if left_lost:
                rospy.logwarn('left lost')

            # not lost, update relative pos in target box
            if not left_lost:
                if visual_track is not None:
                    if self._left_relative_pos is None:
                        self._left_relative_pos = (((self._left_box[0] + self._left_box[2] / 2) - (visual_track[0] + visual_track[2] / 2)) / visual_track[2],
                                                   ((self._left_box[1] + self._left_box[3] / 2) - (visual_track[1] + visual_track[3] / 2)) / visual_track[3])
                    else:
                        new_relative_pos = (((self._left_box[0] + self._left_box[2] / 2) - (visual_track[0] + visual_track[2] / 2)) / visual_track[2],
                                            ((self._left_box[1] + self._left_box[3] / 2) - (visual_track[1] + visual_track[3] / 2)) / visual_track[3])
                        updated_relative_pos = (0.9 *
                                                self._left_relative_pos[0] +
                                                0.1 * new_relative_pos[0], 0.9 *
                                                self._left_relative_pos[1] +
                                                0.1 * new_relative_pos[1])
                        self._left_relative_pos = updated_relative_pos
            # # lost, use target box to generate rearlight tracking result
            # else:
            #     if visual_track is not None:
            #         self._left_box[0] = visual_track[0] + visual_track[2] / 2 + \
            #             self._left_relative_pos[0] * \
            #             visual_track[2] - self._left_box[2] / 2
            #         self._left_box[1] = visual_track[1] + visual_track[3] / 2 + \
            #             self._left_relative_pos[1] * \
            #             visual_track[3] - self._left_box[3] / 2
            #         print('generated left relative x: {}, y: {}'.format(
            #             self._left_relative_pos[0], self._left_relative_pos[1]))

            e = time.time()
            print('left total: {} sec'.format(e - s))

            # publish left
            left_track_res = Box2D()
            left_track_res.header = info['header']
            left_track_res.center_x = int(
                self._left_box[1] + self._left_box[3] / 2)
            left_track_res.center_y = int(
                self._left_box[0] + self._left_box[2] / 2)
            left_track_res.height = int(self._left_box[3])
            left_track_res.width = int(self._left_box[2])
            left_track_res.score = left_score if not left_lost else -1.0
            pub_left.publish(left_track_res)
            # publish pseudo left
            if visual_track is not None and self._left_relative_pos is not None:
                left_pseudo_track_res = Box2D()
                left_pseudo_track_res.header = visual_track_header
                pseudo_left = (visual_track[0] + visual_track[2] / 2 +
                               self._left_relative_pos[0] *
                               visual_track[2] - self._left_box[2] / 2,
                               visual_track[1] + visual_track[3] / 2 +
                               self._left_relative_pos[1] *
                               visual_track[3] - self._left_box[3] / 2)
                left_pseudo_track_res.center_x = int(pseudo_left[1] + self._left_box[3] / 2)
                left_pseudo_track_res.center_y = int(pseudo_left[0] + self._left_box[2] / 2)
                left_pseudo_track_res.width = int(self._left_box[2])
                left_pseudo_track_res.height = int(self._left_box[3])
                left_pseudo_track_res.score = 1.0
                pub_pseudo_left.publish(left_pseudo_track_res)
                

        if frame < 50 or frame % 4 == 1 or frame % 4 == 2:
            # right
            # get auxiliary information ready
            s = time.time()
            tic = time.time()
            right_info = {}
            right_info['lost'] = self._right_lost
            right_info['motion_feedback'] = info['right_motion_feedback'] if 'right_motion_feedback' in info else None
            right_info['cam_params'] = {'K': self._cam_K, 'D': self._cam_D}
            right_info['vh_boxes'] = info['vh_boxes'] if 'vh_boxes' in info else None
            right_info['is_night'] = info['is_night'] if 'is_night' in info else False
            toc = time.time()
            print('right pre cost {} sec'.format(toc - tic))

            # track
            print('right track')
            right_output = self._right_tracker.track(
                image, info=right_info, debug=debug, visdom_info=visdom_info, optional_box=right_init_box)
            print('right inference cost {} sec'.format(
                right_output['time'][1]))
            self._right_box = right_output['target_bbox'][1]

            # lost decision-making
            tic = time.time()
            right_score = right_output['score'][1]

            if 'right_err' in info and info['right_err'] is not None:
                self._right_buffer.append([info['right_err'], right_score])
            else:
                self._right_buffer.append([0.0, right_score])
            right_lost = self._side_lost_decide(
                side='right', vh_boxes=info['vh_boxes'] if 'vh_boxes' in info else None)
            toc = time.time()
            print('right lost decider cost {} sec'.format(toc - tic))
            self._right_lost = right_lost
            if right_lost:
                rospy.logwarn('right lost')

            # not lost, update relative pos in target box
            if not right_lost:
                if visual_track is not None:
                    if self._right_relative_pos is None:
                        self._right_relative_pos = (((self._right_box[0] + self._right_box[2] / 2) - (visual_track[0] + visual_track[2] / 2)) / visual_track[2],
                                                    ((self._right_box[1] + self._right_box[3] / 2) - (visual_track[1] + visual_track[3] / 2)) / visual_track[3])
                    else:
                        new_relative_pos = (((self._right_box[0] + self._right_box[2] / 2) - (visual_track[0] + visual_track[2] / 2)) / visual_track[2],
                                            ((self._right_box[1] + self._right_box[3] / 2) - (visual_track[1] + visual_track[3] / 2)) / visual_track[3])
                        updated_relative_pos = (0.9 *
                                                self._right_relative_pos[0] +
                                                0.1 * new_relative_pos[0], 0.9 *
                                                self._right_relative_pos[1] +
                                                0.1 * new_relative_pos[1])
                        self._right_relative_pos = updated_relative_pos
            # # lost, use target box to generate rearlight tracking result
            # else:
            #     if visual_track is not None:
            #         self._right_box[0] = visual_track[0] + visual_track[2] / 2 + \
            #             self._right_relative_pos[0] * \
            #             visual_track[2] - self._right_box[2] / 2
            #         self._right_box[1] = visual_track[1] + visual_track[3] / 2 + \
            #             self._right_relative_pos[1] * \
            #             visual_track[3] - self._right_box[3] / 2
            #         rospy.logwarn('use generated right')
            #         print('generated right relative x: {}, y: {}'.format(
            #             self._right_relative_pos[0], self._right_relative_pos[1]))

            e = time.time()
            print('right total: {} sec'.format(e - s))
            # publish right
            right_track_res = Box2D()
            right_track_res.header = info['header']
            right_track_res.center_x = int(
                self._right_box[1] + self._right_box[3] / 2)
            right_track_res.center_y = int(
                self._right_box[0] + self._right_box[2] / 2)
            right_track_res.height = int(self._right_box[3])
            right_track_res.width = int(self._right_box[2])
            right_track_res.score = right_score if not right_lost else -1.0
            pub_right.publish(right_track_res)
            # publish pseudo right
            if visual_track is not None and self._right_relative_pos is not None:
                right_pseudo_track_res = Box2D()
                right_pseudo_track_res.header = visual_track_header
                pseudo_right = (visual_track[0] + visual_track[2] / 2 +
                               self._right_relative_pos[0] *
                               visual_track[2] - self._right_box[2] / 2,
                               visual_track[1] + visual_track[3] / 2 +
                               self._right_relative_pos[1] *
                               visual_track[3] - self._right_box[3] / 2)
                right_pseudo_track_res.center_x = int(pseudo_right[1] + self._right_box[3] / 2)
                right_pseudo_track_res.center_y = int(pseudo_right[0] + self._right_box[2] / 2)
                right_pseudo_track_res.width = int(self._right_box[2])
                right_pseudo_track_res.height = int(self._right_box[3])
                right_pseudo_track_res.score = 1.0
                pub_pseudo_right.publish(right_pseudo_track_res)

        ##################################Draw box##################################
        if self._left_box is not None:
            cv2.rectangle(image_disp, (int(self._left_box[0]), int(self._left_box[1])), (int(self._left_box[0] + self._left_box[2]), int(self._left_box[1] + self._left_box[3])),
                          (0, 255, 0), 5)
        if self._right_box is not None:
            cv2.rectangle(image_disp, (int(self._right_box[0]), int(self._right_box[1])), (int(self._right_box[0] + self._right_box[2]), int(self._right_box[1] + self._right_box[3])),
                          (0, 255, 0), 5)

        # Display the resulting frame
        t_s = 0
        t_e = 9999999999999999
        if t_s <= info['header'].stamp.to_sec() <= t_e:
            cv2.imshow(self._disp_name, image_disp)
            key = cv2.waitKey(1)
        elif info['header'].stamp.to_sec() > t_e:
            try:
                cv2.destroyWindow(self._disp_name)
            except:
                pass

        # return left_output, right_output, left_lost, right_lost

    def _side_lost_decide(self, side: str, vh_boxes=None, target_box=None):
        if side == 'left':
            if len(self._left_buffer) < self._lost_obs_len:
                return False
            x_left = np.array(list(self._left_buffer)[-self._lost_obs_len:])
            if self._left_seq_dist is None:
                self._left_lost_decider.startprob_ = np.array([1.0, 0.0])
            else:
                self._left_lost_decider.startprob_ = self._left_seq_dist[1, :]
            try:
                z_left = self._left_lost_decider.predict(x_left)
                self._left_seq_dist = self._left_lost_decider.predict_proba(
                    x_left)
            except ValueError:
                rospy.logwarn('lost decider ValueError!')
                self._left_buffer.clear()
                self._left_lost = False
                self._left_seq_dist = None
                return False

            if target_box is not None:
                expanded_target_box = (target_box[0] + target_box[2] / 2 - target_box[2] / 2 * 1.1,
                                       target_box[1] + target_box[3] /
                                       2 - target_box[3] / 2 * 1.1,
                                       target_box[2] * 1.1, target_box[3] * 1.1)
                if not self._is_in_box(expanded_target_box, self._left_box):
                    return False

            valid_left = True
            for box in vh_boxes:
                if box[2] > 800 or box[3] > 500:    # invalid box
                    continue
                if not self._is_in_box(box, self._left_box):
                    valid_left = False
            return z_left[-1] == 1 and valid_left
        else:
            if len(self._right_buffer) < self._lost_obs_len:
                return False
            x_right = np.array(list(self._right_buffer)[-self._lost_obs_len:])
            if self._right_seq_dist is None:
                self._right_lost_decider.startprob_ = np.array([1.0, 0.0])
            else:
                self._right_lost_decider.startprob_ = self._right_seq_dist[1, :]
            try:
                z_right = self._right_lost_decider.predict(x_right)
                self._right_seq_dist = self._right_lost_decider.predict_proba(
                    x_right)
            except ValueError:
                rospy.logwarn('lost decider ValueError!')
                self._right_buffer.clear()
                self._right_lost = False
                self._right_seq_dist = None
                return False

            if target_box is not None:
                expanded_target_box = (target_box[0] + target_box[2] / 2 - target_box[2] / 2 * 1.1,
                                       target_box[1] + target_box[3] /
                                       2 - target_box[3] / 2 * 1.1,
                                       target_box[2] * 1.1, target_box[3] * 1.1)
                if not self._is_in_box(expanded_target_box, self._right_box):
                    return False

            valid_right = True
            for box in vh_boxes:
                if box[2] > 800 or box[3] > 500:    # invalid box
                    continue
                if not self._is_in_box(box, self._right_box):
                    valid_right = False
            return z_right[-1] == 1 and valid_right

    def _lost_decide(self, vh_boxes=None):
        """_summary_

        Returns:
            bool, bool: if rearlight is lost or not
        """
        if len(self._left_buffer) < self._lost_obs_len or len(self._right_buffer) < self._lost_obs_len:
            return False, False

        x_left = np.array(list(self._left_buffer)[-self._lost_obs_len:])
        x_right = np.array(list(self._right_buffer)[-self._lost_obs_len:])
        if self._left_seq_dist is None:
            self._left_lost_decider.startprob_ = np.array([1.0, 0.0])
        else:
            self._left_lost_decider.startprob_ = self._left_seq_dist[1, :]
        if self._right_seq_dist is None:
            self._right_lost_decider.startprob_ = np.array([1.0, 0.0])
        else:
            self._right_lost_decider.startprob_ = self._right_seq_dist[1, :]

        try:
            z_left = self._left_lost_decider.predict(x_left)
            z_right = self._right_lost_decider.predict(x_right)

            self._left_seq_dist = self._left_lost_decider.predict_proba(x_left)
            self._right_seq_dist = self._right_lost_decider.predict_proba(
                x_right)
        except ValueError:
            rospy.logwarn('lost decider ValueError!')
            self._left_buffer.clear()
            self._right_buffer.clear()
            self._left_lost = False
            self._right_lost = False
            self._left_seq_dist = None
            self._right_seq_dist = None
            return False, False

        # if np.any(z_left == 1):
        #     z_left_cp = z_left.reshape((-1, 1))
        #     prob_left_cp = self._left_seq_dist.reshape((-1, 2))
        #     left_info = np.concatenate((z_left_cp, prob_left_cp ,x_left), axis=1)
        #     left_info = pd.DataFrame(left_info, columns=['hidden_state', 'prob_0', 'prob_1', 'obs_err', 'obs_score'])
        #     left_info.to_csv('/media/wuhr/data/platoon_dataset/2022_10_20/evaluation/2d/vis/{}.csv'.format(np.random.rand()), index=False)
        # if np.any(z_right == 1):
        #     z_right_cp = z_right.reshape((-1, 1))
        #     prob_right_cp = self._right_seq_dist.reshape((-1, 2))
        #     right_info = np.concatenate((z_right_cp, prob_right_cp ,x_right), axis=1)
        #     right_info = pd.DataFrame(right_info, columns=['hidden_state', 'prob_0', 'prob_1', 'obs_err', 'obs_score'])
        #     right_info.to_csv('/media/wuhr/data/platoon_dataset/2022_10_20/evaluation/2d/vis/{}.csv'.format(np.random.rand()), index=False)

        if self._left_box is not None and self._right_box is not None:  # 2 boxes too close
            left_center = np.array(
                [self._left_box[0] + self._left_box[2] / 2, self._left_box[1] + self._left_box[3] / 2])
            right_center = np.array(
                [self._right_box[0] + self._right_box[2] / 2, self._right_box[1] + self._right_box[3] / 2])
            dis = np.linalg.norm(left_center - right_center)
            if dis < (self._left_box[2] + self._left_box[3] + self._right_box[2] + self._right_box[3]) / 4:
                return True, True

        valid_left = valid_right = True
        for box in vh_boxes:
            if box[2] > 800 or box[3] > 500:    # invalid box
                continue
            if not self._is_in_box(box, self._left_box):
                valid_left = False
            if not self._is_in_box(box, self._right_box):
                valid_right = False

        # TODO: use lost probability to tune 3d tracking covariance

        return z_left[-1] == 1 and valid_left, z_right[-1] == 1 and valid_right

    def _is_in_box(self, outer_box, inner_box):
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

    # def _map_3d_to_2d(self, points_3d: np.ndarray):
    #     """_summary_
    #     Args:
    #         points_3d (np.ndarray): shape (N, 3), in camera coordinates
    #                 Z^
    #                 |    <
    #                 |   + X
    #                 |  +
    #                 | +
    #                 |+
    #         <--------+
    #         Y         O
    #     return: np.ndarray, shape (N, 2), in image coordinates
    #     """
    #     # convert to OpenCV camera coordinate
    #     points_3d_cv = np.empty_like(points_3d)
    #     points_3d_cv[:, 0] = -points_3d[:, 1]
    #     points_3d_cv[:, 1] = -points_3d[:, 2]
    #     points_3d_cv[:, 2] = points_3d[:, 0]
    #     points_3d_cv = points_3d_cv.reshape((1, -1, 3))

    #     rvec = tvec = np.zeros((1, 1, 3))
    #     points_2d, _ = cv2.fisheye.projectPoints(
    #         points_3d_cv, rvec, tvec, self._cam_K, self._cam_D)

    #     return points_2d[0]


class ComponentsTrackingWrapper:
    def __init__(self, tracker: ComponentsTracking, vis_save_path: str = None):
        rospy.init_node('components_tracking', anonymous=True)
        rospy.Subscriber('/driver/fisheye/front/compressed',
                         sensor_msgs.msg.CompressedImage, self._image_callback, queue_size=1, buff_size=2**24)
        rospy.Subscriber('/tracking/visual_detection',
                         Box2DArray, self._vis_det_callback, queue_size=1)
        rospy.Subscriber('/tracking/err/left', std_msgs.msg.Float32,
                         self._left_err_callback, queue_size=1)
        rospy.Subscriber('/tracking/err/right', std_msgs.msg.Float32,
                         self._right_err_callback, queue_size=1)
        rospy.Subscriber('/tracking/pos/left', geometry_msgs.msg.PointStamped,
                         self._left_pos_callback, queue_size=1)
        rospy.Subscriber('/tracking/pos/right', geometry_msgs.msg.PointStamped,
                         self._right_pos_callback, queue_size=1)
        rospy.Subscriber('/tracking/visual_tracking', Box2D,
                         self._target_vis_tracking_callback, queue_size=1)

        self._pub_rearlight = rospy.Publisher(
            '/tracking/components_tracking/rearlight', Box2DArray, queue_size=1)
        self._pub_left_rearlight = rospy.Publisher(
            '/tracking/components_tracking/rearlight/left', Box2D, queue_size=1)
        self._pub_right_rearlight = rospy.Publisher(
            '/tracking/components_tracking/rearlight/right', Box2D, queue_size=1)
        self._pub_pseudo_left_rearlight = rospy.Publisher(
            '/tracking/components_tracking/rearlight/pseudo/left', Box2D, queue_size=1)
        self._pub_pseudo_right_rearlight = rospy.Publisher(
            '/tracking/components_tracking/rearlight/pseudo/right', Box2D, queue_size=1)
        self._pub_left_score = rospy.Publisher(
            '/tracking/components_tracking/score/left', std_msgs.msg.Float32, queue_size=1)
        self._pub_right_score = rospy.Publisher(
            '/tracking/components_tracking/score/right', std_msgs.msg.Float32, queue_size=1)

        self._timer = rospy.Timer(
            rospy.Duration(0.04), self._tracking_callback)

        self._vis_save_path = vis_save_path

        # load parameters
        cam_f_x = rospy.get_param('/cam_f_x')
        cam_f_y = rospy.get_param('/cam_f_y')
        cam_c_x = rospy.get_param('/cam_c_x')
        cam_c_y = rospy.get_param('/cam_c_y')
        cam_k_1 = rospy.get_param('/cam_k_1')
        cam_k_2 = rospy.get_param('/cam_k_2')
        cam_k_3 = rospy.get_param('/cam_k_3')
        cam_k_4 = rospy.get_param('/cam_k_4')
        K = np.array([
            [cam_f_x, 0.0, cam_c_x],
            [0.0, cam_f_y, cam_c_y],
            [0.0, 0.0, 1.0]])
        D = np.array([cam_k_1, cam_k_2, cam_k_3, cam_k_4]).reshape((-1, 1))

        self._is_night = rospy.get_param('/is_night', False)
        self._vh_box_thresh = rospy.get_param('/vh_box_thresh', 0.2)
        self._use_vh_visual_tracking = rospy.get_param(
            '/use_vh_visual_tracking', True)

        self._curr_image = None
        self._curr_image_header = None
        self._curr_vis_det = []
        self._target_vis_track = None
        self._target_vis_track_header = None

        self._tracker = tracker
        self._tracker.set_cam_params(K, D)
        self._prev_output = OrderedDict()

        self._left_err = 0.0
        self._right_err = 0.0

        self._left_pos = None
        self._right_pos = None

        self._tracker_frame = 0

    def _image_callback(self, data):
        self._curr_image_header = data.header
        # t_now = rospy.Time.now()
        # print('image receive delay: {} sec'.format(
        #     (t_now - self._curr_image_header.stamp).to_sec()))
        
        np_arr = np.fromstring(data.data, dtype=np.uint8)
        self._curr_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # t_now = rospy.Time.now()
        # print('image receive delay 1: {} sec'.format(
        #     (t_now - self._curr_image_header.stamp).to_sec()))

        # # # track
        # self._tracking_callback(None)

    def _left_err_callback(self, data):
        self._left_err = data.data

    def _right_err_callback(self, data):
        self._right_err = data.data

    def _left_pos_callback(self, data):
        t_now = rospy.Time.now()
        dt = t_now - data.header.stamp
        if np.abs(dt.to_sec()) > 0.1:
            self._left_pos = None
        else:
            self._left_pos = np.array([data.point.x, data.point.y, 0.3])

    def _right_pos_callback(self, data):
        t_now = rospy.Time.now()
        dt = t_now - data.header.stamp
        if np.abs(dt.to_sec()) > 0.1:
            self._right_pos = None
        else:
            self._right_pos = np.array([data.point.x, data.point.y, 0.3])

    def _vis_det_callback(self, msg):
        self._curr_vis_det.clear()
        for box in msg.boxes:
            if box.score > self._vh_box_thresh:
                self._curr_vis_det.append(
                    (box.center_y - box.width / 2, box.center_x - box.height / 2, box.width, box.height))

    def _target_vis_tracking_callback(self, msg):
        self._target_vis_track = (
            msg.center_y - msg.width / 2, msg.center_x - msg.height / 2, msg.width, msg.height)
        self._target_vis_track_header = msg.header

    def _tracking_callback(self, timer_event):
        s = time.time()
        if self._curr_image is None:
            return

        track_res = Box2DArray()
        if self._curr_image_header is not None:
            track_res.header = self._curr_image_header

        print('image delay: {} sec'.format(
            (rospy.Time.now() - track_res.header.stamp).to_sec()))
        print('track')

        tic = time.time()
        target_visual_track_valid = self._use_vh_visual_tracking and self._target_vis_track is not None and abs(
            (track_res.header.stamp - self._target_vis_track_header.stamp).to_sec()) < 0.5
        print('use visual tracking: {}'.format(target_visual_track_valid))
        self._tracker.track(
            self._curr_image,
            {
                'left_err': self._left_err,
                'right_err': self._right_err,
                'left_motion_feedback': self._left_pos,
                'right_motion_feedback': self._right_pos,
                'vh_boxes': self._curr_vis_det,
                'is_night': self._is_night,
                'header': track_res.header,
                'visual_track': self._target_vis_track if target_visual_track_valid else None,
                'visual_track_header': self._target_vis_track_header
            },
            self._pub_left_rearlight,
            self._pub_right_rearlight,
            self._pub_pseudo_left_rearlight,
            self._pub_pseudo_right_rearlight,
            self._tracker_frame)
        self._tracker_frame += 1
        # rospy.loginfo('left score: {}, err: {}'.format(
        #     left_output['score'][1], self._left_err))
        # rospy.loginfo('right score: {}, err: {}'.format(
        #     right_output['score'][1], self._right_err))
        toc = time.time()
        print('track cost {} sec'.format(toc - tic))

        # tic = time.time()

        # left_bbox = left_output['target_bbox'][1]  # [x_lt, y_lt, w, h]
        # right_bbox = right_output['target_bbox'][1]
        # left_score = left_output['score'][1]
        # right_score = right_output['score'][1]

        # if left_lost:
        #     rospy.logwarn('left lost!')
        # if right_lost:
        #     rospy.logwarn('right lost!')

        # left_score_msg = std_msgs.msg.Float32()
        # right_score_msg = std_msgs.msg.Float32()
        # left_score_msg.data = left_score
        # right_score_msg = right_score
        # self._pub_left_score.publish(left_score_msg)
        # self._pub_right_score.publish(right_score_msg)

        # left_track_res = Box2D()
        # left_track_res.header = track_res.header
        # left_track_res.center_x = int(left_bbox[1] + left_bbox[3] / 2)
        # left_track_res.center_y = int(left_bbox[0] + left_bbox[2] / 2)
        # left_track_res.height = int(left_bbox[3])
        # left_track_res.width = int(left_bbox[2])
        # left_track_res.score = left_score if not left_lost else -1.0
        # track_res.boxes.append(left_track_res)

        # right_track_res = Box2D()
        # right_track_res.header = track_res.header
        # right_track_res.center_x = int(right_bbox[1] + right_bbox[3] / 2)
        # right_track_res.center_y = int(right_bbox[0] + right_bbox[2] / 2)
        # right_track_res.height = int(right_bbox[3])
        # right_track_res.width = int(right_bbox[2])
        # right_track_res.score = right_score if not right_lost else -1.0
        # track_res.boxes.append(right_track_res)

        # self._pub_rearlight.publish(track_res)

        # toc = time.time()
        # print('publish cost {} sec'.format(toc - tic))

        # # vis
        # if self._vis_save_path is not None:
        #     t_s = time.time()
        #     img_disp_1 = cv2.rectangle(img_disp_1, (int(left_bbox[0]), int(left_bbox[1])), \
        #         (int(left_bbox[0] + left_bbox[2]), int(left_bbox[1] + left_bbox[3])), \
        #             color=(0, 255, 0) if not left_lost else (255, 128, 0), thickness=2)
        #     img_disp_1 = cv2.rectangle(img_disp_1, (int(right_bbox[0]), int(right_bbox[1])), \
        #         (int(right_bbox[0] + right_bbox[2]), int(right_bbox[1] + right_bbox[3])), \
        #             color=(0, 255, 0) if not left_lost else (255, 128, 0), thickness=2)

        #     # img_disp_2 = cv2.rectangle(img_disp_2, (int(left_bbox[0]), int(left_bbox[1])), \
        #     #     (int(left_bbox[0] + left_bbox[2]), int(left_bbox[1] + left_bbox[3])), \
        #     #         color=(0, 255, 0), thickness=2)
        #     # img_disp_2 = cv2.rectangle(img_disp_2, (int(right_bbox[0]), int(right_bbox[1])), \
        #     #     (int(right_bbox[0] + right_bbox[2]), int(right_bbox[1] + right_bbox[3])), \
        #     #         color=(0, 255, 0), thickness=2)

        #     # save vis
        #     timestamp = track_res.header.stamp.to_sec()
        #     cv2.imwrite(os.path.join(self._vis_save_path, 'no_hint_{}.png'.format(timestamp)), img_disp_1)
        #     # cv2.imwrite(os.path.join(self._vis_save_path, '{}.png'.format(timestamp)), img_disp_2)

        #     # cv2.imshow('rearlight tracking', img_disp_1)
        #     # cv2.waitKey(1)
        #     t_e = time.time()
        #     print('vis cost {} sec'.format(t_e - t_s))

        e = time.time()
        rospy.loginfo("track once, cost {} sec".format(e - s))


if __name__ == '__main__':
    # DiMP
    # tracker = ComponentsTracking()

    # RLT-DiMP
    # tracker = ComponentsTracking(tracker_name='dimplt', parameter_name='new')

    # Ours
    tracker = ComponentsTracking(
        tracker_name='motion_dimplt', parameter_name='motion_dimplt')

    # KeepTrack
    # tracker = ComponentsTracking(
    #     tracker_name='keep_track', parameter_name='default_fast')

    # TOMP
    # tracker = ComponentsTracking(
    #     tracker_name='tomp', parameter_name='tomp50')

    tracking_wrapper = ComponentsTrackingWrapper(tracker)
    rospy.spin()
