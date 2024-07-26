#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
# AUTHOR: Haoran Wu
# FILE: ekf_tracking.py
# DATE: 2021/09/29 周三
# TIME: 15:59:20
'''

import rospy
import std_msgs
import geometry_msgs
from visualization_msgs.msg import Marker
from cyber_msgs.msg import Object, ObjectArray, SteerFeedback, SpeedFeedback

import numpy as np
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise
from collections import deque


class EKFTracker:
    """
    state space: [x, y, v_x, v_y]
    measurement: [x, y, v_x, v_y, k]
    """
    def __init__(self) -> None:
        self.x_dim = 4
        self.z_dim = 5
        self.u_dim = 0
        self.dt = 0.1
        self.sigma_obs_x = 0.10
        self.sigma_obs_y = 0.10
        self.sigma_obs_v_x = 0.05
        self.sigma_obs_v_y = 0.05
        self.sigma_obs_k = 0.03

        self.x_c_r = 3.55 - 2.05
        self.y_c_r = 0.40

        self.filter = ExtendedKalmanFilter(dim_x=self.x_dim, dim_z=self.z_dim, dim_u=self.u_dim)
        self.filter.F = np.array([[1.0, 0.0, self.dt, 0.0], 
            [0.0, 1.0, 0.0, self.dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]])
        self.filter.R = np.diag([self.sigma_obs_x, self.sigma_obs_y, self.sigma_obs_v_x, self.sigma_obs_v_y, self.sigma_obs_k])
        self.filter.Q = Q_discrete_white_noise(dim=2, dt=self.dt, var=0.1, block_size=2, order_by_dim=False)
        self.filter.P *= 50.0

        self.radar_to_rear_center = 3.55
        self.wheel_base = 2.56

        self.is_initialized = False
        self.fusion_obj = None
        self.visual_obj_id = None
        self.ego_pose = None    # Pose2D
        self.ego_speed = 0.0 # float, cm/s
        self.ego_steer_angle = 0.0  # in rad; left(+), right(-)
        # self.last_front_yaw = 0.0
        self.last_front_pose = None
        self.front_pose_buffer = deque(maxlen=3)
        
        rospy.init_node('ekf_tracking', anonymous=True)
        rospy.Subscriber('/visual_track', ObjectArray, self._visual_track_callback)
        rospy.Subscriber('/fusion_tracking/object', Object, self._fusion_callback)
        rospy.Subscriber('/control/ego_pose', geometry_msgs.msg.Pose2D, self._ego_pose_callback)
        rospy.Subscriber('/xboxone/rematch', std_msgs.msg.Bool, self._rematch_callback)
        rospy.Subscriber('/rock_can/steer_feedback', SteerFeedback, self._steer_feedback_callback)
        rospy.Subscriber('/rock_can/speed_feedback', SpeedFeedback, self._ego_speed_callback)
        # self.pub = rospy.Publisher('/ekf_tracking/object', cyber_msgs.msg.Object, queue_size=1)
        self.prior_marker_pub = rospy.Publisher('ekf_tracking/prior_marker', Marker, queue_size=1)
        self.post_marker_pub = rospy.Publisher('ekf_tracking/post_marker', Marker, queue_size=1)
        self.front_global_pose_pub = rospy.Publisher('/tracking/front_vehicle/global_pose',
            geometry_msgs.msg.Pose2D, queue_size=1)
        self.front_local_pose_pub = rospy.Publisher('/tracking/front_vehicle/local_pose',
            geometry_msgs.msg.Pose2D, queue_size=1)
        self.front_speed_pub = rospy.Publisher('/tracking/front_vehicle/speed', std_msgs.msg.Float32, queue_size=1)
        self.front_local_velo_pub = rospy.Publisher('/tracking/front_vehicle/local_velo', geometry_msgs.msg.Vector3, queue_size=1)
        self.front_lost_pub = rospy.Publisher('/tracking/front_vehicle/is_lost', std_msgs.msg.Bool, queue_size=1)
        
        rospy.Timer(rospy.Duration(0.1), self._callback)

        rospy.spin()
    
    def _init_filter(self):
        self.filter = ExtendedKalmanFilter(dim_x=self.x_dim, dim_z=self.z_dim, dim_u=self.u_dim)
        self.filter.F = np.array([[1.0, 0.0, self.dt, 0.0], 
            [0.0, 1.0, 0.0, self.dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]])
        self.filter.R = np.diag([self.sigma_obs_x, self.sigma_obs_y, self.sigma_obs_v_x, self.sigma_obs_v_y, self.sigma_obs_k])
        self.filter.Q = Q_discrete_white_noise(dim=2, dt=self.dt, var=0.1, block_size=2, order_by_dim=False)
        self.filter.P *= 50.0

        self.is_initialized = False
        self.fusion_obj = None
        self.visual_obj_id = None
        self.ego_pose = None    # Pose2D
        self.ego_speed = 0.0 # float, cm/s
        self.ego_steer_angle = 0.0
        # self.last_front_yaw = 0.0
        self.last_front_pose = None
        self.front_pose_buffer = deque(maxlen=3)
    
    def _callback(self, event):
        # parse msg
        if self.fusion_obj is not None:
            x = self.fusion_obj.pose.position.x
            y = self.fusion_obj.pose.position.y
            k = self.fusion_obj.pose.position.z
            v_x = self.fusion_obj.velocity.linear.x
            v_y = self.fusion_obj.velocity.linear.y
        else:
            x = 0.0
            y = 0.0
            k = 0.0
            v_x = 0.0
            v_y = 0.0

        # initialize state
        if not self.is_initialized:
            self.filter.x = np.array([x, y, 0.0, 0.0]).reshape(-1, 1)
            self.is_initialized = True

        # predict
        self.filter.predict()
        # measure
        z = np.array([x, y, v_x, v_y, k]).reshape(-1, 1)
        # update
        self.filter.update(z=z, HJacobian=self._HJacobian_at, Hx=self._hx)  # TODO: ValueError, re-initialize
        
        self._publish_marker(x=self.filter.x_post[0, 0], y=self.filter.x_post[1, 0], type='post')
        self._publish_marker(x=self.filter.x_prior[0, 0], y=self.filter.x_prior[1, 0], type='prior')
        
        front_local_pose = geometry_msgs.msg.Pose2D()
        front_local_pose.x = self.filter.x_post[0, 0]
        front_local_pose.y = self.filter.x_post[1, 0]
        front_local_pose.theta = 0.0
        self.front_local_pose_pub.publish(front_local_pose)
        # rospy.loginfo('local_pose: {}, {}'.format(front_local_pose.x, front_local_pose.y))

        front_local_velo = geometry_msgs.msg.Vector3()
        front_local_velo.x = self.filter.x_post[2, 0]
        front_local_velo.y = self.filter.x_post[3, 0]
        self.front_local_velo_pub.publish(front_local_velo)

        rospy.loginfo('Filtered object position x : {}, y: {}'.format(self.filter.x_post[0, 0], self.filter.x_post[1, 0]))

        # front vehicle tracking in global frame
        if self.ego_pose is not None:
            front_global_pose = geometry_msgs.msg.Pose2D()
            ego_yaw = self.ego_pose.theta
            ego_speed = 0.01 * self.ego_speed   # m/s
            ego_wheel_angle = self.ego_steer_angle * np.pi / 180.0 / 15.58

            front_global_pose.x = self.filter.x_post[0, 0] * np.cos(ego_yaw) - self.filter.x_post[1, 0] * np.sin(ego_yaw) + self.ego_pose.x
            front_global_pose.y = self.filter.x_post[0, 0] * np.sin(ego_yaw) + self.filter.x_post[1, 0] * np.cos(ego_yaw) + self.ego_pose.y

            ego_angular_speed = self.radar_to_rear_center * ego_speed * np.tan(ego_wheel_angle) / self.wheel_base
            front_global_velo_x = self.filter.x_post[2, 0] * np.cos(ego_yaw) - self.filter.x_post[3, 0] * np.sin(ego_yaw) + \
                ego_speed * np.cos(ego_yaw) - ego_angular_speed * np.sin(ego_yaw)
            front_global_velo_y = self.filter.x_post[2, 0] * np.sin(ego_yaw) + self.filter.x_post[3, 0] * np.cos(ego_yaw) + \
                ego_speed * np.sin(ego_yaw) + ego_angular_speed * np.cos(ego_yaw)
            
            # front_yaw = np.arctan2(front_global_velo_y, front_global_velo_x)
            front_global_velo = np.array([front_global_velo_x, front_global_velo_y])
            front_global_speed = np.linalg.norm(front_global_velo) # m/s
            # if self.last_front_pose is not None:
            #     front_yaw = self.clip_angle(front_yaw) if front_global_speed >= 0.2 else self.last_front_pose.theta
            # else:
            #     front_yaw = self.clip_angle(front_yaw)
            # front_global_pose.theta = front_yaw
            # self.last_front_yaw = front_yaw
            if len(self.front_pose_buffer) <= 1:
                front_global_pose.theta = 0.0
            else:
                front_global_waypts = np.array([[waypt.x, waypt.y] for waypt in self.front_pose_buffer])
                front_global_waypts = front_global_waypts[::-1, :]
                front_global_pose.theta = self._estimate_yaw(front_global_waypts)

            front_global_speed_msg = std_msgs.msg.Float32()
            front_global_speed_msg.data = front_global_speed
            self.last_front_pose = front_global_pose
            self.front_global_pose_pub.publish(front_global_pose)
            self.front_speed_pub.publish(front_global_speed_msg)
            self.front_pose_buffer.append(front_global_pose)
        is_lost = std_msgs.msg.Bool()
        is_lost.data = False
        self.front_lost_pub.publish(is_lost)
            
    def _HJacobian_at(self, X: np.ndarray) -> np.ndarray:
        """
        compute Jacobian of H at X
        X: state [x, y, v_x, v_y]
        """
        x, y, v_x, v_y = X.flatten()
        gamma = np.arctan2(y, x)

        HJacobian = np.zeros((self.z_dim, self.x_dim))
        HJacobian[:4, :4] = np.eye(4)
        # HJacobian[2, 2] = np.cos(gamma)
        # HJacobian[2, 3] = np.sin(gamma)
        HJacobian[4, 0] = -(y - self.y_c_r) / (x - self.x_c_r) ** 2
        HJacobian[4, 1] = 1.0 / (x - self.x_c_r)

        return HJacobian
    
    def _hx(self, X: np.ndarray) -> np.ndarray:
        """
        compute measurement
        X: state [x, y, v_x, v_y]
        """
        x, y, v_x, v_y = X.flatten()
        gamma = np.arctan2(y, x)

        z = np.array([x, y, v_x, v_y, (y - self.y_c_r) / (x - self.x_c_r)]).reshape(self.z_dim, -1)
        return z

    def _estimate_yaw(self, points: np.ndarray, ego_yaw: float=0.0) -> float:
        """
        points: np.ndarray (3, 2) (t, t - 1, t - 2); or (2, 2) (t, t - 1)
        """
        _2_points_estimation = np.arctan2(points[0, 1] - points[1, 1], points[0, 0] - points[1, 0])
        _2_points_estimation = self.clip_angle(_2_points_estimation)
        if points.shape[0] == 2 or True:
            return _2_points_estimation

        a = points[0, 0] - points[1, 0]
        b = points[0, 1] - points[1, 1]
        c = points[0, 0] - points[2, 0]
        d = points[0, 1] - points[2, 1]
        e = (points[0, 0] ** 2 - points[1, 0] ** 2 + points[0, 1] ** 2 - points[1, 1] ** 2) / 2
        f = (points[0, 0] ** 2 - points[2, 0] ** 2 + points[0, 1] ** 2 - points[2, 1] ** 2) / 2
        det = b * c - a * d
        if np.abs(det) < 1e-3:  # 3 points collinear
            return _2_points_estimation
        x_0 = -(d * e - b * f) / det
        y_0 = -(a * f - c * e) / det
        radial_direction = np.arctan2(points[0, 1] - y_0, points[0, 0] - x_0)
        radial_direction = self.clip_angle(radial_direction)
        _3_points_estimation = self.clip_angle(radial_direction + 0.5 * np.pi) \
            if self.clip_angle(_2_points_estimation - radial_direction) < 0.5 * np.pi \
                else self.clip_angle(radial_direction - 0.5 * np.pi)
        return _3_points_estimation

    def _ego_pose_callback(self, msg):
        self.ego_pose = msg
    
    def _ego_speed_callback(self, msg):
        self.ego_speed = msg.speed_cms

    def _steer_feedback_callback(self, msg):
        self.ego_steer_angle = msg.SteerAngle
    
    def _visual_track_callback(self, msg):
        if len(msg.objects) > 0:
            self.visual_obj_id = msg.objects[0].object_id
        else:
            self.visual_obj_id = -1
        
    def _fusion_callback(self, msg):
        self.fusion_obj = msg

    @staticmethod
    def clip_angle(theta: float) -> float:
        if theta < -np.pi or theta > np.pi:
            theta = np.remainder(theta, 2 * np.pi)
        if theta > np.pi:
            theta -= 2 * np.pi
        
        return theta

    def _publish_marker(self, x: float, y: float, type: str):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'point'
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        
        marker.color.a = 1.0
        if type == 'post':
            marker.color.b = 1.0
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            self.post_marker_pub.publish(marker)
        else:
            marker.color.b = 1.0
            marker.color.r = 1.0
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            self.prior_marker_pub.publish(marker)
    
    def _rematch_callback(self, data):
        if data.data:
            rospy.logwarn('-------------EKF Rematch-------------')
            self._init_filter()



if __name__ == '__main__':
    ekf_tracker = EKFTracker()