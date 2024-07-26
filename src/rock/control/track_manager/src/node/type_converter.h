/**
 * @file type_converter.h
 * @author Chen Hongxin (angelochen@live.c)
 * @brief data type converter between ros msg and track_manager types
 * @version 0.1
 * @date 2023-04-09
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once

#include "tf/transform_datatypes.h"

#include "cyber_msgs/LocalTrajList.h"
#include "cyber_msgs/PlatoonControlDebug.h"
#include "cyber_msgs/PlatoonControlTarget.h"
#include "cyber_msgs/V2VPacket.h"
#include "geometry_msgs/PoseArray.h"

#include "discretized_trajectory.h"
#include "track_manager_type.h"

namespace converter {

inline void to_ros_msg(const cyberc3::planning::PlatoonControlCommand &data,
                       cyber_msgs::PlatoonControlTarget *msg) {
  msg->header.stamp = ros::Time::now();
  msg->distance_ref = data.distance_expect;
  msg->distance_feedback = data.distance_feedback;
  msg->speed_ref = data.speed_ref;
  msg->acc_ref = data.acc_ref;
  msg->speed_limit_curvature = data.speed_limit_curvature;
  msg->acc_limit_curvature = data.acc_limit_curvature;
  msg->speed_limit_steer = data.speed_limit_steer;
  msg->acc_limit_steer = data.acc_limit_steer;
}

inline void to_ros_msg(const cyberc3::planning::PlatoonControlDebug &data,
                       cyber_msgs::PlatoonControlDebug *msg) {
  msg->header.stamp = ros::Time::now();
  msg->distance_expect = data.distance_expect;
  msg->distance_feedback_perception = data.distance_feedback_perception;
  msg->distance_feedback_curve = data.distance_feedback_curve;
  msg->distance_error = data.distance_error;
  msg->speed_ego = data.speed_ego;
  msg->speed_front_v2v = data.speed_front_v2v;
  msg->speed_front_perception = data.speed_front_perception;
  msg->speed_error = data.speed_error;
  msg->acc_front_v2v = data.acc_front_v2v;
  msg->v2v_delay_ms = data.v2v_delay_ms;
  msg->speed_limit_curvature = data.speed_limit_curvature;
  msg->acc_limit_curvature = data.acc_limit_curvature;
  msg->speed_limit_steer = data.speed_limit_steer;
  msg->acc_limit_steer = data.acc_limit_steer;
  msg->lateral_error = data.lateral_error;
  msg->heading_error = data.heading_error;
}

inline void to_ros_msg(const cyberc3::planning::DiscretizedTrajectory &data,
                       cyber_msgs::LocalTrajList *msg) {
  msg->header.stamp = ros::Time::now();
  msg->header.frame_id = "map";
  for (const auto &point2d : data) {
    cyber_msgs::LocalTrajPoint point;
    point.position.x = point2d.path_point().x();
    point.position.y = point2d.path_point().y();
    point.s = point2d.path_point().s();
    point.theta = point2d.path_point().theta();
    point.kappa = point2d.path_point().kappa();
    point.kappa_prime = point2d.path_point().dkappa();
    point.orientation =
        tf::createQuaternionMsgFromYaw(point2d.path_point().theta());
    msg->points.push_back(std::move(point));
  }
}

inline void to_ros_msg(const cyberc3::planning::DiscretizedTrajectory &data,
                       geometry_msgs::PoseArray *msg) {
  msg->header.stamp = ros::Time::now();
  msg->header.frame_id = "map";
  for (const auto &point2d : data) {
    geometry_msgs::Pose pose;
    pose.position.x = point2d.path_point().x();
    pose.position.y = point2d.path_point().y();
    pose.orientation =
        tf::createQuaternionMsgFromYaw(point2d.path_point().theta());
    msg->poses.push_back(std::move(pose));
  }
}

inline void from_ros_msg(const cyber_msgs::V2VPacket &msg,
                         cyberc3::planning::V2VPacket *data) {
  data->is_updated = msg.is_updated;
  data->timestamp = msg.header.stamp.toSec();
  data->speed = msg.speed;
  data->acc_x = msg.accel_x;
  data->acc_y = msg.accel_y;
  data->yaw_rate = msg.angular_velo_z;
  data->steer_angle = msg.steer;
}

}  // namespace converter
