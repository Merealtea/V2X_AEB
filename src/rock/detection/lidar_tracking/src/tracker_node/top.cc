/******************************************************************************
 * Copyright 2019, Ezekiel. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "tracker_node/top.h"
#include "std_msgs/Bool.h"

namespace top {
Top::Top(ros::NodeHandle nh, ros::NodeHandle pnh) : ground_filter_(nh, pnh){
  pnh.param("map_frame_id", map_frame_id_, std::string("map"));
  pnh.param("vehicle_frame_id", vehicle_frame_id_, std::string("vehicle"));
  pnh.param("pose_error_estimation", pose_error_estimation, 0.06);
  pnh.param("heading_error_estimation", heading_error_estimation, 0.5);
  pnh.param("speed_error_estimation", speed_error_estimation, 0.003);
  pnh.param("yaw_rate_error_estimation", yaw_rate_error_estimation, 0.003);
  pnh.param("speed_drive_error_estimation", speed_drive_error_estimation, 5.0);
  pnh.param("yaw_rate_drive_error_estimation", yaw_rate_drive_error_estimation, 0.1);
  tracker_.GetParameters(nh, pnh);
  tracker_behind_.GetBehindParameters(nh, pnh);

  pcl_target_publisher_ = nh.advertise<VPointCloud>("/tracking/front_vehicle/roi_points", 1);
  // pcl_target_behind_publisher_ = nh.advertise<VPointCloud>("/tracking/behind_vehicle/roi_points", 1);
  target_pose_lidar_publisher_ = nh.advertise<geometry_msgs::Pose2D>("/tracking/front_vehicle/global_pose", 1);
  // behind_pose_lidar_publisher_ = nh.advertise<geometry_msgs::Pose2D>("/tracking/behind_vehicle/global_pose", 1);
  target_lost_publisher_ = nh.advertise<std_msgs::Bool>("/tracking/front_vehicle/is_lost", 1);
  // behind_lost_publisher_ = nh.advertise<std_msgs::Bool>("/tracking/behind_vehicle/is_lost", 1);
  relative_pose_publisher_ = nh.advertise<geometry_msgs::Pose2D>("/tracking/front_vehicle/local_pose", 1);
  estimated_speed_publisher_ = nh.advertise<std_msgs::Float32>("/tracking/front_vehicle/speed", 1);
  // behind_speed_publisher_ = nh.advertise<std_msgs::Float32>("/tracking/behind_vehicle/speed", 1);
  ros::Subscriber pcl_subscriber = nh.subscribe("/driver/hesai/pandar", 1, &Top::PclCallback, this);
  subscribers_.push_back(pcl_subscriber);
  ros::Subscriber local_pose_subscriber = nh.subscribe(
      "/localization/estimation", 1, &Top::VehiclePoseCallback, this);
  subscribers_.push_back(local_pose_subscriber);
  ros::Subscriber rematch_subscriber = nh.subscribe("/xboxone/rematch", 1, &Top::RematchCallback, this);
  subscribers_.push_back(rematch_subscriber);

  dynamic_reconfigure::Server<lidar_tracking::ParamLocConfig>::CallbackType f;
  f = boost::bind(&Top::ConfigCallback, this, _1, _2);
  server_.setCallback(f);
}

void Top::TrackBehind(const VPointCloud::Ptr& in) {
  t_now_behind = ros::Time::now().toSec();
  Eigen::VectorXd X_temp;

  if (tracker_behind_.GetTargetLost())
  {
    ROS_WARN("------------------- Start Rematch Because Behind Target Lost ---------------");
    ekf_init_behind = false;
    pose_init_behind = false;
    fusion_behind.Reset();
    tracker_behind_.ResetInitGuessPose();
  }
  else
  {
    if(ekf_init_behind)
    {
      X_temp = fusion_behind.readX(t_now_behind,true);

      geometry_msgs::Pose2D::Ptr init_pose(new geometry_msgs::Pose2D);
      init_pose->x = X_temp(0);
      init_pose->y = X_temp(1);
      init_pose->theta = X_temp(2);
      tracker_behind_.SetInitGuessPoseInMap(init_pose);
    }
  }

  // Measurement
  VPointCloud::Ptr pcl_roi = tracker_behind_.ExtractPointsInRoi(in);
  VPointCloud::Ptr pcl_no_ground(new VPointCloud);
  ground_filter_.remove_local_ground(pcl_roi, pcl_no_ground);

  geometry_msgs::Pose2D::Ptr target_pose_lidar_in_map =
    tracker_behind_.LocateTarget(pcl_no_ground);

  // Fusion update
  if(!ekf_init_behind)
  {
    fusion_behind.setDriveError(speed_drive_error_estimation, yaw_rate_drive_error_estimation);
    Eigen::Vector2d Z_vel;
    Eigen::Matrix2d R_vel;
    Z_vel << 0.001, 0.001;
    R_vel << pow(1.0, 2), 0,
          0, pow(1.0, 2);
    fusion_behind.velStateUpdate(Z_vel, R_vel);
    t_now_behind = ros::Time::now().toSec();
    ekf_init_behind = true;
  }

  Eigen::Vector3d Z_gps;
  Eigen::Matrix3d R_gps;
  Z_gps << target_pose_lidar_in_map->x, target_pose_lidar_in_map->y, target_pose_lidar_in_map->theta;
  if(!pose_init_behind) Z_gps << target_pose_lidar_in_map->x, target_pose_lidar_in_map->y, 0;
  R_gps << pow(pose_error_estimation, 2), 0, 0,
          0, pow(pose_error_estimation, 2), 0,
          0, 0, pow(heading_error_estimation, 2);

  fusion_behind.gpsStateUpdate(Z_gps, R_gps, t_now_behind);
  if(!pose_init_behind) pose_init_behind = true;
  Eigen::VectorXd X;
  X = fusion_behind.readX(t_now_behind,false);

  geometry_msgs::Pose2D::Ptr target_pose_fused(new geometry_msgs::Pose2D);
  target_pose_fused->x= X(0);
  target_pose_fused->y= X(1);
  target_pose_fused->theta= X(2);
  behind_pose_lidar_publisher_.publish(target_pose_fused);

  double estimated_speed = X(3);
  // ROS_INFO("estimated speed: %f cm/s", estimated_speed * 100);
  std_msgs::Float32 estimated_speed_msg;
  estimated_speed_msg.data = estimated_speed;
  behind_speed_publisher_.publish(estimated_speed_msg);

  // 显示匹配的目标点云
  if (is_display_) {
    if (!pcl_no_ground->empty()) {
      pcl_no_ground->header.frame_id = vehicle_frame_id_;
      pcl_target_behind_publisher_.publish(pcl_no_ground);
    }
  }
}

void Top::PclCallback(const VPointCloud::ConstPtr &rec) {
  if (is_vehicle_pose_ok_) {
    Timing("start", false);
    VPointCloud::Ptr in(new VPointCloud);
    //Transform pcl to vehicle frame
    try {
      pcl_ros::transformPointCloud(vehicle_frame_id_, *rec, *in, tf_listener_);
    } catch (tf::TransformException &e) {
      std::cout << e.what() << std::endl;
    }

    std_msgs::Bool target_lost_;
    target_lost_.data = tracker_.GetTargetLost();
    target_lost_publisher_.publish(target_lost_);
    // std_msgs::Bool behind_lost_;
    // behind_lost_.data = tracker_behind_.GetTargetLost();
    // behind_lost_publisher_.publish(behind_lost_);

    // Prediction update
    t_now = ros::Time::now().toSec();
    Eigen::VectorXd X_temp;

    if (tracker_.GetTargetLost())
    {
      ROS_WARN("------------------- Start Rematch Because Target Lost ---------------");
      is_vehicle_pose_ok_ = false;
      ekf_init = false;
      pose_init = false;
      fusion.Reset();
      tracker_.ResetInitGuessPose();
    }
    else
    {
      if(ekf_init)
      {
        X_temp = fusion.readX(t_now,true);

        geometry_msgs::Pose2D::Ptr init_pose(new geometry_msgs::Pose2D);
        init_pose->x = X_temp(0);
        init_pose->y = X_temp(1);
        init_pose->theta = X_temp(2);
        tracker_.SetInitGuessPoseInMap(init_pose);
      }
    }

    // Measurement
    VPointCloud::Ptr pcl_roi = tracker_.ExtractPointsInRoi(in);
    VPointCloud::Ptr pcl_no_ground(new VPointCloud);
    ground_filter_.remove_local_ground(pcl_roi, pcl_no_ground);

    geometry_msgs::Pose2D::Ptr target_pose_lidar_in_map =
      tracker_.LocateTarget(pcl_no_ground);

    // Fusion update
    if(!ekf_init)
    {
      fusion.setDriveError(speed_drive_error_estimation, yaw_rate_drive_error_estimation);
      Eigen::Vector2d Z_vel;
      Eigen::Matrix2d R_vel;
      Z_vel << 0.001, 0.001;
      R_vel << pow(1.0, 2), 0,
            0, pow(1.0, 2);
      fusion.velStateUpdate(Z_vel, R_vel);
      t_now = ros::Time::now().toSec();
      ekf_init = true;
    }

    Eigen::Vector3d Z_gps;
    Eigen::Matrix3d R_gps;
    Z_gps << target_pose_lidar_in_map->x, target_pose_lidar_in_map->y, target_pose_lidar_in_map->theta;
    if(!pose_init) Z_gps << target_pose_lidar_in_map->x, target_pose_lidar_in_map->y, 0;
    R_gps << pow(pose_error_estimation, 2), 0, 0,
            0, pow(pose_error_estimation, 2), 0,
            0, 0, pow(heading_error_estimation, 2);

    fusion.gpsStateUpdate(Z_gps, R_gps, t_now);
    if(!pose_init) pose_init = true;
    Eigen::VectorXd X;
    X = fusion.readX(t_now,false);

    geometry_msgs::Pose2D::Ptr target_pose_fused(new geometry_msgs::Pose2D);
    target_pose_fused->x= X(0);
    target_pose_fused->y= X(1);
    target_pose_fused->theta= X(2);
    target_pose_lidar_publisher_.publish(target_pose_fused);
    tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(X(0), X(1), 0.0));
    transform.setRotation(tf::createQuaternionFromYaw(X(2)));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                          map_frame_id_, "target"));

    double estimated_speed = X(3);
    // ROS_INFO("estimated speed: %f cm/s", estimated_speed * 100);
    std_msgs::Float32 estimated_speed_msg;
    estimated_speed_msg.data = estimated_speed;
    estimated_speed_publisher_.publish(estimated_speed_msg);

    geometry_msgs::Pose2D relative_pose = tracker_.GetRelativePose(target_pose_fused);
    relative_pose_publisher_.publish(relative_pose);

    // 显示匹配的目标点云
    if (is_display_) {
      if (!pcl_no_ground->empty()) {
        pcl_no_ground->header.frame_id = vehicle_frame_id_;
        pcl_target_publisher_.publish(pcl_no_ground);
      }
    }

    // TrackBehind(in);
    // Timing("end", true);
  }
}

void Top::VehiclePoseCallback(
    const cyber_msgs::LocalizationEstimateConstPtr &rec) {
  is_vehicle_pose_ok_ = true;
  geometry_msgs::Pose2D::Ptr vehicle_pose(new geometry_msgs::Pose2D);
  vehicle_pose->x = rec->pose.position.x;
  vehicle_pose->y = rec->pose.position.y;
  vehicle_pose->theta = tf::getYaw(rec->pose.orientation);
  tracker_.SetVehiclePose(vehicle_pose);
  tracker_behind_.SetVehiclePose(vehicle_pose);
}

void Top::RematchCallback(const std_msgs::Bool::ConstPtr &rec) {
  if(rec->data) {
    // 重匹配操作
    ROS_WARN("------------------- Start Rematch By Xboxone ---------------");
    is_vehicle_pose_ok_ = false;
    ekf_init = false;
    pose_init = false;
    fusion.Reset();
    tracker_.ResetInitGuessPose();

    ekf_init_behind = false;
    pose_init_behind = false;
    fusion_behind.Reset();
    tracker_behind_.ResetInitGuessPose();

  }
}

void Top::ConfigCallback(const lidar_tracking::ParamLocConfig &config, uint32_t level) {
  is_display_ = config.is_displaying_pcl;
  ground_filter_.window_width = config.window_width;
  ground_filter_.cov_thres = config.cov_thres;
  ground_filter_.grid_length = config.grid_length;
  ground_filter_.grid_width = config.grid_width;
  tracker_.SetRoi(config.area_length, config.area_width);
  tracker_behind_.SetRoi(config.area_length, config.area_width);
}
}  // namespace top
