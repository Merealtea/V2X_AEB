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

#ifndef TRACKER_TOP_H_
#define TRACKER_TOP_H_

#include <mrpt_bridge/mrpt_bridge.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include "e100_msgs/zigbee.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "cyber_msgs/LocalizationEstimate.h"

#include <dynamic_reconfigure/server.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>  // Converts PCL to ROS messages essentially.
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>

#include <chrono>
#include <vector>
#include <memory>
#include <map>
#include <string>

#include "velodyne.h"
#include "lidar_tracking/ParamLocConfig.h"
#include "ground_filter/ground_filter.h"
#include "tracker/tracker.h"
#include "ekf/ekf_pose.h"

namespace top {
class Top {
 public:
  Top(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~Top() {};

 private:
  void ConfigCallback(const lidar_tracking::ParamLocConfig &config, uint32_t level);
  void PclCallback(const VPointCloud::ConstPtr &rec);
  void TargetVelocityCallback(const e100_msgs::zigbee::ConstPtr &rec);
  void VehiclePoseCallback(
      const cyber_msgs::LocalizationEstimate::ConstPtr &rec);
  void RematchCallback(const std_msgs::Bool::ConstPtr &rec);
  void TrackBehind(const VPointCloud::Ptr& in);

  template<typename T>
  T X2(const T &in) const {return in * in;}
  double GetTimestamp() const {
    const auto t = std::chrono::system_clock::now().time_since_epoch();
    const auto ms = std::chrono::duration_cast<std::chrono::microseconds>(t);
    return ms.count() * 1e-6;
  }
  void Timing(const std::string &event, bool is_print = true) const {
    static double time_old = GetTimestamp();
    double time_now = GetTimestamp();
    if (is_print) {
      printf("%s:%.3fms\n", event.c_str(), (time_now - time_old) * 1000);
    }
    time_old = time_now;
  }

  std::string vehicle_frame_id_;
  std::string map_frame_id_;
  std::string target_frame_id_;
  tf::TransformListener tf_listener_;

  dynamic_reconfigure::Server<lidar_tracking::ParamLocConfig> server_;

  std::vector<ros::Subscriber> subscribers_;

  ros::Publisher pcl_target_publisher_;
  ros::Publisher pcl_target_behind_publisher_;
  ros::Publisher target_pose_lidar_publisher_;
  ros::Publisher target_lost_publisher_;
  ros::Publisher behind_lost_publisher_;
  ros::Publisher relative_pose_publisher_;
  ros::Publisher estimated_speed_publisher_;
  ros::Publisher behind_pose_lidar_publisher_;
  ros::Publisher behind_speed_publisher_;

  tracker::Tracker<VPoint> tracker_;
  tracker::Tracker<VPoint> tracker_behind_;
  Lidar::Ground_filter ground_filter_;
  bool is_display_;
  bool is_vehicle_pose_ok_ = false;
  bool ekf_init = false;
  bool ekf_init_behind = false;
  bool pose_init = false;
  bool pose_init_behind = false;
  All_EKF_Pose fusion;
  All_EKF_Pose fusion_behind;
  double t_now;
  double t_now_behind;
  double pose_error_estimation;
  double heading_error_estimation;
  double speed_error_estimation;
  double yaw_rate_error_estimation;
  double speed_drive_error_estimation;
  double yaw_rate_drive_error_estimation;
  double target_speed;

  tf::TransformBroadcaster tf_broadcaster;
};
}  // namespace top
#endif  // TRACKER_TOP_H_
