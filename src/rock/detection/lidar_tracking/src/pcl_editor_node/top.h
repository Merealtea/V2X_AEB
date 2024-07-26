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

#ifndef PCL_EDITOR_NODE_TOP_H_
#define PCL_EDITOR_NODE_TOP_H_

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>  // Converts PCL to ROS messages essentially.
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <dynamic_reconfigure/server.h>
#include <lidar_tracking/PclEditorConfig.h>

#include <chrono>
#include <vector>

#include "box_roi.h"

namespace top {
using VPoint = pcl::PointXYZI;
using VPointCloud = pcl::PointCloud<VPoint>;
using VPointCloudPtr = pcl::PointCloud<VPoint>::Ptr;
class Top {
 public:
  Top(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~Top() {};

 private:
  void PclCallback(const VPointCloud::ConstPtr &rec);
  void ConfigCallback(const lidar_tracking::PclEditorConfig &config, uint32_t level);
  void TimerCallback(const ros::TimerEvent &event);
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

  std::string target_frame_id_;
  std::string vehicle_frame_id_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;

  ros::Timer timer_;

  std::vector<ros::Subscriber> subscribers_;

  ros::Publisher pcl_target_publisher_;
  ros::Publisher pcl_raw_publisher_;

  dynamic_reconfigure::Server<lidar_tracking::PclEditorConfig> server_;

  std::string file_name_;
  std::string file_path_;
  bool is_saved_;
  filters::BoxRoi<VPoint> box_;
  double target_x_;
  double target_y_;
  double target_z_;
  double target_roll_;
  double target_pitch_;
  double target_yaw_;
  ros::Time old_timestamp_;
};
}  // namespace top
#endif  // PCL_EDITOR_NODE_TOP_H_
