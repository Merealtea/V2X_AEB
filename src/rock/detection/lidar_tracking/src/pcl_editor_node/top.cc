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

#include "pcl_editor_node/top.h"

namespace top {
Top::Top(ros::NodeHandle nh, ros::NodeHandle pnh):
  file_name_("crop_out.ply"), is_saved_(true) {
  std::string topic;
  int queue_size;

  Timing("", false);
  pnh.param("ros_default_queue_size", queue_size, 1);

  pnh.param("target_frame_id", target_frame_id_, std::string("target"));
  pnh.param("vehicle_frame_id", vehicle_frame_id_, std::string("base_link"));

  pnh.param("file_path", file_path_, std::string("~"));
  if (file_path_.back() != '/') {
    file_path_.push_back('/');
  }

  pnh.param("pcl_target_topic", topic, std::string("/target_points"));
  pcl_target_publisher_ = nh.advertise<VPointCloud>(topic, queue_size);

  pnh.param("pcl_raw_topic", topic, std::string("pcl_raw"));
  pcl_raw_publisher_ = nh.advertise<VPointCloud>(topic, queue_size);

  pnh.param("pcl_topic", topic, std::string("pcl"));
  ros::Subscriber pcl_subscriber =
    nh.subscribe(topic, queue_size, &Top::PclCallback, this);
  subscribers_.push_back(pcl_subscriber);

  timer_ = nh.createTimer(ros::Duration(0.1), &Top::TimerCallback, this);

  dynamic_reconfigure::Server<lidar_tracking::PclEditorConfig>::CallbackType f;
  f = boost::bind(&Top::ConfigCallback, this, _1, _2);
  server_.setCallback(f);
}
void Top::PclCallback(const VPointCloud::ConstPtr &rec) {
  const std::string full_name = file_path_ + file_name_;
  if (!full_name.empty() && (full_name.length() > 4)) {
    VPointCloudPtr in(new VPointCloud(*rec));
    in->header.frame_id = target_frame_id_;
    in->header.stamp = pcl_conversions::toPCL(old_timestamp_);
    // in->header.stamp = pcl_conversions::toPCL(ros::Time::now());
    VPointCloudPtr pcl_in_vehicle(new VPointCloud);
    //Transform pcl to vehicle frame
    try {
      pcl_ros::transformPointCloud(vehicle_frame_id_, *in, *pcl_in_vehicle, tf_listener_);
    } catch (tf::TransformException &e) {
      std::cout << e.what() << std::endl;
    }

    VPointCloudPtr out = box_.Filter(pcl_in_vehicle);
    out->header.frame_id = vehicle_frame_id_;
    out->header.stamp = pcl_conversions::toPCL(ros::Time::now());
    pcl_raw_publisher_.publish(in);
    if(!out->points.empty())
      pcl_target_publisher_.publish(out);

    if (!is_saved_) {
      if (full_name.substr(full_name.length() - 4) == std::string(".pcd")) {
        pcl::io::savePCDFileASCII(full_name, *out);
      } else if (full_name.substr(full_name.length() - 4) == std::string(".ply")) {
        pcl::io::savePLYFileASCII(full_name, *out);
      } else {
        ROS_INFO("Unknown file format");
      }
    }
  } else {
    ROS_INFO("Failed to save file:%s", full_name.c_str());
  }
}
void Top::ConfigCallback(const lidar_tracking::PclEditorConfig &config, uint32_t level) {
  box_.SetBox(config.crop_box_min_x,
              config.crop_box_min_y,
              config.crop_box_min_z,
              config.crop_box_max_x,
              config.crop_box_max_y,
              config.crop_box_max_z);

  target_x_ = config.target_pose_x;
  target_y_ = config.target_pose_y;
  target_z_ = config.target_pose_z;
  target_roll_ = config.target_pose_roll;
  target_pitch_ = config.target_pose_pitch;
  target_yaw_ = config.target_pose_yaw;

  if (config.file_name == file_name_) {
    is_saved_ = true;
  } else if (is_saved_ == true) {
    is_saved_ = false;
    file_name_ = config.file_name;
  }
}
void Top::TimerCallback(const ros::TimerEvent &event) {
  //pub target frame
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(target_x_, target_y_, target_z_) );
  tf::Quaternion q;
  q.setRPY(target_roll_, target_pitch_, target_yaw_);
  transform.setRotation(q);
  old_timestamp_ = ros::Time::now();
  old_timestamp_ = pcl_conversions::fromPCL(pcl_conversions::toPCL(old_timestamp_));
  tf_broadcaster_.sendTransform(tf::StampedTransform(transform,
                                old_timestamp_,
                                vehicle_frame_id_,
                                target_frame_id_));
}
}  // namespace top
