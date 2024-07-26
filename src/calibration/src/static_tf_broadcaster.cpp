/**
 * @file static_tf_broadcaster.cpp
 * @author Chen Hongxin (angelochen@live.cn)
 * @brief
 * @version 0.1
 * @date 2022-10-17
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <string>

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>

#include "common/utils/find_path.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "calibration_tf_broadcaster");
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;
  std::string config_file_path =
      expand_catkin_ws("/src/calibration/params/calibration_params.yaml");
  YAML::Node config_node = YAML::LoadFile(config_file_path);
  for (const auto& item : config_node["calibration"]) {
    std::string frame_name = item.first.as<std::string>();
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "base_link";
    static_transformStamped.child_frame_id = frame_name;
    static_transformStamped.transform.translation.x =
        item.second["x"].as<double>();
    static_transformStamped.transform.translation.y =
        item.second["y"].as<double>();
    static_transformStamped.transform.translation.z =
        item.second["z"].as<double>();
    tf2::Quaternion quat;
    quat.setRPY(item.second["roll"].as<double>(),
                item.second["pitch"].as<double>(),
                item.second["yaw"].as<double>());
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();
    static_broadcaster.sendTransform(static_transformStamped);
  }

  ros::spin();
  return 0;
};
