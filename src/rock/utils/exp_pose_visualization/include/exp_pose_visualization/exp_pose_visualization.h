#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class AblationOfflinePoseVisualizer {
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_pose_normal_;
  ros::Subscriber sub_pose_ablation_;
  ros::Publisher pub_marker_normal_;
  ros::Publisher pub_marker_ablation_;
  tf2_ros::TransformBroadcaster br_;

  void poseNormalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void poseAblationCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

public:
  AblationOfflinePoseVisualizer(ros::NodeHandle node_handle,
                                ros::NodeHandle private_node_handle);
};
