#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class CompareOfflinePoseVisualizer {
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_pose_ours_;
  ros::Subscriber sub_pose_compare_A_;
  ros::Subscriber sub_pose_compare_B_;
  ros::Publisher pub_marker_ours_;
  ros::Publisher pub_marker_compare_A_;
  ros::Publisher pub_marker_compare_B_;
  tf2_ros::TransformBroadcaster br_;

  void poseOursCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void poseCompareACallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void poseCompareBCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

public:
  CompareOfflinePoseVisualizer(ros::NodeHandle node_handle,
                                ros::NodeHandle private_node_handle);
};
