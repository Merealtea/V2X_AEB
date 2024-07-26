#include "exp_pose_vis_compare/exp_pose_vis_compare.h"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

const double ARROW_LEN = 2.0;
const double ARROW_SCALE_X = 0.15;
const double ARROW_SCALE_Y = 0.3;
const double SPHERE_SCALE = 0.5;
const double Z = -1.3;

CompareOfflinePoseVisualizer::CompareOfflinePoseVisualizer(
    ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
    : nh_(node_handle), pnh_(private_node_handle) {
  sub_pose_ours_ =
      nh_.subscribe("/tracking/pose_lidar/ours", 10,
                    &CompareOfflinePoseVisualizer::poseOursCallback, this);
  sub_pose_compare_A_ =
      nh_.subscribe("/tracking/pose_lidar/compare_A", 10,
                    &CompareOfflinePoseVisualizer::poseCompareACallback, this);
  sub_pose_compare_B_ =
      nh_.subscribe("/tracking/pose_lidar/compare_B", 10,
                    &CompareOfflinePoseVisualizer::poseCompareBCallback, this);
  pub_marker_ours_ = nh_.advertise<visualization_msgs::Marker>(
      "/tracking/visualization/ours", 10);
  pub_marker_compare_A_ = nh_.advertise<visualization_msgs::Marker>(
      "/tracking/visualization/compare_A", 10);
  pub_marker_compare_B_ = nh_.advertise<visualization_msgs::Marker>(
      "/tracking/visualization/compare_B", 10);
}

void CompareOfflinePoseVisualizer::poseOursCallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {
  ROS_INFO("receive ours pose");
  double x = msg->pose.position.x, y = msg->pose.position.y;
  double roll, pitch, yaw;
  tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
                    msg->pose.orientation.z, msg->pose.orientation.w);
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  visualization_msgs::Marker marker_1, marker_2;
  marker_1.header.frame_id = "Pandar40";
  marker_1.header.stamp = msg->header.stamp;
  marker_1.ns = "compare";
  marker_1.id = 0;
  marker_1.action = visualization_msgs::Marker::ADD;
  marker_1.type = visualization_msgs::Marker::ARROW;

  marker_1.scale.x = ARROW_SCALE_X;
  marker_1.scale.y = ARROW_SCALE_Y;

  marker_1.color.r = 0.0;
  marker_1.color.g = 1.0;
  marker_1.color.b = 0.0;
  marker_1.color.a = 1.0;

  geometry_msgs::Point p_1, p_2;
  p_1.x = x;
  p_1.y = y;
  p_1.z = Z;
  p_2.x = x + ARROW_LEN * std::cos(yaw);
  p_2.y = y + ARROW_LEN * std::sin(yaw);
  p_2.z = Z;
  marker_1.points.push_back(p_1);
  marker_1.points.push_back(p_2);

  marker_2.header.frame_id = "Pandar40";
  marker_2.header.stamp = msg->header.stamp;
  marker_2.ns = "compare";
  marker_2.id = 1;
  marker_2.action = visualization_msgs::Marker::ADD;
  marker_2.type = visualization_msgs::Marker::SPHERE;
  marker_2.pose.position.x = x;
  marker_2.pose.position.y = y;
  marker_2.pose.position.z = Z;
  marker_2.scale.x = marker_2.scale.y = marker_2.scale.z = SPHERE_SCALE;
  marker_2.color.r = 0.0;
  marker_2.color.g = 1.0;
  marker_2.color.b = 0.0;
  marker_2.color.a = 1.0;

  pub_marker_ours_.publish(marker_1);
  pub_marker_ours_.publish(marker_2);

  // tf
  geometry_msgs::TransformStamped t;
  t.header.stamp = ros::Time::now();
  t.header.frame_id = "Pandar40";
  t.child_frame_id = "ours";
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = Z;
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();
  br_.sendTransform(t);
}

void CompareOfflinePoseVisualizer::poseCompareACallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {
  ROS_INFO("receive A pose");
  double x = msg->pose.position.x, y = msg->pose.position.y;
  double roll, pitch, yaw;
  tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
                    msg->pose.orientation.z, msg->pose.orientation.w);
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  visualization_msgs::Marker marker_1, marker_2;
  marker_1.header.frame_id = "Pandar40";
  marker_1.header.stamp = msg->header.stamp;
  marker_1.ns = "compare";
  marker_1.id = 2;
  marker_1.action = visualization_msgs::Marker::ADD;
  marker_1.type = visualization_msgs::Marker::ARROW;

  marker_1.scale.x = ARROW_SCALE_X;
  marker_1.scale.y = ARROW_SCALE_Y;

  marker_1.color.r = 0.0;
  marker_1.color.g = 0.0;
  marker_1.color.b = 1.0;
  marker_1.color.a = 1.0;

  geometry_msgs::Point p_1, p_2;
  p_1.x = x;
  p_1.y = y;
  p_1.z = Z;
  p_2.x = x + ARROW_LEN * std::cos(yaw);
  p_2.y = y + ARROW_LEN * std::sin(yaw);
  p_2.z = Z;
  marker_1.points.push_back(p_1);
  marker_1.points.push_back(p_2);

  marker_2.header.frame_id = "Pandar40";
  marker_2.header.stamp = msg->header.stamp;
  marker_2.ns = "compare";
  marker_2.id = 3;
  marker_2.action = visualization_msgs::Marker::ADD;
  marker_2.type = visualization_msgs::Marker::SPHERE;
  marker_2.pose.position.x = x;
  marker_2.pose.position.y = y;
  marker_2.pose.position.z = Z;
  marker_2.scale.x = marker_2.scale.y = marker_2.scale.z = SPHERE_SCALE;
  marker_2.color.r = 0.0;
  marker_2.color.g = 0.0;
  marker_2.color.b = 1.0;
  marker_2.color.a = 1.0;

  pub_marker_compare_A_.publish(marker_1);
  pub_marker_compare_A_.publish(marker_2);

  // tf
  geometry_msgs::TransformStamped t;
  t.header.stamp = ros::Time::now();
  t.header.frame_id = "Pandar40";
  t.child_frame_id = "A";
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = Z;
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();
  br_.sendTransform(t);
}

void CompareOfflinePoseVisualizer::poseCompareBCallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {
  ROS_INFO("receive B pose");
  double x = msg->pose.position.x, y = msg->pose.position.y;
  double roll, pitch, yaw;
  tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
                    msg->pose.orientation.z, msg->pose.orientation.w);
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  visualization_msgs::Marker marker_1, marker_2;
  marker_1.header.frame_id = "Pandar40";
  marker_1.header.stamp = msg->header.stamp;
  marker_1.ns = "compare";
  marker_1.id = 4;
  marker_1.action = visualization_msgs::Marker::ADD;
  marker_1.type = visualization_msgs::Marker::ARROW;

  marker_1.scale.x = ARROW_SCALE_X;
  marker_1.scale.y = ARROW_SCALE_Y;

  marker_1.color.r = 1.0;
  marker_1.color.g = 0.0;
  marker_1.color.b = 0.0;
  marker_1.color.a = 1.0;

  geometry_msgs::Point p_1, p_2;
  p_1.x = x;
  p_1.y = y;
  p_1.z = Z;
  p_2.x = x + ARROW_LEN * std::cos(yaw);
  p_2.y = y + ARROW_LEN * std::sin(yaw);
  p_2.z = Z;
  marker_1.points.push_back(p_1);
  marker_1.points.push_back(p_2);

  marker_2.header.frame_id = "Pandar40";
  marker_2.header.stamp = msg->header.stamp;
  marker_2.ns = "compare";
  marker_2.id = 5;
  marker_2.action = visualization_msgs::Marker::ADD;
  marker_2.type = visualization_msgs::Marker::SPHERE;
  marker_2.pose.position.x = x;
  marker_2.pose.position.y = y;
  marker_2.pose.position.z = Z;
  marker_2.scale.x = marker_2.scale.y = marker_2.scale.z = SPHERE_SCALE;
  marker_2.color.r = 1.0;
  marker_2.color.g = 0.0;
  marker_2.color.b = 0.0;
  marker_2.color.a = 1.0;

  pub_marker_compare_B_.publish(marker_1);
  pub_marker_compare_B_.publish(marker_2);

  // tf
  geometry_msgs::TransformStamped t;
  t.header.stamp = ros::Time::now();
  t.header.frame_id = "Pandar40";
  t.child_frame_id = "B";
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = Z;
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();
  br_.sendTransform(t);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "exp_pose_vis_compare");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");

  CompareOfflinePoseVisualizer viz(node_handle, private_node_handle);
  ros::spin();
}