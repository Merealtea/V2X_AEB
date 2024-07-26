/**
  *top.h
  *brief:top layer of localization pipeline
  *author:Chen Xiaofeng
  *date:20191028
  **/

#include <ros/ros.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/TimeReference.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/Marker.h>
#include <cyber_msgs/LocalizationEstimate.h>
#include <cyber_msgs/SpeedFeedback.h>
#include <cyber_msgs/SteerFeedback.h>
#include <cyber_msgs/V2VPacket.h>
#include <std_msgs/Float64.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "common/map_frame.h"
#include "ekf/ekf_pose.h"
#include "common/wgs84_to_utm.h"

#include <iostream>

#ifndef TOP_H
#define TOP_H

class Top
{
public:
  Top(ros::NodeHandle &nh, ros::NodeHandle &nh_priv);
  ~Top();

private:
  bool have_inited_vel_ = false;
  bool have_inited_gps_ = false;
  bool command_no_gps_ = false;
  bool gps_with_problem_ = false;
  bool localization_debug_ = false;

  double curr_vel_ = 0;
  double curr_yaw_rate_ = 0;
  geometry_msgs::Vector3 angular_velocity_;
  geometry_msgs::Vector3 linear_acceleration_;

  All_EKF_Pose fusion_;

  // parameters of vehicle
  double steer_ratio_ = 15.5; // steer to wheel ratio
  double wheelbase_ = 2.56;  // vehicle wheelbase

  // parameters for ekf
  double slam_fix_param_ = 0.3;
  double slam_error_param_ = 0.2;
  double imu_err_ = 0.004; //rad velocity of IMU, unit: rad/s
  double speed_err_ = 0.001; // unit: m/s
  double steer_err_ = 0.001; // unit: m/s
  double gps_yaw_err_fix_ = 0.2 * M_PI / 180.0; //from gps, unit: rad
  double gps_yaw_err_normal_ = 2.0 * M_PI / 180.0; //from gps, unit: rad
  double slam_err_fix_ = 0.1; //slam error, unit: m
  double slam_err_normal_ = 1.0; //slam error, unit: m
  double slam_yaw_err_fix_ = 0.5 * M_PI / 180.0; //from slam, unit: rad
  double slam_yaw_err_normal_ = 5 * M_PI / 180.0; //from slam, unit: rad

  double t_vel_ = 0.0;
  double t_gps_ = 0.0;
  double t_slam_ = 0.0;
  double t_output_ = 0.0;
  double t_diff_ = 0.0;

  double global_heading_ = 0;

  int slam_pose_cnt = 0;
  int gps_pose_cnt = 0;
  double obs_yaw = 0;

  std::string vehicle_frame_id_;

  std::vector<Eigen::Vector3d> v_slam_pose;
  std::vector<Eigen::Vector3d> v_gps_pose;

  ros::Timer filter_timer_;

  ros::Publisher pub_localization_estimation_;
  ros::Publisher pub_gps_marker_;
  ros::Publisher pub_gps_pose_;
  ros::Publisher pub_target_gps_pose_;
  ros::Publisher pub_slam_marker_;
  ros::Publisher pub_slam_pose_;
  ros::Publisher pub_filter_marker_;
  ros::Publisher pub_filter_pose_;
  ros::Publisher pub_gps_vel_;
  ros::Publisher pub_gps_angle_;
  ros::Publisher pub_slam_angle_;
  ros::Publisher pub_output_angle_;
  ros::Publisher pub_slam_pose_xyz_;
  ros::Publisher pub_gps_pose_xyz_;

  ros::Subscriber vel_gps_sub_;
  // ros::Subscriber vel_can_sub_;
  // ros::Subscriber steer_can_sub_;
  ros::Subscriber command_sub_;
  ros::Subscriber target_gps_sub_;

  typedef message_filters::sync_policies::ApproximateTime<cyber_msgs::SpeedFeedback,cyber_msgs::SteerFeedback > CANPolicy;
  message_filters::Subscriber<cyber_msgs::SpeedFeedback>* sub_vel_can_;
  message_filters::Subscriber<cyber_msgs::SteerFeedback>* sub_steer_can_;
  message_filters::Synchronizer<CANPolicy>* can_sync_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix,sensor_msgs::Imu > GPSPolicy;
  message_filters::Subscriber<sensor_msgs::NavSatFix>* sub_gps_fix_;
  message_filters::Subscriber<sensor_msgs::Imu>* sub_imu_;
  message_filters::Synchronizer<GPSPolicy>* gps_sync_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, sensor_msgs::Imu> SlamPolicy;
  message_filters::Subscriber<sensor_msgs::NavSatFix>* sub_slam_fix_;
  message_filters::Subscriber<sensor_msgs::Imu>* sub_slam_heading_;
  message_filters::Synchronizer<SlamPolicy>* slam_sync_;

  geometry_msgs::PoseArray gps_poses_;
  geometry_msgs::PoseArray target_gps_poses_;
  visualization_msgs::Marker gps_marker_traj_;
  tf::TransformBroadcaster br_gps_;
  geometry_msgs::PoseArray slam_poses_;
  visualization_msgs::Marker slam_marker_traj_;
  tf::TransformBroadcaster br_slam_;
  geometry_msgs::PoseArray filter_poses_;
  visualization_msgs::Marker filter_marker_traj_;
  tf::TransformBroadcaster br_filter_;
  tf2_ros::StaticTransformBroadcaster static_br_;
  std::ofstream offile;

  double get_time_now();
  void command_callback(const std_msgs::BoolConstPtr &bool_in);
  void vel_gps_callback(const geometry_msgs::TwistWithCovarianceStampedConstPtr &vel_gps_in);
  void vel_can_callback(const cyber_msgs::SpeedFeedbackConstPtr &vel_can_in,const cyber_msgs::SteerFeedbackConstPtr &steer_can_in);
  void imu_callback(const sensor_msgs::ImuConstPtr &imu_in);
  void gps_callback(const sensor_msgs::NavSatFixConstPtr &gps_in, const sensor_msgs::ImuConstPtr &imu_in);
  void target_gps_callback(const cyber_msgs::V2VPacketConstPtr &gps_in);
  // void slam_callback(const sensor_msgs::NavSatFixConstPtr &slam_in, const sensor_msgs::ImuConstPtr &heading_in);
  void filter_callback(const ros::TimerEvent&);
  Eigen::VectorXd slam_fusion_result();
  Eigen::VectorXd gps_fusion_result();
}; //class Top

#endif
