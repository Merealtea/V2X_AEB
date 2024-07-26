#ifndef _PURE_PURSUIT_H
#define _PURE_PURSUIT_H

#include <stdio.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <vector>

#include "cyber_msgs/LocalTrajList.h"
#include "cyber_msgs/LocalTrajPoint.h"
#include "cyber_msgs/LocalizationEstimate.h"
#include "cyber_msgs/SteerFeedback.h"
#include "cyber_msgs/steercmd.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "tf/transform_datatypes.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include "steer_controller/pursuit_paramConfig.h"

using namespace std;

const int STEER_ERROR_QUEUE_SIZE=50;

class PurePursuit {
 public:
  PurePursuit();
  ~PurePursuit();

 private:
  //---------定义成员变量----------------
  bool path_flag;

  double reference_distance_last;
  int current_point_global;  //车辆（后轮）当前位置点
  double steer_feedback;
  double steer_error;
  double inter_steer_error;
  double former_steer_cmd;
  double former_reference_distance;
  double steer_cmd;
  double final_steer_cmd;
  queue<double> steer_error_queue;

  // cyber_msgs::steercmd steer_cmd_;

  double Min_ref_speed;
  double Min_preview_distance;
  double K_ref_0_20;
  double K_ref_20_40;
  double K_ref_40_60;
  double Kp_error;
  double Ki_error;
  double filter_param;
  double Kp_wheel;

  double wheelbase;
  double wheel_max;   // 0.1 度
  double wheel_zero;  //方向盘零位，向左为负，单位0.1 度

  double current_yaw;
  double current_vel;
  geometry_msgs::PoseStamped current_pose;
  geometry_msgs::PoseStamped reference_pose;
  geometry_msgs::PoseStamped target_pose;
  geometry_msgs::PoseArray target_path_global;

  //--------定义订阅者和发布者-----------
  ros::NodeHandle nh;
  ros::Publisher pub_steer_cmd;
  ros::Publisher pub_reference_pose;
  ros::Publisher pub_target_pose;
  ros::Publisher pub_control_target_point_;

  ros::Subscriber sub_local_trajectory;
  ros::Subscriber sub_pose;
  ros::Subscriber sub_steer;

  //---------参数服务相关变量------------
  dynamic_reconfigure::Server<steer_controller::pursuit_paramConfig> dr_srv;
  dynamic_reconfigure::Server<
      steer_controller::pursuit_paramConfig>::CallbackType cb;

  //---------定义成员函数----------------
  void localTrajectoryCallback(
      const cyber_msgs::LocalTrajList::ConstPtr& path_in);
  void poseCallback(const cyber_msgs::LocalizationEstimate::ConstPtr& pose_in);
  void SteerCallback(const cyber_msgs::SteerFeedbackConstPtr& steer_in);
  void configCallback(steer_controller::pursuit_paramConfig& config,
                      uint32_t level);
  void ShowTargetPoint(const geometry_msgs::PoseStamped in_target_point);
};

#endif
