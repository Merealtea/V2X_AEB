#ifndef _STEER_CONTROLLER_H_
#define _STEER_CONTROLLER_H_

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/String.h"

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
//#include <controller/controllerConfig.h>
#include <speed_controller/speed_controllerConfig.h>

#include <deque>

#include "cyber_msgs/LocalTrajList.h"
#include "cyber_msgs/LocalizationEstimate.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"

const int THROTTLE_MAX = 3600;  //高架需改成1800
const int THROTTLE_MIN = 1700;
const double SPEED_MAX = 40;  // 15km/h
const double SPEED_MIN = 0;
const int BRAKE_MAX = 100;
const int BRAKE_MIN = 1;
const int BRAKE_MAX_GAP =
    3;  // 两帧之间最大刹车增加直用于平滑刹车，越小越平滑，越小越危险, > 1
struct Speed {
  double kp;
  double ki;
  double kd;
  double kv;
  double ka;

  double speed_curr;
  double speed_ref;
  double err_curr;
  double err_sum;

  int max;
  int min;
  int state;
  double cmd;
  int brake_cmd;
  int last_brake_cmd;

  int emergency_slow_brake;
  int emergency_stop_brake;
};

class SpeedController {
 public:
  explicit SpeedController(ros::NodeHandle* nh);
  ~SpeedController() {}

  void localTrajectoryCallback(
      const cyber_msgs::LocalTrajList::ConstPtr& path_in);

  void velocityCallback(cyber_msgs::LocalizationEstimate vel_in);

  void StartCallback(const std_msgs::Int8ConstPtr& start_in);

  void FinishedCallback(const std_msgs::BoolConstPtr& finished_in);

  void emergencyTypeCallback(std_msgs::Int8 emergency_type_in);

  void configCallback(speed_controller::speed_controllerConfig& config,
                      uint32_t level);

  void stagerModeCallback(const std_msgs::Int8 mode_in);

  void cal_speed_cmd(Speed& sp);
  void publish_cmd(const Speed Sp);
  void print_info();

  void Timer1Callback(const ros::TimerEvent&);

 private:
  //--------定义订阅者和发布者-----------
  ros::NodeHandle* nh_;

  ros::Subscriber sub_local_trajectory;
  ros::Subscriber sub_velocity;
  ros::Subscriber sub_emergency_type;
  ros::Subscriber sub_stager_mode;

  ros::Subscriber sub_parking_start_;
  ros::Subscriber sub_parking_finished_;

  ros::Publisher pub_speed_cmd;

  //---------参数服务相关变量------------
  dynamic_reconfigure::Server<speed_controller::speed_controllerConfig> dr_srv;
  dynamic_reconfigure::Server<
      speed_controller::speed_controllerConfig>::CallbackType cb;

  //---------定义成员变量----------------
  Speed sp_;
  double min_speed_;
  int stager_mode_;
  unsigned int cnt_;
  ros::Timer timer_;

  bool obstacle_emergency_ = false;
  bool obstacle_close_ = false;
  bool bparking_ = false;
};

#endif
