#pragma once

#include <algorithm>
 
#include "cyber_msgs/brakecmd.h"
#include "cyber_msgs/speedcmd.h"
#include "hycan_msgs/Localization.h"
#include "hycan_msgs/DetectionResults.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"

const int TORQU_MAX = 3000;  //底层最大可到5000
const int TORQU_MIN = 0;
const double SPEED_MAX = 36;  // 36km/h
const double SPEED_MIN = 0;
const double BRAKE_MAX = 0.0;
const double BRAKE_MIN = -4.0;
const double BRAKE_MAX_GAP =
    -0.04;  // 两帧之间最大刹车增加直用于平滑刹车，绝对值越小越平滑，也越危险

struct SpeedTest {
  double kp;
  double ki;
  double kd;
  double kv;
  double ka;

  // scale of speeds : m/s
  double speed_curr;
  double speed_ref;
  double err_curr;
  double err_sum;
  // the state of speed_feedback
  bool state;

  double max;
  double min;

  // scale : Nm  from[0,5000]
  double cmd;

  // scale of brake_cmds: cm/s2, must be minus
  double brake_cmd;
  double last_brake_cmd;
};

class SpeedControllerTest {
 public:
  SpeedControllerTest(ros::NodeHandle* nh);
  ~SpeedControllerTest() {}

 private:
  ros::NodeHandle* nh_;
  ros::Subscriber localization_sub_;
  ros::Subscriber detection_sub_;
  ros::Publisher pub_brake_cmd_;
  ros::Publisher pub_speed_cmd_;
  ros::Timer timer_;
  SpeedTest sp_;

  int8_t cnt_;
  cyber_msgs::speedcmd speed_cmd_;
  cyber_msgs::brakecmd brake_cmd_;
  double utm_x, utm_y, heading;

  //---------参数服务相关变量------------

  bool flag = false;
  int stager_mode_=0;

  void LocalizationCallback(const hycan_msgs::Localization& msg);
  void DectectionCallback(
      const hycan_msgs::DetectionResults& detection_result);
};
