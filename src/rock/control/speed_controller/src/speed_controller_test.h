#pragma once

#include <dynamic_reconfigure/server.h>
#include <speed_controller/speed_controller_testConfig.h>

#include <algorithm>

#include "cyber_msgs/LocalTrajList.h"
#include "cyber_msgs/SpeedFeedback.h"
#include "cyber_msgs/brakecmd.h"
#include "cyber_msgs/speedcmd.h"
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
  void configCallback(speed_controller::speed_controller_testConfig& config,
                      uint32_t level);

 private:
  ros::NodeHandle* nh_;
  ros::Subscriber sub_vel_;
  ros::Subscriber sub_target_;
  ros::Subscriber sub_local_trajectory_;
  ros::Subscriber sub_stager_mode_;
  ros::Publisher pub_speed_cmd_;
  ros::Publisher pub_brake_cmd_;
  ros::Timer timer_;
  SpeedTest sp_;
  cyber_msgs::speedcmd speed_cmd_;
  cyber_msgs::brakecmd brake_cmd_;
  int cnt_;

  //---------参数服务相关变量------------
  dynamic_reconfigure::Server<speed_controller::speed_controller_testConfig>
      dr_srv;
  dynamic_reconfigure::Server<
      speed_controller::speed_controller_testConfig>::CallbackType cb;

  bool flag = false;
  int stager_mode_=0;

  void SpeedCallback(const cyber_msgs::SpeedFeedback& msg);
  void TargetCallback(const std_msgs::Float64& msg);
  void TimerCallback(const ros::TimerEvent&);
  void LocalTrajectoryCallback(
      const cyber_msgs::LocalTrajList::ConstPtr& path_in);
  void StagerModeCallback(const std_msgs::Int8ConstPtr& mode_in);
};
