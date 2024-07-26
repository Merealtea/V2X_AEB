#ifndef CANENCODER_H
#define CANENCODER_H

#include <ros/ros.h>

#include "cyber_msgs/BodyworkControl.h"
#include "cyber_msgs/BrakeFeedback.h"
#include "cyber_msgs/BrakeStateFeedback.h"
#include "cyber_msgs/SpeedFeedback.h"
#include "cyber_msgs/SteerFeedback.h"
#include "cyber_msgs/SteerStateFeedback.h"
#include "cyber_msgs/brakecmd.h"
#include "cyber_msgs/canframe.h"
#include "cyber_msgs/speedcmd.h"
#include "cyber_msgs/steercmd.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"

class can_encoder {
 public:
  can_encoder();
  void controlsourcecallback(const std_msgs::Int8& msg);
  void steercmdcallback(const cyber_msgs::steercmd& msg);
  void speedcmdcallback(const cyber_msgs::speedcmd& msg);
  void brakecmdcallback(const cyber_msgs::brakecmd& msg);
  void bodyworkcmdcallback(const cyber_msgs::BodyworkControl& msg);

  void canid_0x111_timer_callback(const ros::TimerEvent&);
  void canid_0x112_timer_callback(const ros::TimerEvent&);

  void getsteercallback(const cyber_msgs::SteerFeedback& msg);
  void getspeedcallback(const cyber_msgs::SpeedFeedback& msg);
  void getsteerstatecallback(const cyber_msgs::SteerStateFeedback& msg);
  void getbrakestatecallback(const cyber_msgs::BrakeStateFeedback& msg);
  void releasecallback(const std_msgs::Bool& msg);
  void autodrivecallback(const std_msgs::Bool& msg);
  void canid_0x111_msg_init(void);
  void canid_0x112_msg_init(void);
  void bodywork_control_msg_init(void);

 private:
  bool auto_drive_ = false;  //遥控硬件开关，相应topic下发之后进入自动驾驶模式
  bool auto_speed_ = false;
  bool auto_steer_ = false;
  bool bodywork_control_ = false;

  bool EPS_brake_controllable_ = false;

  double steer_feedback_;
  double speed_feedback_;

  // double last_time_ = 0.0;
  double last_steer_time_ = 0.0;
  double last_speed_time_ = 0.0;
  double last_brake_time_ = 0.0;

  bool manual_intervention_ = false;
  bool release_manual_intervention_ = false;

  cyber_msgs::canframe can_id_0x111_;  // acceleration and steering control
  cyber_msgs::canframe can_id_0x112_;  // vehicle bodywork control
  cyber_msgs::canframe bodywork_control_msg_;

  bool IsEPSControlable_ = false;

  cyber_msgs::SteerStateFeedback steer_state_fbmsg_;
  cyber_msgs::steercmd steer_cmd_;
  cyber_msgs::speedcmd speed_cmd_;
  cyber_msgs::brakecmd brake_cmd_;
  std_msgs::Int32 gas_value_msg_;

  ros::NodeHandle nh_;
  ros::Subscriber sub_steerstatefeedback_;
  ros::Subscriber sub_brakestatefeedback_;
  ros::Subscriber sub_steercmd_;
  ros::Subscriber sub_steerfeedback_;
  ros::Subscriber sub_speedcmd_;
  ros::Subscriber sub_brakecmd_;
  ros::Subscriber sub_speedfeedback_;
  ros::Subscriber sub_bodywork_control_;
  ros::Subscriber sub_gas_test_;
  ros::Subscriber sub_release_manual_intervention_;
  ros::Subscriber sub_auto_drive_;
  ros::Timer canid_0x111_timer_;
  ros::Timer canid_0x112_timer_;
  ros::Publisher pub_canWrite_0x111_;
  ros::Publisher pub_canWrite_0x112_;
};

#endif
