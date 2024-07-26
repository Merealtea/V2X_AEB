
#include <iostream>

#include "cyber_msgs/BrakeFeedback.h"
#include "cyber_msgs/BrakeStateFeedback.h"
#include "cyber_msgs/SpeedFeedback.h"
#include "cyber_msgs/SteerFeedback.h"
#include "cyber_msgs/SteerStateFeedback.h"
#include "cyber_msgs/canframe.h"
#include "ros/ros.h"

using namespace std;
class CanFrameDecoder {
 public:
  CanFrameDecoder();
  void canFrameCallback(const cyber_msgs::canframe &can_msg);
  void SpeedPubCallback(const ros::TimerEvent &);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_can_msg_;
  ros::Publisher pub_speed_;
  ros::Publisher pub_steer_;
  ros::Publisher pub_brake_;
  ros::Publisher pub_brake_state_;
  ros::Publisher pub_steer_state_;

  cyber_msgs::SpeedFeedback speed_msg_;

  int id_;
  int len_;
  unsigned char data_[8];
  bool speed_updated_ = false;
};

CanFrameDecoder::CanFrameDecoder() {
  sub_can_msg_ =
      nh_.subscribe("/can_frame", 1, &CanFrameDecoder::canFrameCallback, this);
  pub_steer_ =
      nh_.advertise<cyber_msgs::SteerFeedback>("/rock_can/steer_feedback", 1);
  pub_speed_ =
      nh_.advertise<cyber_msgs::SpeedFeedback>("/rock_can/speed_feedback", 1);
  pub_brake_ =
      nh_.advertise<cyber_msgs::BrakeFeedback>("/rock_can/brake_feedback", 1);
  pub_steer_state_ = nh_.advertise<cyber_msgs::SteerStateFeedback>(
      "/rock_can/steer_state_feedback", 1);
  pub_brake_state_ = nh_.advertise<cyber_msgs::BrakeStateFeedback>(
      "/rock_can/brake_state_feedback", 1);

  ros::Timer timer = nh_.createTimer(ros::Duration(0.02),
                                     &CanFrameDecoder::SpeedPubCallback, this);
  ros::NodeHandle nh_priv_("~");
  ros::spin();
}

void CanFrameDecoder::SpeedPubCallback(const ros::TimerEvent &) {
  if (speed_updated_) {
    pub_speed_.publish(speed_msg_);
  }
  speed_updated_ = false;
}

void CanFrameDecoder::canFrameCallback(const cyber_msgs::canframe &can_msg) {
  id_ = can_msg.id;
  len_ = can_msg.len;
  float speed_cmps_;
  float speed_kmph_;
  float steer_0p1d_;
  for (int i = 0; i < len_; i++) data_[i] = can_msg.data[i];

  cyber_msgs::SteerFeedback steer_msg;
  cyber_msgs::BrakeFeedback brake_msg;
  cyber_msgs::BrakeStateFeedback brake_state_msg;
  cyber_msgs::SteerStateFeedback steer_state_msg;

  switch (id_) {
    case 0x2a3:  // steer_state

      steer_state_msg.header.stamp = can_msg.header.stamp;
      steer_state_msg.EPS_EPSFailed = (data_[1] >> 7) & 0x01;
      steer_state_msg.EPS_ACUAbortFeedback = ((data_[1] & 0x70) >> 4);
      steer_state_msg.EPS_ACUEpasFailed = (data_[1] >> 1) & 0x01;
      steer_state_msg.EPS_ACUControllerFeedback = (data_[3] >> 5) & 0x01;
      steer_state_msg.SteeringTorque = data_[2] * 0.1794 - 22.78;  // scale : Nm

      pub_steer_state_.publish(steer_state_msg);
      break;

    case 0x2a4:  // steer

      steer_msg.header.stamp = can_msg.header.stamp;
      steer_msg.SteerAngle = ((data_[0] & 0x3f) * 256 + data_[1]) * 0.15 -
                             1228.65;  // scale : degree
      steer_msg.SteerAngulaSpeed =
          ((data_[3] & 0x03) * 256 + data_[4]) * 5 - 2555;  // scale : deg/s

      pub_steer_.publish(steer_msg);
      break;

    case 0x108:  // gear
      speed_msg_.gear = ((data_[4] & 0x1e) >> 1);
      break;

    case 0x21b:  // speed

      speed_msg_.header.stamp = can_msg.header.stamp;
      speed_msg_.IsValid = (data_[4] >> 5) & 0x01;
      speed_msg_.speed_kmh = ((data_[4] & 0x1f) * 256 + data_[5]) * 0.05625;
      speed_msg_.speed_cms = speed_msg_.speed_kmh * 1000 / 36.0;

      speed_updated_ = true;

      // pub_speed_.publish(speed_msg);
      break;

    case 0x25b:  // brake

      brake_msg.header.stamp = can_msg.header.stamp;
      brake_msg.BrakeValid = (data_[0] >> 7) & 0x01;
      brake_msg.BrakePressure = ((data_[0] & 0x0f) * 256 + data_[1]) * 0.1;

      pub_brake_.publish(brake_msg);
      break;

    case 0x27a:  // stop_control

      brake_state_msg.header.stamp = can_msg.header.stamp;
      brake_state_msg.ESP_QDCACC = (data_[3] >> 1) & 0x03;
      pub_brake_state_.publish(brake_state_msg);
      break;

    default:
      break;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "can_decoder");
  CanFrameDecoder DecoderObject;

  return 0;
}
