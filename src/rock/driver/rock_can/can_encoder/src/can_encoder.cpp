#include "driver/rock_can/can_encoder/src/can_encoder.h"

can_encoder::can_encoder() {
  ros::NodeHandle nh_priv("~");
  nh_priv.param<bool>("auto_speed", auto_speed_, false);
  nh_priv.param<bool>("auto_steer", auto_steer_, false);
  nh_priv.param<bool>("bodywork_control", bodywork_control_, false);

  canid_0x111_msg_init();
  canid_0x112_msg_init();
  bodywork_control_msg_init();

  steer_cmd_.steer_cmd = 0.0;
  steer_cmd_.enable_auto_steer = false;
  steer_cmd_.is_updated = false;

  speed_cmd_.speed_cmd = 0.0;
  speed_cmd_.enable_auto_speed = false;
  speed_cmd_.is_updated = false;
  speed_cmd_.gear = 0;

  brake_cmd_.enable_auto_brake = false;
  brake_cmd_.deceleration = 0.0;

  auto_drive_ = false;

  if (auto_speed_ || auto_steer_) {
    if (auto_speed_) {
      ROS_INFO("Enable auto speed!");
      sub_speedfeedback_ = nh_.subscribe("/rock_can/speed_feedback", 1,
                                         &can_encoder::getspeedcallback, this);
      sub_speedcmd_ = nh_.subscribe("/rock_can/speed_command", 1,
                                    &can_encoder::speedcmdcallback, this);
      sub_brakestatefeedback_ =
          nh_.subscribe("/rock_can/brake_state_feedback", 1,
                        &can_encoder::getbrakestatecallback, this);
      sub_brakecmd_ = nh_.subscribe("/rock_can/brake_command", 1,
                                    &can_encoder::brakecmdcallback, this);
    } else
      ROS_INFO("Disable auto speed!");

    if (auto_steer_) {
      ROS_INFO("Enable auto steer!");
      sub_steerfeedback_ = nh_.subscribe("/rock_can/steer_feedback", 1,
                                         &can_encoder::getsteercallback, this);
      sub_steercmd_ = nh_.subscribe("/rock_can/steer_command", 1,
                                    &can_encoder::steercmdcallback, this);
      sub_steerstatefeedback_ =
          nh_.subscribe("/rock_can/steer_state_feedback", 1,
                        &can_encoder::getsteerstatecallback, this);

    } else
      ROS_INFO("Disable auto steer!");
    sub_release_manual_intervention_ =
        nh_.subscribe("/rock_can/release_manual_intervention", 1,
                      &can_encoder::releasecallback, this);
    sub_auto_drive_ = nh_.subscribe("/rock_can/auto_drive", 1,
                                    &can_encoder::autodrivecallback, this);
    pub_canWrite_0x111_ =
        nh_.advertise<cyber_msgs::canframe>("/rock_can/can_write_0x111", 1);
    canid_0x111_timer_ = nh_.createTimer(
        ros::Duration(0.02), &can_encoder::canid_0x111_timer_callback, this);

  } else
    ROS_INFO("Disable auto speed & auto steer!");

  if (bodywork_control_) {
    ROS_INFO("Enable bodywork control!");
    sub_bodywork_control_ =
        nh_.subscribe("/rock_can/bodywork_control", 1,
                      &can_encoder::bodyworkcmdcallback, this);
  } else {
    ROS_INFO("Disable bodywork control!");
  }
  pub_canWrite_0x112_ =
      nh_.advertise<cyber_msgs::canframe>("/rock_can/can_write_0x112", 1);
  canid_0x112_timer_ = nh_.createTimer(
      ros::Duration(0.02), &can_encoder::canid_0x112_timer_callback, this);
}

void can_encoder::controlsourcecallback(const std_msgs::Int8& msg) {}
void can_encoder::steercmdcallback(const cyber_msgs::steercmd& msg) {
  last_steer_time_ = ros::Time::now().toSec();
  steer_cmd_.enable_auto_steer = msg.enable_auto_steer;
  steer_cmd_.is_updated = msg.is_updated;
  steer_cmd_.steer_cmd = msg.steer_cmd;
}
void can_encoder::speedcmdcallback(const cyber_msgs::speedcmd& msg) {
  last_speed_time_ = ros::Time::now().toSec();
  speed_cmd_.enable_auto_speed = msg.enable_auto_speed;
  speed_cmd_.is_updated = msg.is_updated;
  speed_cmd_.speed_cmd = msg.speed_cmd;  //此处不是速度，而是转矩
  speed_cmd_.gear = msg.gear;
}
void can_encoder::brakecmdcallback(const cyber_msgs::brakecmd& msg) {
  last_brake_time_ = ros::Time::now().toSec();
  brake_cmd_.enable_auto_brake = msg.enable_auto_brake;
  brake_cmd_.deceleration = msg.deceleration;
}

void can_encoder::bodyworkcmdcallback(const cyber_msgs::BodyworkControl& msg) {
  if (msg.Horns_updated)
    bodywork_control_msg_.data[1] =
        (bodywork_control_msg_.data[1] & 0xfe) + (msg.BCM_Horns & 0x01);
  if (msg.Door_updated)
    bodywork_control_msg_.data[2] =
        (bodywork_control_msg_.data[2] & 0xf9) + (msg.BCM_Door & 0x03) << 1;
  if (msg.Wipers_updated)
    bodywork_control_msg_.data[2] =
        (bodywork_control_msg_.data[2] & 0x1f) + (msg.BCM_Wipers & 0x07) << 5;
  if (msg.HeadLight_updated)
    bodywork_control_msg_.data[3] =
        (bodywork_control_msg_.data[3] & 0xfc) + (msg.LCM_HeadLight & 0x03);
  if (msg.LowLight_updated)
    bodywork_control_msg_.data[3] =
        (bodywork_control_msg_.data[3] & 0xf3) + (msg.LCM_LowLight & 0x03) << 2;
  if (msg.SideLamps_updated)
    bodywork_control_msg_.data[3] =
        (bodywork_control_msg_.data[3] & 0xcf) + (msg.LCM_SideLamps & 0x03)
        << 4;
  if (msg.TurnLight_updated)
    bodywork_control_msg_.data[3] =
        (bodywork_control_msg_.data[3] & 0x3f) + (msg.LCM_TurnLight & 0x03)
        << 6;
  if (msg.RearFogLamps_updated)
    bodywork_control_msg_.data[4] =
        (bodywork_control_msg_.data[4] & 0xfc) + (msg.LCM_RearFogLamps & 0x03);
}

void can_encoder::canid_0x111_timer_callback(const ros::TimerEvent&) {
  canid_0x111_msg_init();
  if (auto_drive_) {
    if (auto_steer_ && IsEPSControlable_ && !manual_intervention_) {
      // SteerAgEna
      if (steer_state_fbmsg_.EPS_ACUControllerFeedback == 0) {
        can_id_0x111_.data[5] = (can_id_0x111_.data[5] & 0xfc) + 0x01;
      } else if (steer_state_fbmsg_.EPS_ACUControllerFeedback == 1) {
        can_id_0x111_.data[5] = (can_id_0x111_.data[5] & 0xfc) + 0x02;
      }

      // if (manual_intervention_ && release_manual_intervention_) {
      //   can_id_0x111_.data[5] = (can_id_0x111_.data[5] & 0xfc) + 0x00;
      //   manual_intervention_ = false;
      //   release_manual_intervention_ = false;
      // }

      double steer_angle = steer_cmd_.steer_cmd;
      //连续两帧限幅
      if (steer_angle - steer_feedback_ > 15.0) {
        steer_angle = steer_feedback_ + 15.0;
      } else if (steer_angle - steer_feedback_ < -15.0) {
        steer_angle = steer_feedback_ - 15.0;
      }

      //最终输出限幅
      if (steer_angle > 460.0)
        steer_angle = 460.0;
      else if (steer_angle < -460.0)
        steer_angle = -460.0;

      // AutDrvSteerAgReq
      auto steer_write = (int)(steer_angle * 10);
      if (steer_angle > 0.0) {
        can_id_0x111_.data[6] = steer_write / 256;
        can_id_0x111_.data[7] = (steer_write) % 256;
      } else {
        int temp = 65536 + steer_write;
        can_id_0x111_.data[6] = temp / 256;
        can_id_0x111_.data[7] = (temp) % 256;
      }
    } else {
      can_id_0x111_.data[5] = (can_id_0x111_.data[5] & 0xfc) + 0x00;
    }

    if (auto_speed_) {
      if (speed_cmd_.enable_auto_speed) {
        // Enable AutDrvTrqReqActv
        can_id_0x111_.data[5] = (can_id_0x111_.data[5] & 0xef) + 0x20;
        // AutDrvTrqReq
        if (speed_cmd_.speed_cmd > 5000)
          speed_cmd_.speed_cmd = 5000;
        else if (speed_cmd_.speed_cmd < -5000)
          speed_cmd_.speed_cmd = -5000;
        int temp = speed_cmd_.speed_cmd + 5000;
        int diwei = temp % 4;
        int gaowei = temp / 1024;
        int zhongwei = (temp - gaowei * 1024 - diwei) / 4;
        can_id_0x111_.data[5] =
            (can_id_0x111_.data[5] & 0x3f) + ((0x03 & diwei) << 6);
        can_id_0x111_.data[4] = zhongwei;
        can_id_0x111_.data[3] = (can_id_0x111_.data[3] & 0xf0) + gaowei;
      } else {
        // Disable AutDrvTrqReqActv
        can_id_0x111_.data[5] = (can_id_0x111_.data[5] & 0xef) + 0x00;
        int temp = 5000;
        int diwei = temp % 4;
        int gaowei = temp / 1024;
        int zhongwei = (temp - gaowei * 1024 - diwei) / 4;
        can_id_0x111_.data[5] =
            (can_id_0x111_.data[5] & 0x3f) + ((0x03 & diwei) << 6);
        can_id_0x111_.data[4] = zhongwei;
        can_id_0x111_.data[3] = (can_id_0x111_.data[3] & 0xf0) + gaowei;
      }

      if (brake_cmd_.enable_auto_brake == true &&
          EPS_brake_controllable_ == true) {
        // Enable AutDrvCDDAxEna
        can_id_0x111_.data[3] = (can_id_0x111_.data[3] & 0x7f) + 0x80;
        // AutDrvTarAccr
        if (brake_cmd_.deceleration > 7.75) {
          can_id_0x111_.data[0] = 255;
        } else if (brake_cmd_.deceleration < -5.0) {
          can_id_0x111_.data[0] = 0;
        } else {
          auto temp = (unsigned char)(20 * (brake_cmd_.deceleration + 5));
          printf("acceleration : %d\n", temp);
          can_id_0x111_.data[0] = temp;
        }
      } else {
        // Disable AutDrvCDDAxEna
        can_id_0x111_.data[3] = (can_id_0x111_.data[3] & 0x7f) + 0x00;
      }
    }

    if (auto_steer_ || auto_speed_)
      can_id_0x111_.data[5] = (can_id_0x111_.data[5] & 0xe3) + 0x0c;
    else
      can_id_0x111_.data[5] = (can_id_0x111_.data[5] & 0xe3) + 0x00;
  }

  // // if ((auto_steer_ || auto_speed_) &&
  // //     (steer_cmd_.enable_auto_steer || speed_cmd_.enable_auto_speed ||
  // //      brake_cmd_.enable_auto_brake))
  // // auto_drive_来源于外部的硬件遥控器
  // if (auto_steer_ || auto_speed_)
  //   // AutDrvMode
  //   can_id_0x111_.data[5] = (can_id_0x111_.data[5] & 0xe3) + 0x0c;
  // else
  //   can_id_0x111_.data[5] = (can_id_0x111_.data[5] & 0xe3) + 0x00;

  //若超过3s没有收到速度控制指令，则切换出速度自动控制
  if (ros::Time::now().toSec() - last_speed_time_ >= 3.0) {
    // can_id_0x111_.data[5] = (can_id_0x111_.data[5] & 0xe3) + 0x00;
    speed_cmd_.enable_auto_speed = false;
    ROS_INFO("speed command time out!!!");
  }

  //若超过3s没有收到刹车控制指令，则切换出刹车自动控制
  if (ros::Time::now().toSec() - last_brake_time_ >= 3.0) {
    brake_cmd_.enable_auto_brake = false;
    ROS_INFO("brake command time out!!!");
  }

  pub_canWrite_0x111_.publish(can_id_0x111_);
}

void can_encoder::canid_0x112_timer_callback(const ros::TimerEvent&) {
  canid_0x112_msg_init();
  if (bodywork_control_) {
    for (int i = 0; i < 5; i++) {
      can_id_0x112_.data[i] = bodywork_control_msg_.data[i];
    }
  }
  if (auto_drive_) {
    //只要进行速度控制就需要控制档位，具体的档位由给定速度的数据结构来确定
    if (auto_speed_ && speed_cmd_.enable_auto_speed) {
      can_id_0x112_.data[5] = (can_id_0x112_.data[5] & 0x07) + 0x88;  // Enable
      if (speed_cmd_.gear == 1 && speed_feedback_ >= 0.0) {
        ROS_INFO("Gear Shifts to D!");
        can_id_0x112_.data[5] = (can_id_0x112_.data[5] & 0x8f) + 0x40;  // D
      } else if (speed_cmd_.gear == 2 && speed_feedback_ <= 0.0) {
        ROS_INFO("Gear shifts to R!");
        can_id_0x112_.data[5] = (can_id_0x112_.data[5] & 0x8f) + 0x20;  // R
      }
    } else {
      can_id_0x112_.data[5] = (can_id_0x112_.data[5] & 0x07) + 0x00;  // Disable
    }
  }
  pub_canWrite_0x112_.publish(can_id_0x112_);
}

void can_encoder::getsteercallback(const cyber_msgs::SteerFeedback& msg) {
  steer_feedback_ = msg.SteerAngle;
}
void can_encoder::releasecallback(const std_msgs::Bool& msg) {
  release_manual_intervention_ = msg.data;
  manual_intervention_ = false;
}

void can_encoder::autodrivecallback(const std_msgs::Bool& msg) {
  auto_drive_ = msg.data;
  ROS_WARN("auto drive: %d", auto_drive_);
  steer_cmd_.steer_cmd = 0.0;
  steer_cmd_.enable_auto_steer = false;
  steer_cmd_.is_updated = false;

  speed_cmd_.speed_cmd = 0.0;
  speed_cmd_.enable_auto_speed = false;
  speed_cmd_.is_updated = false;
  speed_cmd_.gear = 0;

  brake_cmd_.enable_auto_brake = false;
  brake_cmd_.deceleration = 0.0;
}
void can_encoder::getspeedcallback(const cyber_msgs::SpeedFeedback& msg) {
  speed_feedback_ = msg.speed_cms;
}
void can_encoder::getsteerstatecallback(
    const cyber_msgs::SteerStateFeedback& msg) {
  steer_state_fbmsg_.EPS_ACUAbortFeedback = msg.EPS_ACUAbortFeedback;
  steer_state_fbmsg_.EPS_ACUControllerFeedback = msg.EPS_ACUControllerFeedback;
  steer_state_fbmsg_.EPS_ACUEpasFailed = msg.EPS_ACUEpasFailed;
  steer_state_fbmsg_.EPS_EPSFailed = msg.EPS_EPSFailed;
  steer_state_fbmsg_.SteeringTorque = msg.SteeringTorque;
  if (steer_state_fbmsg_.EPS_EPSFailed == 0 &&
      steer_state_fbmsg_.EPS_ACUEpasFailed == 0 &&
      steer_state_fbmsg_.EPS_ACUAbortFeedback == 0)
    IsEPSControlable_ = true;
  else
    IsEPSControlable_ = false;

  if (steer_state_fbmsg_.EPS_ACUAbortFeedback == 1) {
    manual_intervention_ = true;
    ROS_WARN("manual intervention!");
  }

  if (steer_state_fbmsg_.EPS_ACUEpasFailed == 1) {
    switch (steer_state_fbmsg_.EPS_ACUAbortFeedback) {
      case 0x02:
        ROS_ERROR("speed too high!");
        break;
      case 0x03:
        ROS_ERROR("angular too large!");
        break;
      case 0x04:
        ROS_ERROR("angular speed too large!");
        break;
      case 0x05:
        ROS_ERROR("dae thermal security!");
        break;
      case 0x06:
        ROS_ERROR("dae limit security!");
        break;
      case 0x07:
        ROS_ERROR("other default!");
        break;
      default:
        ROS_ERROR("default!");
        break;
    }
    ROS_ERROR("Need to restart the car after 10 seconds");
  }
}

void can_encoder::getbrakestatecallback(
    const cyber_msgs::BrakeStateFeedback& msg) {
  int brake_valid = msg.ESP_QDCACC;
  if (brake_valid == 0)
    EPS_brake_controllable_ = true;
  else {
    EPS_brake_controllable_ = false;
    ROS_ERROR("EPS_QDC(Brake) is not controllable!!!");
  }
}

void can_encoder::canid_0x111_msg_init(void) {
  can_id_0x111_.id = 0x111;
  can_id_0x111_.len = 8;
  for (int i = 0; i < 8; i++) can_id_0x111_.data[i] = 0;
}

void can_encoder::canid_0x112_msg_init(void) {
  can_id_0x112_.id = 0x112;
  can_id_0x112_.len = 8;
  for (int i = 0; i < 8; i++) can_id_0x112_.data[i] = 0;
}

void can_encoder::bodywork_control_msg_init(void) {
  bodywork_control_msg_.id = 0x112;
  bodywork_control_msg_.len = 8;
  for (int i = 0; i < 8; i++) {
    bodywork_control_msg_.data[i] = 0;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "can_encoder");
  can_encoder EncoderObject;
  ros::spin();
  return 0;
}
