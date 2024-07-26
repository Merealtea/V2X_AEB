#include "speed_controller.h"

// USB Key，只在生成发布版的时候才解注释
// #include "../../../common/Pwd_8/SoftkeyPWD.h"

using namespace std;

SpeedController::SpeedController(ros::NodeHandle *nh) : nh_(nh) {
  //------成员变量初始化-------------
  sp_.speed_curr = 0.0;
  sp_.speed_ref = 0.0;
  sp_.err_curr = 0.0;
  sp_.err_sum = 0.0;
  sp_.state = 0;
  sp_.cmd = SPEED_MIN;
  sp_.brake_cmd = BRAKE_MIN;
  sp_.last_brake_cmd = BRAKE_MIN;
  min_speed_ = 10.0;

  //------运行时可更改的参数---------
  //------配置动态更改参数服务-------
  cb = boost::bind(&SpeedController::configCallback, this, _1, _2);
  dr_srv.setCallback(cb);
  //------配置默认参数--------------
  nh_->param("speed_kp", sp_.kp, 300.0);
  nh_->param("speed_ki", sp_.ki, 5.0);
  nh_->param("speed_kd", sp_.kd, 0.0);
  nh_->param("speed_kv", sp_.kv, 40.0);  //需修改！！！
  nh_->param("speed_ka", sp_.ka, 400.0);
  // nh_->param("speed_max", sp_.max, 15);
  nh_->param("speed_max", sp_.max, 25);
  nh_->param("speed_min", sp_.min, 5);
  nh_->param("emergency_slow_brake", sp_.emergency_slow_brake, 65);
  nh_->param("emergency_stop_brake", sp_.emergency_stop_brake, 80);

  pub_speed_cmd = nh_->advertise<std_msgs::Int32MultiArray>("/speed_cmd", 5);

  sub_local_trajectory = nh_->subscribe(
      "/control/local_trajectory", 1, &SpeedController::localTrajectoryCallback, this);
  sub_velocity = nh_->subscribe("/localization/estimation", 1,
                                &SpeedController::velocityCallback, this);
  sub_emergency_type = nh_->subscribe(
      "/emergency_type", 2, &SpeedController::emergencyTypeCallback, this);
  sub_stager_mode = nh_->subscribe("/stager_mode", 5,
                                   &SpeedController::stagerModeCallback, this);

  sub_parking_start_ = nh_->subscribe("/parking/start", 5,
                                      &SpeedController::StartCallback, this);

  sub_parking_finished_ = nh_->subscribe(
      "/parking/finished", 5, &SpeedController::FinishedCallback, this);

  cnt_ = 0;
  timer_ = nh->createTimer(ros::Duration(0.1), &SpeedController::Timer1Callback,
                           this);
}

void SpeedController::StartCallback(const std_msgs::Int8ConstPtr &start_in) {
  bparking_ = true;
}

void SpeedController::FinishedCallback(
    const std_msgs::BoolConstPtr &finished_in) {
  bparking_ = false;
}

// 获取紧急刹车mode
void SpeedController::stagerModeCallback(const std_msgs::Int8 mode_in) {
  stager_mode_ = mode_in.data;
}

void SpeedController::Timer1Callback(const ros::TimerEvent &) {
  // 离障碍物太近时紧急刹车
  if (obstacle_emergency_) {
    std::cout << "obstacle is emergency, stop !!!" << std::endl;
    Speed sp;
    sp.cmd = SPEED_MIN;
    sp.brake_cmd = 100;
    publish_cmd(sp);
    return;
  }

  cnt_++;
  //超过1秒没有全局路径规划了
  if (cnt_ > 10) {
    Speed sp;
    sp.cmd = SPEED_MIN;
    sp.brake_cmd = 100;
    publish_cmd(sp);
  }
}

void SpeedController::cal_speed_cmd(Speed &sp) {
  ///如果没有速度反馈
  if (!sp.state)
    sp.cmd = SPEED_MIN;
  else {
    /// D* RRT路段的怠速
    if (sp.speed_ref > 4.95 && sp.speed_ref < 5.05 && sp.speed_curr < 7) {
      sp.brake_cmd = 0;
      // sp.cmd = SPEED_MIN;
      sp.cmd = sp.speed_ref;
    } else {
      // //----------e100直接对速度控制--------------------------
      sp.cmd = sp.speed_ref;
      //速度误差计算
      sp.err_curr = sp.speed_ref - sp.speed_curr;
      sp.err_sum += sp.err_curr;
      if (sp.err_sum > 40) sp.err_sum = 40;
      if (sp.err_sum < -40) sp.err_sum = -40;

      //-----------减速控制-----------------------------------
      //静止状态
      if (sp.speed_ref <= 0 && fabs(sp.err_curr) < 10) {
        sp.brake_cmd = 100;
        sp.err_sum = 0;
        sp.brake_cmd =
            min(sp.last_brake_cmd + 10 * BRAKE_MAX_GAP, sp.brake_cmd);
      } else if ((sp.err_curr > -2) && (sp.err_curr <= -1)) {
        sp.brake_cmd = 100;
        sp.err_sum = 0;
        sp.cmd = 0;
        sp.brake_cmd = min(sp.last_brake_cmd + 1 * BRAKE_MAX_GAP, sp.brake_cmd);
      } else if ((sp.err_curr > -3) && (sp.err_curr <= -2)) {
        sp.brake_cmd = 100;
        sp.err_sum = 0;
        sp.cmd = 0;
        sp.brake_cmd = min(sp.last_brake_cmd + 1 * BRAKE_MAX_GAP, sp.brake_cmd);
      } else if ((sp.err_curr > -4) && (sp.err_curr <= -3)) {
        sp.brake_cmd = 100;
        sp.err_sum = 0;
        sp.cmd = 0;
        sp.brake_cmd = min(sp.last_brake_cmd + 1 * BRAKE_MAX_GAP, sp.brake_cmd);
      } else if ((sp.err_curr > -6) && (sp.err_curr <= -4)) {
        sp.brake_cmd = 100;
        sp.err_sum = 0;
        sp.cmd = 0;
        sp.brake_cmd = min(sp.last_brake_cmd + 3 * BRAKE_MAX_GAP, sp.brake_cmd);
      } else if ((sp.err_curr > -8) && (sp.err_curr <= -6)) {
        sp.brake_cmd = 100;
        sp.err_sum = 0;
        sp.cmd = 0;
        sp.brake_cmd = min(sp.last_brake_cmd + 5 * BRAKE_MAX_GAP, sp.brake_cmd);
      } else if (sp.err_curr <= -8) {
        sp.brake_cmd = 100;
        sp.err_sum = 0;
        sp.cmd = 0;
        sp.brake_cmd = min(sp.last_brake_cmd + 7 * BRAKE_MAX_GAP, sp.brake_cmd);
      } else
        sp.brake_cmd = BRAKE_MIN;

      sp.state = 0;
    }
  }

  //-------以下将输出命令做一个限幅-------------
  if (sp.cmd < SPEED_MIN) sp.cmd = SPEED_MIN;
  if (sp.cmd > SPEED_MAX) sp.cmd = SPEED_MAX;
  if (sp.brake_cmd > BRAKE_MAX) sp.brake_cmd = BRAKE_MAX;
  sp.last_brake_cmd = sp.brake_cmd;
}

void SpeedController::publish_cmd(const Speed Sp) {
  if (bparking_) return;
  std_msgs::Int32MultiArray speed_cmd;
  speed_cmd.data.push_back(Sp.cmd / 3.6 * 100);  // cm/s
  speed_cmd.data.push_back(Sp.brake_cmd);
  pub_speed_cmd.publish(speed_cmd);
}

void SpeedController::localTrajectoryCallback(
    const cyber_msgs::LocalTrajList::ConstPtr &path_in) {
  if (stager_mode_ == 20) {  // 紧急刹停
    std::cout << "Emergency stop!!!" << std::endl;
    Speed sp;
    sp.cmd = SPEED_MIN;
    sp.brake_cmd = sp_.emergency_stop_brake;
    std::cout << "V_ref: " << 0 << " V_cur: " << sp_.speed_curr
              << " cmd: " << sp.cmd << " brake: " << sp.brake_cmd
              << " stager_mode: " << stager_mode_ << std::endl;
    publish_cmd(sp);
    return;
  } else if (stager_mode_ == 21) {  //紧急减速
    std::cout << "Emergency slow down!!!" << std::endl;
    Speed sp;
    sp.cmd = SPEED_MIN;
    sp.brake_cmd = sp_.emergency_slow_brake;
    std::cout << "V_ref: " << 0 << " V_cur: " << sp_.speed_curr
              << " cmd: " << sp.cmd << " brake: " << sp.brake_cmd
              << " stager_mode: " << stager_mode_ << std::endl;
    publish_cmd(sp);
    return;
  } else {
    // 离障碍物太近时紧急刹车,冗余判断，e100项目未启用
    if (obstacle_emergency_) return;

    if (path_in->points.size() < 5) {
      //如果路径太短
      Speed sp;
      sp.cmd = SPEED_MIN;
      sp.brake_cmd = 50;
      publish_cmd(sp);
      return;
    }

    cnt_ = 0;
    ///预瞄2米外的一目标点的速度
    for (auto pt : path_in->points) {
      // std::cout << "pt s" << pt.s << "pt v:" << pt.velocity << std::endl;
      if (pt.s > std::max(2.0, 1.5 * sp_.speed_curr / 3.6)) {
        sp_.speed_ref = 3.6 * pt.velocity;
        break;
      }
      sp_.speed_ref = 3.6 * pt.velocity;
    }

    sp_.speed_ref = std::min(sp_.speed_ref, 3.6 * min_speed_);

    if (obstacle_close_) {
      sp_.speed_ref = std::min(sp_.speed_ref, 5.0);  // 靠近障碍物时怠速
      std::cout << "obstacle is very close, slow down !!!" << std::endl;
    }

    cal_speed_cmd(sp_);
    std::cout << "V_ref: " << sp_.speed_ref << " V_cur: " << sp_.speed_curr
              << std::endl;
    std::cout << "forward control distance : "
              << std::max(2.0, 1.5 * sp_.speed_curr / 3.6) << std::endl;
    std::cout << "speed_cmd: " << sp_.cmd << " brake: " << sp_.brake_cmd
              << std::endl;
    std::cout << " stager_mode: " << stager_mode_ << std::endl;
    publish_cmd(sp_);
  }
}

void SpeedController::velocityCallback(
    cyber_msgs::LocalizationEstimate vel_in) {
  sp_.speed_curr = 3.6 * vel_in.velocity.linear.x;
  sp_.state = 1;
}

void SpeedController::emergencyTypeCallback(std_msgs::Int8 emergency_type_in) {
  if (emergency_type_in.data == 0) {
    obstacle_emergency_ = false;
    obstacle_close_ = false;
  } else if (emergency_type_in.data == 1) {
    obstacle_emergency_ = false;
    obstacle_close_ = true;
  } else if (emergency_type_in.data == 2) {
    obstacle_emergency_ = true;
    obstacle_close_ = true;
  } else {
    std::cout << "emergency type" << emergency_type_in.data << " error! "
              << std::endl;
  }
}

void SpeedController::configCallback(
    speed_controller::speed_controllerConfig &config, uint32_t level) {
  sp_.kp = config.speed_kp;
  sp_.ki = config.speed_ki;
  sp_.kd = config.speed_kd;
  sp_.kv = config.speed_kv;
  sp_.ka = config.speed_ka;
  sp_.max = config.speed_max;
  sp_.min = config.speed_min;
}

void SpeedController::print_info() {
  cout << "speed_kp\t" << sp_.kp << endl;
  cout << "speed_ki\t" << sp_.ki << endl;
  cout << "speed_kd\t" << sp_.kd << endl;
  cout << "speed_ka\t" << sp_.ka << endl;
  cout << "speed_kv\t" << sp_.kv << endl;
  cout << "speed_max\t" << sp_.max << endl;
  cout << "speed_min\t" << sp_.min << endl;
  cout << "speed_state\t" << sp_.state << endl;
  cout << "speed_cmd\t" << sp_.cmd << endl;
  cout << "brake_cmd\t" << sp_.brake_cmd << endl;
  cout << "speed_ref\t" << sp_.speed_ref << endl;
  cout << "speed_curr\t" << sp_.speed_curr << endl;
  cout << "speed_err\t" << sp_.err_curr << endl;
  cout << "speed_err_sum\t" << sp_.err_sum << endl;
}

int main(int argc, char **argv) {
  // 只在生成发布版的时候才解注释，否则在自己电脑上无法调试
  // if(!checkUSBKey()) return 0;

  ros::init(argc, argv, "speed_controller");
  ros::NodeHandle pnh("~");

  SpeedController SpeedController_obj(&pnh);

  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();
  return 0;
}
