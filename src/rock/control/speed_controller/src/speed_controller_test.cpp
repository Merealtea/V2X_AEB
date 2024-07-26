#include "speed_controller_test.h"

using namespace std;

SpeedControllerTest::SpeedControllerTest(ros::NodeHandle* nh) : nh_(nh) {
  // initial
  sp_.speed_curr = 0.0;
  sp_.speed_ref = 0.0;
  sp_.err_curr = 0.0;
  sp_.err_sum = 0.0;
  sp_.state = false;
  cnt_ = 0;

  //运行时可更改参数
  //配置动态更改参数服务
  cb = boost::bind(&SpeedControllerTest::configCallback, this, _1, _2);
  dr_srv.setCallback(cb);

  //配置默认参数
  nh_->param("speed_kp", sp_.kp, 600.0);
  nh_->param("speed_ki", sp_.ki, 3.0);
  nh_->param("speed_kd", sp_.kd, 0.0);
  nh_->param("speed_max", sp_.max, 25.0);  // km/h

  std::cout << "kp : " << sp_.kp << std::endl;
  std::cout << "ki : " << sp_.ki << std::endl;
  std::cout << "kd : " << sp_.kd << std::endl;
  std::cout << "max : " << sp_.max << std::endl;

  sub_vel_ = nh_->subscribe("/rock_can/speed_feedback", 1,
                            &SpeedControllerTest::SpeedCallback, this);
  sub_target_ = nh_->subscribe("/control/speed_target", 1,
                               &SpeedControllerTest::TargetCallback, this);
  // sub_local_trajectory_ =
  //     nh_->subscribe("/control/local_trajectory", 1,
  //                    &SpeedControllerTest::LocalTrajectoryCallback, this);
  sub_stager_mode_ = nh_->subscribe(
      "/stager_mode", 1, &SpeedControllerTest::StagerModeCallback, this);
  pub_speed_cmd_ =
      nh_->advertise<cyber_msgs::speedcmd>("/rock_can/speed_command", 5);
  pub_brake_cmd_ =
      nh_->advertise<cyber_msgs::brakecmd>("/rock_can/brake_command", 5);
  timer_ = nh_->createTimer(ros::Duration(0.05),
                            &SpeedControllerTest::TimerCallback, this);
  ros::spin();
}

void SpeedControllerTest::SpeedCallback(const cyber_msgs::SpeedFeedback& msg) {
  sp_.speed_curr = msg.speed_cms / 100.0;  // m/s
  sp_.state = true;
}

void SpeedControllerTest::TargetCallback(const std_msgs::Float64& msg) {
  flag = true;
  cnt_ = 0;
  sp_.speed_ref = std::min(msg.data, sp_.max / 3.6);
}

void SpeedControllerTest::StagerModeCallback(
    const std_msgs::Int8ConstPtr& mode_in) {
  stager_mode_ = mode_in->data;
}

void SpeedControllerTest::LocalTrajectoryCallback(
    const cyber_msgs::LocalTrajList::ConstPtr& path_in) {
  flag = true;
  cnt_ = 0;
  if (path_in->points.size() < 5) {
    SpeedTest sp;
    sp.cmd = SPEED_MIN;
    sp.brake_cmd = 0.0;
    return;
  }

  for (auto pt : path_in->points) {
    if (pt.s > std::max(2.0, 1.5 * sp_.speed_curr)) {
      sp_.speed_ref = pt.velocity;
      break;
    }
    sp_.speed_ref = pt.velocity;
  }
  sp_.speed_ref = std::min(sp_.speed_ref, sp_.max / 3.6);
}

void SpeedControllerTest::TimerCallback(const ros::TimerEvent&) {
  if (flag == true) {
    if (!sp_.state) {
      sp_.cmd = TORQU_MIN;
    } else {
      //输出到底层的speed_cmd的取值范围是[0,5000]
      //输出到底层的brake_cmd的取值范围是[-5,0]
      sp_.err_curr = sp_.speed_ref - sp_.speed_curr;
      if (sp_.err_curr < fabs(sp_.speed_ref * 0.5)) sp_.err_sum += sp_.err_curr;
      if (sp_.err_sum > 50.0) sp_.err_sum = 50.0;
      if (sp_.err_sum < -50.0) sp_.err_sum = -50.0;
      sp_.cmd = sp_.kp * sp_.err_curr + sp_.ki * sp_.err_sum;

      sp_.brake_cmd = BRAKE_MAX;
      brake_cmd_.deceleration = sp_.brake_cmd;
      brake_cmd_.enable_auto_brake = false;

      if (sp_.speed_ref <= 0.0 && fabs(sp_.err_curr) < 2.5) {
        sp_.err_sum = 0;
        sp_.brake_cmd = max(sp_.last_brake_cmd + 10 * BRAKE_MAX_GAP, BRAKE_MIN);
        brake_cmd_.deceleration = sp_.brake_cmd;
        brake_cmd_.enable_auto_brake = true;
      } else if (sp_.err_curr > -1.2 && sp_.err_curr <= -0.3) {
        sp_.err_sum = 0;
        sp_.cmd = 0;
        sp_.brake_cmd = max(sp_.last_brake_cmd + 1 * BRAKE_MAX_GAP, BRAKE_MIN);
        brake_cmd_.deceleration = sp_.brake_cmd;
        brake_cmd_.enable_auto_brake = true;
      } else if (sp_.err_curr > -1.8 && sp_.err_curr <= -1.2) {
        sp_.err_sum = 0;
        sp_.cmd = 0;
        sp_.brake_cmd = max(sp_.last_brake_cmd + 2 * BRAKE_MAX_GAP, BRAKE_MIN);
        brake_cmd_.deceleration = sp_.brake_cmd;
        brake_cmd_.enable_auto_brake = true;
      } else if (sp_.err_curr > -2.4 && sp_.err_curr <= -1.8) {
        sp_.err_sum = 0;
        sp_.cmd = 0;
        sp_.brake_cmd = max(sp_.last_brake_cmd + 3 * BRAKE_MAX_GAP, BRAKE_MIN);
        brake_cmd_.deceleration = sp_.brake_cmd;
        brake_cmd_.enable_auto_brake = true;
      } else if (sp_.err_curr <= -2.4) {
        sp_.err_sum = 0;
        sp_.cmd = 0;
        sp_.brake_cmd = max(sp_.last_brake_cmd + 4 * BRAKE_MAX_GAP, BRAKE_MIN);
        brake_cmd_.deceleration = sp_.brake_cmd;
        brake_cmd_.enable_auto_brake = true;
      }
      sp_.state = false;
    }
    if (stager_mode_ == 20) {  //紧急刹停
      sp_.cmd = SPEED_MIN;
      sp_.err_sum = 0.0;
      sp_.brake_cmd = BRAKE_MIN * 0.8;
      brake_cmd_.deceleration = sp_.brake_cmd;
      brake_cmd_.enable_auto_brake = true;

    } else if (stager_mode_ == 21) {  //紧急减速
      sp_.cmd = SPEED_MIN;
      sp_.err_sum = 0.0;
      sp_.brake_cmd = BRAKE_MIN * 0.5;
      brake_cmd_.deceleration = sp_.brake_cmd;
      brake_cmd_.enable_auto_brake = true;
    }

    if (sp_.cmd < TORQU_MIN) sp_.cmd = TORQU_MIN;
    if (sp_.cmd > TORQU_MAX) sp_.cmd = TORQU_MAX;
    if (sp_.brake_cmd < BRAKE_MIN) sp_.brake_cmd = BRAKE_MIN;
    sp_.last_brake_cmd = sp_.brake_cmd;

    speed_cmd_.speed_cmd = sp_.cmd;
    speed_cmd_.enable_auto_speed = true;
    speed_cmd_.is_updated = true;
    speed_cmd_.gear = 1;  // D档
    if (cnt_ >= 10) {     //超过1秒没有轨迹，刹停
      sp_.cmd = SPEED_MIN;
      sp_.err_sum = 0.0;
      sp_.brake_cmd = BRAKE_MIN * 0.8;
      speed_cmd_.speed_cmd = TORQU_MIN;
      speed_cmd_.enable_auto_speed = true;
      speed_cmd_.is_updated = true;
      speed_cmd_.gear = 1;
      brake_cmd_.deceleration = sp_.brake_cmd;
      brake_cmd_.enable_auto_brake = true;
    }

    if (cnt_ <= 15) {  //避免没有轨迹刹停后一直发刹车指令
      pub_speed_cmd_.publish(speed_cmd_);
      pub_brake_cmd_.publish(brake_cmd_);
    }
    cnt_++;
    // std::cout << "err_curr : " << sp_.err_curr << "  err_sum : " <<
    // sp_.err_sum
    //           << "   brake : " << sp_.brake_cmd << "  brake_status : "
    //           << static_cast<int>(brake_cmd_.enable_auto_brake)
    //           << "  cmd : " << sp_.cmd
    //           << "  gear : " << static_cast<int>(speed_cmd_.gear) <<
    //           std::endl;

    std::cout << "V_ref: " << sp_.speed_ref << " V_cur: " << sp_.speed_curr
              << std::endl;
    std::cout << "err_curr : " << sp_.err_curr << "  err_sum : " << sp_.err_sum
              << std::endl;
    std::cout << "forward control distance : "
              << std::max(2.0, 1.5 * sp_.speed_curr / 3.6) << std::endl;
    std::cout << "speed_cmd: " << sp_.cmd << " brake: " << sp_.brake_cmd
              << std::endl;
    std::cout << "brake_status : "
              << static_cast<int>(brake_cmd_.enable_auto_brake)
              << " gear : " << static_cast<int>(speed_cmd_.gear) << std::endl;
    std::cout << " stager_mode: " << stager_mode_ << std::endl;
  }
}

void SpeedControllerTest::configCallback(
    speed_controller::speed_controller_testConfig& config, uint32_t level) {
  if (config.update_param_) {
    sp_.kp = config.speed_kp;
    sp_.ki = config.speed_ki;
    sp_.kd = config.speed_kd;
    sp_.max = config.speed_max;
    std::cout << "----------reconfigure----------"<<std::endl;
    std::cout << "speed_kp\t" << sp_.kp << std::endl;
    std::cout << "speed_ki\t" << sp_.ki << std::endl;
    std::cout << "speed_kd\t" << sp_.kd << std::endl;
    config.update_param_ = false;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "speed_controller_test");
  ros::NodeHandle pnh("~");

  SpeedControllerTest SpeedControllerTest_obj(&pnh);

  return 0;
}
