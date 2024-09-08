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
  // cb = boost::bind(&SpeedControllerTest::configCallback, this, _1, _2);
  // dr_srv.setCallback(cb);

  localization_sub_ = nh_->subscribe("/rock_utm_localization", 1,
                                     &SpeedControllerTest::LocalizationCallback,
                                     this);
  detection_sub_ = nh_->subscribe("fusion_results", 1,
                                  &SpeedControllerTest::DectectionCallback, this);

  pub_speed_cmd_ =
      nh_->advertise<cyber_msgs::speedcmd>("/rock_can/speed_command", 5);
  pub_brake_cmd_ =
      nh_->advertise<cyber_msgs::brakecmd>("/rock_can/brake_command", 5);
  // timer_ = nh_->createTimer(ros::Duration(0.05),
  //                           &SpeedControllerTest::TimerCallback, this);
  ros::spin();
}

void SpeedControllerTest::LocalizationCallback(const hycan_msgs::Localization& msg) {
  utm_x = msg.utm_x;
  utm_y = msg.utm_y;
  heading = msg.heading;
}

void SpeedControllerTest::DectectionCallback(
    const hycan_msgs::DetectionResults& detection_result) {
  int8_t num_boxes = detection_result.num_boxes;
  std::vector<std::vector<float>> boxes;

  float min_distance = 1000;

  for (int i = 0; i < num_boxes; i++) {
    std::vector<float> box;
    box.push_back(detection_result.box3d_array[i].center_x);
    box.push_back(detection_result.box3d_array[i].center_y);
    box.push_back(detection_result.box3d_array[i].width);
    box.push_back(detection_result.box3d_array[i].height);
    boxes.push_back(box);

    float distance = sqrt(pow(utm_x - box[0], 2) + pow(utm_y - box[1], 2));
    if (distance < min_distance) {
      min_distance = distance;
    }
  }

  ROS_INFO("min_distance: %f", min_distance);

  if (min_distance < 5) {
    // start to brake

    sp_.brake_cmd = BRAKE_MIN * 0.8;
    brake_cmd_.deceleration = sp_.brake_cmd;
    brake_cmd_.enable_auto_brake = true;

    cnt_ ++;
    if (cnt_ <= 15) {
      sp_.cmd = SPEED_MIN;
      sp_.err_sum = 0.0;
      speed_cmd_.speed_cmd = TORQU_MIN;
      speed_cmd_.enable_auto_speed = true;
      speed_cmd_.is_updated = true;
      speed_cmd_.gear = 1;
      pub_speed_cmd_.publish(speed_cmd_);
      pub_brake_cmd_.publish(brake_cmd_);
    }
  }
}