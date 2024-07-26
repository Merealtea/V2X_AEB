//
// 2021.08.24 zhu-hu
//
#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <dynamic_reconfigure/server.h>
#include <lqr/lqrConfig.h>
#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>

// #include "common/log.h"
#include "ros/publisher.h"
#include "tf/transform_datatypes.h"

#include "cyber_msgs/LocalTrajList.h"
#include "cyber_msgs/LocalTrajPoint.h"
#include "cyber_msgs/LocalizationEstimate.h"
#include "cyber_msgs/LqrDebug.h"
#include "cyber_msgs/SpeedFeedback.h"
#include "cyber_msgs/SteerFeedback.h"
#include "cyber_msgs/steercmd.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "common/math/filters/digital_filter.h"
#include "common/math/filters/digital_filter_coefficients.h"
#include "common/math/linear_quadratic_regulator.h"
#include "leadlag_controller.h"

namespace controller {
namespace LQR {
using namespace std;
using Matrix = Eigen::MatrixXd;
class LQR {
 public:
  LQR();
  ~LQR();

 private:
  struct TrajPoint {
    geometry_msgs::Pose point;
    double heading;
    double kappa;
    double velocity;
  };

  struct Pose {
    double x;
    double y;
    double yaw;
    double linear_v;
    double angular_v;
    double kappa;
  };

  struct Model_configs {
    double ts;
    double cf;
    double cr;
    double mass_fl;
    double mass_fr;
    double mass_rl;
    double mass_rr;
    double eps;
    double max_iteration;
  };

  double mass_ = 0.0;
  double lf_ = 0.0;
  double lr_ = 0.0;
  double iz_ = 0.0;

  ros::NodeHandle pnh_;
  ros::Subscriber sub_localization_;
  ros::Subscriber sub_localtraj_;
  ros::Subscriber sub_steerfeedback_;
  ros::Subscriber sub_stager_mode;
  ros::Subscriber sub_speedfeedback_;
  ros::Publisher pub_steer_cmd_;

  // debug pub
  // ros::Publisher pub_lat_error_;
  // ros::Publisher pub_lat_error_rate_;
  // ros::Publisher pub_heading_error_;
  // ros::Publisher pub_heading_error_rate_;
  ros::Publisher pub_point_marker_;
  // ros::Publisher pub_lqr_output_;
  // ros::Publisher pub_current_speed_;
  // ros::Publisher pub_filter_lqr_output_;
  // ros::Publisher pub_ref_curvature_;
  // ros::Publisher pub_ref_heading_;
  ros::Publisher pub_debug_msg_;

  cyber_msgs::LqrDebug debug_msg_;
  cyber_msgs::LocalTrajList target_traj_;
  cyber_msgs::LocalizationEstimate localization_;
  enum vehicle_gear { R = 9, D = 11, P = 10, N = 0 };
  int8_t gear_;  // gear_为9是R档，11是D档，10是P档，0是空档

  bool traj_updated_ = false;

  Pose current_pose_;

  Pose target_pose_;

  double current_wheel_degree_;
  double current_wheel_rad_;

  double wheel_degree_feedback_;
  double wheel_rad_feedback_;

  Model_configs model_configs_;
  int stager_mode_;
  int current_index_ = 0;
  double filter_param_;
  double wheelbase_;
  double k_wheel2steering_;
  double max_delta_steering_degree_;
  double max_steering_degree_;
  double wheel_zero_;

  double final_steering_degree_;
  double former_steering_degree_ = 0.0;

  const int state_size_ = 4;
  Matrix matrix_state_;
  Matrix matrix_a_;
  Matrix matrix_ad_;
  Matrix matrix_a_coeff_;
  Matrix matrix_b_;
  Matrix matrix_bd_;
  Matrix matrix_q_;
  Matrix matrix_reverse_q_;  // 倒档情况下的q矩阵
  Matrix matrix_r_;
  Matrix matrix_k_;

  double weight_of_lat_error_;
  double weight_of_lat_error_rate_;
  double weight_of_heading_error_;
  double weight_of_heading_error_rate_;
  double weight_of_control_;
  bool enable_gain_scheduler_ = false;
  vector<double> gain_ref_speeds_;
  vector<double> lat_error_gains_;
  vector<double> heading_error_gains_;

  // 数字滤波器
  cyberc3::common::math::DigitalFilter digital_filter_;
  int preview_window_ = 0;
  double lookahead_station_low_speed_ = 1.4224;
  double lookback_station_low_speed_ = 2.8448;
  double lookahead_station_high_speed_ = 1.4224;
  double lookback_station_high_speed_ = 2.8448;
  double max_lat_acc_ = 5.0;
  double low_speed_bound_ = 3.0;
  double low_speed_window_ = 1.0;

  double query_relative_time_ = 0.8;
  double minimum_speed_protection_ = 0.1;
  double k_minimum_curvature_ =
      0.1;  // minimum curvature for feedforward calcualtion

  double lqr_eps_ = 0.01;
  int lqr_max_iteration_ = 150;

  double lateral_error_;
  double lateral_error_rate_;
  double heading_error_;
  double heading_error_rate_;
  double lateral_error_feedback_;
  double heading_error_feedback_;
  double ref_heading_;
  double ref_heading_rate_;
  double heading_rate_;
  double ref_curvature_;

  bool enable_lookahead_back_control_;

  bool enable_leadlag_control_;

  double look_forward_distance_;
  double look_forward_time_;

  double reverse_look_forward_distance_;

  LeadlagController leadlag_controller_;

  LeadlagConf leadlag_config_{3000.0, 1.0, 1.0, 0.0};

  cyber_msgs::steercmd steer_cmd_;

  dynamic_reconfigure::Server<lqr::lqrConfig> dr_srv;

  dynamic_reconfigure::Server<lqr::lqrConfig>::CallbackType cb;

  void configCallback(lqr::lqrConfig &config, uint32_t level);

  void load_params();
  void matrix_init();
  void update_matrix();
  // void update_target_point();

  void update_state();
  void compute_control_command();

  void compute_lat_error();

  double compute_feed_forward(double ref_curvature) const;

  void TrajectoryCallback(const cyber_msgs::LocalTrajList::ConstPtr &traj_ptr);

  void LocalizationCallback(
      const cyber_msgs::LocalizationEstimate::ConstPtr &pose_in);

  void SteerCallback(const cyber_msgs::SteerFeedback::ConstPtr &msg);

  void SpeedCallback(const cyber_msgs::SpeedFeedback::ConstPtr &msg);

  void StagerModeCallback(const std_msgs::Int8 mode_in);

  inline double Steering2Wheel(double steering_angle) {
    return steering_angle / k_wheel2steering_;
  }

  inline double Wheel2Steering(double wheel_angle) {
    return wheel_angle * k_wheel2steering_;
  }

  // 和轨迹处理相关的函数
  double PointDistanceSquare(const cyber_msgs::LocalTrajPoint &point,
                             const double x, const double y);

  cyber_msgs::LocalTrajPoint QueryNearestPointByRelativeTime(const double t);

  cyber_msgs::LocalTrajPoint QueryNearestPointByPosition(const double x,
                                                         const double y,
                                                         const double distance);

  inline void ComputeCOMPosition(cyber_msgs::LocalTrajPoint &point,
                                 double rear_to_com_distance);
};
}  // namespace LQR
}  // namespace controller
