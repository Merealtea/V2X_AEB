#pragma once

#include <deque>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <cyber_msgs/Box2D.h>
#include <cyber_msgs/Box2DArray.h>
#include <cyber_msgs/SpeedFeedback.h>
#include <cyber_msgs/V2VPacket.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/message_filter.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <opencv2/opencv.hpp>
// #include <opencv2/tracking.hpp>

// #include <mrpt/bayes/CParticleFilter.h>

// #include "components_visual_tracking/components_tracking.h"
#include "ekf/ekf_pose.h"
#include "part_based_tracking/part_based_tracking.h"

// const double EPS = 1.0e-6;

namespace tracking {
double wrapToPi(const double x) {
  if (x < -M_PI) {
    return x + 2 * M_PI;
  }
  if (x >= M_PI) {
    return x - 2 * M_PI;
  }
  return x;
}

class VehicleTracking {
public:
  VehicleTracking() {}
  VehicleTracking(
      const int num_particles, const double t, const double std_xy,
      const double std_v, const double std_theta, const double std_omega,
      const double measurement_std_xy, const double measurement_std_theta,
      const double measurement_std_v, const double measurement_std_omega,
      const double prediction_std_xy, const double prediction_std_theta,
      const std::unordered_map<part_based_tracking::VehiclePartType, double>
          &measurement_noise_std,
      const std::unordered_map<part_based_tracking::VehiclePartType,
                               cv::Point2d> &geometry_model,
      const part_based_tracking::SensorPose &cam_pose, const cv::Mat &K,
      const cv::Mat &D, const cv::Mat &K_new,
      const part_based_tracking::SensorPose &radar_pose,
      const bool delay_compensation, const bool use_target_vis_tracking);

  VehicleTracking &operator=(const VehicleTracking &other) {
    tracker_frame_ = 0;
    // // particles_ = other.particles_;
    cam_K_ = other.cam_K_;
    cam_D_ = other.cam_D_;
    cam_K_new_ = other.cam_K_new_;
    cam_external_ = other.cam_external_;
    radar_external_ = other.radar_external_;
    measurement_std_xy_ = other.measurement_std_xy_;
    measurement_std_theta_ = other.measurement_std_theta_;
    measurement_std_v_ = other.measurement_std_v_;
    measurement_std_omega_ = other.measurement_std_omega_;
    prediction_std_xy_ = other.prediction_std_xy_;
    prediction_std_theta_ = other.prediction_std_theta_;
    std::cout << "m_xy: " << measurement_std_xy_
              << " m_theta: " << measurement_std_theta_
              << " m_v: " << measurement_std_v_
              << " m_omega: " << measurement_std_omega_ << std::endl;
    std::cout << "p_xy: " << prediction_std_xy_
              << " p_theta: " << prediction_std_theta_ << std::endl;

    // initialize particles and filter
    // mrpt::bayes::CParticleFilter::TParticleFilterOptions pf_options;
    // pf_options.adaptiveSampleSize = false;
    // pf_options.PF_algorithm =
    // mrpt::bayes::CParticleFilter::pfStandardProposal;
    // pf_options.resamplingMethod = mrpt::bayes::CParticleFilter::prSystematic;
    // pf_options.BETA = 0.50;
    // particles_.initializeParticles(other.num_particles_, 7.0, 0.0, 0.5, 0.0,
    //                                0.0);
    // num_particles_ = other.num_particles_;
    // particles_.setEgoPose(cv::Point2d(0.0, 0.0), 0.0);
    // particles_.updateAction(0.0, 0.0);
    // particles_.updateTimestamp(ros::Time::now().toSec());
    // pf_.m_options = pf_options;

    // geometry model
    geometry_model_ = other.geometry_model_;

    // initialize historical radar point
    last_radar_pt_.x = last_radar_pt_.y = 0.0;

    init_target_yaw_ = 0.0;

    // initialize front vh bbox
    vh_bbox_.width = 0.0;
    vh_bbox_.height = 0.0;

    // initialize v2v msg
    front_vh_speed_ = front_vh_angular_velo_ = 0.0;

    direction_left_rearlight_ = 0.0;
    direction_right_rearlight_ = 0.0;
    direction_vh_ = 0.0;

    left_valid_ = right_valid_ = true;

    tracker_init_ = false;
    match_fail_count_ = 0;
    left_rearlight_fail_count_ = right_rearlight_fail_count_ = 0;

    delay_compensation_ = other.delay_compensation_;
    use_target_vis_tracking_ = other.use_target_vis_tracking_;

    ekf_.Reset();
    ekf_.timeUpdate(ros::Time::now().toSec());

    // parameters for ablation study
    use_rearlight_all_ = true;
    use_rearlight_left_ = true;
    use_rearlight_right_ = true;
    use_radar_ = true;
    use_v2v_ = true;
  }

  ~VehicleTracking() = default;

private:
  All_EKF_Pose ekf_;
  int tracker_frame_;
  int match_fail_count_;
  int left_rearlight_fail_count_, right_rearlight_fail_count_;

  bool tracker_init_;
  int num_particles_;
  // part_based_tracking::PartParticleFilter particles_;
  // mrpt::bayes::CParticleFilter pf_;
  // std::deque<part_based_tracking::PartParticleFilter> particles_buffer_;
  cv::Mat cam_K_, cam_D_, cam_K_new_;
  part_based_tracking::SensorPose cam_external_, radar_external_;
  std::unordered_map<part_based_tracking::VehiclePartType, cv::Point2d>
      geometry_model_;
  double measurement_std_xy_, measurement_std_theta_, measurement_std_v_,
      measurement_std_omega_;
  double prediction_std_xy_, prediction_std_theta_;

  cv::Point2d last_radar_pt_;
  cv::Rect2d vh_bbox_;
  bool vh_bbox_valid_;
  std_msgs::Header vh_bbox_header_;
  cv::Rect2d pseudo_left_box_;
  cv::Rect2d pseudo_right_box_;
  double direction_left_rearlight_, direction_right_rearlight_;
  double direction_pseudo_left_rearlight_, direction_pseudo_right_rearight_;
  double direction_vh_;
  double front_vh_speed_, front_vh_angular_velo_;
  double front_vh_yaw_;
  double init_target_yaw_;
  cv::Point2d ego_pos_global_;
  double ego_yaw_global_;
  cv::Point2d ego_pos_latest_global_;
  double ego_yaw_latest_global_;
  double ego_velocity_, ego_angular_velo_;
  std::deque<std::pair<double, double>> ego_angular_velo_buffer_;
  std::deque<std::pair<double, double>> front_vh_angular_velo_buffer_;

  bool rearlight_valid_;
  bool left_valid_;
  bool right_valid_;

  cv::Point2d front_vh_rear_center_proposal_;
  cv::Point2d front_vh_rear_center_proposal_global_;

  bool delay_compensation_;
  bool use_target_vis_tracking_;

  double stop_update_time_;

  // parameters for ablation study
  bool use_rearlight_all_;
  bool use_rearlight_left_;
  bool use_rearlight_right_;
  bool use_radar_;
  bool use_v2v_;

  void unprojectTo3D(const std::vector<cv::Point2d> &distorted_pts,
                     std::vector<cv::Point2d> &undistorted_pts,
                     const double z = 1.0) {
    // cv::Mat K_new;
    // cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
    //     cam_K_, cam_D_, cv::Size(1920, 1080), cv::Mat::eye(cv::Size(3, 3),
    //     CV_32FC1), K_new);
    cv::fisheye::undistortPoints(distorted_pts, undistorted_pts, cam_K_, cam_D_,
                                 cv::Mat(), cam_K_new_);
    // std::cout << "undistorted pt: " << undistorted_pts[0] << std::endl;
    // std::cout << "distorted pt: " << distorted_pts[0] << std::endl;
    // std::cout << "new K: " << cam_K_new_ << std::endl;
    auto f_x = cam_K_new_.at<double>(0, 0);
    auto f_y = cam_K_new_.at<double>(1, 1);
    auto c_x = cam_K_new_.at<double>(0, 2);
    auto c_y = cam_K_new_.at<double>(1, 2);
    for (auto &pt : undistorted_pts) {
      pt.x = (pt.x - c_x) / f_x * z;
      pt.y = (pt.y - c_y) / f_y * z;
    }
    // std::cout << "pt: " << undistorted_pts[0] << std::endl;
  }

  void matchRadarPoint(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &radar,
                       cv::Point2d &match,
                       const cv::Point2d &sugg = cv::Point2d(-1.0, 0.0));

public:
  bool processRearlight(
      const std::unordered_map<part_based_tracking::VehiclePartType, cv::Rect2d>
          &obs_boxes,
      std::vector<bool> &obs_valid, const double t, double &left_err,
      double &right_err, const std::string &method = std::string("our")) {
    std::cout << "num  of rearlights: " << obs_boxes.size() << std::endl;
    std::cout << "A" << std::endl;
    bool rearlight_valid = true;
    bool radar_valid = true;
    // check validity of rearlight
    // calc rearlight direction in local coordinates
    // calc rearlight time delay compensation
    auto t_now = ros::Time::now().toSec();
    double direc_compensation = 0.0;
    if (delay_compensation_) {
      if (t < t_now - 0.02) {
        for (int i = 0; i < front_vh_angular_velo_buffer_.size(); ++i) {
          auto curr_t = front_vh_angular_velo_buffer_[i].second;
          auto curr_angular_velo = front_vh_angular_velo_buffer_[i].first;
          if (curr_t >= t && curr_t <= t_now) {
            if (i == 0) {
              direc_compensation += curr_angular_velo * (curr_t - t);
            } else if (front_vh_angular_velo_buffer_[i - 1].second <= t) {
              direc_compensation += curr_angular_velo * (curr_t - t);
            } else if (front_vh_angular_velo_buffer_[i - 1].second > t) {
              direc_compensation +=
                  curr_angular_velo *
                  (curr_t - front_vh_angular_velo_buffer_[i - 1].second);
            }
          }
        }
        for (int i = 0; i < ego_angular_velo_buffer_.size(); ++i) {
          auto curr_t = ego_angular_velo_buffer_[i].second;
          auto curr_angular_velo = ego_angular_velo_buffer_[i].first;
          if (curr_t >= t && curr_t <= t_now) {
            if (i == 0) {
              direc_compensation -= curr_angular_velo * (curr_t - t);
            } else if (ego_angular_velo_buffer_[i - 1].second <= t) {
              direc_compensation -= curr_angular_velo * (curr_t - t);
            } else if (ego_angular_velo_buffer_[i - 1].second > t) {
              direc_compensation -=
                  curr_angular_velo *
                  (curr_t - ego_angular_velo_buffer_[i - 1].second);
            }
          }
        }
      }
    }
    std::cout << "B" << std::endl;

    for (auto it = obs_boxes.begin(); it != obs_boxes.end(); ++it) {
      auto center_u = it->second.x + it->second.width / 2;
      auto center_v = it->second.y + it->second.height / 2;
      cv::Point2d center_pt{center_u, center_v};
      std::vector<cv::Point2d> distorted_pts{center_pt};
      std::vector<cv::Point2d> undistorted_pts;
      unprojectTo3D(distorted_pts, undistorted_pts);
      auto x = undistorted_pts[0].x;
      auto direc = wrapToPi(std::atan2(-x, 1.0));
      if (it->first == part_based_tracking::VehiclePartType::LEFTREARLIGHT) {
        direction_left_rearlight_ =
            direc + cam_external_.yaw + direc_compensation;
        std::cout << "calc left: " << direction_left_rearlight_ << std::endl;
      }
      if (it->first == part_based_tracking::VehiclePartType::RIGHTREARLIGHT) {
        direction_right_rearlight_ =
            direc + cam_external_.yaw + direc_compensation;
        std::cout << "calc right: " << direction_right_rearlight_ << std::endl;
      }
    }
    std::cout << "C" << std::endl;
    // judge if visual tracking needs rematching
    if (std::abs(wrapToPi(direction_left_rearlight_ -
                          direction_right_rearlight_)) < 1.0 / 180.0 * M_PI ||
        std::abs(wrapToPi(direction_left_rearlight_ -
                          direction_right_rearlight_)) > 60.0 / 180.0 * M_PI) {
      left_rearlight_fail_count_ = right_rearlight_fail_count_ = 0;
      std::cout << "left: " << direction_left_rearlight_ << std::endl;
      std::cout << "right: " << direction_right_rearlight_ << std::endl;
      ROS_WARN("2 rearlight too close or far");
      rearlight_valid = false;
      obs_valid[0] = obs_valid[1] = false;
    }
    Eigen::VectorXd X;
    X = ekf_.readX(t, false);
    geometry_msgs::Pose2D pred_target_pose_global;
    geometry_msgs::Pose2D pred_left_rearlight_pose_global,
        pred_right_rearlight_pose_global;
    pred_target_pose_global.x = X(0);
    pred_target_pose_global.y = X(1);
    pred_target_pose_global.theta = X(2);
    calcPartPosition(pred_target_pose_global,
                     part_based_tracking::VehiclePartType::LEFTREARLIGHT,
                     pred_left_rearlight_pose_global);
    calcPartPosition(pred_target_pose_global,
                     part_based_tracking::VehiclePartType::RIGHTREARLIGHT,
                     pred_right_rearlight_pose_global);
    cv::Point2d pred_left_rearlight_pos_global{
        pred_left_rearlight_pose_global.x, pred_left_rearlight_pose_global.y},
        pred_right_rearlight_pos_global{pred_right_rearlight_pose_global.x,
                                        pred_right_rearlight_pose_global.y};
    cv::Point2d pred_left_rearlight_pos_local, pred_right_rearlight_pos_local;
    transformGlobal2Local(pred_left_rearlight_pos_global,
                          pred_left_rearlight_pos_local);
    transformGlobal2Local(pred_right_rearlight_pos_global,
                          pred_right_rearlight_pos_local);
    double pred_direction_left_rearlight =
        std::atan2(pred_left_rearlight_pos_local.y - cam_external_.y,
                   pred_left_rearlight_pos_local.x - cam_external_.x);
    double pred_direction_right_rearlight =
        std::atan2(pred_right_rearlight_pos_local.y - cam_external_.y,
                   pred_right_rearlight_pos_local.x - cam_external_.x);
    double left_rearlight_error =
        std::abs(pred_direction_left_rearlight - direction_left_rearlight_);
    double right_rearlight_error =
        std::abs(pred_direction_right_rearlight - direction_right_rearlight_);

    left_err = pred_direction_left_rearlight - direction_left_rearlight_;
    right_err = pred_direction_right_rearlight - direction_right_rearlight_;
    // std::cout << "left_err: " << left_err << std::endl;
    // std::cout << "right_err: " << right_err << std::endl;

    if (left_rearlight_error > 10.0 / 180.0 * M_PI && tracker_frame_ > 100) {
      ++left_rearlight_fail_count_;
      // obs_valid[0] = false;
      // ROS_WARN("left rearlight visual tracking error!");
    }
    if (right_rearlight_error > 10.0 / 180.0 * M_PI && tracker_frame_ > 100) {
      ++right_rearlight_fail_count_;
      // obs_valid[1] = false;
      // ROS_WARN("right rearlight visual tracking error!");
    }
    if (left_rearlight_fail_count_ > 10 || right_rearlight_fail_count_ > 10) {
      left_rearlight_fail_count_ = right_rearlight_fail_count_ = 0;
      // ROS_WARN("visual tracking error, request rematching!");
      rearlight_valid = false;
    }

    auto input_left_obs = obs_valid[0] ? direction_left_rearlight_
                                       : pred_direction_left_rearlight;
    auto input_right_obs = obs_valid[1] ? direction_right_rearlight_
                                        : pred_direction_right_rearlight;
    if (!obs_valid[0]) {
      ROS_ERROR("left lost!!!!!!!!!!!!!!");
    }
    if (!obs_valid[1]) {
      ROS_ERROR("right lost!!!!!!!!!!!!!!");
    }
    rearlight_valid_ = obs_valid[0] && obs_valid[1];

    obs_valid[0] = obs_valid[0] && use_rearlight_all_ && use_rearlight_left_;
    obs_valid[1] = obs_valid[1] && use_rearlight_all_ && use_rearlight_right_;

    rearlight_valid = obs_valid[0] || obs_valid[1];

    // check if radar point is valid
    if ((obs_valid[0] && direction_left_rearlight_ > 65.0 / 180.0 * M_PI) ||
        (obs_valid[1] && direction_right_rearlight_ < -65.0 / 180.0 * M_PI))
      radar_valid = false;

    radar_valid = radar_valid && use_radar_;

    if (!rearlight_valid && !radar_valid) {
      ++tracker_frame_;
      return false;
    }

    std::cout << "method: " << method << std::endl;
    // get optimal rear center obs by searching
    cv::Point2d target_rear_center;
    double yaw;
    if (method == "simple") {
      simpleFindCenter(target_rear_center);
    } else if (method == "combine") {
      // if (!rearlight_valid) {
      //   combineFindCenter(target_rear_center, pred_direction_left_rearlight,
      //                     pred_direction_right_rearlight);
      // } else if (!radar_valid) {
      //   cv::Point2d pred_rearcenter_pos_local =
      //       (pred_left_rearlight_pos_local + pred_right_rearlight_pos_local)
      //       / 2;
      //   combineFindCenter(target_rear_center, direction_left_rearlight_,
      //                     direction_right_rearlight_,
      //                     pred_rearcenter_pos_local);
      // } else {
      //   combineFindCenter(target_rear_center);
      // }
      if (!radar_valid) {
        cv::Point2d pred_rearcenter_pos_local =
            (pred_left_rearlight_pos_local + pred_right_rearlight_pos_local) /
            2;
        combineFindCenter(target_rear_center, input_left_obs, input_right_obs,
                          pred_rearcenter_pos_local);
      } else {
        combineFindCenter(target_rear_center, input_left_obs, input_right_obs);
      }
    } else {
      // if (!rearlight_valid) {
      //   std::cout << "our" << std::endl;
      //   searchYawCenter(front_vh_yaw_, target_rear_center,
      //                   pred_direction_left_rearlight,
      //                   pred_direction_right_rearlight);
      // } else if (!radar_valid) {
      //   cv::Point2d pred_rearcenter_pos_local =
      //       (pred_left_rearlight_pos_local + pred_right_rearlight_pos_local)
      //       / 2;
      //   searchYawCenter(front_vh_yaw_, target_rear_center,
      //                   input_left_obs, input_right_obs,
      //                   pred_rearcenter_pos_local);
      // } else {
      //   // both measurement valid
      //   searchYawCenter(front_vh_yaw_, target_rear_center);
      // }

      if (!radar_valid) {
        cv::Point2d pred_rearcenter_pos_local =
            (pred_left_rearlight_pos_local + pred_right_rearlight_pos_local) /
            2;
        yaw = pred_target_pose_global.theta;
        searchYawCenter(yaw, target_rear_center, input_left_obs,
                        input_right_obs, pred_rearcenter_pos_local);
      } else {
        yaw = pred_target_pose_global.theta;
        searchYawCenter(yaw, target_rear_center, input_left_obs,
                        input_right_obs);
      }

      if (!rearlight_valid) {
        target_rear_center = last_radar_pt_;
      }
    }

    cv::Point2d target_rear_center_global;
    double target_yaw_global;
    if (target_rear_center.x > 0.0) {
      transformLocal2Global(target_rear_center, yaw - ego_yaw_latest_global_, target_rear_center_global,
                            target_yaw_global);
    } else { // find no valid rear center
      target_rear_center_global =
          (pred_left_rearlight_pos_global + pred_right_rearlight_pos_global) /
          2;
    }
    front_vh_rear_center_proposal_global_ = target_rear_center_global;
    std::cout << "target_rc: " << target_rear_center << std::endl;
    std::cout << "target_rc_global: " << target_rear_center_global << std::endl;
    std::cout << "ego curr pos:" << ego_pos_latest_global_ << std::endl;

    geometry_msgs::Pose2D target_part_global, target_state_obs_global;
    target_part_global.x = target_rear_center_global.x;
    target_part_global.y = target_rear_center_global.y;
    X = ekf_.readX(t, false);
    calcStateFromPart(target_part_global,
                      part_based_tracking::VehiclePartType::REARCENTER, X(2),
                      target_state_obs_global);
    auto tmp_yaw = std::atan2(target_state_obs_global.y - target_rear_center_global.y,
    target_state_obs_global.x - target_rear_center_global.x);
    std::cout << "tmp_yaw: " << tmp_yaw << " pred yaw: " << yaw << std::endl;
    // front_vh_rear_center_proposal_global_.x = target_state_obs_global.x;
    // front_vh_rear_center_proposal_global_.y = target_state_obs_global.y;

    // update with ekf
    Eigen::Vector3d Z_gps;
    Eigen::Matrix3d R_gps;
    t_now = ros::Time::now().toSec();
    X = ekf_.readX(t_now, false);
    if (method == "our") {
      // Z_gps << target_state_obs_global.x, target_state_obs_global.y,
      //     target_state_obs_global.theta;
      auto theta = X(2);
      Z_gps << target_state_obs_global.x, target_state_obs_global.y, theta;
      std::cout << "obs-state discrenpency: "
                << cv::norm(cv::Point2d(target_state_obs_global.x - X(0),
                                        target_state_obs_global.y - X(1)))
                << std::endl;
      if (cv::norm(cv::Point2d(target_state_obs_global.x - X(0),
                               target_state_obs_global.y - X(1))) > 0.5 &&
          tracker_init_ &&
          tracker_frame_ > 100) { // observation is far from tracking state
        std::cout << "observation is far from tracking state" << std::endl;
        Z_gps << target_state_obs_global.x, target_state_obs_global.y, theta;
      }
    } else {
      auto theta = X(2);
      Z_gps << target_state_obs_global.x, target_state_obs_global.y, theta;
    }
    // Z_gps << target_state_obs_global.x, target_state_obs_global.y,
    //       target_state_obs_global.theta;
    R_gps << std::pow(measurement_std_xy_, 2), 0, 0, 0,
        std::pow(measurement_std_xy_, 2), 0, 0, 0,
        std::pow(measurement_std_theta_, 2);
    std::cout << "Z_gps " << Z_gps << std::endl;
    ekf_.gpsStateUpdate(Z_gps, R_gps, t_now);
    ++tracker_frame_;

    return rearlight_valid;
  }

  bool
  processSingleRearlight(const cv::Rect2d &obs_box,
                         const part_based_tracking::VehiclePartType &obs_type,
                         bool &obs_valid, const double t, double &err,
                         const std::string &method = std::string("our")) {
    bool rearlight_valid = true;
    bool radar_valid = true;
    bool left_valid, right_valid;
    // check validity of rearlight
    // calc rearlight direction in local coordinates
    // calc rearlight time delay compensation
    auto t_now = ros::Time::now().toSec();
    double direc_compensation = 0.0;
    if (delay_compensation_) {
      if (t < t_now - 0.02) {
        for (int i = 0; i < front_vh_angular_velo_buffer_.size(); ++i) {
          auto curr_t = front_vh_angular_velo_buffer_[i].second;
          auto curr_angular_velo = front_vh_angular_velo_buffer_[i].first;
          if (curr_t >= t && curr_t <= t_now) {
            if (i == 0) {
              direc_compensation += curr_angular_velo * (curr_t - t);
            } else if (front_vh_angular_velo_buffer_[i - 1].second <= t) {
              direc_compensation += curr_angular_velo * (curr_t - t);
            } else if (front_vh_angular_velo_buffer_[i - 1].second > t) {
              direc_compensation +=
                  curr_angular_velo *
                  (curr_t - front_vh_angular_velo_buffer_[i - 1].second);
            }
          }
        }
        for (int i = 0; i < ego_angular_velo_buffer_.size(); ++i) {
          auto curr_t = ego_angular_velo_buffer_[i].second;
          auto curr_angular_velo = ego_angular_velo_buffer_[i].first;
          if (curr_t >= t && curr_t <= t_now) {
            if (i == 0) {
              direc_compensation -= curr_angular_velo * (curr_t - t);
            } else if (ego_angular_velo_buffer_[i - 1].second <= t) {
              direc_compensation -= curr_angular_velo * (curr_t - t);
            } else if (ego_angular_velo_buffer_[i - 1].second > t) {
              direc_compensation -=
                  curr_angular_velo *
                  (curr_t - ego_angular_velo_buffer_[i - 1].second);
            }
          }
        }
      }
    }

    auto center_u = obs_box.x + obs_box.width / 2;
    auto center_v = obs_box.y + obs_box.height / 2;
    cv::Point2d center_pt{center_u, center_v};
    std::vector<cv::Point2d> distorted_pts{center_pt};
    std::vector<cv::Point2d> undistorted_pts;
    unprojectTo3D(distorted_pts, undistorted_pts);
    auto x = undistorted_pts[0].x;
    auto direc = wrapToPi(std::atan2(-x, 1.0));
    if (obs_type == part_based_tracking::VehiclePartType::LEFTREARLIGHT) {
      direction_left_rearlight_ =
          direc + cam_external_.yaw + direc_compensation;
    } else if (obs_type ==
               part_based_tracking::VehiclePartType::RIGHTREARLIGHT) {
      direction_right_rearlight_ =
          direc + cam_external_.yaw + direc_compensation;
    }

    // calculate pseudo rearlight direction
    double direction_pseudo_rearlight = 1e6;
    if (obs_type == part_based_tracking::VehiclePartType::LEFTREARLIGHT) {
      auto center_u = pseudo_left_box_.x + pseudo_left_box_.width / 2;
      auto center_v = pseudo_right_box_.y = pseudo_left_box_.height / 2;
      cv::Point2d center_pt{center_u, center_v};
      std::vector<cv::Point2d> distorted_pts{center_pt};
      std::vector<cv::Point2d> undistorted_pts;
      unprojectTo3D(distorted_pts, undistorted_pts);
      auto x = undistorted_pts[0].x;
      auto direc = wrapToPi(std::atan2(-x, 1.0));
      direction_pseudo_rearlight =
          direc + cam_external_.yaw + direc_compensation;
      direction_pseudo_left_rearlight_ = direction_pseudo_rearlight;
    } else {
      auto center_u = pseudo_right_box_.x + pseudo_right_box_.width / 2;
      auto center_v = pseudo_right_box_.y = pseudo_right_box_.height / 2;
      cv::Point2d center_pt{center_u, center_v};
      std::vector<cv::Point2d> distorted_pts{center_pt};
      std::vector<cv::Point2d> undistorted_pts;
      unprojectTo3D(distorted_pts, undistorted_pts);
      auto x = undistorted_pts[0].x;
      auto direc = wrapToPi(std::atan2(-x, 1.0));
      direction_pseudo_rearlight =
          direc + cam_external_.yaw + direc_compensation;
      direction_pseudo_right_rearight_ = direction_pseudo_rearlight;
    }

    // judge if visual tracking needs rematching
    if (std::abs(wrapToPi(direction_left_rearlight_ -
                          direction_right_rearlight_)) < 1.0 / 180.0 * M_PI ||
        std::abs(wrapToPi(direction_left_rearlight_ -
                          direction_right_rearlight_)) > 30.0 / 180.0 * M_PI) {
      left_rearlight_fail_count_ = right_rearlight_fail_count_ = 0;
      ROS_WARN("2 rearlight too close or far");
      rearlight_valid = false;
      obs_valid = false;
    }
    Eigen::VectorXd X;
    X = ekf_.readX(t, false);
    geometry_msgs::Pose2D pred_target_pose_global;
    geometry_msgs::Pose2D pred_left_rearlight_pose_global,
        pred_right_rearlight_pose_global;
    pred_target_pose_global.x = X(0);
    pred_target_pose_global.y = X(1);
    pred_target_pose_global.theta = X(2);
    calcPartPosition(pred_target_pose_global,
                     part_based_tracking::VehiclePartType::LEFTREARLIGHT,
                     pred_left_rearlight_pose_global);
    calcPartPosition(pred_target_pose_global,
                     part_based_tracking::VehiclePartType::RIGHTREARLIGHT,
                     pred_right_rearlight_pose_global);
    cv::Point2d pred_left_rearlight_pos_global{
        pred_left_rearlight_pose_global.x, pred_left_rearlight_pose_global.y},
        pred_right_rearlight_pos_global{pred_right_rearlight_pose_global.x,
                                        pred_right_rearlight_pose_global.y};
    cv::Point2d pred_left_rearlight_pos_local, pred_right_rearlight_pos_local;
    transformGlobal2Local(pred_left_rearlight_pos_global,
                          pred_left_rearlight_pos_local);
    transformGlobal2Local(pred_right_rearlight_pos_global,
                          pred_right_rearlight_pos_local);
    double pred_direction_left_rearlight =
        std::atan2(pred_left_rearlight_pos_local.y - cam_external_.y,
                   pred_left_rearlight_pos_local.x - cam_external_.x);
    double pred_direction_right_rearlight =
        std::atan2(pred_right_rearlight_pos_local.y - cam_external_.y,
                   pred_right_rearlight_pos_local.x - cam_external_.x);
    double left_rearlight_error =
        std::abs(pred_direction_left_rearlight - direction_left_rearlight_);
    double right_rearlight_error =
        std::abs(pred_direction_right_rearlight - direction_right_rearlight_);

    double left_err = pred_direction_left_rearlight - direction_left_rearlight_;
    double right_err =
        pred_direction_right_rearlight - direction_right_rearlight_;
    if (obs_type == part_based_tracking::VehiclePartType::LEFTREARLIGHT) {
      err = left_err;
    } else {
      err = right_err;
    }
    // std::cout << "left_err: " << left_err << std::endl;
    // std::cout << "right_err: " << right_err << std::endl;

    if (left_rearlight_error > 10.0 / 180.0 * M_PI && tracker_frame_ > 100) {
      ++left_rearlight_fail_count_;
      // obs_valid[0] = false;
      // ROS_WARN("left rearlight visual tracking error!");
    }
    if (right_rearlight_error > 10.0 / 180.0 * M_PI && tracker_frame_ > 100) {
      ++right_rearlight_fail_count_;
      // obs_valid[1] = false;
      // ROS_WARN("right rearlight visual tracking error!");
    }
    if (left_rearlight_fail_count_ > 10 || right_rearlight_fail_count_ > 10) {
      left_rearlight_fail_count_ = right_rearlight_fail_count_ = 0;
      // ROS_WARN("visual tracking error, request rematching!");
      rearlight_valid = false;
    }

    if (obs_type == part_based_tracking::VehiclePartType::LEFTREARLIGHT) {
      left_valid_ = obs_valid;
    } else {
      right_valid_ = obs_valid;
    }

    // get input rearlight directions
    double input_left_obs, input_right_obs;
    if (obs_type == part_based_tracking::VehiclePartType::LEFTREARLIGHT) {
      if (use_target_vis_tracking_ && direction_pseudo_rearlight >= -M_PI &&
          direction_pseudo_rearlight < M_PI) {
        input_left_obs =
            obs_valid ? direction_left_rearlight_ : direction_pseudo_rearlight;
        if (!obs_valid) {
          ROS_WARN("use pseudo direc");
        }
      } else {
        input_left_obs = obs_valid ? direction_left_rearlight_
                                   : pred_direction_left_rearlight;
      }
      input_right_obs = pred_direction_right_rearlight;
      if (!obs_valid) {
        ROS_ERROR("left lost!!!!!!!!!!");
      }
    } else {
      input_left_obs = pred_direction_left_rearlight;
      if (use_target_vis_tracking_ && direction_pseudo_rearlight >= -M_PI &&
          direction_pseudo_rearlight < M_PI) {
        input_right_obs =
            obs_valid ? direction_right_rearlight_ : direction_pseudo_rearlight;
        if (!obs_valid) {
          ROS_WARN("use pseudo direc");
        }
      } else {
        input_right_obs = obs_valid ? direction_right_rearlight_
                                   : pred_direction_right_rearlight;
      }
      if (!obs_valid) {
        ROS_ERROR("right lost!!!!!!!!!!");
      }
    }

    rearlight_valid_ = obs_valid;
    if (obs_type == part_based_tracking::VehiclePartType::LEFTREARLIGHT) {
      left_valid = obs_valid && use_rearlight_all_ && use_rearlight_left_;
      right_valid = right_valid_ && use_rearlight_all_ && use_rearlight_right_;
    } else {
      left_valid = left_valid_ && use_rearlight_all_ && use_rearlight_left_;
      right_valid = obs_valid && use_rearlight_all_ && use_rearlight_right_;
    }

    rearlight_valid = left_valid || right_valid;

    // check if radar point is valid
    if ((left_valid && direction_left_rearlight_ > 65.0 / 180.0 * M_PI) ||
        (right_valid && direction_right_rearlight_ < -65.0 / 180.0 * M_PI))
      radar_valid = false;

    radar_valid = radar_valid && use_radar_;

    if (!rearlight_valid && !radar_valid) {
      ++tracker_frame_;
      return false;
    }

    std::cout << "method: " << method << std::endl;
    // get optimal rear center obs by searching
    cv::Point2d target_rear_center;
    double yaw;
    if (method == "simple") {
      simpleFindCenter(target_rear_center);
    } else if (method == "combine") {
      // if (!rearlight_valid) {
      //   combineFindCenter(target_rear_center, pred_direction_left_rearlight,
      //                     pred_direction_right_rearlight);
      // } else if (!radar_valid) {
      //   cv::Point2d pred_rearcenter_pos_local =
      //       (pred_left_rearlight_pos_local + pred_right_rearlight_pos_local)
      //       / 2;
      //   combineFindCenter(target_rear_center, direction_left_rearlight_,
      //                     direction_right_rearlight_,
      //                     pred_rearcenter_pos_local);
      // } else {
      //   combineFindCenter(target_rear_center);
      // }
      if (!radar_valid) {
        cv::Point2d pred_rearcenter_pos_local =
            (pred_left_rearlight_pos_local + pred_right_rearlight_pos_local) /
            2;
        combineFindCenter(target_rear_center, input_left_obs, input_right_obs,
                          pred_rearcenter_pos_local);
      } else {
        combineFindCenter(target_rear_center, input_left_obs, input_right_obs);
      }
    } else {
      // if (!rearlight_valid) {
      //   std::cout << "our" << std::endl;
      //   searchYawCenter(front_vh_yaw_, target_rear_center,
      //                   pred_direction_left_rearlight,
      //                   pred_direction_right_rearlight);
      // } else if (!radar_valid) {
      //   cv::Point2d pred_rearcenter_pos_local =
      //       (pred_left_rearlight_pos_local + pred_right_rearlight_pos_local)
      //       / 2;
      //   searchYawCenter(front_vh_yaw_, target_rear_center,
      //                   input_left_obs, input_right_obs,
      //                   pred_rearcenter_pos_local);
      // } else {
      //   // both measurement valid
      //   searchYawCenter(front_vh_yaw_, target_rear_center);
      // }

      if (!radar_valid) {
        cv::Point2d pred_rearcenter_pos_local =
            (pred_left_rearlight_pos_local + pred_right_rearlight_pos_local) /
            2;
        yaw = pred_target_pose_global.theta;
        searchYawCenter(yaw, target_rear_center, input_left_obs,
                        input_right_obs, pred_rearcenter_pos_local);
      } else {
        yaw = pred_target_pose_global.theta;
        searchYawCenter(yaw, target_rear_center, input_left_obs,
                        input_right_obs);
      }

      if (!rearlight_valid) {
        target_rear_center = last_radar_pt_;
      }
    }

    cv::Point2d target_rear_center_global;
    double target_yaw_global;
    if (target_rear_center.x > 0.0) {
      transformLocal2Global(target_rear_center, yaw, target_rear_center_global,
                            target_yaw_global);
    } else { // find no valid rear center
      target_rear_center_global =
          (pred_left_rearlight_pos_global + pred_right_rearlight_pos_global) /
          2;
    }

    geometry_msgs::Pose2D target_part_global, target_state_obs_global;
    target_part_global.x = target_rear_center_global.x;
    target_part_global.y = target_rear_center_global.y;
    calcStateFromPart(target_part_global,
                      part_based_tracking::VehiclePartType::REARCENTER, yaw,
                      target_state_obs_global);
    // update with ekf
    Eigen::Vector3d Z_gps;
    Eigen::Matrix3d R_gps;
    t_now = ros::Time::now().toSec();
    X = ekf_.readX(t_now, false);
    if (method == "our") {
      // Z_gps << target_state_obs_global.x, target_state_obs_global.y,
      //     target_state_obs_global.theta;
      auto theta = X(2);
      Z_gps << target_state_obs_global.x, target_state_obs_global.y, theta;
      std::cout << "obs-state discrenpency: "
                << cv::norm(cv::Point2d(target_state_obs_global.x - X(0),
                                        target_state_obs_global.y - X(1)))
                << std::endl;
      if (cv::norm(cv::Point2d(target_state_obs_global.x - X(0),
                               target_state_obs_global.y - X(1))) > 0.5 &&
          tracker_init_ &&
          tracker_frame_ > 100) { // observation is far from tracking state
        std::cout << "observation is far from tracking state" << std::endl;
        Z_gps << target_state_obs_global.x, target_state_obs_global.y, theta;
      }
    } else {
      auto theta = X(2);
      Z_gps << target_state_obs_global.x, target_state_obs_global.y, theta;
    }
    // Z_gps << target_state_obs_global.x, target_state_obs_global.y,
    //       target_state_obs_global.theta;
    R_gps << std::pow(measurement_std_xy_, 2), 0, 0, 0,
        std::pow(measurement_std_xy_, 2), 0, 0, 0,
        std::pow(measurement_std_theta_, 2);
    ekf_.gpsStateUpdate(Z_gps, R_gps, t_now);
    ++tracker_frame_;

    return rearlight_valid;
  }

  void processRadar(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &radar,
                    const double t) {
    ROS_INFO("process radar");
    if (vh_bbox_.area() > 0) {
      ROS_INFO("vh > 0");
      Eigen::VectorXd X;
      X = ekf_.readX(t, false);
      cv::Point2d pred_target_pos_global{X(0), X(1)}, pred_target_pos_local;
      transformGlobal2Local(pred_target_pos_global, pred_target_pos_local);

      cv::Point2d radar_pt;
      if (tracker_frame_ > 100) {
        matchRadarPoint(radar, radar_pt, pred_target_pos_local);
      } else {
        matchRadarPoint(radar,
                        radar_pt); // match radar record automatically
      }
      std::cout << "radar pt: " << radar_pt << std::endl;
      if (!tracker_init_ && radar_pt.x > 0) {
        // init
        Eigen::Vector3d Z_gps;
        Eigen::Matrix3d R_gps;
        auto t_now = ros::Time::now().toSec();
        std::cout << "init t: " << t_now << std::endl;
        double global_yaw;
        cv::Point2d radar_pt_global;
        transformLocal2Global(last_radar_pt_, 0.0, radar_pt_global, global_yaw);
        Z_gps << radar_pt_global.x, radar_pt_global.y, ego_yaw_global_;
        R_gps << std::pow(measurement_std_xy_, 2), 0, 0, 0,
            std::pow(measurement_std_xy_, 2), 0, 0, 0,
            std::pow(measurement_std_theta_, 2);
        ekf_.setDriveError(prediction_std_xy_, prediction_std_theta_);
        ekf_.gpsStateUpdate(Z_gps, R_gps, t_now);

        Eigen::Vector2d Z_vel;
        Eigen::Matrix2d R_vel;
        Z_vel << 0.0, 0.0;
        R_vel << std::pow(measurement_std_v_, 2), 0, 0,
            std::pow(measurement_std_omega_, 2);
        ekf_.velStateUpdate(Z_vel, R_vel);
        tracker_init_ = true;
        ROS_WARN("init filter with radar");
        std::cout << "init with: " << radar_pt_global << std::endl;
        X = ekf_.readX(ros::Time::now().toSec(), false);
        std::cout << "after init: " << X(0) << " " << X(1) << std::endl;
      }
    }
  }

  void ekfReset() { ekf_.Reset(); }
  void ekfSetDriveError(const double speed_drive_error_estimation_set,
                        const double yaw_rate_drive_error_estimation_set) {
    ekf_.setDriveError(speed_drive_error_estimation_set,
                       yaw_rate_drive_error_estimation_set);
  }
  void ekfReadX(const double t, bool prediction, Eigen::VectorXd &X) {
    X = ekf_.readX(t, prediction);
  }
  void ekfReadP(const double t, bool prediction, Eigen::MatrixXd &P) {
    P = ekf_.readP(t, prediction);
  }
  void ekfGpsStateUpdate(Eigen::Vector3d &Z_gps, const Eigen::Matrix3d &R_gps,
                         const double t) {
    ekf_.gpsStateUpdate(Z_gps, R_gps, t);
  }
  void ekfVelStateUpdate(const Eigen::Vector2d &Z_vel,
                         const Eigen::Matrix2d &R_vel) {
    ekf_.velStateUpdate(Z_vel, R_vel);
  }

  void calcPartPosition(const geometry_msgs::Pose2D pose,
                        const part_based_tracking::VehiclePartType part_type,
                        geometry_msgs::Pose2D &part_pose) {
    part_pose.x = pose.x + geometry_model_[part_type].x * std::cos(pose.theta) -
                  geometry_model_[part_type].y * std::sin(pose.theta);
    part_pose.y = pose.y + geometry_model_[part_type].x * std::sin(pose.theta) +
                  geometry_model_[part_type].y * std::cos(pose.theta);
    part_pose.theta = 0.0;
  }

  void calcStateFromPart(const geometry_msgs::Pose2D &part_pose,
                         const part_based_tracking::VehiclePartType part_type,
                         const double yaw, geometry_msgs::Pose2D &state) {
    // part_pose: part pose in global coordinates
    // yaw: target yaw in global coordinates
    // state: target center state in global coordinates
    std::cout << "calcStateFromPart yaw: " << yaw << std::endl;
    state.theta = yaw;
    state.x = part_pose.x - geometry_model_[part_type].x * std::cos(yaw) +
              geometry_model_[part_type].y * std::sin(yaw);
    state.y = part_pose.y - geometry_model_[part_type].x * std::sin(yaw) -
              geometry_model_[part_type].y * std::cos(yaw);
  }

  void transformLocal2Global(const geometry_msgs::Pose2D &local_pose,
                             geometry_msgs::Pose2D &global_pose) {
    global_pose.theta = local_pose.theta + ego_yaw_global_;
    global_pose.x = ego_pos_global_.x +
                    local_pose.x * std::cos(ego_yaw_global_) -
                    local_pose.y * std::sin(ego_yaw_global_);
    global_pose.y = ego_pos_global_.y +
                    local_pose.x * std::sin(ego_yaw_global_) +
                    local_pose.y * std::cos(ego_yaw_global_);
  }

  void transformGlobal2Local(const cv::Point2d &global_pos,
                             cv::Point2d &local_pos) {
    auto diff = global_pos - ego_pos_global_;
    local_pos.x =
        diff.x * std::cos(ego_yaw_global_) + diff.y * std::sin(ego_yaw_global_);
    local_pos.y = -diff.x * std::sin(ego_yaw_global_) +
                  diff.y * std::cos(ego_yaw_global_);
  }

  void transformGlobal2Cam(const cv::Point2d &global_pos,
                           cv::Point2d &local_pos) {
    auto diff = global_pos - ego_pos_global_;
    local_pos.x =
        diff.x * std::cos(ego_yaw_global_) + diff.y * std::sin(ego_yaw_global_);
    local_pos.y = -diff.x * std::sin(ego_yaw_global_) +
                  diff.y * std::cos(ego_yaw_global_);
    local_pos.x -= cam_external_.x;
    local_pos.y -= cam_external_.y;
  }

  void predict() {
    // auto t_curr = ros::Time::now().toSec();
    // particles_.predictOnce(t_curr);
    // particles_buffer_.emplace_back(particles_);
    // if (particles_buffer_.size() > 50) {
    //   particles_buffer_.pop_front();
    // }
  }
  void getCurrState(double &x, double &y, double &v, double &theta,
                    double &omega) const {
    // ROS_INFO("get curr state");
    // particles_.getMean(x, y, v, theta, omega);
    // std::cout << "curr state: " << x << " " << y << " " << v << " " << theta
    //           << " " << omega << std::endl;
    // ROS_INFO("get curr state done");
  }
  void getCurrState(double &x, double &y, double &v, double &theta,
                    double &omega, double &r_x, double &r_y, double &r_v,
                    double &r_theta, double &r_omega,
                    cv::Point2d &left_rearlight, cv::Point2d &right_rearlight) {
    // ROS_INFO("get curr state");
    // particles_.getMeanRadius(x, y, v, theta, omega, r_x, r_y, r_v, r_theta,
    //                          r_omega, left_rearlight, right_rearlight);
    // // if (tracker_frame_ > 1000) {
    // //   particles_.getMeanRadius(front_vh_rear_center_proposal_, x, y, v,
    // //   theta,
    // //                            omega, r_x, r_y, r_v, r_theta, r_omega,
    // //                            left_rearlight, right_rearlight);
    // // } else {
    // //   particles_.getMeanRadius(x, y, v, theta, omega, r_x, r_y, r_v,
    // //   r_theta,
    // //                            r_omega, left_rearlight, right_rearlight);
    // // }

    // std::cout << "curr state: " << x << " " << y << " " << v << " " << theta
    //           << " " << omega << std::endl;
    // tracker_frame_++;
    // ROS_INFO("get curr state done");
  }

  // double getTimestamp() const { return particles_.getTimestamp(); }

  double getEgoYaw() const { return ego_yaw_global_; }

  void setStopUpdateTime(const double t) { stop_update_time_ = t; }

  bool getInitFlag() const { return tracker_init_; }

  void getBufferTimestamp(std::vector<double> &timestamps) const {
    // for (const auto p : particles_buffer_) {
    //   timestamps.emplace_back(p.getTimestamp());
    // }
  }

  void getRearCenterProposal(cv::Point2d &pt) {
    pt = front_vh_rear_center_proposal_global_;
  }

  void predictUpdate(
      const std::unordered_map<part_based_tracking::VehiclePartType, cv::Rect2d>
          &boxes,
      const double timestamp);
  void predictUpdate(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &radar,
                     const double timestamp);
  void updateVhBbox(const cv::Rect2d &box, const std_msgs::Header &box_header,
                    const bool box_valid) {
    vh_bbox_ = box;
    vh_bbox_header_ = box_header;
    vh_bbox_valid_ = box_valid;

    auto center_u = box.x + box.width / 2;
    auto center_v = box.y + box.height / 2;
    cv::Point2d center_pt{center_u, center_v};
    std::vector<cv::Point2d> distorted_pts{center_pt};
    std::vector<cv::Point2d> undistorted_pts;
    unprojectTo3D(distorted_pts, undistorted_pts);
    auto x = undistorted_pts[0].x;
    auto direc = wrapToPi(std::atan2(-x, 1.0));
    direction_vh_ = direc + cam_external_.yaw;
  }

  void updatePseudoLeftBox(const cv::Rect2d &box) { pseudo_left_box_ = box; }
  void updatePseudoRightBox(const cv::Rect2d &box) { pseudo_right_box_ = box; }

  void updateEgoState(const cv::Point2d &pos, const double yaw) {
    // particles_.setEgoPose(pos, yaw);
    ego_yaw_global_ = yaw;
    ego_pos_global_ = pos;
  }

  void updateEgoLatestState(const cv::Point2d &pos, const double yaw) {
    // particles_.setEgoPose(pos, yaw);
    ego_yaw_latest_global_ = yaw;
    ego_pos_latest_global_ = pos;
  }

  void updateEgoVelo(const double velo) {
    if (tracker_frame_ < 50) {
      ego_velocity_ = velo;
    } else {
      ego_velocity_ = 0.5 * ego_velocity_ + 0.5 * velo;
    }
  }
  void updateEgoAngularVelo(const double angular_velo, const double t = -1.0) {
    if (tracker_frame_ < 50) {
      ego_angular_velo_ = angular_velo;
    } else {
      ego_angular_velo_ = 0.5 * ego_angular_velo_ + 0.5 * angular_velo;

      // add into buffer
      if (t > 0.0) {
        ego_angular_velo_buffer_.emplace_back(
            std::make_pair(ego_angular_velo_, t));
        if (ego_angular_velo_buffer_.size() > 100) {
          ego_angular_velo_buffer_.pop_front();
        }
      }
    }
  }

  void updateV2V(const double front_speed, const double front_angular_velo,
                 const double t = -1.0) {
    if (tracker_frame_ < 50) {
      front_vh_speed_ = front_speed;
      front_vh_angular_velo_ = front_angular_velo;
    } else {
      front_vh_speed_ = 0.5 * front_vh_speed_ + 0.5 * front_speed;
      front_vh_angular_velo_ =
          0.5 * front_vh_angular_velo_ + 0.5 * front_angular_velo;
      // add into buffer
      if (t > 0.0) {
        front_vh_angular_velo_buffer_.emplace_back(
            std::make_pair(front_vh_angular_velo_, t));
        if (front_vh_angular_velo_buffer_.size() > 100) {
          front_vh_angular_velo_buffer_.pop_front();
        }
      }
    }

    // particles_.updateAction(front_speed, front_angular_velo);

    // update with ekf
    // std::cout << "use v2v: " << use_v2v_ << std::endl;
    if (use_v2v_) {
      Eigen::Vector2d Z_vel;
      Eigen::Matrix2d R_vel;
      Z_vel << front_speed, front_angular_velo;
      R_vel << std::pow(measurement_std_v_, 2), 0, 0,
          std::pow(measurement_std_omega_, 2);
      ekf_.velStateUpdate(Z_vel, R_vel);
      ROS_INFO("update with v2v");
      std::cout << "update with: " << Z_vel << std::endl;
      Eigen::VectorXd X;
      X = ekf_.readX(t, false);
      std::cout << "after update: " << X << std::endl;
    }
  }

  void updateV2VImu(const double yaw) {
    if (!tracker_init_) {
      init_target_yaw_ = yaw;
    }
    // particles_.updateYaw(yaw - init_target_yaw_);
    front_vh_yaw_ = yaw - init_target_yaw_;
    std::cout << "Update target yaw: " << yaw - init_target_yaw_ << std::endl;
  }

  void getRearlightDirection(std::vector<double> &directions) const {
    if (directions.size() > 0)
      directions.clear();
    directions.emplace_back(direction_left_rearlight_);
    directions.emplace_back(direction_right_rearlight_);
  }

  void getVehicleDirection(double &direction) const {
    direction = direction_vh_;
  }

  void getMatchedRadarPoint(cv::Point2d &match) const {
    match.x = last_radar_pt_.x;
    match.y = last_radar_pt_.y;
  }

  void getCamExternal(part_based_tracking::SensorPose &param) const {
    param = cam_external_;
  }

  void transformLocal2Global(const cv::Point2d &local_pos,
                             const double local_yaw, cv::Point2d &global_pos,
                             double &global_yaw) {
    global_yaw = local_yaw + ego_yaw_latest_global_;

    global_pos.x = ego_pos_latest_global_.x + local_pos.x * std::cos(ego_yaw_latest_global_) -
                   local_pos.y * std::sin(ego_yaw_latest_global_);
    global_pos.y = ego_pos_latest_global_.y + local_pos.x * std::sin(ego_yaw_latest_global_) +
                   local_pos.y * std::cos(ego_yaw_latest_global_);
  }

  void reInit(const cv::Point2d &init_pos, const double init_yaw) {
    // particles_buffer_.clear();
    // particles_.initializeParticles(num_particles_, init_pos.x, init_pos.y,
    // 0.5,
    //                                init_yaw, 0.0);
    // ROS_WARN("re-init!");
  }

  void calcLineIntersection(const cv::Point2d &pt_1, const cv::Point2d &pt_2,
                            const double direc_1, const double direc_2,
                            cv::Point2d &intersec) {
    if (std::abs(direc_1 - M_PI / 2) < 1e-2 ||
        std::abs(direc_1 + M_PI / 2) < 1e-2) {
      // std::cout << "A" << std::endl;
      intersec.x = pt_1.x;
      double k_2 = std::tan(direc_2);
      intersec.y = k_2 * (pt_1.x - pt_2.x) + pt_2.y;
      return;
    }
    if (std::abs(direc_2 - M_PI / 2) < 1e-2 ||
        std::abs(direc_2 + M_PI / 2) < 1e-2) {
      // std::cout << "B" << std::endl;
      intersec.x = pt_2.x;
      double k_1 = std::tan(direc_1);
      intersec.y = k_1 * (pt_2.x - pt_1.x) + pt_1.y;
      return;
    }
    // std::cout << "C" << std::endl;
    double k_1 = std::tan(direc_1), k_2 = std::tan(direc_2);
    intersec.x = (-pt_1.y + pt_2.y + k_1 * pt_1.x - k_2 * pt_2.x) / (k_1 - k_2);
    intersec.y = (k_1 * pt_2.y - k_2 * pt_1.y + k_1 * k_2 * (pt_1.x - pt_2.x)) /
                 (k_1 - k_2);
  }

  void searchYawOnce(const double yaw, double &dis, cv::Point2d &intersec_1,
                     cv::Point2d &intersec_2,
                     const double direc_left = 2 * M_PI,
                     const double direc_right = 2 * M_PI,
                     const cv::Point2d &radar_pt = cv::Point2d(-1.0, 0.0)) {
    auto tangent = yaw + M_PI / 2;
    // std::cout << "search radar pt: " << last_radar_pt_ << std::endl;
    cv::Point2d radar_pt_apply = radar_pt.x > 0.0 ? radar_pt : last_radar_pt_;
    double direc_left_apply =
               direc_left < M_PI ? direc_left : direction_left_rearlight_,
           direc_right_apply =
               direc_right < M_PI ? direc_right : direction_right_rearlight_;

    calcLineIntersection(radar_pt_apply,
                         cv::Point2d(cam_external_.x, cam_external_.y), tangent,
                         direc_left_apply, intersec_1);
    calcLineIntersection(radar_pt_apply,
                         cv::Point2d(cam_external_.x, cam_external_.y), tangent,
                         direc_right_apply, intersec_2);
    dis = cv::norm(intersec_1 - intersec_2);
  }

  void searchYawCenter(double &yaw, cv::Point2d &center,
                       const double direc_left = 2 * M_PI,
                       const double direc_right = 2 * M_PI,
                       const cv::Point2d &radar_pt = cv::Point2d(-1.0, 0.0),
                       const double search_step = 0.01744,
                       const double yaw_range = 0.1744) {
    std::cout << "search left: " << direc_left << " search right: " << direc_right << std::endl;
    auto valid_radar_pt = radar_pt.x > 0.0 ? radar_pt : last_radar_pt_;
    if (std::abs(direc_left - direc_right) < 5.0 / 180 * M_PI) {
      center = valid_radar_pt;
      if (!checkCenterValid(center, direc_left, direc_right)) {
        center.x = -1.0;
      }
      front_vh_rear_center_proposal_ = center;
      std::cout << "two rearlight too close" << std::endl;
      return;
    }
    auto yaw_local = yaw - ego_yaw_global_;
    // std::cout << "search yaw center: " << yaw_local << std::endl;
    auto yaw_low = yaw_local - yaw_range;
    auto yaw_high = yaw_local + yaw_range;
    std::vector<double> candidates_dis;
    std::vector<std::pair<cv::Point2d, cv::Point2d>> candidates_pts;
    auto left_right_dis = cv::norm(
        geometry_model_[part_based_tracking::VehiclePartType::LEFTREARLIGHT] -
        geometry_model_[part_based_tracking::VehiclePartType::RIGHTREARLIGHT]);
    // std::cout << "expected dis = " << left_right_dis << std::endl;
    bool is_valid_exist = false;
    for (double theta = yaw_low; theta <= yaw_high; theta += search_step) {
      double dis;
      cv::Point2d intersec_1, intersec_2;
      searchYawOnce(theta, dis, intersec_1, intersec_2, direc_left, direc_right,
                    radar_pt);
      auto c_pt = (intersec_1 + intersec_2) / 2;
      auto direc_c_pt =
          std::atan2(c_pt.y - radar_external_.y, c_pt.x - radar_external_.x);
      if (direc_c_pt > direc_left + 10.0 / 180.0 * M_PI ||
          direc_c_pt < direc_right - 10.0 / 180.0 * M_PI) {
        // invalid candidate
        candidates_dis.emplace_back(DBL_MAX);
      } else {
        candidates_dis.emplace_back(std::abs(dis - left_right_dis));
        is_valid_exist = true;
      }
      candidates_pts.emplace_back(std::make_pair(intersec_1, intersec_2));
      std::cout << "theta: " << theta << " dis: " << dis << std::endl;
      std::cout << "center: " << c_pt << std::endl;
    }

    if (!is_valid_exist) {
      center = valid_radar_pt;
      if (!checkCenterValid(center, direc_left, direc_right)) {
        center.x = -1.0;
      }
      front_vh_rear_center_proposal_ = center;
      std::cout << "no valid candidate found" << std::endl;
      return;
    }
    int best_idx =
        std::min_element(candidates_dis.begin(), candidates_dis.end()) -
        candidates_dis.begin();
    center =
        (candidates_pts[best_idx].first + candidates_pts[best_idx].second) / 2;
    if (cv::norm(center - cv::Point2d(radar_external_.x, radar_external_.y)) <
        1.5) {
      center = valid_radar_pt;
      front_vh_rear_center_proposal_ = center;
      std::cout << "too close to radar sensor" << std::endl;
    } else if (cv::norm(center - valid_radar_pt) > 2.0) {
      center = valid_radar_pt;
      front_vh_rear_center_proposal_ = center;
      std::cout << "too far" << std::endl;
    }

    if (!checkCenterValid(center, direc_left, direc_right)) {
      center.x = -1.0;
    }

    std::cout << "best yaw: " << yaw_low + search_step * best_idx << std::endl;
    std::cout << "best center: " << center << std::endl;

    front_vh_rear_center_proposal_ = center;
    yaw = yaw_low + best_idx * search_step;
  }

  bool checkCenterValid(const cv::Point2d center, const double direc_left,
                        const double direc_right) const {
    auto direc_center =
        std::atan2(center.y - radar_external_.y, center.x - radar_external_.x);
    if (direc_center > direc_left + 5.0 / 180.0 * M_PI ||
        direc_center < direc_right - 5.0 / 180.0 * M_PI) {
      return false;
    }
    return true;
  }

  void simpleFindCenter(cv::Point2d &center, const double direc_left = 2 * M_PI,
                        const double direc_right = 2 * M_PI,
                        const cv::Point2d &radar_pt = cv::Point2d(-1.0, 0.0)) {
    center = radar_pt.x > 0.0 ? radar_pt : last_radar_pt_;
  }

  void combineFindCenter(cv::Point2d &center,
                         const double direc_left = 2 * M_PI,
                         const double direc_right = 2 * M_PI,
                         const cv::Point2d &radar_pt = cv::Point2d(-1.0, 0.0)) {
    double direc_left_apply =
        direc_left < M_PI ? direc_left : direction_left_rearlight_;
    double direc_right_apply =
        direc_right < M_PI ? direc_right : direction_right_rearlight_;
    cv::Point2d radar_pt_apply = radar_pt.x > 0.0 ? radar_pt : last_radar_pt_;

    cv::Point2d left_pt{radar_pt_apply.x, 0.0}, right_pt{radar_pt_apply.x, 0.0};
    left_pt.y = std::tan(direc_left_apply) * (left_pt.x - cam_external_.x) +
                cam_external_.y;
    right_pt.y = std::tan(direc_right_apply) * (right_pt.x - cam_external_.x) +
                 cam_external_.y;

    center = (left_pt + right_pt) / 2;
  }

  // function for ablation study
  void setRearlightAllUse(const bool use) { use_rearlight_all_ = use; }
  void setRearlightLeftUse(const bool use) { use_rearlight_left_ = use; }
  void setRearlightRightUse(const bool use) { use_rearlight_right_ = use; }
  void setRadarUse(const bool use) { use_radar_ = use; }
  void setV2VUse(const bool use) { use_v2v_ = use; }
};

class VehicleTrackingWrapper {
public:
  VehicleTrackingWrapper(ros::NodeHandle node_handle,
                         ros::NodeHandle private_node_handle);
  ~VehicleTrackingWrapper() = default;

  void getCurrState(const ros::TimerEvent &event);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_image_;
  ros::Subscriber sub_radar_;
  ros::Subscriber sub_vis_det_;
  ros::Subscriber sub_rearlight_bbox_;
  ros::Subscriber sub_rearlight_left_;
  ros::Subscriber sub_rearlight_right_;
  ros::Subscriber sub_rearlight_pseudo_left_;
  ros::Subscriber sub_rearlight_pseudo_right_;
  ros::Subscriber sub_vh_bbox_;
  ros::Subscriber sub_v2v_;
  ros::Subscriber sub_leader_imu_;
  ros::Subscriber sub_ego_imu_;
  ros::Subscriber sub_ego_speed_;
  ros::Subscriber sub_lidar_;
  // subscriber for ablation study
  ros::Subscriber sub_ablation_rearlight_all_;
  ros::Subscriber sub_ablation_rearlight_left_;
  ros::Subscriber sub_ablation_rearlight_right_;
  ros::Subscriber sub_ablation_radar_;
  ros::Subscriber sub_ablation_v2v_;

  ros::Publisher pub_marker_;
  ros::Publisher pub_vis_rematch_;
  ros::Publisher pub_vh_vis_rematch_;
  ros::Publisher pub_pose_lidar_;
  ros::Publisher pub_target_track_vis_;
  ros::Publisher pub_left_err_, pub_right_err_;
  ros::Publisher pub_left_pos_, pub_right_pos_;
  ros::Publisher pub_front_relative_pose_;
  ros::Publisher pub_front_global_pose_;
  ros::Publisher pub_lost_;
  ros::Timer timer_;

  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster br_;
  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  VehicleTracking tracker_;
  std::string method_;

  cv::Mat image_;
  std_msgs::Header image_msg_header_;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr
      radar_; // radar transformed to vehicle frame(base_link)
  std_msgs::Header radar_msg_header_;
  std_msgs::Header lidar_msg_header_;
  std::vector<std::pair<cyber_msgs::Box2D, std_msgs::Header>> vis_det_;
  std::vector<std::pair<cv::Rect2d, std_msgs::Header>> rearlight_bbox_;
  std::pair<cv::Rect2d, std_msgs::Header> vh_bbox_;

  cv::Point2d last_target_pos_;
  double last_target_yaw_;

  int frame_;

  std::deque<geometry_msgs::Pose2D> target_pose_buffer_;

  void image_callback(const sensor_msgs::CompressedImageConstPtr &image_in);
  void radar_callback(const sensor_msgs::PointCloud2ConstPtr &radar_in);
  void v2v_callback(const cyber_msgs::V2VPacketConstPtr &msg);
  void leader_imu_callback(const sensor_msgs::ImuConstPtr &msg);
  void ego_imu_callback(const sensor_msgs::ImuConstPtr &msg);
  void ego_speed_callback(const cyber_msgs::SpeedFeedbackConstPtr &msg);
  void vis_det_callback(const cyber_msgs::Box2DArrayConstPtr &det_in);
  void rearlight_callback(const cyber_msgs::Box2DArrayConstPtr &msg);
  void rearlight_left_callback(const cyber_msgs::Box2DConstPtr &msg);
  void rearlight_right_callback(const cyber_msgs::Box2DConstPtr &msg);
  void rearlight_pseudo_left_callback(const cyber_msgs::Box2DConstPtr &msg);
  void rearlight_pseudo_right_callback(const cyber_msgs::Box2DConstPtr &msg);
  void vh_callback(const cyber_msgs::Box2DConstPtr &msg);
  void lidar_callback(const sensor_msgs::PointCloud2ConstPtr &msg);
  // callback function for ablation study
  void ablation_rearlight_all_callback(const std_msgs::Bool &msg) {
    tracker_.setRearlightAllUse(!msg.data);
    ROS_WARN("ablation rearlight_all_callback");
  }
  void ablation_rearlight_left_callback(const std_msgs::Bool &msg) {
    tracker_.setRearlightLeftUse(!msg.data);
    ROS_WARN("ablation rearlight_left_callback");
  }
  void ablation_rearlight_right_callback(const std_msgs::Bool &msg) {
    tracker_.setRearlightRightUse(!msg.data);
    ROS_WARN("ablation rearlight_right_callback");
  }
  void ablation_radar_callback(const std_msgs::Bool &msg) {
    tracker_.setRadarUse(!msg.data);
    ROS_WARN("ablation radar_callback");
  }
  void ablation_v2v_callback(const std_msgs::Bool &msg) {
    tracker_.setV2VUse(!msg.data);
    ROS_WARN("ablation v2v_callback");
  }

  void visualize_ray(const double angle, const int id,
                     const std::string &color = "r") const;
  void visualize_point(const cv::Point2d &position, const int id,
                       const std::string &color = "g") const;
  void visualize_point(const cv::Point2d &position, const double r_x,
                       const double r_y, const int id,
                       const std::string &color = "g") const;
  void updateTargetPoseBuffer(const geometry_msgs::Pose2D &pose) {
    target_pose_buffer_.emplace_back(pose);
    if (target_pose_buffer_.size() > 1000)
      target_pose_buffer_.pop_front();
  }
};

} // namespace tracking
