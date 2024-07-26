/**
 * @file julia_mpc_adpter.cc
 * @author Chen Hongxin (angelochen@live.cn)
 * @brief adapter class for communication with Julia Altro MPC solver by zmq
 * @version 0.1
 * @date 2023-10-15
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "julia_mpc_adapter.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <mutex>
#include <string>

#include <zmq.hpp>

#include "ros/ros.h"
#include "tf/transform_datatypes.h"

#include "common/log.h"
#include "common/math/math_utils.h"
#include "track_manager_type.h"

namespace cyberc3 {
namespace planning {

constexpr size_t X = 0;  // index of X in MPC state defintion
constexpr size_t Y = 1;
constexpr size_t S = 2;
constexpr size_t V = 3;
constexpr size_t A = 4;
constexpr size_t YAW = 5;
constexpr size_t STEER = 6;

constexpr double steer_ratio = 16.5;
constexpr double steer_limit = 460.0 / 180 * M_PI / steer_ratio;
constexpr double velocity_limit = 10;
constexpr double acc_limit_upper = 2.5;
constexpr double acc_limit_lower = -4.0;

namespace vehicle_gear_cmd {
constexpr signed char D = 1;
constexpr signed char R = 2;
}  // namespace vehicle_gear_cmd

void MapAccToCmd(const double target_acc, cyber_msgs::speedcmd* speed_cmd,
                 cyber_msgs::brakecmd* brake_cmd) {
  constexpr int THROTTLE_MAX = 3000;
  constexpr int THROTTLE_MIN = 0;
  constexpr double BRAKE_MAX = 0;
  constexpr double BRAKE_MIN = -5.0;
  double k0 = 51.45;
  double k1 = 794.8;

  double throttle_cmd = k0 + k1 * target_acc;
  if (throttle_cmd > 0) {
    if (throttle_cmd > THROTTLE_MAX) {
      throttle_cmd = THROTTLE_MAX;
    } else if (throttle_cmd < THROTTLE_MIN) {
      throttle_cmd = THROTTLE_MIN;
    }
    speed_cmd->speed_cmd = throttle_cmd;
    speed_cmd->gear = vehicle_gear_cmd::D;
    speed_cmd->enable_auto_speed = true;
    speed_cmd->is_updated = true;

    brake_cmd->deceleration = 0;
    brake_cmd->enable_auto_brake = false;
  } else {
    double brake_acc = target_acc;
    if (brake_acc > BRAKE_MAX) {
      brake_acc = BRAKE_MAX;
    } else if (brake_acc < BRAKE_MIN) {
      brake_acc = BRAKE_MIN;
    }
    speed_cmd->speed_cmd = 0;
    speed_cmd->enable_auto_speed = false;
    speed_cmd->is_updated = true;

    brake_cmd->deceleration = brake_acc;
    brake_cmd->enable_auto_brake = true;
  }
  speed_cmd->acc_cmd = target_acc;  // not used, only for debug
}

JuliaMPCAdapter::JuliaMPCAdapter(
    const ExpectDistanceConfig& dis_config,
    const std::shared_ptr<BaseSpeedPredictor>& speed_predictor_ptr)
    : dis_config_(dis_config), speed_predictor_(speed_predictor_ptr) {
  zmq_thread_ = std::thread(&JuliaMPCAdapter::ZMQLoop, this);
}

JuliaMPCAdapter::~JuliaMPCAdapter() {
  running_ = false;
  zmq_thread_.join();
}

void JuliaMPCAdapter::UpdateSpline(const SplineSmoothResult& spline,
                                   const DiscretizedTrajectory& traj) {
  std::lock_guard<std::mutex> lock_guard(input_mutex_);
  smoothed_spline_ = spline;
  smoothed_track_ = traj;
}

void JuliaMPCAdapter::UpdateEgoState(const VehicleState& ego_state) {
  std::lock_guard<std::mutex> lock_guard(input_mutex_);
  ego_state_ = ego_state;
}

bool JuliaMPCAdapter::CalculateEgoSInSpline(double* ego_s_in_spline) const {
  const common::math::Vec2d ego_vec = {ego_state_.x, ego_state_.y};
  const size_t ego_idx_in_spline = smoothed_track_.QueryNearestPoint(ego_vec);
  if (ego_idx_in_spline == 0 ||
      ego_idx_in_spline == smoothed_track_.NumOfPoints() - 1) {
    AERROR << "Ego vehicle is out of smoothed track!";
    return false;
  }
  *ego_s_in_spline =
      smoothed_track_.TrajectoryPointAt(ego_idx_in_spline).path_point().s();
  return true;
}

bool JuliaMPCAdapter::CalculateSVRefs(
    const double timestamp, std::array<double, N_HORIZON>* s_refs,
    std::array<double, N_HORIZON>* v_refs) const {
  //  predict target speeds and distances in MPC horizon
  std::array<double, N_HORIZON> predicted_target_s{};
  std::array<double, N_HORIZON> predicted_speeds{};
  if (!speed_predictor_->GetPredictedSpeedProfiles(
          timestamp, &predicted_target_s, &predicted_speeds)) {
    AWARN << "Get target predicted speed profiles failed!";
    return false;
  }
  for (int i = 0; i < N_HORIZON; ++i) {
    double distance_expect = dis_config_.dis_base +
                             dis_config_.dis_speed_factor * predicted_speeds[i];
    distance_expect = std::min(distance_expect, dis_config_.dis_max);
    distance_expect = std::max(distance_expect, dis_config_.dis_min);
    s_refs->at(i) =
        predicted_target_s[i] - distance_expect - dis_config_.dis_offset;
    v_refs->at(i) = predicted_speeds[i];
  }
  return true;
}

geometry_msgs::PoseArray JuliaMPCAdapter::GetMPCTrajectory() {
  std::lock_guard<std::mutex> lock_guard(result_mutex_);
  if (mpc_result_.status.empty() ||
      ros::Time::now().toSec() > mpc_result_.timestamp + T_final) {
    return {};
  }
  geometry_msgs::PoseArray mpc_traj;
  mpc_traj.header.frame_id = "map";
  mpc_traj.header.stamp = ros::Time::now();
  mpc_traj.poses.reserve(N_HORIZON);
  for (const auto& state : mpc_result_.states) {
    geometry_msgs::Pose pose;
    pose.position.x = state[X];
    pose.position.y = state[Y];
    pose.orientation = tf::createQuaternionMsgFromYaw(state[YAW]);
    mpc_traj.poses.push_back(pose);
  }
  return mpc_traj;
}

bool JuliaMPCAdapter::GetMPCCommands(cyber_msgs::steercmd* steer_cmd,
                                     cyber_msgs::brakecmd* brake_cmd,
                                     cyber_msgs::speedcmd* speed_cmd) {
  std::lock_guard<std::mutex> lock_guard(result_mutex_);
  const double cur_time = ros::Time::now().toSec();
  if (mpc_result_.status.empty() ||
      cur_time > mpc_result_.timestamp + T_final) {
    AERROR << "MPC result timeout! time diff: "
           << cur_time - mpc_result_.timestamp;
    return false;
  }
  const int idx = static_cast<int>((cur_time - mpc_result_.timestamp) / T_step);
  const double ratio = (cur_time - mpc_result_.timestamp) / T_step - idx;
  if (idx < 0 || idx >= N_HORIZON - 1 || ratio < 0 || ratio > 1) {
    AERROR << "MPC current index is not valid! cur idx: " << idx
           << " ratio: " << ratio << " Horizon: " << N_HORIZON;
    return false;
  }
  double wheel_angle_rad = (1 - ratio) * mpc_result_.states[idx][STEER] +
                           ratio * mpc_result_.states[idx + 1][STEER];
  double steer_angle_deg = wheel_angle_rad / M_PI * 180 * steer_ratio;
  double target_v = (1 - ratio) * mpc_result_.states[idx][V] +
                    ratio * mpc_result_.states[idx + 1][V];
  double target_acc = (1 - ratio) * mpc_result_.states[idx][A] +
                      ratio * mpc_result_.states[idx + 1][A];
  // check control limits is satisfied
  constexpr double eps = 1e-3;
  if (std::abs(wheel_angle_rad) > steer_limit + eps ||
      target_v > (velocity_limit + eps) || target_v < -eps ||
      target_acc > (acc_limit_upper + eps) ||
      target_acc < (acc_limit_lower - eps)) {
    AERROR << "MPC state constrain check failed!";
    if (std::abs(wheel_angle_rad) > steer_limit) {
      AERROR << "steer angle check failed: " << wheel_angle_rad
             << "\nlimit: " << steer_limit;
    }
    if (target_v > velocity_limit || target_v < 0) {
      AERROR << "velocity check failed! " << target_v
             << "\nlimit: " << velocity_limit;
    }
    if (target_acc > acc_limit_upper || target_acc < acc_limit_lower) {
      AERROR << "acc check failed! " << target_acc
             << "\nlimit: " << acc_limit_upper << " " << acc_limit_lower;
    }
    return false;
  }

  double leader_s = 0.0;
  double leader_speed = 0.0;
  double leader_acc = 0.0;
  if (!speed_predictor_->GetTargetStates(cur_time, &leader_s, &leader_speed,
                                         &leader_acc)) {
    AWARN << "Get target states failed!";
    return false;
  }
  // check acc for stop state
  constexpr double stop_ref_speed_threshold = 0.15;
  constexpr double stop_acc_cmd = -1.5;
  double target_acc_with_stop = target_acc;
  if (leader_speed < stop_ref_speed_threshold) {
    target_acc_with_stop = std::min(target_acc, stop_acc_cmd);
  }

  steer_cmd->header.stamp = ros::Time::now();
  steer_cmd->enable_auto_steer = true;
  steer_cmd->is_updated = true;
  steer_cmd->steer_cmd = steer_angle_deg;
  MapAccToCmd(target_acc_with_stop, speed_cmd, brake_cmd);
  last_acc_cmd_.store(target_acc);
  last_steer_cmd_.store(wheel_angle_rad);
  return true;
}

void JuliaMPCAdapter::ZMQLoop() {
  zmq::context_t context(1);
  zmq::socket_t socket(context, zmq::socket_type::req);
  AINFO << "Connecting to Julia MPC server...";
  socket.connect("tcp://localhost:5555");
  double last_success_time = 0;
  double last_log_time = 0;
  constexpr double send_interval = 0.05;  // 20Hz
  constexpr double log_interval = 0.5;    // 2Hz
  size_t seq = 0;
  while (running_ && ros::ok()) {
    const double cur_time = ros::Time::now().toSec();
    if (cur_time - last_success_time < send_interval) {
      continue;
    }
    {
      // check input states
      std::lock_guard<std::mutex> lock_guard(input_mutex_);
      if (cur_time - ego_state_.timestamp > 0.1) {
        if (cur_time - last_log_time > log_interval) {
          if (ego_state_.timestamp < 1) {
            AWARN << "Ego state is not ready!";
          } else {
            AWARN << "Ego state timeout! diff: "
                  << cur_time - ego_state_.timestamp;
          }
          last_log_time = cur_time;
        }
        continue;
      }
      if (!speed_predictor_->CheckTargetStateValidty(cur_time)) {
        if (cur_time - last_log_time > log_interval) {
          AWARN << "Target state is not ready!";
          last_log_time = cur_time;
        }
        continue;
      }
      if (smoothed_spline_.x_coefs.empty()) {
        if (cur_time - last_log_time > log_interval) {
          AWARN << "spline is not ready!";
          last_log_time = cur_time;
        }
        continue;
      }
      double ego_s_in_spline = 0;
      if (!CalculateEgoSInSpline(&ego_s_in_spline)) {
        if (cur_time - last_log_time > log_interval) {
          AWARN << "Target track is not valid!";
          last_log_time = cur_time;
        }
        continue;
      }
      PlatoonMPCInput mpc_input;
      if (!CalculateSVRefs(cur_time, &mpc_input.s_refs, &mpc_input.v_refs)) {
        AWARN << "Calculate s and speed references failed!";
        continue;
      }
      mpc_input.timestamp = ego_state_.timestamp;
      mpc_input.seq = seq++;
      mpc_input.spline = smoothed_spline_;
      mpc_input.initial_state[X] = ego_state_.x;
      mpc_input.initial_state[Y] = ego_state_.y;
      // limit input states to avoid insolveable initial state
      const double speed_limited =
          std::min(std::max(ego_state_.speed, 0.0), velocity_limit);
      double acc_limited = std::min(std::max(ego_state_.acc_x, acc_limit_lower),
                                    acc_limit_upper);
      double steer_limited =
          std::min(std::max(ego_state_.steer_angle, -steer_limit), steer_limit);
      // in auto drive mode, use last cmd to avoid sudden change of commands
      if (is_auto_drive_) {
        acc_limited = std::min(std::max(last_acc_cmd_.load(), acc_limit_lower),
                               acc_limit_upper);
        steer_limited = std::min(std::max(last_steer_cmd_.load(), -steer_limit),
                                 steer_limit);
      }
      mpc_input.initial_state[V] = speed_limited;
      mpc_input.initial_state[A] = acc_limited;
      mpc_input.initial_state[YAW] = ego_state_.theta;
      mpc_input.initial_state[STEER] = steer_limited;
      // compensate time delay between perception and control
      mpc_input.initial_state[S] = GetEgoSWithPrediction(cur_time);
      // compensate s offset between raw track and smoothed track
      mpc_input.spline.start_s +=
          (mpc_input.initial_state[S] - ego_s_in_spline);
      std::string input_string = json(mpc_input).dump();
      socket.send(input_string.data(), input_string.size());
      AINFO << "Send MPC request: " << mpc_input.seq;
    }
    {
      // get mpc result from zmq socket
      zmq::message_t reply;
      socket.recv(&reply);
      auto j = json::parse(static_cast<char*>(reply.data()),
                           static_cast<char*>(reply.data()) + reply.size());
      const std::string solve_status = j["status"];
      if (solve_status == "SOLVE_SUCCEEDED" ||
          solve_status == "SOLVE_SUCCEEDED SOLVE_SUCCEEDED") {
        std::lock_guard<std::mutex> lock(result_mutex_);
        mpc_result_ = PlatoonMPCResult(j);
        last_success_time = cur_time;
      } else {
        AWARN << "MPC solve failed, status: " << solve_status;
        // todo(chx): check whether result is usable
      }
      AINFO << "Received MPC result: " << solve_status;
      AINFO << "MPC solve time: " << mpc_result_.solve_time_ms << "ms";
      AINFO << "MPC sum time: " << (ros::Time::now().toSec() - cur_time) * 1000
            << "ms";
    }
  }
}

void JuliaMPCAdapter::Reset() {
  // stop zmq thread
  running_.store(false);
  zmq_thread_.join();
  {
    std::lock_guard<std::mutex> lock_guard(input_mutex_);
    ego_state_ = {};
    smoothed_spline_ = {};
    smoothed_track_ = {};
  }
  {
    std::lock_guard<std::mutex> lock_guard(result_mutex_);
    mpc_result_ = {};
  }
  last_acc_cmd_.store(0);
  last_steer_cmd_.store(0);
  // restart zmq thread
  running_.store(true);
  zmq_thread_ = std::thread(&JuliaMPCAdapter::ZMQLoop, this);
}

bool JuliaMPCAdapter::GetMPCDebug(cyber_msgs::PlatoonMPCDebug* mpc_debug) {
  static uint32_t last_seq = -1;
  const double cur_time = ros::Time::now().toSec();
  if (mpc_debug == nullptr) {
    AERROR << "input pointer is nullptr!";
    return false;
  }
  // skip if mpc result is not updated
  // if (mpc_result_.seq == last_seq) {
  //   return false;
  // }
  {
    std::lock_guard<std::mutex> lock(result_mutex_);
    if (mpc_result_.timestamp < 1) {
      AWARN << "MPC result empty!";
      return false;
    }
    mpc_debug->header.stamp.fromSec(mpc_result_.timestamp);
    mpc_debug->solve_time_ms = mpc_result_.solve_time_ms;
    last_seq = mpc_result_.seq;
  }
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    mpc_debug->speed_feedback = ego_state_.speed;
    mpc_debug->acc_expect = ego_state_.acc_x;
    mpc_debug->s_ego = GetEgoSWithPrediction(cur_time);
    common::math::Vec2d ego_vec(ego_state_.x, ego_state_.y);
    const auto nearest_idx = smoothed_track_.QueryNearestPoint(ego_vec);
    const auto& nearest_point = smoothed_track_[nearest_idx];
    common::math::Vec2d ref_point(nearest_point.path_point().x(),
                                  nearest_point.path_point().y());
    auto ref_direction = common::math::Vec2d::CreateUnitVec2d(
        nearest_point.path_point().theta());
    mpc_debug->lateral_error = (ego_vec - ref_point).CrossProd(ref_direction);
    mpc_debug->heading_error = common::math::NormalizeAngle(
        nearest_point.path_point().theta() - ego_state_.theta);
  }
  if (!speed_predictor_->GetTargetStates(cur_time, &mpc_debug->s_target,
                                         &mpc_debug->speed_expect,
                                         &mpc_debug->acc_expect)) {
    AWARN << "Get target states failed!";
    return false;
  }
  mpc_debug->imm_probs = speed_predictor_->GetIMMProbs();
  mpc_debug->state_predicts = speed_predictor_->GetStatePredictDebugs(cur_time);
  mpc_debug->distance_expect =
      std::max(dis_config_.dis_min,
               std::min(dis_config_.dis_max,
                        dis_config_.dis_base + dis_config_.dis_speed_factor *
                                                   mpc_debug->speed_expect));
  mpc_debug->distance_feedback =
      mpc_debug->s_target - mpc_debug->s_ego - dis_config_.dis_offset;
  mpc_debug->distance_error =
      -(mpc_debug->distance_expect - mpc_debug->distance_feedback);
  mpc_debug->speed_error = mpc_debug->speed_expect - mpc_debug->speed_feedback;
  mpc_debug->s_expect =
      mpc_debug->s_target - mpc_debug->distance_expect - dis_config_.dis_offset;
  mpc_debug->acc_cmd = last_acc_cmd_.load();
  mpc_debug->steer_cmd = last_steer_cmd_.load() / M_PI * 180 * steer_ratio;
  return true;
}

}  // namespace planning
}  // namespace cyberc3
