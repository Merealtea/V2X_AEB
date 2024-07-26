/**
 *common.h
 *brief: common structs
 *author:Chen Xiaofeng
 *date:20201021
 **/

#pragma once

#include <array>
#include <cstddef>
#include <ostream>
#include <vector>

#include "json.hpp"

using json = nlohmann::json;

namespace cyberc3 {
namespace planning {

enum class SmootherType { Discrete, Spline };
struct SmoothConfig {
  double resmooth_threshold = 2.0;
  double behind_distance = 2.0;  // distance behind ego vehicle used for smooth
  SmootherType smoother_type = SmootherType::Spline;
};

struct ExpectDistanceConfig {
  double dis_base = 4.0;
  double dis_speed_factor = 0.5;
  double dis_max = 8.5;
  double dis_min = 4.0;
  double dis_offset = 3.55;  // distance offset between ego rear and head
};

struct V2VConfig {
  double acc_filter_cutoff_freq = 2.0;
  double use_acc_threshold = 0.15;
  double delay_threshold = 0.1;
};

struct TrackManagerConfig {
  double track_interval = 0.05;
  int max_track_size = 2000;
  double timeout_threshold = 1.0;
  double distance_filter_cutoff_freq = 2.0;
  ExpectDistanceConfig dis_config;
  SmoothConfig smooth_config;
  V2VConfig v2v_config;
  bool enable_mpc_adapter = false;
};

struct V2VPacket {
  bool is_updated;
  double timestamp;
  double acc_x;
  double acc_y;
  double speed;
  double yaw_rate;
  double steer_angle;
};

struct VehicleState {
  double timestamp;
  double
      timestamp_perception;  // timestamp of perception, only for target vehilce
  double x;
  double y;
  double theta;
  double speed;
  double speed_perception;  // speed from perception, only for target vehilce
  double acc_x;
  double acc_y;
  double yaw_rate;
  double steer_angle;
  double s;  // cumulative distance in the target track
};

struct PlatoonControlCommand {
  double distance_expect;
  double distance_feedback;
  double speed_ref;
  double acc_ref;
  double speed_limit_curvature;
  double acc_limit_curvature;
  double speed_limit_steer;
  double acc_limit_steer;
};

struct PlatoonControlDebug {
  double distance_expect;
  double distance_feedback_curve;
  double distance_feedback_perception;
  double distance_error;
  double speed_ego;
  double speed_front_perception;
  double speed_front_v2v;
  double speed_error;
  double acc_front_v2v;
  double v2v_delay_ms;
  double speed_limit_curvature;
  double acc_limit_curvature;
  double speed_limit_steer;
  double acc_limit_steer;
  double lateral_error;
  double heading_error;
};

constexpr int N_SPLINE = 6;  // number of spline coefs, should be spline_order+1
constexpr int N_HORIZON = 61;
constexpr int N_STATES = 7;
constexpr int N_CONTROLS = 2;

constexpr double T_final = 3.0;  // MPC prediction time
constexpr double T_step = T_final / N_HORIZON;

struct SplineSmoothResult {
  std::array<double, N_SPLINE> x_coefs;
  std::array<double, N_SPLINE> y_coefs;
  double start_s;  // t= (s-start_s)/length
  double length;
};

struct PlatoonMPCInput {
  double timestamp = 0;
  uint64_t seq = 0;
  SplineSmoothResult spline{};
  std::array<double, N_STATES> initial_state{};
  std::array<double, N_HORIZON> s_refs{};
  std::array<double, N_HORIZON> v_refs{};
};

struct PlatoonMPCResult {
  double timestamp = 0;
  uint64_t seq = 0;
  std::string status;
  double solve_time_ms;
  std::array<std::array<double, N_STATES>, N_HORIZON> states;
  std::array<std::array<double, N_CONTROLS>, N_HORIZON - 1> controls;
  // std::array<std::array<std::array<double, N_STATES>, N_CONTROLS>,
  //            N_HORIZON - 1>
  //     feedback_gains;
  std::array<std::vector<std::vector<double>>, N_HORIZON - 1> feedback_gains;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SplineSmoothResult, x_coefs, y_coefs,
                                   start_s, length);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PlatoonMPCInput, timestamp, seq, spline,
                                   initial_state, s_refs, v_refs);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PlatoonMPCResult, timestamp, seq, status,
                                   solve_time_ms, states, controls,
                                   feedback_gains);

}  // namespace planning
}  // namespace cyberc3
