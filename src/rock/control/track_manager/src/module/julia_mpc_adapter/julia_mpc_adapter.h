/**
 * @file julia_mpc_adpter.h
 * @author Chen Hongxin (angelochen@live.cn)
 * @brief adapter class for communication with Julia Altro MPC solver by zmq
 * @version 0.1
 * @date 2023-10-15
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <atomic>
#include <mutex>
#include <thread>

#include "cyber_msgs/PlatoonMPCDebug.h"
#include "cyber_msgs/brakecmd.h"
#include "cyber_msgs/speedcmd.h"
#include "cyber_msgs/steercmd.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"

#include "discretized_trajectory.h"
#include "speed_predictor.h"
#include "track_manager_type.h"

namespace cyberc3 {
namespace planning {

class JuliaMPCAdapter {
 public:
  explicit JuliaMPCAdapter(
      const ExpectDistanceConfig& dis_config,
      const std::shared_ptr<BaseSpeedPredictor>& speed_predictor_ptr);
  ~JuliaMPCAdapter();
  void UpdateSpline(const SplineSmoothResult& spline,
                    const DiscretizedTrajectory& traj);
  void UpdateEgoState(const VehicleState& ego_state);
  geometry_msgs::PoseArray GetMPCTrajectory();
  bool GetMPCCommands(cyber_msgs::steercmd* steer_cmd,
                      cyber_msgs::brakecmd* brake_cmd,
                      cyber_msgs::speedcmd* speed_cmd);
  bool GetMPCDebug(cyber_msgs::PlatoonMPCDebug* mpc_debug);
  void SetAutoDriveMode(const bool is_auto_drive) {
    is_auto_drive_.store(is_auto_drive);
  }
  void Reset();

 private:
  bool CalculateEgoSInSpline(double* ego_s_in_spline) const;
  bool CalculateSVRefs(const double timestamp,
                       std::array<double, N_HORIZON>* s_refs,
                       std::array<double, N_HORIZON>* v_refs) const;
  double GetEgoSWithPrediction(double cur_time) const {
    const double ego_predict_time = cur_time - ego_state_.timestamp;
    return ego_state_.s + ego_state_.speed * ego_predict_time;
  }
  void ZMQLoop();

  // PlatoonMPCInput mpc_input_ = {};
  PlatoonMPCResult mpc_result_ = {};
  std::thread zmq_thread_;
  std::mutex input_mutex_;
  std::mutex result_mutex_;
  std::atomic<bool> running_{true};
  std::atomic<bool> is_auto_drive_{
      false};  // whether the vehicle is in auto drive mode

  std::atomic<double> last_acc_cmd_{0};    // latest published acc command
  std::atomic<double> last_steer_cmd_{0};  // latest published steer command
  VehicleState ego_state_{};
  SplineSmoothResult smoothed_spline_{};
  DiscretizedTrajectory smoothed_track_;
  ExpectDistanceConfig dis_config_;
  std::shared_ptr<BaseSpeedPredictor> speed_predictor_;
};
}  // namespace planning
}  // namespace cyberc3
