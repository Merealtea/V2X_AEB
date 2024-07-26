/**
 * @file speed_predictor.h
 * @author Chen Hongxin (angelochen@live.cn)
 * @brief
 * @version 0.1
 * @date 2023-11-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <array>
#include <mutex>

#include "common/log.h"
#include "track_manager_type.h"

namespace cyberc3 {
namespace planning {

class BaseSpeedPredictor {
 public:
  BaseSpeedPredictor() = default;
  virtual ~BaseSpeedPredictor() = default;
  virtual void FeedTargetV2V(const V2VPacket& v2v_msg);
  virtual void FeedTargetPerception(const VehicleState& target_state);
  virtual bool GetPredictedSpeedProfiles(
      double timestamp, std::array<double, N_HORIZON>* predicted_s,
      std::array<double, N_HORIZON>* predicted_speeds);
  virtual bool GetTargetStates(double timestamp, double* target_s,
                               double* target_speed,
                               double* target_acc);  // only for debug
  virtual std::vector<double> GetIMMProbs() { return {}; };
  virtual std::vector<double> GetStatePredictDebugs(double /*timestamp*/) {
    return {};
  };

  bool CheckTargetStateValidty(double cur_time) const {
    static double last_log_time = 0.0;
    constexpr double log_interval = 0.5;
    constexpr double v2v_timeout_threshold = 0.5;
    constexpr double perception_timeout_threshold = 1.0;
    if (target_state_.timestamp < 1.0) {
      if (cur_time - last_log_time > log_interval) {
        AERROR << "Target V2V is not ready!";
        last_log_time = cur_time;
      }
      return false;
    }
    if (target_state_.timestamp_perception < 1.0) {
      if (cur_time - last_log_time > log_interval) {
        AERROR << "Target Perception is not ready!";
        last_log_time = cur_time;
      }
      return false;
    }
    if (cur_time - target_state_.timestamp > v2v_timeout_threshold) {
      if (cur_time - last_log_time > log_interval) {
        AERROR << "Target V2V timeout!";
        last_log_time = cur_time;
      }
      return false;
    }
    if (cur_time - target_state_.timestamp_perception >
        perception_timeout_threshold) {
      if (cur_time - last_log_time > log_interval) {
        AERROR << "Target V2V timeout!";
        last_log_time = cur_time;
      }
      return false;
    }
    return true;
  }

 protected:
  std::mutex mutex_;
  VehicleState target_state_;
};
}  // namespace planning
}  // namespace cyberc3
