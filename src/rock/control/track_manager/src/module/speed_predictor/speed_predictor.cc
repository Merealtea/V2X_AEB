/**
 * @file speed_predictor.cc
 * @author Chen Hongxin (angelochen@live.cn)
 * @brief
 * @version 0.1
 * @date 2023-11-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "speed_predictor.h"

#include "common/log.h"

namespace cyberc3 {
namespace planning {

void BaseSpeedPredictor::FeedTargetV2V(const V2VPacket& v2v_msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  target_state_.timestamp = v2v_msg.timestamp;
  target_state_.speed = v2v_msg.speed;
  target_state_.acc_x = v2v_msg.acc_x;
}

void BaseSpeedPredictor::FeedTargetPerception(
    const VehicleState& target_state) {
  std::lock_guard<std::mutex> lock(mutex_);
  target_state_.timestamp_perception = target_state.timestamp_perception;
  target_state_.s = target_state.s;
}

// predict target s and speed using const velocity model
bool BaseSpeedPredictor::GetPredictedSpeedProfiles(
    double timestamp, std::array<double, N_HORIZON>* predicted_s,
    std::array<double, N_HORIZON>* predicted_speeds) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (predicted_s == nullptr || predicted_speeds == nullptr) {
    AERROR << "input pointer is nullptr!";
    return false;
  }
  if (!CheckTargetStateValidty(timestamp)) {
    return false;
  }
  // compensate time delay between perception and control
  const double predict_time = timestamp - target_state_.timestamp_perception;
  const double target_s_predicted =
      target_state_.s + target_state_.speed * predict_time;
  for (int i = 0; i < N_HORIZON; ++i) {
    predicted_s->at(i) = target_s_predicted + target_state_.speed * i * T_step;
    predicted_speeds->at(i) = target_state_.speed;
  }
  return true;
}

bool BaseSpeedPredictor::GetTargetStates(double timestamp, double* target_s,
                                         double* target_speed,
                                         double* target_acc) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!CheckTargetStateValidty(timestamp)) {
    return false;
  }
  // compensate time delay between perception and control
  const double predict_time = timestamp - target_state_.timestamp_perception;
  *target_s = target_state_.s + target_state_.speed * predict_time;
  *target_speed = target_state_.speed;
  *target_acc = target_state_.acc_x;
  return true;
}

}  // namespace planning
}  // namespace cyberc3
