/**
 * @file track_manager.h
 * @author Chen Hongxin (angelochen@live.cn)
 * @brief Trajectory Manager and Generator for CybeC3 Platoon Task
 * @version 0.1
 * @date 2023-04-04
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once

#include <atomic>
#include <mutex>

#include "common/math/filters/digital_filter.h"
#include "common/math/vec2d.h"
#include "common/struct/pnc_point.h"
#include "discretized_trajectory.h"
#include "julia_mpc_adapter.h"
#include "kalman_speed_predictor.h"
#include "track_manager_type.h"
#include "track_smoother.h"

namespace cyberc3 {
namespace planning {

class TrackManager {
 public:
  TrackManager(const TrackManagerConfig &config);
  ~TrackManager() = default;
  TrackManager(const TrackManager &) = delete;
  TrackManager operator=(const TrackManager &) = delete;
  TrackManager(TrackManager &&) = delete;
  TrackManager operator=(TrackManager &&) = delete;

  void Init(const TrackManagerConfig &config);
  void UpdateConfig(const TrackManagerConfig &config);
  void Reset();

  void FeedTargetPose(common::TrajectoryPoint &target_pose);
  void FeedEgoPose(const common::TrajectoryPoint &ego_pose);
  void FeedEgoSteerAngle(const float steer_angle);
  void FeedTargetV2V(const V2VPacket &v2v_msg);
  void FeedTargetLostState(const bool target_lost);
  void FeedTargetPerceptionSpeed(const float speed);
  void FeedAutoDriveMode(const bool auto_drive_mode);

  bool GetTargetTrack(DiscretizedTrajectory *track_out) const;
  bool GetEgoTrack(DiscretizedTrajectory *track_out) const;
  bool GetFrontTrack(DiscretizedTrajectory *track_out);
  bool GetControlCommand(const double cur_time, PlatoonControlCommand *cmd,
                         PlatoonControlDebug *debug);
  VehicleState GetEgoState() const;

  std::unique_ptr<JuliaMPCAdapter> mpc_adapter_;

 private:
  double CalculateExpectDistance(double ego_speed, double target_speed) const;
  void UpdateTargetTrack(common::TrajectoryPoint target_pose);
  void UpdateEgoTrack(common::TrajectoryPoint ego_pose);
  void UpdateSmoothedTrack();
  void UpdateNearestIndex(const common::TrajectoryPoint &ego_pose);
  // limit reference speed by curvature to avoid sudden speed change in turn
  void CalculateSpeedAccLimitByCurvature(double *speed_limit,
                                         double *acc_limit) const;
  void UpdateEgoS(const common::TrajectoryPoint &ego_pose);

  std::vector<common::math::Vec2d> GenerateBezierPoints(
      const common::TrajectoryPoint &start_pose,
      const common::TrajectoryPoint &end_pose) const;

  mutable std::mutex ego_track_mutex_;
  mutable std::mutex target_track_mutex_;
  mutable std::mutex localization_mutex_;
  mutable std::mutex v2v_mutex_;
  mutable std::mutex smoothed_track_mutex_;

  bool smoothed_track_updated_;
  std::atomic<bool> target_lost_;

  size_t nearest_index_ = 0;
  size_t smoothed_nearest_index_ = 0;

  DiscretizedTrajectory target_track_;
  DiscretizedTrajectory ego_track_;
  DiscretizedTrajectory smoothed_track_;

  TrackManagerConfig config_;

  VehicleState target_state_;
  VehicleState ego_state_;

  common::math::DigitalFilter v2v_acc_filter_;
  common::math::DigitalFilter distance_feedback_filter_;
  PlatoonTrackSmoother track_smoother_;
  std::vector<double> ref_curvatures_;
  std::vector<double> ref_steer_angles_;
  std::vector<double> ref_speed_limits_;
  std::vector<double> ref_acc_limits_;
  std::shared_ptr<BaseSpeedPredictor> speed_predictor_;
};
}  // namespace planning
}  // namespace cyberc3
