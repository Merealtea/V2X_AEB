/**
 * @file track_manager_node.cpp
 * @author Chen Hongxin (angelochen@livw.cn)
 * @brief
 * @version 0.1
 * @date 2023-04-04
 *
 * @copyright Copyright (c) 2023
 *
 */

#define PUB_TARGET_FRAME false

// app
#include "common/utils/find_path.h"
#include "track_manager.h"
#include "type_converter.h"
// ros
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "cyber_msgs/LocalTrajList.h"
#include "cyber_msgs/LocalizationEstimate.h"
#include "cyber_msgs/PlatoonControlDebug.h"
#include "cyber_msgs/PlatoonControlTarget.h"
#include "cyber_msgs/SteerFeedback.h"
#include "cyber_msgs/V2VPacket.h"
#include "std_msgs/Bool.h"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
// standard c++
#include <memory>

// app
std::unique_ptr<cyberc3::planning::TrackManager> manager;

// declare publishers
ros::Publisher cmd_pub;
ros::Publisher front_track_publisher;
ros::Publisher debug_pub;
ros::Publisher ego_track_draw_publisher;
ros::Publisher target_track_draw_publisher;
ros::Publisher front_track_draw_publisher;
ros::Publisher mpc_traj_publisher;
ros::Publisher mpc_speedcmd_publisher;
ros::Publisher mpc_brakecmd_publisher;
ros::Publisher mpc_steercmd_publisher;
ros::Publisher mpc_debug_publisher;

bool target_use_global_pose;
bool enable_mpc_adapter;

void localization_callback(
    const cyber_msgs::LocalizationEstimateConstPtr &msg) {
  cyberc3::common::TrajectoryPoint ego_pose;
  ego_pose.set_relative_time(msg->header.stamp.toSec());
  ego_pose.mutable_path_point()->set_x(msg->pose.position.x);
  ego_pose.mutable_path_point()->set_y(msg->pose.position.y);
  ego_pose.mutable_path_point()->set_z(msg->pose.position.z);
  ego_pose.mutable_path_point()->set_theta(tf::getYaw(msg->pose.orientation));
  ego_pose.set_v(msg->velocity.linear.x);
  ego_pose.set_a(msg->acceleration.linear.x);
  ego_pose.set_steer(msg->velocity.angular.z);
  manager->FeedEgoPose(ego_pose);
}

void steer_feedback_callback(const cyber_msgs::SteerFeedbackConstPtr &msg) {
  // convert steer wheel angle to front wheel angle (rad)
  constexpr double steer_ratio = 16.5;
  const double wheel_angle_rad = msg->SteerAngle / steer_ratio / 180.0 * M_PI;
  manager->FeedEgoSteerAngle(wheel_angle_rad);
}

void target_pose_callback(const geometry_msgs::Pose2DConstPtr &msg) {
  cyberc3::planning::VehicleState ego_state = manager->GetEgoState();
  if (ros::Time::now().toSec() - ego_state.timestamp > 1.0) {
    AWARN << "------------------- Localization Timeout"
             "-------------------";
    return;
  }

  // transform to global frame
  double global_x, global_y, global_theta;
  if (target_use_global_pose) {
    global_x = msg->x;
    global_y = msg->y;
    global_theta = msg->theta;
  } else {
    global_x = ego_state.x + msg->x * std::cos(ego_state.theta) -
               msg->y * std::sin(ego_state.theta);
    global_y = ego_state.y + msg->x * std::sin(ego_state.theta) +
               msg->y * std::cos(ego_state.theta);
    global_theta = ego_state.theta + msg->theta;
  }
  cyberc3::common::TrajectoryPoint target_pose;
  target_pose.set_relative_time(ros::Time::now().toSec());
  target_pose.mutable_path_point()->set_x(global_x);
  target_pose.mutable_path_point()->set_y(global_y);
  target_pose.mutable_path_point()->set_theta(global_theta);
  manager->FeedTargetPose(target_pose);

// pub target frame
#if PUB_TARGET_FRAME
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(global_x, global_y, 0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, global_theta);
  transform.setRotation(q);
  static tf::TransformBroadcaster tf_broadcaster;
  tf_broadcaster.sendTransform(
      tf::StampedTransform(transform, ros::Time::now(), "map", "target"));
#endif
}

void target_lost_callback(const std_msgs::BoolConstPtr &msg) {
  manager->FeedTargetLostState(msg->data);
  // publish empty command if target lost
  if (msg->data) {
    AWARN << "------------------- Target Lost -------------------";
    cyber_msgs::LocalTrajList traj_msg;
    front_track_publisher.publish(traj_msg);
    cyber_msgs::PlatoonControlTarget speed_command;
    cmd_pub.publish(speed_command);
  }
}

void target_v2v_callback(const cyber_msgs::V2VPacketConstPtr &msg) {
  cyberc3::planning::V2VPacket v2v_packet;
  converter::from_ros_msg(*msg, &v2v_packet);
  manager->FeedTargetV2V(v2v_packet);
}

void rematch_callback(const std_msgs::BoolConstPtr &msg) {
  if (msg->data) {
    AWARN << "------------------- Start Manual Rematch -------------------";
    manager->Reset();
    // publish empty command
    cyber_msgs::LocalTrajList traj_msg;
    front_track_publisher.publish(traj_msg);
    cyber_msgs::PlatoonControlTarget speed_command;
    cmd_pub.publish(speed_command);
  }
}

void autodrive_callback(const std_msgs::BoolConstPtr &msg) {
  manager->FeedAutoDriveMode(msg->data);
}

void perception_speed_callback(const std_msgs::Float32ConstPtr &msg) {
  manager->FeedTargetPerceptionSpeed(msg->data);
}

void cmd_timer_callback(const ros::TimerEvent &event) {
  cyberc3::planning::PlatoonControlCommand cmd;
  cyberc3::planning::PlatoonControlDebug debug;
  if (manager->GetControlCommand(ros::Time::now().toSec(), &cmd, &debug)) {
    cyber_msgs::PlatoonControlTarget cmd_msg;
    converter::to_ros_msg(cmd, &cmd_msg);
    cmd_pub.publish(cmd_msg);
    cyber_msgs::PlatoonControlDebug debug_msg;
    converter::to_ros_msg(debug, &debug_msg);
    debug_pub.publish(debug_msg);
  }
  cyberc3::planning::DiscretizedTrajectory front_track;
  if (manager->GetFrontTrack(&front_track)) {
    cyber_msgs::LocalTrajList traj_msg;
    converter::to_ros_msg(front_track, &traj_msg);
    front_track_publisher.publish(traj_msg);
    // publish front track pose for visualization
    geometry_msgs::PoseArray front_track_msg;
    converter::to_ros_msg(front_track, &front_track_msg);
    front_track_draw_publisher.publish(front_track_msg);
  }
  if (enable_mpc_adapter) {
    cyber_msgs::speedcmd speed_cmd;
    cyber_msgs::brakecmd brake_cmd;
    cyber_msgs::steercmd steer_cmd;
    if (!manager->mpc_adapter_->GetMPCCommands(&steer_cmd, &brake_cmd,
                                               &speed_cmd)) {
      AERROR << "Get MPC Commands Failed! Emergency Brake Now!";
      speed_cmd.enable_auto_speed = false;
      steer_cmd.enable_auto_steer = false;
      brake_cmd.enable_auto_brake = true;
      brake_cmd.deceleration = -2.0;
    }
    mpc_speedcmd_publisher.publish(speed_cmd);
    mpc_brakecmd_publisher.publish(brake_cmd);
    mpc_steercmd_publisher.publish(steer_cmd);
    cyber_msgs::PlatoonMPCDebug mpc_debug_msg;
    if (manager->mpc_adapter_->GetMPCDebug(&mpc_debug_msg)) {
      mpc_debug_publisher.publish(mpc_debug_msg);
    }
  }
}

void visualize_timer_callback(const ros::TimerEvent &event) {
  cyberc3::planning::DiscretizedTrajectory ego_track;
  if (manager->GetEgoTrack(&ego_track)) {
    geometry_msgs::PoseArray ego_track_msg;
    converter::to_ros_msg(ego_track, &ego_track_msg);
    ego_track_draw_publisher.publish(ego_track_msg);
  }
  cyberc3::planning::DiscretizedTrajectory target_track;
  if (manager->GetTargetTrack(&target_track)) {
    geometry_msgs::PoseArray target_track_msg;
    converter::to_ros_msg(target_track, &target_track_msg);
    target_track_draw_publisher.publish(target_track_msg);
  }
  if (enable_mpc_adapter) {
    mpc_traj_publisher.publish(manager->mpc_adapter_->GetMPCTrajectory());
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "track_manager_node");
  ros::NodeHandle nh("~");

  // if (!google::IsGoogleLoggingInitialized()) {
  google::InitGoogleLogging("track_manager");
  // }

  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  FLAGS_log_prefix = true;
  FLAGS_logbufsecs = 0;
  FLAGS_max_log_size = 1024;
  FLAGS_log_dir = expand_catkin_ws("/log/");

  cyberc3::planning::TrackManagerConfig config;
  // load param
  bool load_ret = true;
  std::string smoother_type;
  load_ret &= nh.getParam("smoother_type", smoother_type);
  if (smoother_type == "Spline") {
    config.smooth_config.smoother_type =
        cyberc3::planning::SmootherType::Spline;
  } else if (smoother_type == "Discrete") {
    config.smooth_config.smoother_type =
        cyberc3::planning::SmootherType::Discrete;
  } else {
    AERROR << "Unknown smoother type!";
    load_ret = false;
  }
  load_ret &= nh.getParam("resmooth_threshold",
                          config.smooth_config.resmooth_threshold);
  load_ret &=
      nh.getParam("behind_distance", config.smooth_config.behind_distance);
  load_ret &= nh.getParam("dis_base", config.dis_config.dis_base);
  load_ret &=
      nh.getParam("dis_speed_factor", config.dis_config.dis_speed_factor);
  load_ret &= nh.getParam("dis_max", config.dis_config.dis_max);
  load_ret &= nh.getParam("dis_min", config.dis_config.dis_min);
  load_ret &= nh.getParam("dis_offset", config.dis_config.dis_offset);

  load_ret &= nh.getParam("v2v_acc_filter_cutoff_freq",
                          config.v2v_config.acc_filter_cutoff_freq);
  load_ret &=
      nh.getParam("v2v_delay_threshold", config.v2v_config.delay_threshold);
  load_ret &=
      nh.getParam("v2v_use_acc_threshold", config.v2v_config.use_acc_threshold);

  load_ret &= nh.getParam("track_interval", config.track_interval);
  load_ret &= nh.getParam("max_track_size", config.max_track_size);
  load_ret &= nh.getParam("timeout_threshold", config.timeout_threshold);
  load_ret &= nh.getParam("target_use_global_pose", target_use_global_pose);
  load_ret &= nh.getParam("distance_filter_cutoff_freq",
                          config.distance_filter_cutoff_freq);
  load_ret &= nh.getParam("enable_mpc_adapter", enable_mpc_adapter);
  config.enable_mpc_adapter = enable_mpc_adapter;

  if (!load_ret) {
    AERROR << "Load track manager config failed!";
    return -1;
  }

  manager = std::make_unique<cyberc3::planning::TrackManager>(config);

  // subscribers
  ros::Subscriber localization_sub =
      nh.subscribe("/localization/estimation", 1, &localization_callback);
  ros::Subscriber target_pose_sub;
  if (target_use_global_pose) {
    target_pose_sub = nh.subscribe("/tracking/front_vehicle/global_pose", 1,
                                   &target_pose_callback);
  } else {
    target_pose_sub = nh.subscribe("/tracking/front_vehicle/local_pose", 1,
                                   &target_pose_callback);
  }
  ros::Subscriber steer_feedback_sub =
      nh.subscribe("/rock_can/steer_feedback", 1, &steer_feedback_callback);
  ros::Subscriber target_lost_sub =
      nh.subscribe("/tracking/front_vehicle/is_lost", 1, &target_lost_callback);
  ros::Subscriber target_v2v_sub =
      nh.subscribe("/V2V/leader", 1, &target_v2v_callback);
  ros::Subscriber tagert_tracking_speed_sub = nh.subscribe(
      "/tracking/front_vehicle/speed", 1, &perception_speed_callback);
  ros::Subscriber rematch_sub =
      nh.subscribe("/xboxone/rematch", 1, &rematch_callback);
  ros::Subscriber autodrivemode_sub =
      nh.subscribe("/rock_can/auto_drive", 1, &autodrive_callback);

  // publishers for control
  cmd_pub = nh.advertise<cyber_msgs::PlatoonControlTarget>(
      "/control/control_target", 1);
  front_track_publisher =
      nh.advertise<cyber_msgs::LocalTrajList>("/control/local_trajectory", 1);
  // publishers for debug and visualization
  debug_pub =
      nh.advertise<cyber_msgs::PlatoonControlDebug>("/control_debug", 1);
  ego_track_draw_publisher =
      nh.advertise<geometry_msgs::PoseArray>("/control/ego_track", 1);
  target_track_draw_publisher =
      nh.advertise<geometry_msgs::PoseArray>("/control/target_track", 1);
  front_track_draw_publisher = nh.advertise<geometry_msgs::PoseArray>(
      "/control/front_track_smoothed", 1);

  if (config.enable_mpc_adapter) {
    mpc_traj_publisher =
        nh.advertise<geometry_msgs::PoseArray>("/control/mpc_trajectory", 1);
    mpc_speedcmd_publisher =
        nh.advertise<cyber_msgs::speedcmd>("/rock_can/speed_command", 1);
    mpc_brakecmd_publisher =
        nh.advertise<cyber_msgs::brakecmd>("/rock_can/brake_command", 1);
    mpc_steercmd_publisher =
        nh.advertise<cyber_msgs::steercmd>("/rock_can/steer_command", 1);
    mpc_debug_publisher =
        nh.advertise<cyber_msgs::PlatoonMPCDebug>("/mpc_debug", 1);
  }

  // timers
  ros::Timer cmd_timer =
      nh.createTimer(ros::Duration(0.02), &cmd_timer_callback);

  ros::Timer visualize_timer =
      nh.createTimer(ros::Duration(0.1), &visualize_timer_callback);

  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();

  return 0;
}
