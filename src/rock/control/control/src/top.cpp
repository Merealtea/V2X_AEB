/**
  *top.cpp
  *brief:top layer of software, ROS plantform related part
  *author:Jianlin Zhang
  *date:20171031
  **/

#include "top.h"
#include "cyber_msgs/PlatoonControlTarget.h"

namespace Top
{
Top::Top(ros::NodeHandle nh, ros::NodeHandle pnh)
  :vehicle(nh,pnh),
   mapping(nh,pnh),
   control(nh,pnh)
{
  pnh.param("vehicle_frame_id", vehicle_frame_id, std::string("base_link"));
  pnh.param("map_frame_id", map_frame_id, std::string("map"));
  pnh.param("target_frame_id", target_frame_id, std::string("target"));
  pnh.param("behind_frame_id", behind_frame_id, std::string("behind"));
  pnh.param("wheelbase", wheelbase, 2.56);
  pnh.param("speed_cmd_min", speed_cmd_min, 0.0);
  pnh.param("brake_cmd_max", brake_cmd_max, 4.0); // brake_cmd: [-4, 0]
  pnh.param("consider_behind", consider_behind, false);
  pnh.param("dis_base", dis_base, 6.0);
  pnh.param("dis_speed_factor", dis_speed_factor, 0.25);
  pnh.param("dis_max", dis_max, 7.0);
  pnh.param("dis_min", dis_min, 4.0);
  pnh.param("dis_offset", dis_offset, 3.55);
  pnh.param("k_acc_filter", k_acc_filter, 0.1);
  pnh.param("k_resmooth_threshold", k_resmooth_threshold, 2.0);
  pnh.param<bool>("flag_enable_smooth", flag_enable_smooth, true);

  // steer_publisher = nh.advertise<cyber_msgs::steercmd>("/rock_can/steer_command", 1);
  speed_command_publisher = nh.advertise<cyber_msgs::PlatoonControlTarget>(
      "/control/control_target", 1);
  // brake_command_publisher = nh.advertise<cyber_msgs::brakecmd>("/rock_can/brake_command", 1);
  vehicle_track_draw_publisher = nh.advertise<geometry_msgs::PoseArray>("/control/ego_track", 1);
  front_track_draw_publisher = nh.advertise<geometry_msgs::PoseArray>("/control/front_track", 1);
  front_track_smoothed_draw_publisher = nh.advertise<geometry_msgs::PoseArray>("/control/front_track_smoothed", 1);
  // front_track_interpolated_draw_publisher = nh.advertise<geometry_msgs::PoseArray>("/control/front_track_interpolated", 1);
  target_track_draw_publisher = nh.advertise<geometry_msgs::PoseArray>("/control/target_track", 1);
  // behind_track_draw_publisher = nh.advertise<geometry_msgs::PoseArray>("/control/behind_track", 1);
  // map_draw_publisher = nh.advertise<visualization_msgs::MarkerArray>("/control/target_track", 1);
  // preview_draw_publisher = nh.advertise<visualization_msgs::MarkerArray>("/control/preview_point", 1);
  // pure_pursuit_draw_publisher = nh.advertise<visualization_msgs::Marker>("/control/pure_pursuit_circle", 1);
  local_pose_publisher = nh.advertise<geometry_msgs::Pose2D>("/control/ego_pose",1);
  front_traj_publisher = nh.advertise<cyber_msgs::LocalTrajList>("/control/local_trajectory",1);
  ego_speed_publisher = nh.advertise<std_msgs::Float32>("/control/ego_vehicle/speed",1);
  front_distance_publisher = nh.advertise<std_msgs::Float32>("/control/front_vehicle/distance",1);
  curve_distance_publisher = nh.advertise<std_msgs::Float32>("/control/front_vehicle/curve_distance",1);
  behind_distance_publisher = nh.advertise<std_msgs::Float32>("/control/behind_vehicle/distance",1);
  front_distance_expect_publisher = nh.advertise<std_msgs::Float32>("/control/front_vehicle/distance_expect",1);
  control_debug_publisher =
      nh.advertise<cyber_msgs::PlatoonControlDebug>("/control_debug", 1);
  // behind_distance_expect_publisher = nh.advertise<std_msgs::Float32>("/control/behind_vehicle/distance_expect",1);
  // steercmd_display_publisher = nh.advertise<std_msgs::Float32>("/control/display/steer_cmd",1);
  // speedcmd_display_publisher = nh.advertise<std_msgs::Float32>("/control/display/speed_cmd",1);
  // brakecmd_display_publisher = nh.advertise<std_msgs::Float32>("/control/display/brake_cmd",1);

  // steer_subscriber = nh.subscribe("/e100_can/steer_feedback", 1, &Top::steer_callback, this);
  // steer_subscriber = nh.subscribe("/rock_can/steer_feedback", 1, &Top::steer_callback, this);
  target_pose_lidar_subscriber = nh.subscribe("/tracking/front_vehicle/global_pose", 1, &Top::target_pose_lidar_callback, this);
  target_local_pose_subscriber = nh.subscribe("/tracking/front_vehicle/local_pose", 1, &Top::target_local_pose_callback, this);
  behind_pose_lidar_subscriber = nh.subscribe("/tracking/behind_vehicle/global_pose", 1, &Top::behind_pose_lidar_callback, this);
  estimated_speed_subscriber = nh.subscribe("/tracking/front_vehicle/speed", 1, &Top::estimated_speed_callback, this);
  behind_speed_subscriber = nh.subscribe("/tracking/behind_vehicle/speed", 1, &Top::behind_speed_callback, this);
  // self_speed_subscriber = nh.subscribe("/e100_can/speed_feedback", 1, &Top::self_speed_callback, this);
  // self_speed_subscriber = nh.subscribe("/rock_can/speed_feedback", 1, &Top::self_speed_callback, this);
  target_lost_subscriber = nh.subscribe("/tracking/front_vehicle/is_lost", 1, &Top::target_lost_callback, this);
  behind_lost_subscriber = nh.subscribe("/tracking/behind_vehicle/is_lost", 1, &Top::behind_lost_callback, this);
  rematch_subscriber = nh.subscribe("/xboxone/rematch", 1, &Top::RematchCallback, this);
  target_v2v_subscriber = nh.subscribe("/V2V/leader", 1, &Top::target_v2v_callback, this);
  localization_subscriber = nh.subscribe("/localization/estimation", 10, &Top::localization_callback, this);

  timer=nh.createTimer(ros::Duration(0.1),&Top::timer_callback,this);

  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  cyberc3::common::math::LpfCoefficients(0.02, k_acc_filter, &den, &num);

  v2v_acc_filter_.set_coefficients(den, num);

  dynamic_reconfigure::Server<control::ParamConfig>::CallbackType f;
  f = boost::bind(&Top::config_callback, this, _1, _2);
  server.setCallback(f);
}

void Top::config_callback(control::ParamConfig &config, uint32_t level)
{
  control.speed_k1 = config.speed_k1;
  control.speed_k2 = config.speed_k2;
  control.k_front_distance = config.k_front_distance; // 车距的加权
  control.k_behind_distance = config.k_behind_distance;
  control.c_front_velocity = config.c_front_velocity; // 速度差加权
  control.c_behind_velocity = config.c_behind_velocity;
  control.k_speed_output = config.k_speed_output; // 速度响应灵敏度
  control.k_brake_output = config.k_brake_output; // 刹车响应灵敏度
  control.min_preview_distance_for_search = config.min_preview_distance_for_search;
  vehicle.steer_ratio = config.steer_ratio;
  control.heading_cost_factor = config.heading_cost_factor;
  control.straight_threshold = config.straight_threshold;
  control.k_curvature_factor = config.k_curvature_factor;
  dis_base = config.dis_base;
  dis_speed_factor = config.dis_speed_factor;
  dis_max = config.dis_max;
  control.preview_distance_base = config.preview_distance_base;
  control.preview_time = config.preview_time;
  control.zero_brake_acc_threshold = config.zero_brake_acc_threshold;
  control.zero_brake_speed_threshold = config.zero_brake_speed_threshold;
  control.k_acc_threshold = config.k_acc_threshold;
  control.k_feedforward_acc = config.k_feedforward_acc;
  if (k_acc_filter != config.k_acc_filter) {
    k_acc_filter = config.k_acc_filter;
    std::vector<double> den(3, 0.0);
    std::vector<double> num(3, 0.0);
    cyberc3::common::math::LpfCoefficients(0.02, k_acc_filter, &den, &num);
    v2v_acc_filter_.set_coefficients(den, num);
    v2v_acc_filter_.reset_values();
  }
  k_resmooth_threshold = config.k_resmooth_threshold;
  ROS_INFO("reconfigure!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
}

void Top::timer_callback(const ros::TimerEvent &event)
{
  double t_current = ros::Time::now().toSec();

  if(!init_localization)
  {
    ROS_ERROR_THROTTLE(1,"Wait for localization data!");
    return;
  }
  else if(t_current - t_localization > 1)
  {
    init_localization = false;
    mapping.map.clear();
    mapping.front_track_raw.clear();
    cyber_msgs::PlatoonControlTarget msg;
    speed_command_publisher.publish(msg);
    ROS_ERROR("Localization data lost!");
    return;
  }

  if (init_v2v && t_current - t_v2v > 1) {
    ROS_WARN("V2V msg lost!");
    target_acc=0.0;
    init_v2v = false;
    v2v_updated = false;
    v2v_acc_filter_.reset_values();
  }

  // query target_point by relative time
  if (!start_mode_flag)
    ROS_INFO("time to target point: %.2f s",
             mapping.front_track_raw.back().t_ - mapping.closest_point->t_);

  //update vehicle status
  control.x_vehicle=vehicle.x;
  control.y_vehicle=vehicle.y;
  control.theta_vehicle=vehicle.theta;
  control.speed_vehicle=vehicle.speed;

  if(behind_lost && consider_behind) {
    behind_distance_expect = -1.0;
    mapping.behind_track.clear();
    ROS_ERROR("Behind lost, ignore impact of behind!!!");
  }

  if(target_lost)
  {
    vehicle.speed_cmd = 0;
    mapping.map.clear();
    mapping.front_track_raw.clear();
    ROS_ERROR("Target lost, please rematch!!!");
  } else {
    if (init_v2v && v2v_updated) {
      control.compute_speed_cmd_V2V(mapping.curve_distance_max - dis_offset,
                                    front_distance_expect_new, front_speed_V2V,
                                    target_acc, self_speed, start_mode_flag);
    } else {
      control.compute_speed_cmd_V2V(mapping.curve_distance_max - dis_offset,
                                    front_distance_expect_new, front_speed, 0.0,
                                    self_speed, start_mode_flag);
    }
    vehicle.speed_cmd = control.speed_cmd;
  }
  cyber_msgs::PlatoonControlTarget lon_control_msg;
  lon_control_msg.header.stamp = ros::Time::now();
  lon_control_msg.distance_ref = front_distance_expect;
  lon_control_msg.distance_feedback = mapping.curve_distance_max - dis_offset;
  if (init_v2v && v2v_updated) {
    lon_control_msg.speed_ref = front_speed_V2V;
    lon_control_msg.acc_ref =
        (std::fabs(target_acc) > control.k_acc_threshold) ? target_acc : 0;
  } else {
    ROS_WARN_THROTTLE(1, "V2V msg not available!");
    lon_control_msg.speed_ref = front_speed;
    lon_control_msg.acc_ref = 0.0;
  }
  speed_command_publisher.publish(lon_control_msg);
  ROS_INFO("distance ref: %.2f, speed_ref: %.2f,target_acc: %.2f",
           lon_control_msg.distance_ref, lon_control_msg.speed_ref,
           lon_control_msg.acc_ref);
  ROS_INFO("distance err: %f, speed_err: %f",
           lon_control_msg.distance_ref - lon_control_msg.distance_feedback,
           lon_control_msg.speed_ref - vehicle.speed);

  if (!target_lost) {
    cyber_msgs::PlatoonControlDebug debug_msg;
    debug_msg.header.stamp = ros::Time::now();
    debug_msg.speed_ego = vehicle.speed;
    debug_msg.speed_front_perception = front_speed;
    debug_msg.speed_front_v2v = front_speed_V2V;
    debug_msg.speed_target = vehicle.speed_cmd;
    debug_msg.speed_error = vehicle.speed_cmd - vehicle.speed;
    debug_msg.distance_feedback_curve = mapping.curve_distance_max - dis_offset;
    debug_msg.distance_feedback_perception = front_distance;
    debug_msg.distance_expect = front_distance_expect;
    debug_msg.distance_expect_new = front_distance_expect_new;
    debug_msg.distance_error =
        debug_msg.distance_expect - debug_msg.distance_feedback_curve;
    debug_msg.acc_front_v2v = target_acc;
    debug_msg.v2v_delay_ms = v2v_delay * 1000.0;
    debug_msg.time_to_front =
        mapping.front_track_raw.back().t_ - mapping.closest_point->t_;

    control_debug_publisher.publish(debug_msg);
  }
}

void Top::localization_callback(
    const cyber_msgs::LocalizationEstimate::ConstPtr &rec) {
  t_localization = ros::Time::now().toSec();
  if (!init_localization) init_localization = true;
  vehicle.x = rec->pose.position.x;
  vehicle.y = rec->pose.position.y;
  vehicle.theta = tf::getYaw(rec->pose.orientation);
  vehicle.speed = rec->velocity.linear.x;
  self_speed = rec->velocity.linear.x;
  vehicle.update_track();
  vehicle.draw_track(vehicle_track_draw_publisher);

  geometry_msgs::Pose2D local_pose;
  local_pose.x = vehicle.x;
  local_pose.y = vehicle.y;
  local_pose.theta = vehicle.theta;
  local_pose_publisher.publish(local_pose);

  if (!mapping.front_track_raw.empty())
    mapping.remove_passed_points(local_pose);

  std_msgs::Float32 speed_msg;
  speed_msg.data = self_speed;
  ego_speed_publisher.publish(speed_msg);

  front_distance_expect = calc_distance_expect(self_speed);
  std_msgs::Float32 dis_msg;
  dis_msg.data = front_distance_expect;
  front_distance_expect_publisher.publish(dis_msg);

  front_distance_expect_new = calc_distance_expect_by_time(dis_speed_factor);

  if (!mapping.front_track_raw.empty())
    check_start_mode_flag(vehicle.speed, mapping.start_s,
                          mapping.closest_point->curve_distance_);
}

void Top::target_pose_lidar_callback(const geometry_msgs::Pose2D::ConstPtr &rec)
{
  if(target_lost) return;
  geometry_msgs::Pose2D local_pose;
  local_pose.x = vehicle.x;
  local_pose.y = vehicle.y;
  local_pose.theta = vehicle.theta;

  mapping.update_front_track(rec, local_pose);
  mapping.draw_track(mapping.front_track_raw, front_track_draw_publisher);

  cyber_msgs::LocalTrajList traj_msg;
  if (flag_enable_smooth && mapping.front_track_raw.size() > 5) {
    if (last_smooth_s - mapping.closest_point->curve_distance_ <
        k_resmooth_threshold) {
      if (cyberc3::planning::SmoothPlatoonTrack(mapping.front_track_raw,
                                                mapping.front_track_smoothed)) {
        mapping.draw_track(mapping.front_track_smoothed,
                           front_track_smoothed_draw_publisher);
        if (mapping.convert_trajectory(mapping.front_track_smoothed,
                                       traj_msg)) {
          front_traj_publisher.publish(traj_msg);
          last_smooth_s = mapping.front_track_raw.back().curve_distance_;
        }
      }
    }
  } else {
    if (mapping.convert_trajectory(mapping.front_track_raw, traj_msg))
      front_traj_publisher.publish(traj_msg);
  }

  std_msgs::Float32 dis_msg;
  dis_msg.data = mapping.curve_distance_max - dis_offset;
  curve_distance_publisher.publish(dis_msg);

  mapping.update_target_track(rec);
  // mapping.update_map(vehicle.x,vehicle.y,vehicle.theta);
  mapping.draw_target_track(target_track_draw_publisher);
  // mapping.draw_map(map_draw_publisher);

  //pub target frame
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(rec->x, rec->y, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, rec->theta);
  transform.setRotation(q);
  tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), map_frame_id, target_frame_id));

  //pub closest_point frame
  transform.setOrigin(tf::Vector3(mapping.closest_point->x_, mapping.closest_point->y_, 0.0));
  q.setRPY(0, 0, mapping.closest_point->theta_);
  transform.setRotation(q);
  tf_broadcaster.sendTransform(tf::StampedTransform(
      transform, ros::Time::now(), map_frame_id, "closest_point"));

}

void Top::target_local_pose_callback(const geometry_msgs::Pose2D::ConstPtr &rec)
{
  if(target_lost) return;
  front_distance = sqrt(pow(rec->x, 2) + pow(rec->y, 2)) - dis_offset - 0.45;

  std_msgs::Float32 dis_msg;
  dis_msg.data = front_distance;
  front_distance_publisher.publish(dis_msg);
}

void Top::behind_pose_lidar_callback(const geometry_msgs::Pose2D::ConstPtr &rec)
{
  if(behind_lost) {
    mapping.behind_track.clear();
    mapping.behind_curve_distance_max = -1.0;
    return;
  }
  geometry_msgs::Pose2D local_pose;
  local_pose.x = vehicle.x;
  local_pose.y = vehicle.y;
  local_pose.theta = vehicle.theta;
  mapping.update_behind_track(rec, local_pose);
  mapping.draw_behind_track(behind_track_draw_publisher);
  // ROS_ERROR("behind track size: %d, dis: %f", mapping.behind_track.size(), mapping.behind_curve_distance_max);
  std_msgs::Float32 dis_msg;
  dis_msg.data = mapping.behind_curve_distance_max;
  behind_distance_publisher.publish(dis_msg);

  tf::Transform transform;
  transform.setOrigin( tf::Vector3(rec->x, rec->y, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, rec->theta);
  transform.setRotation(q);
  tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), map_frame_id, behind_frame_id));
}

double Top::calc_distance_expect(double curr_speed) {
  // 计算时距
  double distance_expect = dis_base + dis_speed_factor * curr_speed;
  distance_expect = std::min(distance_expect, dis_max);
  distance_expect = std::max(distance_expect, dis_min);
  return distance_expect;
}

double Top::calc_distance_expect_by_time(double relative_time) {
  // 计算时距
  double distance_expect =
      dis_base + mapping.front_track_raw.back().curve_distance_ -
      mapping.query_target_point_by_relative_time(dis_speed_factor)
          .curve_distance_;
  distance_expect = std::min(distance_expect, dis_max);
  distance_expect = std::max(distance_expect, dis_min);
  return distance_expect;
}

void Top::estimated_speed_callback(const std_msgs::Float32::ConstPtr &rec)
{
  control.speed_target = rec->data;
  front_speed = rec->data;
}

void Top::behind_speed_callback(const std_msgs::Float32::ConstPtr &rec)
{
  control.behind_speed = rec->data;
  behind_speed = rec->data;
  behind_distance_expect = calc_distance_expect(control.behind_speed);
  std_msgs::Float32 dis_msg;
  dis_msg.data = behind_distance_expect;
  behind_distance_expect_publisher.publish(dis_msg);
}

void Top::target_lost_callback(const std_msgs::Bool::ConstPtr &rec)
{
  target_lost = rec->data;
}

void Top::behind_lost_callback(const std_msgs::Bool::ConstPtr &rec)
{
  behind_lost = rec->data;
}


void Top::RematchCallback(const std_msgs::Bool::ConstPtr &rec) {
  if(rec->data) {
    ROS_WARN("------------------- Start Rematch By Xboxone ---------------");
    t_localization = 0.0;
    init_localization = false;
    mapping.clear_map();
    mapping.clear_target_track();
    mapping.front_track_raw.clear();
    mapping.behind_track.clear();
    mapping.behind_curve_distance_max = -1.0;
    mapping.reset_time_offset();
    vehicle.clear_track();
    vehicle.reset_local_pose();
    start_mode_flag = true;
    stop_time_count = ros::Time::now().toSec();
    last_smooth_s = 0.0;
    v2v_acc_filter_.reset_values();

    // publish empty msg
    cyber_msgs::LocalTrajList traj_msg;
    front_traj_publisher.publish(traj_msg);
    cyber_msgs::PlatoonControlTarget speed_command;
    speed_command_publisher.publish(speed_command);
    mapping.draw_track(mapping.front_track_raw,front_track_draw_publisher);
    mapping.draw_target_track(target_track_draw_publisher);
  }
}

void Top::target_v2v_callback(const cyber_msgs::V2VPacket::ConstPtr &rec) {
  v2v_delay = (ros::Time::now() - rec->header.stamp).toSec();
  if (std::fabs(v2v_delay) < 0.1 && rec->is_updated) {
    t_v2v = ros::Time::now().toSec();
    if (!init_v2v)
      init_v2v = true;
    double imu_offset = 0.0;
    double acc = rec->accel_x - imu_offset;
    target_acc = v2v_acc_filter_.Filter(acc);
    front_speed_V2V = rec->speed_cmps / 100.0;
    v2v_updated = true;
    // ROS_INFO("V2V communication delay is %.1f ms! Accept !",
    //          (ros::Time::now() - rec->header.stamp).toSec() * 1000);
  } else {
    v2v_updated = false;
    ROS_WARN("V2V communication delay is %.1f ms! Discard !",(ros::Time::now() - rec->header.stamp).toSec()*1000);
  }
}

void Top::check_start_mode_flag(const double current_speed,
                                const double target_s, const double current_s) {
  // in start mode, check s for status change
  if (start_mode_flag) {
    if (current_s >= target_s + 0.5) {
      start_mode_flag = false;
      ROS_WARN_STREAM("Change start_mode_flag to: " << start_mode_flag);
    }
  }
  // in running mode, check ego speed for status change
  else {
    //车速小于0.1m/s超过5s后认为已停车
    if (std::fabs(current_speed) < 0.1) {
      if (ros::Time::now().toSec() - stop_time_count > 5.0) {
        start_mode_flag = true;
        ROS_WARN_STREAM("Change start_mode_flag to: " << start_mode_flag);
        mapping.start_s = mapping.front_track_raw.back().curve_distance_;
      }
    } else {
      stop_time_count = ros::Time::now().toSec();
    }
  }
}
} // namespace Top
