#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

#include "vehicle_mapping/vehicle_mapping.h"

namespace tracking {
VehicleMapping::VehicleMapping(ros::NodeHandle node_handle,
                               ros::NodeHandle private_node_handle)
    : nh_(node_handle), pnh_(private_node_handle) {
  timer_ =
      nh_.createTimer(ros::Duration(1.0 / 50), &VehicleMapping::mapping, this);
  sub_speed_ = nh_.subscribe("/rock_can/speed_feedback", 2,
                             &VehicleMapping::speedCallback, this);
  sub_steer_ = nh_.subscribe("/rock_can/steer_feedback", 2,
                             &VehicleMapping::steerCallback, this);
  sub_imu_ = nh_.subscribe("/Inertial/imu/data", 2,
                           &VehicleMapping::imuCallback, this);
  sub_imu_wit_ =
      nh_.subscribe("/wit/imu", 2, &VehicleMapping::imuWitCallback, this);
  sub_leader_wit_imu_ = nh_.subscribe("/leader/wit/imu", 2, &VehicleMapping::leaderImuCallback, this);

  sub_v2v_ =
      nh_.subscribe("/V2V/leader", 2, &VehicleMapping::V2VCallback, this);
  pub_track_vis_ =
      nh_.advertise<geometry_msgs::PoseArray>("/mapping/track/ego", 10);
  pub_target_track_vis_ =
      nh_.advertise<geometry_msgs::PoseArray>("/mapping/track/target", 10);
  pub_orientation_debug_ = nh_.advertise<std_msgs::Float64>("/leader/yaw_debug", 10);
  pub_front_speed_ = nh_.advertise<std_msgs::Float64>("/tracking/front_vehicle/speed", 10);

  ego_init_ = false;
  init_yaw_ = 0.0;
  pose_.x = pose_.y = pose_.theta = 0.0;
  curr_speed_ = 0.0;
  curr_angular_velo_ = 0.0;
  timestamp_ = -1.0;

  target_pose_.x = target_pose_.y = target_pose_.theta = 0.0;
  target_curr_speed_ = 0.0;
  target_curr_angular_velo_ = 0.0;

  ego_pose_buffer_.clear();
  target_pose_buffer_.clear();
}

void VehicleMapping::speedCallback(
    const cyber_msgs::SpeedFeedback::ConstPtr &msg) {
  curr_speed_ = msg->speed_cms / 100.0;

  std_msgs::Float64 front_speed_msg;
  front_speed_msg.data = curr_speed_;
  pub_front_speed_.publish(front_speed_msg);
}

void VehicleMapping::steerCallback(
    const cyber_msgs::SteerFeedback::ConstPtr &msg) {
  return;
}

void VehicleMapping::imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  curr_angular_velo_ = msg->angular_velocity.z;
}

void VehicleMapping::imuWitCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  curr_angular_velo_wit_ = msg->angular_velocity.z;
  auto orientation = msg->orientation;
  double roll, pitch, yaw;
  tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  if (!ego_init_)
  {
    init_yaw_ = yaw;
    ego_init_ = true;
  }
  last_yaw_ = pose_.theta;
  // pose_.theta = yaw - init_yaw_;
  std::cout << "ego yaw: " << yaw << std::endl;
}

void VehicleMapping::leaderImuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  auto orientation = msg->orientation;
  double roll, pitch, yaw;
  tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  std_msgs::Float64 yaw_msg;
  yaw_msg.data = yaw;
  pub_orientation_debug_.publish(yaw_msg);
}

void VehicleMapping::V2VCallback(const cyber_msgs::V2VPacketConstPtr &msg) {
  target_curr_speed_ = msg->speed;
  target_curr_angular_velo_ = msg->angular_velo_z;
}

void VehicleMapping::mapping(const ros::TimerEvent &event) {
  if (timestamp_ < 0.0) {
    timestamp_ = ros::Time::now().toSec();
    return;
  }
  auto curr_t = ros::Time::now().toSec();
  auto dt = curr_t - timestamp_;
  dt = std::min(std::max(0.0, dt), 0.1);

  // ego
  // if (curr_speed_ > 0.04) {
  //   pose_.x += curr_speed_ * std::cos(pose_.theta) * dt;
  //   pose_.y += curr_speed_ * std::sin(pose_.theta) * dt;
  // }

  // auto delta_theta = curr_angular_velo_ * dt;

  // // std::cout << "curr speed: " << curr_speed_ << std::endl;
  // // std::cout << "dt: " << dt << std::endl;
  auto delta_theta = curr_angular_velo_wit_ * dt;
  if (curr_speed_ > 0.04) {
    if (std::abs(delta_theta) < 0.01) {
      pose_.x += curr_speed_ * std::cos(pose_.theta) * dt;
      pose_.y += curr_speed_ * std::sin(pose_.theta) * dt;
      pose_.theta += delta_theta;
    } else {
      pose_.x +=
          (curr_speed_ * dt / delta_theta *
           (std::sin(pose_.theta + delta_theta) - std::sin(pose_.theta)));
      pose_.y +=
          (curr_speed_ * dt / delta_theta *
           (std::cos(pose_.theta) - std::cos(pose_.theta + delta_theta)));
      pose_.theta += delta_theta;
    }
    if (pose_.theta >= M_PI)
      pose_.theta -= 2 * M_PI;
    else if (pose_.theta < -M_PI)
      pose_.theta += 2 * M_PI;
  }

  // target
  auto target_delta_theta = target_curr_angular_velo_ * dt;
  if (target_curr_speed_ > 0.04) {
    if (std::abs(target_delta_theta) < 0.01) {
      target_pose_.x += target_curr_speed_ * std::cos(target_pose_.theta) * dt;
      target_pose_.y += target_curr_speed_ * std::sin(target_pose_.theta) * dt;
      target_pose_.theta += target_delta_theta;
    } else {
      target_pose_.x += (target_curr_speed_ * dt / target_delta_theta *
                         (std::sin(target_pose_.theta + target_delta_theta) -
                          std::sin(target_pose_.theta)));
      target_pose_.y += (target_curr_speed_ * dt / target_delta_theta *
                         (std::cos(target_pose_.theta) -
                          std::cos(target_pose_.theta + target_delta_theta)));
      target_pose_.theta += target_delta_theta;
    }
    if (target_pose_.theta >= M_PI)
      target_pose_.theta -= 2 * M_PI;
    else if (target_pose_.theta < -M_PI)
      target_pose_.theta += 2 * M_PI;
  }
  std::cout << "target pose: " << target_pose_ << std::endl;

  timestamp_ = curr_t;
  ego_pose_buffer_.emplace_back(pose_);
  if (ego_pose_buffer_.size() > 1000) {
    ego_pose_buffer_.pop_front();
  }
  target_pose_buffer_.emplace_back(target_pose_);
  if (target_pose_buffer_.size() > 1000) {
    target_pose_buffer_.pop_front();
  }

  // publish tf
  geometry_msgs::TransformStamped transform;
  transform.header.stamp = ros::Time::now();
  transform.header.frame_id = "map";
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = pose_.x;
  transform.transform.translation.y = pose_.y;
  transform.transform.translation.z = 0.0;
  std::cout << "x: " << transform.transform.translation.x
            << " y: " << transform.transform.translation.y << std::endl;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose_.theta);
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();

  br_.sendTransform(transform);

  // publish ego track visualization
  geometry_msgs::PoseArray::Ptr ego_track_output(
      new geometry_msgs::PoseArray());
  for (const auto &pose : ego_pose_buffer_) {
    geometry_msgs::Pose p;
    p.position.x = pose.x;
    p.position.y = pose.y;
    p.orientation.w = std::cos(pose.theta / 2);
    p.orientation.x = 0;
    p.orientation.y = 0;
    p.orientation.z = std::sin(pose.theta / 2);
    ego_track_output->poses.emplace_back(p);
  }
  ego_track_output->header.frame_id = "map";
  ego_track_output->header.stamp = ros::Time::now();
  pub_track_vis_.publish(ego_track_output);

  // // publish target track visualization
  // geometry_msgs::PoseArray::Ptr target_track_output(
  //     new geometry_msgs::PoseArray());
  // std::cout << "size: " << target_pose_buffer_.size() << std::endl;
  // for (const auto &pose : target_pose_buffer_) {
  //   geometry_msgs::Pose p;
  //   p.position.x = pose.x;
  //   p.position.y = pose.y;
  //   p.orientation.w = std::cos(pose.theta / 2);
  //   p.orientation.x = 0;
  //   p.orientation.y = 0;
  //   p.orientation.z = std::sin(pose.theta / 2);
  //   ego_track_output->poses.emplace_back(p);
  // }
  // target_track_output->header.frame_id = "map";
  // target_track_output->header.stamp = ros::Time::now();
  // pub_target_track_vis_.publish(target_track_output);
  // std::cout << "target: " << target_pose_ << std::endl;

  ROS_INFO("publish vehicle frame");
}

} // namespace tracking

int main(int argc, char **argv) {
  ros::init(argc, argv, "vehicle_mapping");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");

  tracking::VehicleMapping mapper(node_handle, private_node_handle);
  ros::spin();
  return 0;
}