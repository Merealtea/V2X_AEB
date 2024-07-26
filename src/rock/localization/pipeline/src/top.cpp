/**
 *top.h
 *brief:top layer of localization pipeline
 *author:Chen Xiaofeng
 *date:20191028
 **/

#include "top.h"

Top::Top(ros::NodeHandle &nh, ros::NodeHandle &nh_priv) {
  //   nh_priv.param<bool>("command_no_gps",command_no_gps_,false);
  nh_priv.param<bool>("localization_debug", localization_debug_, false);
  nh_priv.param<double>("speed_err", speed_err_, 0.001);
  nh_priv.param<double>("steer_err", steer_err_, 0.001);
  nh_priv.param<double>("imu_err", imu_err_, 0.004);
  nh_priv.param<double>("gps_yaw_err_fix", gps_yaw_err_fix_,
                        0.05 * M_PI / 180.0);
  nh_priv.param<double>("gps_yaw_err_normal", gps_yaw_err_normal_,
                        2.0 * M_PI / 180.0);  // offile.open("../odom.txt");
  nh_priv.param<double>("steer_ratio", steer_ratio_, 15.5);
  nh_priv.param("vehicle_frame_id", vehicle_frame_id_, std::string("base_link"));

  pub_localization_estimation_ = nh.advertise<cyber_msgs::LocalizationEstimate>(
      "/localization/estimation", 10);
  //   pub_gps_vel_ = nh.advertise<const
  //   cyber_msgs::SpeedFeedback>("/gps_vel",10);
  pub_gps_angle_ = nh.advertise<const std_msgs::Float64>("/localization/gps_angle", 1);
  //   pub_slam_angle_ = nh.advertise<const std_msgs::Float64>("/slam_angle",1);
  //   pub_output_angle_ = nh.advertise<const
  //   std_msgs::Float64>("/output_angle",1);

  if (localization_debug_) {
    pub_gps_marker_ =
        nh.advertise<visualization_msgs::Marker>("/localization/gps_markers", 2);
    pub_gps_pose_ = nh.advertise<geometry_msgs::PoseArray>("/localization/gps_poses", 2);
    pub_target_gps_pose_ = nh.advertise<geometry_msgs::PoseArray>("/localization/target_gps_poses", 2);
    pub_filter_marker_ =
        nh.advertise<visualization_msgs::Marker>("/localization/filter_markers", 2);
    pub_filter_pose_ =
        nh.advertise<geometry_msgs::PoseArray>("/localization/filter_poses", 2);
    pub_slam_pose_xyz_ =
        // nh.advertise<geometry_msgs::Pose>("/slam_poses_xyz", 2);
    pub_gps_pose_xyz_ = nh.advertise<geometry_msgs::Pose>("/localization/gps_poses_xyz", 2);
    command_sub_ =
        nh.subscribe("/localization/reset_localpose", 1, &Top::command_callback, this);
    target_gps_sub_ =
        nh.subscribe("/V2V/leader", 1, &Top::target_gps_callback, this);
  }

  //   vel_gps_sub_ = nh.subscribe("/Inertial/gps/vel", 1000,
  //   &Top::vel_gps_callback, this); vel_can_sub_ =
  //   nh.subscribe("/rock_can/speed_feedback", 1000, &Top::vel_can_callback,
  //   this);
  sub_vel_can_ = new message_filters::Subscriber<cyber_msgs::SpeedFeedback>(
      nh, "/rock_can/speed_feedback", 1);
  sub_steer_can_ = new message_filters::Subscriber<cyber_msgs::SteerFeedback>(
      nh, "/rock_can/steer_feedback", 1);
  can_sync_ = new message_filters::Synchronizer<CANPolicy>(
      CANPolicy(10), *sub_vel_can_, *sub_steer_can_);
  can_sync_->registerCallback(
      boost::bind(&Top::vel_can_callback, this, _1, _2));

  sub_gps_fix_ = new message_filters::Subscriber<sensor_msgs::NavSatFix>(
      nh, "/Inertial/gps/fix", 1);
  sub_imu_ = new message_filters::Subscriber<sensor_msgs::Imu>(
      nh, "/Inertial/imu/data", 1);
  sub_imu_->registerCallback(boost::bind(&Top::imu_callback, this, _1));
  gps_sync_ = new message_filters::Synchronizer<GPSPolicy>(
      GPSPolicy(10), *sub_gps_fix_, *sub_imu_);
  gps_sync_->registerCallback(boost::bind(&Top::gps_callback, this, _1, _2));

  filter_timer_ =
      nh.createTimer(ros::Duration(0.02), &Top::filter_callback, this);

  ROS_INFO("Localization Pipeline begin!");

  //   double test_lon = 121.43639320451585;
  //   double test_lat = 31.03063158746496;
  //   char zone;
  //   double test_x = 0;
  //   double test_y = 0;
  //   LLtoUTM(test_lat, test_lon, test_y, test_x, &zone);
  //   std::cout << "testx:" << test_x << " test y:" << test_y << std::endl;
}

Top::~Top() {
  delete sub_gps_fix_;
  delete sub_imu_;
  delete gps_sync_;
  // delete sub_slam_fix_;
  // delete sub_slam_heading_;
  // delete slam_sync_;
  delete sub_vel_can_;
  delete sub_steer_can_;
  delete can_sync_;
}

double Top::get_time_now() { return ros::Time::now().toSec(); }

void Top::command_callback(const std_msgs::BoolConstPtr &bool_in)
{
    if(bool_in->data)
    {
      fusion_.resetState();
      have_inited_gps_=false;
      have_inited_vel_=false;
      gps_poses_.poses.clear();
      gps_marker_traj_.points.clear();
      gps_marker_traj_.colors.clear();
      filter_poses_.poses.clear();
      filter_marker_traj_.points.clear();
      filter_marker_traj_.colors.clear();
    }
}

void Top::vel_can_callback(
    const cyber_msgs::SpeedFeedbackConstPtr &vel_can_in,
    const cyber_msgs::SteerFeedbackConstPtr &steer_can_in) {
  // if (!have_inited_gps_){
  //     ROS_INFO("wait for GPS input!");
  //     return;
  // }
  curr_vel_ = abs(double(vel_can_in->speed_cms) * 0.01);

  // 9倒档，11 D档
  if (int(vel_can_in->gear) == 9) curr_vel_ = -curr_vel_;

  double steer_angle = steer_can_in->SteerAngle * M_PI / 180.0 / steer_ratio_;
  double yaw_rate = curr_vel_ * tan(steer_angle) / wheelbase_;

  Eigen::Vector2d Z_vel;
  Eigen::Matrix2d R_vel;

  // double temp_yaw_rate = 0;
  if(yaw_rate<0.00001 && yaw_rate>-0.00001)
      yaw_rate = 0.00001;
  // else
  //     temp_yaw_rate = curr_yaw_rate_;

  Z_vel << curr_vel_, yaw_rate;
  R_vel << pow(speed_err_, 2), 0, 0, pow(steer_err_, 2);

  t_vel_ = get_time_now();
  if (fabs(curr_vel_) > 0.01)
    fusion_.velStateUpdate(Z_vel, R_vel, t_vel_);
  else
    fusion_.timeUpdate(t_vel_);

  have_inited_vel_ = true;
}

void Top::imu_callback(const sensor_msgs::ImuConstPtr &imu_in) {
  angular_velocity_ = imu_in->angular_velocity;
  linear_acceleration_ = imu_in->linear_acceleration;

  if (!have_inited_vel_) {
    ROS_INFO_THROTTLE(1,"wait for initial");
    return;
  }

  Eigen::Vector2d Z_vel;
  Eigen::Matrix2d R_vel;

  double yaw_rate = imu_in->angular_velocity.z;
  double yaw_rate_cov = imu_in->angular_velocity_covariance[0];

  if (yaw_rate < 0.00001 && yaw_rate > -0.00001) yaw_rate = 0.00001;
  if (std::fabs(yaw_rate_cov) < 1e-9) yaw_rate_cov = 1e-9;

  Z_vel << -1, yaw_rate;
  R_vel << -1, 0, 0, yaw_rate_cov;

  t_vel_ = get_time_now();
  if (fabs(curr_vel_) > 0.01)
    fusion_.velStateUpdate(Z_vel, R_vel, t_vel_, true);
}

// use GPS data only for validation now !!!
void Top::gps_callback(const sensor_msgs::NavSatFixConstPtr &gps_in,
                       const sensor_msgs::ImuConstPtr &imu_in) {
  static double x_offset, y_offset, yaw_offset;
  // double obs_yaw;
  if (!have_inited_gps_) {
    char zone;
    LLtoUTM(gps_in->latitude, gps_in->longitude, y_offset, x_offset, &zone);

    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(imu_in->orientation.x, imu_in->orientation.y,
                                 imu_in->orientation.z, imu_in->orientation.w))
        .getRPY(roll, pitch, yaw);
    yaw = yaw + M_PI / 2;
    if (yaw >= M_PI)
      yaw -= 2 * M_PI;
    else if (yaw <= -M_PI)
      yaw += 2 * M_PI;
    yaw_offset = yaw;
    have_inited_gps_ = true;

    tf::Transform transform_gps_static;
    tf::Quaternion q_gps_static;
    transform_gps_static.setOrigin(tf::Vector3(x_offset, y_offset, 0));
    q_gps_static.setRPY(0, 0, yaw_offset);
    transform_gps_static.setRotation(q_gps_static);
    auto atf = tf::StampedTransform(transform_gps_static, ros::Time::now(),
                                    "world", "map");
    geometry_msgs::TransformStamped stf;
    tf::transformStampedTFToMsg(atf,stf);
    static_br_.sendTransform(stf);
  } else {
    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(imu_in->orientation.x, imu_in->orientation.y,
                                 imu_in->orientation.z, imu_in->orientation.w))
        .getRPY(roll, pitch, yaw);
    yaw = yaw + M_PI / 2;
    if (yaw >= M_PI)
      yaw -= 2 * M_PI;
    else if (yaw <= -M_PI)
      yaw += 2 * M_PI;
    obs_yaw = yaw;
  }

  std_msgs::Float64 gps_angle;
  gps_angle.data = obs_yaw / M_PI * 180;
  pub_gps_angle_.publish(gps_angle);

  t_gps_ = get_time_now();
  // std::cout<<"Z_gps="<<Z_gps<<std::endl;

  if (have_inited_gps_ && localization_debug_) {
    geometry_msgs::PoseStamped original_pose;
    original_pose.header.frame_id = "world";
    original_pose.header.stamp.fromSec(t_gps_);
    char zone;
    LLtoUTM(gps_in->latitude, gps_in->longitude, original_pose.pose.position.y,
            original_pose.pose.position.x, &zone);
    original_pose.pose.position.z = 0;
    original_pose.pose.orientation = tf::createQuaternionMsgFromYaw(obs_yaw);

    tf::Transform transform_gps;
    tf::Quaternion q_gps;
    transform_gps.setOrigin(tf::Vector3(original_pose.pose.position.x,
                                        original_pose.pose.position.y,
                                        original_pose.pose.position.z));
    q_gps.setRPY(0, 0, obs_yaw);
    transform_gps.setRotation(q_gps);
    br_gps_.sendTransform(tf::StampedTransform(transform_gps, ros::Time::now(),
                                               "world", "gps"));

    geometry_msgs::Point gps_marker = original_pose.pose.position;
    gps_marker_traj_.header.frame_id = "world";
    gps_marker_traj_.header.stamp.fromSec(t_gps_);
    gps_marker_traj_.ns = "gps";
    gps_marker_traj_.id = 100;
    gps_marker_traj_.type = 6;
    gps_marker_traj_.action = visualization_msgs::Marker::ADD;
    gps_marker_traj_.scale.x = 0.02;
    gps_marker_traj_.scale.y = 0.02;
    gps_marker_traj_.scale.z = 0.02;
    gps_marker_traj_.points.push_back(gps_marker);
    if (gps_marker_traj_.points.size() > 5000)
      gps_marker_traj_.points.erase(gps_marker_traj_.points.begin());
    std_msgs::ColorRGBA gps_color;
    gps_color.r = 1;
    gps_color.g = 0;
    gps_color.b = 0;
    gps_color.a = 1;
    gps_marker_traj_.colors.push_back(gps_color);
    if (gps_marker_traj_.colors.size() > 5000)
      gps_marker_traj_.colors.erase(gps_marker_traj_.colors.begin());
    pub_gps_marker_.publish(gps_marker_traj_);

    gps_poses_.poses.push_back(original_pose.pose);
    if (gps_poses_.poses.size() > 5000)
      gps_poses_.poses.erase(gps_poses_.poses.begin());
    gps_poses_.header.stamp.fromSec(t_gps_);
    gps_poses_.header.frame_id = "world";
    pub_gps_pose_.publish(gps_poses_);

    ////位置输出调试
    geometry_msgs::Pose gps_pose_xyz_;
    char gps_zone;
    LLtoUTM(gps_in->latitude, gps_in->longitude, gps_pose_xyz_.position.y,
            gps_pose_xyz_.position.x, &gps_zone);
    pub_gps_pose_xyz_.publish(gps_pose_xyz_);
  }
}

void Top::filter_callback(const ros::TimerEvent &) {
  if (!have_inited_vel_) {
    ROS_INFO_THROTTLE(1,"wait for initial!");
    return;
  }

  t_output_ = get_time_now();
  if (t_output_ - t_vel_ > 0.5) {  // 3.0
    ROS_WARN_THROTTLE(1,"no velocity input!");
    return;
  }

  Eigen::VectorXd X;

  if (fabs(curr_vel_) > 0.01)
    X = fusion_.readX(t_output_, true);
  else
    X = fusion_.readX(t_output_, false);

  // X(2) = global_slam_angle;

  MapFrame output_pos(X(0), X(1));
  double output_yaw = X(2);
  double t_pub = get_time_now();
  // std::cout<<"X="<<X<<std::endl;

  geometry_msgs::Pose output_pose;
  // LLtoUTM(output_gps.latitude, output_gps.longitude, output_pose.position.y,
  // output_pose.position.x, &zone); output_pose.position.z = 0;
  // output_pose.orientation =
  // tf::createQuaternionMsgFromYaw(global_slam_angle);
  output_pose.position.x = X(0);
  output_pose.position.y = X(1);
  output_pose.orientation = tf::createQuaternionMsgFromYaw(output_yaw);

  if (output_yaw < -M_PI || output_yaw > M_PI || isnan(output_yaw) ||
      isinf(output_yaw)) {
    ROS_INFO("output heading invalid");
    return;
  }

  cyber_msgs::LocalizationEstimate localization_estimation;
  localization_estimation.header.frame_id = "map";
  localization_estimation.header.stamp.fromSec(t_pub);
  localization_estimation.latitude = 0;
  localization_estimation.longitude = 0;
  localization_estimation.altitude = 0;
  localization_estimation.pose = output_pose;
  localization_estimation.velocity.linear.x = curr_vel_;
  localization_estimation.velocity.linear.y = 0;
  localization_estimation.velocity.linear.z = 0;
  localization_estimation.velocity.angular = angular_velocity_;
  localization_estimation.acceleration.linear = linear_acceleration_;
  localization_estimation.acceleration.angular.x = 0;
  localization_estimation.acceleration.angular.y = 0;
  // localization_estimation.acceleration.angular.z =
  // global_slam_angle/M_PI*180;
  localization_estimation.acceleration.angular.z = 0;
  pub_localization_estimation_.publish(localization_estimation);

  tf::Transform transform_filter;
  tf::Quaternion q_filter;
  transform_filter.setOrigin(tf::Vector3(
      output_pose.position.x, output_pose.position.y, output_pose.position.z));
  q_filter.setRPY(0, 0, output_yaw);
  transform_filter.setRotation(q_filter);
  br_filter_.sendTransform(tf::StampedTransform(
      transform_filter, ros::Time::now(), "map",vehicle_frame_id_));
  // if(command_no_gps_){
  //  std_msgs::Float64 output_angle;
  //  output_angle.data = output_yaw/M_PI*180;
  //  pub_output_angle_.publish(output_angle);
  // }

  // offile << std::fixed << std::setprecision(4) << ros::Time().fromSec(t_pub)
  // << '\t' << output_pose.position.x
  //                                                                                     << '\t' << output_pose.position.y
  //                                                                                         << std::endl;  //??

  if(localization_debug_)
  {

      geometry_msgs::Point filter_marker = output_pose.position;
      filter_marker_traj_.header.frame_id = "map";
      filter_marker_traj_.header.stamp.fromSec(t_pub);
      filter_marker_traj_.ns = "filter";
      filter_marker_traj_.id = 102;
      filter_marker_traj_.type = 6;
      filter_marker_traj_.action = visualization_msgs::Marker::ADD;
      filter_marker_traj_.scale.x = 0.02;
      filter_marker_traj_.scale.y = 0.02;
      filter_marker_traj_.scale.z = 0.02;
      filter_marker_traj_.points.push_back(filter_marker);
      if(filter_marker_traj_.points.size()>5000)
      filter_marker_traj_.points.erase(filter_marker_traj_.points.begin());
      std_msgs::ColorRGBA filter_color;
      filter_color.r = 0; filter_color.g = 0; filter_color.b = 1;
      filter_color.a = 1; filter_marker_traj_.colors.push_back(filter_color);
      if(filter_marker_traj_.colors.size()>5000)
      filter_marker_traj_.colors.erase(filter_marker_traj_.colors.begin());
      pub_filter_marker_.publish(filter_marker_traj_);

      filter_poses_.poses.push_back(output_pose);
      if(filter_poses_.poses.size()>5000)
      filter_poses_.poses.erase(filter_poses_.poses.begin());
      filter_poses_.header.stamp.fromSec(t_pub);
      filter_poses_.header.frame_id = "map";
      pub_filter_pose_.publish(filter_poses_);
  }
}

void Top::target_gps_callback(const cyber_msgs::V2VPacketConstPtr &gps_in) {
  if (have_inited_gps_ && localization_debug_) {
    geometry_msgs::PoseStamped original_pose;
    original_pose.header.frame_id = "world";
    original_pose.header.stamp = ros::Time::now();
    char zone;
    LLtoUTM(gps_in->latitude, gps_in->longitude, original_pose.pose.position.y,
            original_pose.pose.position.x, &zone);
    original_pose.pose.position.z = 0;
    original_pose.pose.orientation = tf::createQuaternionMsgFromYaw(obs_yaw); //use ego yaw as approximation

    tf::Transform transform_gps;
    tf::Quaternion q_gps;
    transform_gps.setOrigin(tf::Vector3(original_pose.pose.position.x,
                                        original_pose.pose.position.y,
                                        original_pose.pose.position.z));
    q_gps.setRPY(0, 0, obs_yaw);
    transform_gps.setRotation(q_gps);
    br_gps_.sendTransform(
        tf::StampedTransform(transform_gps, ros::Time::now(), "world", "target_gps"));

    target_gps_poses_.poses.push_back(original_pose.pose);
    if (target_gps_poses_.poses.size() > 5000)
      target_gps_poses_.poses.erase(target_gps_poses_.poses.begin());
    target_gps_poses_.header.stamp.fromSec(t_gps_);
    target_gps_poses_.header.frame_id = "world";
    pub_target_gps_pose_.publish(target_gps_poses_);

  }
}
