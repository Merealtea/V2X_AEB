#include <cmath>

// // #include <mrpt/bayes/CParticleFilter.h>
// // #include <mrpt/math/wrap2pi.h>
// // #include <mrpt/obs/CSensoryFrame.h>

#include <pcl/filters/passthrough.h>

#include "vehicle_tracking.h"

namespace tracking
{

  VehicleTrackingWrapper::VehicleTrackingWrapper(
      ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
      : nh_(node_handle), pnh_(private_node_handle), tf2_listener_(tf2_buffer_)
  {
    // parse parameters
    pnh_.param("/method", method_, std::string("our"));
    ROS_WARN("method: %s", method_.c_str());
    // camera parameters
    int cam_frame_width, cam_frame_height;
    double cam_f_x, cam_f_y, cam_c_x, cam_c_y, cam_k_1, cam_k_2, cam_k_3, cam_k_4;
    double cam_x, cam_y, cam_z, cam_yaw, cam_pitch, cam_roll;
    pnh_.param("/cam_frame_width", cam_frame_width, 0);
    pnh_.param("/cam_frame_height", cam_frame_height, 0);
    pnh_.param("/cam_f_x", cam_f_x, 0.0);
    pnh_.param("/cam_f_x", cam_f_x, 0.0);
    pnh_.param("/cam_f_y", cam_f_y, 0.0);
    pnh_.param("/cam_c_x", cam_c_x, 0.0);
    pnh_.param("/cam_c_y", cam_c_y, 0.0);
    pnh_.param("/cam_k_1", cam_k_1, 0.0);
    pnh_.param("/cam_k_2", cam_k_2, 0.0);
    pnh_.param("/cam_k_3", cam_k_3, 0.0);
    pnh_.param("/cam_k_4", cam_k_4, 0.0);
    pnh_.param("/cam_x", cam_x, 0.0);
    pnh_.param("/cam_y", cam_y, 0.0);
    pnh_.param("/cam_z", cam_z, 0.0);
    pnh_.param("/cam_yaw", cam_yaw, 0.0);
    pnh_.param("/cam_pitch", cam_pitch, 0.0);
    pnh_.param("/cam_roll", cam_roll, 0.0);
    cv::Mat cam_K(3, 3, CV_32F), cam_D(4, 1, CV_32F), cam_K_new;
    cam_K = (cv::Mat_<double>(3, 3) << cam_f_x, 0.0, cam_c_x, 0.0, cam_f_y,
             cam_c_y, 0.0, 0.0, 1.0);
    cam_D = (cv::Mat_<double>(4, 1) << cam_k_1, cam_k_2, cam_k_3, cam_k_4);
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
        cam_K, cam_D, cv::Size(cam_frame_width, cam_frame_height),
        cv::Mat::eye(cv::Size(3, 3), CV_32FC1), cam_K_new);

    part_based_tracking::SensorPose cam_pose = {cam_x, cam_y, cam_z,
                                                cam_pitch, cam_roll, cam_yaw};
    // target geometry parameters
    double target_left_rearlight_x, target_left_rearlight_y,
        target_right_rearlight_x, target_right_rearlight_y, target_rear_x,
        target_rear_y;
    pnh_.param("/target_left_rearlight_x", target_left_rearlight_x, 0.0);
    pnh_.param("/target_left_rearlight_y", target_left_rearlight_y, 0.0);
    pnh_.param("/target_right_rearlight_x", target_right_rearlight_x, 0.0);
    pnh_.param("/target_right_rearlight_y", target_right_rearlight_y, 0.0);
    pnh_.param("/target_rear_x", target_rear_x, 0.0);
    pnh_.param("/target_rear_y", target_rear_y, 0.0);
    std::cout << "left x: " << target_left_rearlight_x
              << " left y: " << target_left_rearlight_y << std::endl;
    std::cout << "right x: " << target_right_rearlight_x
              << " right y: " << target_right_rearlight_y << std::endl;
    std::unordered_map<part_based_tracking::VehiclePartType, cv::Point2d>
        target_geometry_model = {
            {part_based_tracking::VehiclePartType::LEFTREARLIGHT,
             cv::Point2d(target_left_rearlight_x, target_left_rearlight_y)},
            {part_based_tracking::VehiclePartType::RIGHTREARLIGHT,
             cv::Point2d(target_right_rearlight_x, target_right_rearlight_y)},
            {part_based_tracking::VehiclePartType::REAR,
             cv::Point2d(target_rear_x, target_rear_y)},
            {part_based_tracking::VehiclePartType::REARCENTER,
             cv::Point2d(target_rear_x, target_rear_y)}};
    // parted_based tracker parameters
    int filter_num_particles;
    double filter_model_std_xy, filter_model_std_v, filter_model_std_theta,
        filter_model_std_omega;
    double filter_measurement_std_left_rearlight,
        filter_measurement_std_right_rearlight, filter_measurement_std_rear,
        filter_measurement_std_rear_center;
    pnh_.param("/filter_num_particles", filter_num_particles, 0);
    pnh_.param("/filter_model_std_xy", filter_model_std_xy, 0.0);
    pnh_.param("/filter_model_std_v", filter_model_std_v, 0.0);
    pnh_.param("/filter_model_std_theta", filter_model_std_theta, 0.0);
    pnh_.param("/filter_model_std_omega", filter_model_std_omega, 0.0);
    pnh_.param("/filter_measurement_std_left_rearlight",
               filter_measurement_std_left_rearlight, 0.0);
    pnh_.param("/filter_measurement_std_right_rearlight",
               filter_measurement_std_right_rearlight, 0.0);
    pnh_.param("/filter_measurement_std_rear", filter_measurement_std_rear, 0.0);
    pnh_.param("/filter_measurement_std_rear_center",
               filter_measurement_std_rear_center, 0.0);
    double measurement_std_xy, measurement_std_theta, measurement_std_v,
        measurement_std_omega;
    double prediction_std_xy, prediction_std_theta;
    pnh_.param("/filter_model_std_xy", prediction_std_xy, 0.0);
    pnh_.param("/filter_model_std_theta", prediction_std_theta, 0.0);
    pnh_.param("/filter_measurement_std_xy", measurement_std_xy, 0.0);
    pnh_.param("/filter_measurement_std_theta", measurement_std_theta, 0.0);
    pnh_.param("/filter_measurement_std_v", measurement_std_v, 0.0);
    pnh_.param("/filter_measurement_std_omega", measurement_std_omega, 0.0);

    std::unordered_map<part_based_tracking::VehiclePartType, double>
        measurement_noise_std = {
            {part_based_tracking::VehiclePartType::LEFTREARLIGHT,
             filter_measurement_std_left_rearlight},
            {part_based_tracking::VehiclePartType::RIGHTREARLIGHT,
             filter_measurement_std_right_rearlight},
            {part_based_tracking::VehiclePartType::REAR,
             filter_measurement_std_rear},
            {part_based_tracking::VehiclePartType::REARCENTER,
             filter_measurement_std_rear_center}};

    // radar parameters
    double radar_x, radar_y, radar_z, radar_yaw, radar_pitch, radar_roll;
    pnh_.param("/radar_x", radar_x, 0.0);
    pnh_.param("/radar_y", radar_y, 0.0);
    pnh_.param("/radar_z", radar_z, 0.0);
    pnh_.param("/radar_yaw", radar_yaw, 0.0);
    pnh_.param("/radar_pitch", radar_pitch, 0.0);
    pnh_.param("/radar_roll", radar_roll, 0.0);
    part_based_tracking::SensorPose radar_pose = {
        radar_x, radar_y, radar_z, radar_pitch, radar_roll, radar_yaw};

    double stop_update_time;
    pnh_.param("/filter_stop_update_t", stop_update_time, 9000000000.0);
    tracker_.setStopUpdateTime(stop_update_time);

    bool delay_compensation;
    pnh_.param("/delay_compensation", delay_compensation, true);

  bool use_target_vis_tracking;
  pnh_.param("use_vh_visual_tracking", use_target_vis_tracking, true);

  // initialize tracker
  tracker_ = VehicleTracking(
      filter_num_particles, ros::Time::now().toSec(), filter_model_std_xy,
      filter_model_std_v, filter_model_std_theta, filter_model_std_omega,
      measurement_std_xy, measurement_std_theta, measurement_std_v,
      measurement_std_omega, prediction_std_xy, prediction_std_theta,
      measurement_noise_std, target_geometry_model, cam_pose, cam_K, cam_D,
      cam_K_new, radar_pose, delay_compensation, use_target_vis_tracking);

    //
    frame_ = 0;

    last_target_pos_ = cv::Point2d(0.0, 0.0);
    last_target_yaw_ = 0.0;

    target_pose_buffer_.clear();

  // ros components
  sub_image_ = nh_.subscribe("/driver/fisheye/front/compressed", 1,
                             &VehicleTrackingWrapper::image_callback, this);
  sub_radar_ = nh_.subscribe("/radar_pc", 1,
                             &VehicleTrackingWrapper::radar_callback, this);
  sub_vis_det_ = nh_.subscribe("/tracking/visual_detection", 1,
                               &VehicleTrackingWrapper::vis_det_callback, this);
  sub_rearlight_bbox_ =
      nh_.subscribe("/tracking/components_tracking/rearlight", 1,
                    &VehicleTrackingWrapper::rearlight_callback, this);
  sub_rearlight_left_ =
      nh_.subscribe("/tracking/components_tracking/rearlight/left", 1,
                    &VehicleTrackingWrapper::rearlight_left_callback, this);
  sub_rearlight_right_ =
      nh_.subscribe("/tracking/components_tracking/rearlight/right", 1,
                    &VehicleTrackingWrapper::rearlight_right_callback, this);
  sub_vh_bbox_ = nh_.subscribe("/tracking/visual_tracking", 1,
                               &VehicleTrackingWrapper::vh_callback, this);
  sub_rearlight_pseudo_left_ = nh_.subscribe(
      "/tracking/components_tracking/rearlight/pseudo/left", 1,
      &VehicleTrackingWrapper::rearlight_pseudo_left_callback, this);
  sub_rearlight_pseudo_right_ = nh_.subscribe(
      "/tracking/components_tracking/rearlight/pseudo/right", 1,
      &VehicleTrackingWrapper::rearlight_pseudo_right_callback, this);
  sub_v2v_ = nh_.subscribe("/V2V/leader", 1,
                           &VehicleTrackingWrapper::v2v_callback, this);
  sub_leader_imu_ = nh_.subscribe(
      "/leader/wit/imu", 1, &VehicleTrackingWrapper::leader_imu_callback, this);
  sub_ego_imu_ = nh_.subscribe("/wit/imu", 1,
                               &VehicleTrackingWrapper::ego_imu_callback, this);
  sub_ego_speed_ =
      nh_.subscribe("/rock_can/speed_feedback", 1,
                    &VehicleTrackingWrapper::ego_speed_callback, this);
  sub_lidar_ = nh_.subscribe("/driver/hesai/pandar", 1,
                             &VehicleTrackingWrapper::lidar_callback, this);
  // subscriber for ablation study
  sub_ablation_rearlight_all_ = nh_.subscribe(
      "/tracking/ablation/rearlight_all", 1,
      &VehicleTrackingWrapper::ablation_rearlight_all_callback, this);
  sub_ablation_rearlight_left_ = nh_.subscribe(
      "/tracking/ablation/rearlight_left", 1,
      &VehicleTrackingWrapper::ablation_rearlight_left_callback, this);
  sub_ablation_rearlight_right_ = nh_.subscribe(
      "/tracking/ablation/rearlight_right", 1,
      &VehicleTrackingWrapper::ablation_rearlight_right_callback, this);
  sub_ablation_radar_ =
      nh_.subscribe("/tracking/ablation/radar", 1,
                    &VehicleTrackingWrapper::ablation_radar_callback, this);
  sub_ablation_v2v_ =
      nh_.subscribe("/tracking/ablation/v2v", 1,
                    &VehicleTrackingWrapper::ablation_v2v_callback, this);

    pub_marker_ =
        nh_.advertise<visualization_msgs::Marker>("/tracking/visualization", 10);
    pub_vis_rematch_ = nh_.advertise<std_msgs::Bool>(
        "/tracking/components_tracking/rematch", 10);
    pub_vh_vis_rematch_ =
        nh_.advertise<cyber_msgs::Box2D>("/tracking/vehicle_vis/rematch", 10);
    pub_pose_lidar_ =
        nh_.advertise<geometry_msgs::PoseStamped>("/tracking/pose_lidar", 10);
    pub_target_track_vis_ =
        nh_.advertise<geometry_msgs::PoseArray>("/mapping/track/target", 10);
    pub_left_err_ = nh_.advertise<std_msgs::Float32>("/tracking/err/left", 10);
    pub_right_err_ = nh_.advertise<std_msgs::Float32>("/tracking/err/right", 10);
    pub_left_pos_ =
        nh_.advertise<geometry_msgs::PointStamped>("/tracking/pos/left", 10);
    pub_right_pos_ =
        nh_.advertise<geometry_msgs::PointStamped>("/tracking/pos/right", 10);
    pub_front_relative_pose_ = nh_.advertise<geometry_msgs::Pose2D>(
        "/tracking/front_vehicle/local_pose", 1);
    pub_front_global_pose_ = nh_.advertise<geometry_msgs::Pose2D>("/tracking/front_vehicle/global_pose", 1);
    pub_lost_ =
        nh_.advertise<std_msgs::Bool>("/tracking/front_vehicle/is_lost", 10);
    timer_ = nh_.createTimer(ros::Duration(1.0 / 50),
                             &VehicleTrackingWrapper::getCurrState, this);

    // tracking init params, for offline
    double vh_1_x, vh_1_y, vh_2_x, vh_2_y;
    bool use_init_param;
    pnh_.param("/use_init_param", use_init_param, false);
    if (use_init_param)
    {
      pnh_.param("/vh_1_x", vh_1_x, -1.0);
      pnh_.param("/vh_1_y", vh_1_y, -1.0);
      pnh_.param("/vh_2_x", vh_2_x, -1.0);
      pnh_.param("/vh_2_y", vh_2_y, -1.0);
      cv::Rect2d init_vh_bbox{cv::Point2d(vh_1_x, vh_1_y),
                              cv::Point2d(vh_2_x, vh_2_y)};
      std_msgs::Header dummy_header;
    tracker_.updateVhBbox(init_vh_bbox, dummy_header, true);
    }
  }

  void VehicleTrackingWrapper::image_callback(
      const sensor_msgs::CompressedImageConstPtr &image_in)
  {
    // ROS_INFO("image callback");
    image_ = cv::imdecode(cv::Mat(image_in->data), 1);
    image_msg_header_ = image_in->header;

    // // det and track components
    // cv::Rect2d bbox_vh, bbox_left, bbox_right;
    // comp_tracker_.detect(image_, bbox_vh, bbox_left, bbox_right);
    // std::unordered_map<part_based_tracking::VehiclePartType, cv::Rect2d> boxes;
    // // TODO: check validity of boxes
    // if (bbox_left.area() > 0)
    //   boxes.insert(
    //       {part_based_tracking::VehiclePartType::LEFTREARLIGHT, bbox_left});
    // if (bbox_right.area() > 0)
    //   boxes.insert(
    //       {part_based_tracking::VehiclePartType::RIGHTREARLIGHT, bbox_right});

    // tracker_.updateVhBbox(bbox_vh);

    // if (boxes.size() > 0)
    //   tracker_.predictUpdate(boxes, image_msg_header_.stamp.toSec());
    // ROS_INFO("image callback done");
  }

  void VehicleTrackingWrapper::radar_callback(
      const sensor_msgs::PointCloud2ConstPtr &radar_in)
  {
    ROS_INFO("radar callback");

    sensor_msgs::PointCloud2 transformed_msg;
    pcl::PointCloud<pcl::PointXYZINormal> radar_pc;
    // try {
    //   auto transform = tf_buffer_.lookupTransform("radar", "base_link",
    //   ros::Time(0)); tf2::doTransform(*radar_in, transformed_msg, transform);
    // }
    // catch (tf2::TransformException &ex){
    //   return;
    // }
    pcl_ros::transformPointCloud("base_link", *radar_in, transformed_msg,
                                 tf_listener_);
    pcl::fromROSMsg(transformed_msg, radar_pc);
    ROS_INFO("radar make shared");
    radar_ = radar_pc.makeShared();
    ROS_INFO("radar make shared done");
    radar_msg_header_ = radar_in->header;

    // update ego vehicle state
    try
    {
      geometry_msgs::TransformStamped transform;
      transform = tf2_buffer_.lookupTransform("map", "base_link", ros::Time(0));
      double roll, pitch, yaw;
      tf2::Quaternion q(
          transform.transform.rotation.x, transform.transform.rotation.y,
          transform.transform.rotation.z, transform.transform.rotation.w);
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      tracker_.updateEgoState(cv::Point2d(transform.transform.translation.x,
                                          transform.transform.translation.y),
                              yaw);
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
    }
    tracker_.processRadar(radar_, radar_in->header.stamp.toSec());
    // tracker_.predictUpdate(radar_, radar_msg_header_.stamp.toSec());
    ROS_INFO("radar callback done");
  }

  void VehicleTrackingWrapper::v2v_callback(
      const cyber_msgs::V2VPacketConstPtr &msg)
  {
    ROS_INFO("v2v callback");
    tracker_.updateV2V(msg->speed,
                       msg->angular_velo_z, ros::Time::now().toSec());
    ROS_INFO("v2v callback done");
  }

  void VehicleTrackingWrapper::leader_imu_callback(
      const sensor_msgs::ImuConstPtr &msg)
  {
    // tracker_.updateV2VImu(msg->angular_velocity.z);
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z,
                      msg->orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    tracker_.updateV2VImu(yaw + 0.0 / 180.0 * M_PI);
  }

  void VehicleTrackingWrapper::ego_imu_callback(
      const sensor_msgs::ImuConstPtr &msg)
  {
    tracker_.updateEgoAngularVelo(msg->angular_velocity.z,
                                  ros::Time::now().toSec());
  }

  void VehicleTrackingWrapper::ego_speed_callback(
      const cyber_msgs::SpeedFeedbackConstPtr &msg)
  {
    tracker_.updateEgoVelo(msg->speed_cms / 100.0);
  }

  void VehicleTrackingWrapper::vis_det_callback(
      const cyber_msgs::Box2DArrayConstPtr &det_in)
  {
    ROS_INFO("vis_det callback");
    vis_det_.clear();
    std::vector<cyber_msgs::Box2D> boxes_vec;
    for (const auto box_pair : det_in->boxes)
    {
      vis_det_.emplace_back(std::make_pair(box_pair, det_in->header));
      boxes_vec.emplace_back(box_pair);
    }
    // comp_tracker_.update_vis_det(boxes_vec);
  }

void VehicleTrackingWrapper::rearlight_pseudo_left_callback(
    const cyber_msgs::Box2DConstPtr &msg) {
  cv::Rect2d box;
  box.x = static_cast<double>(msg->center_y - msg->width / 2);
  box.y = static_cast<double>(msg->center_x - msg->height / 2);
  box.width = static_cast<double>(msg->width);
  box.height = static_cast<double>(msg->height);
  tracker_.updatePseudoLeftBox(box);
}

void VehicleTrackingWrapper::rearlight_pseudo_right_callback(
    const cyber_msgs::Box2DConstPtr &msg) {
  cv::Rect2d box;
  box.x = static_cast<double>(msg->center_y - msg->width / 2);
  box.y = static_cast<double>(msg->center_x - msg->height / 2);
  box.width = static_cast<double>(msg->width);
  box.height = static_cast<double>(msg->height);
  tracker_.updatePseudoRightBox(box);
}

void VehicleTrackingWrapper::rearlight_callback(
    const cyber_msgs::Box2DArrayConstPtr &msg) {
  // order: left, right
  ROS_INFO("rearlight callback");
  std::cout << "image delay: " << (ros::Time::now() - msg->header.stamp).toSec()
            << " sec" << std::endl;
  std::vector<bool> rearlight_valid;
  rearlight_bbox_.clear();
  for (const auto &box : msg->boxes) {
    cv::Rect2d cv_box;
    cv_box.x = static_cast<double>(box.center_y - box.width / 2);
    cv_box.y = static_cast<double>(box.center_x - box.height / 2);
    cv_box.width = static_cast<double>(box.width);
    cv_box.height = static_cast<double>(box.height);
    rearlight_bbox_.emplace_back(std::make_pair(cv_box, msg->header));
    std::cout << "reveive box: " << cv_box << " box score: " << box.score
              << std::endl;
    if (box.score < 0.8) {
      rearlight_valid.emplace_back(false);
    } else {
      rearlight_valid.emplace_back(true);
    }
  }

  std::unordered_map<part_based_tracking::VehiclePartType, cv::Rect2d>
      obs_boxes;
    obs_boxes.insert({part_based_tracking::VehiclePartType::LEFTREARLIGHT,
                      rearlight_bbox_[0].first});
    obs_boxes.insert({part_based_tracking::VehiclePartType::RIGHTREARLIGHT,
                      rearlight_bbox_[1].first});

    // update ego vehicle state
    try
    {
      geometry_msgs::TransformStamped transform;
      transform = tf2_buffer_.lookupTransform("map", "base_link", msg->header.stamp);
      double roll, pitch, yaw;
      tf2::Quaternion q(
          transform.transform.rotation.x, transform.transform.rotation.y,
          transform.transform.rotation.z, transform.transform.rotation.w);
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      tracker_.updateEgoState(cv::Point2d(transform.transform.translation.x,
                                          transform.transform.translation.y),
                              yaw);
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
    }
  try {
    geometry_msgs::TransformStamped transform;
    transform =
        tf2_buffer_.lookupTransform("map", "base_link", ros::Time(0));
    double roll, pitch, yaw;
    tf2::Quaternion q(
        transform.transform.rotation.x, transform.transform.rotation.y,
        transform.transform.rotation.z, transform.transform.rotation.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    tracker_.updateEgoLatestState(cv::Point2d(transform.transform.translation.x,
                                        transform.transform.translation.y),
                            yaw);
  } catch (tf::TransformException &ex) {
    ROS_WARN("%s", ex.what());
  }

    double left_err, right_err;
    bool succ = tracker_.processRearlight(obs_boxes, rearlight_valid,
                                          msg->header.stamp.toSec(), left_err,
                                          right_err, method_);

    std_msgs::Float32 left_err_msg, right_err_msg;
    left_err_msg.data = left_err;
    right_err_msg.data = right_err;
    pub_left_err_.publish(left_err_msg);
    pub_right_err_.publish(right_err_msg);

    if (!succ)
    {
      std_msgs::Bool rematch_msg;
      rematch_msg.data = true;
      pub_vis_rematch_.publish(rematch_msg);
    }

  // if (obs_boxes.size() > 0) {
  //   tracker_.predictUpdate(obs_boxes, msg->header.stamp.toSec());
  // }
  ROS_INFO("rearlight callback done");
}

void VehicleTrackingWrapper::rearlight_left_callback(
    const cyber_msgs::Box2DConstPtr &msg) {
  ROS_INFO("left rearlight callback");
  auto t_now = ros::Time::now();
  std::cout << "left delay " << (t_now - msg->header.stamp).toSec() << " sec"
            << std::endl;
  bool rearlight_valid;
  rearlight_bbox_.clear();
  cv::Rect2d cv_box;
  cv_box.x = static_cast<double>(msg->center_y - msg->width / 2);
  cv_box.y = static_cast<double>(msg->center_x - msg->height / 2);
  cv_box.width = static_cast<double>(msg->width);
  cv_box.height = static_cast<double>(msg->height);
  // rearlight_bbox_.emplace_back(std::make_pair(cv_box, msg->header));
  std::cout << "reveive box: " << cv_box << " box score: " << msg->score
            << std::endl;
  if (msg->score < 0.7) {
    rearlight_valid = false;
  } else {
    rearlight_valid = true;
  }

  // update ego vehicle state
  try {
    geometry_msgs::TransformStamped transform;
    transform =
        tf2_buffer_.lookupTransform("map", "base_link", msg->header.stamp);
    double roll, pitch, yaw;
    tf2::Quaternion q(
        transform.transform.rotation.x, transform.transform.rotation.y,
        transform.transform.rotation.z, transform.transform.rotation.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    tracker_.updateEgoState(cv::Point2d(transform.transform.translation.x,
                                        transform.transform.translation.y),
                            yaw);
  } catch (tf::TransformException &ex) {
    ROS_WARN("%s", ex.what());
  }
  try {
    geometry_msgs::TransformStamped transform;
    transform =
        tf2_buffer_.lookupTransform("map", "base_link", ros::Time(0));
    double roll, pitch, yaw;
    tf2::Quaternion q(
        transform.transform.rotation.x, transform.transform.rotation.y,
        transform.transform.rotation.z, transform.transform.rotation.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    tracker_.updateEgoLatestState(cv::Point2d(transform.transform.translation.x,
                                        transform.transform.translation.y),
                            yaw);
  } catch (tf::TransformException &ex) {
    ROS_WARN("%s", ex.what());
  }

  double err;
  bool succ = tracker_.processSingleRearlight(
      cv_box, part_based_tracking::VehiclePartType::LEFTREARLIGHT,
      rearlight_valid, msg->header.stamp.toSec(), err, method_);

  std_msgs::Float32 left_err_msg;
  left_err_msg.data = err;
  pub_left_err_.publish(left_err_msg);

  if (!succ) {
    std_msgs::Bool rematch_msg;
    rematch_msg.data = true;
    pub_vis_rematch_.publish(rematch_msg);
  }
  ROS_INFO("left rearlight callback done");
}

void VehicleTrackingWrapper::rearlight_right_callback(
    const cyber_msgs::Box2DConstPtr &msg) {
  ROS_INFO("right rearlight callback");
  auto t_now = ros::Time::now();
  std::cout << "right delay " << (t_now - msg->header.stamp).toSec() << " sec"
            << std::endl;
  bool rearlight_valid;
  rearlight_bbox_.clear();
  cv::Rect2d cv_box;
  cv_box.x = static_cast<double>(msg->center_y - msg->width / 2);
  cv_box.y = static_cast<double>(msg->center_x - msg->height / 2);
  cv_box.width = static_cast<double>(msg->width);
  cv_box.height = static_cast<double>(msg->height);
  // rearlight_bbox_.emplace_back(std::make_pair(cv_box, msg->header));
  std::cout << "reveive box: " << cv_box << " box score: " << msg->score
            << std::endl;
  if (msg->score < 0.7) {
    rearlight_valid = false;
  } else {
    rearlight_valid = true;
  }

  // update ego vehicle state
  try {
    geometry_msgs::TransformStamped transform;
    transform =
        tf2_buffer_.lookupTransform("map", "base_link", msg->header.stamp);
    double roll, pitch, yaw;
    tf2::Quaternion q(
        transform.transform.rotation.x, transform.transform.rotation.y,
        transform.transform.rotation.z, transform.transform.rotation.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    tracker_.updateEgoState(cv::Point2d(transform.transform.translation.x,
                                        transform.transform.translation.y),
                            yaw);
  } catch (tf::TransformException &ex) {
    ROS_WARN("%s", ex.what());
  }
  try {
    geometry_msgs::TransformStamped transform;
    transform =
        tf2_buffer_.lookupTransform("map", "base_link", ros::Time(0));
    double roll, pitch, yaw;
    tf2::Quaternion q(
        transform.transform.rotation.x, transform.transform.rotation.y,
        transform.transform.rotation.z, transform.transform.rotation.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    tracker_.updateEgoLatestState(cv::Point2d(transform.transform.translation.x,
                                        transform.transform.translation.y),
                            yaw);
  } catch (tf::TransformException &ex) {
    ROS_WARN("%s", ex.what());
  }

  double err;
  bool succ = tracker_.processSingleRearlight(
      cv_box, part_based_tracking::VehiclePartType::RIGHTREARLIGHT,
      rearlight_valid, msg->header.stamp.toSec(), err, method_);

  std_msgs::Float32 right_err_msg;
  right_err_msg.data = err;
  pub_right_err_.publish(right_err_msg);

  if (!succ) {
    std_msgs::Bool rematch_msg;
    rematch_msg.data = true;
    pub_vis_rematch_.publish(rematch_msg);
  }
  ROS_INFO("right rearlight callback done");
}

void VehicleTrackingWrapper::vh_callback(const cyber_msgs::Box2DConstPtr &msg) {
  cv::Rect2d box;
  box.x = static_cast<double>(msg->center_y - msg->width / 2);
  box.y = static_cast<double>(msg->center_x - msg->height / 2);
  box.width = static_cast<double>(msg->width);
  box.height = static_cast<double>(msg->height);
  vh_bbox_ = std::make_pair(box, msg->header);
  bool vh_box_valid = msg->score > 0.7;

  tracker_.updateVhBbox(box, msg->header, vh_box_valid);
}

  void VehicleTrackingWrapper::lidar_callback(
      const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    lidar_msg_header_ = msg->header;
  }

  void VehicleTrackingWrapper::getCurrState(const ros::TimerEvent &event)
  {
    ROS_INFO("timer callback");
    ++frame_;
    // update ego vehicle state
    try
    {
      geometry_msgs::TransformStamped transform;
      transform = tf2_buffer_.lookupTransform("map", "base_link", ros::Time(0));
      double roll, pitch, yaw;
      tf2::Quaternion q(
          transform.transform.rotation.x, transform.transform.rotation.y,
          transform.transform.rotation.z, transform.transform.rotation.w);
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      tracker_.updateEgoState(cv::Point2d(transform.transform.translation.x,
                                          transform.transform.translation.y),
                              yaw);
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
    }

    if (frame_ < 50)
      return;

    auto t_now = ros::Time::now();
    Eigen::VectorXd X;
    tracker_.ekfReadX(t_now.toSec(), true, X);
    geometry_msgs::Pose2D state;
    state.x = X(0);
    state.y = X(1);
    state.theta = X(2);
    std::cout << "curr state x: " << state.x << " y: " << state.y
              << " yaw: " << state.theta << std::endl;

    geometry_msgs::PoseStamped target_pose_map;
    target_pose_map.header.frame_id = "map";
    target_pose_map.header.stamp = t_now;
    target_pose_map.pose.position.x = state.x;
    target_pose_map.pose.position.y = state.y;
    target_pose_map.pose.position.z = 0.0;
    tf2::Quaternion q_target_map;
    q_target_map.setRPY(0.0, 0.0, state.theta);
    target_pose_map.pose.orientation.x = q_target_map.x();
    target_pose_map.pose.orientation.y = q_target_map.y();
    target_pose_map.pose.orientation.z = q_target_map.z();
    target_pose_map.pose.orientation.w = q_target_map.w();

    // publish to control module
    // lost msg
    std_msgs::Bool lost_msg;
    lost_msg.data = false;
    pub_lost_.publish(lost_msg);

    // relative pose msg
    geometry_msgs::PoseStamped target_pose_relative;
    try
    {
      target_pose_relative = tf2_buffer_.transform(target_pose_map, "base_link");

      geometry_msgs::Pose2D target_pose_relative_msg;
      target_pose_relative_msg.x = target_pose_relative.pose.position.x;
      target_pose_relative_msg.y = target_pose_relative.pose.position.y;
      tf2::Quaternion target_q_relative(target_pose_relative.pose.orientation.x,
                                        target_pose_relative.pose.orientation.y,
                                        target_pose_relative.pose.orientation.z,
                                        target_pose_relative.pose.orientation.w);
      double relative_roll, relative_pitch, relative_yaw;
      tf2::Matrix3x3(target_q_relative)
          .getRPY(relative_roll, relative_pitch, relative_yaw);
      target_pose_relative_msg.theta = relative_yaw;

      pub_front_relative_pose_.publish(target_pose_relative_msg);
    }
    catch (...)
    {
      ROS_WARN("transform from map to base_link fail!");
    }

    // global pose msg
    geometry_msgs::Pose2D target_pose_global_msg;
    target_pose_global_msg.x = state.x;
    target_pose_global_msg.y = state.y;
    target_pose_global_msg.theta = state.theta;
    pub_front_global_pose_.publish(target_pose_global_msg);

    updateTargetPoseBuffer(state);

    // publish target track
    geometry_msgs::PoseArray::Ptr target_track_output(
        new geometry_msgs::PoseArray());
    for (const auto &pose : target_pose_buffer_)
    {
      geometry_msgs::Pose p;
      p.position.x = pose.x;
      p.position.y = pose.y;
      p.orientation.w = std::cos(pose.theta / 2);
      p.orientation.x = 0;
      p.orientation.y = 0;
      p.orientation.z = std::sin(pose.theta / 2);
      target_track_output->poses.emplace_back(p);
    }
    target_track_output->header.frame_id = "map";
    target_track_output->header.stamp = ros::Time::now();
    pub_target_track_vis_.publish(target_track_output);

    // get state in Lidar frame, for experiment only
    geometry_msgs::PoseStamped target_pose_lidar;
    try
    {
      target_pose_lidar = tf2_buffer_.transform(target_pose_map, "Pandar40");
      // geometry_msgs::Pose2D target_pose2d_lidar;
      // target_pose2d_lidar.x = target_pose_lidar.pose.position.x;
      // target_pose2d_lidar.y = target_pose_lidar.pose.position.y;
      // tf2::Quaternion target_q_lidar(target_pose_lidar.pose.orientation.x,
      //                                target_pose_lidar.pose.orientation.y,
      //                                target_pose_lidar.pose.orientation.z,
      //                                target_pose_lidar.pose.orientation.w);
      // double roll, pitch, yaw;
      // tf2::Matrix3x3(target_q_lidar).getRPY(roll, pitch, yaw);
      // target_pose2d_lidar.theta = yaw;
      target_pose_lidar.header.stamp = lidar_msg_header_.stamp;
      pub_pose_lidar_.publish(target_pose_lidar);
    }
    catch (...)
    {
      ROS_WARN("transform from map to Pandar40 fail!");
    }

    cv::Point2d rear_center_proposal;
    tracker_.getRearCenterProposal(rear_center_proposal);

    // auto curr_t = tracker_.getTimestamp();
    // double x, y, v, theta, omega;
    // double r_x, r_y, r_v, r_theta, r_omega;
    // cv::Point2d left_rearlight, right_rearlight;
    // try {
    //   tracker_.getCurrState(x, y, v, theta, omega, r_x, r_y, r_v, r_theta,
    //                         r_omega, left_rearlight, right_rearlight);
    // } catch (...) {
    //   tracker_.reInit(last_target_pos_, tracker_.getEgoYaw());
    //   return;
    // }
    // last_target_pos_.x = x;
    // last_target_pos_.y = y;
    // last_target_yaw_ = theta;

    // // search
    // cv::Point2d target_rear_center;
    // tracker_.searchYawCenter(theta, target_rear_center);
    // cv::Point2d target_rear_center_global;
    // double target_yaw_global;
    // tracker_.transformLocal2Global(target_rear_center, 0.0,
    //                                target_rear_center_global,
    //                                target_yaw_global);
    // if (cv::norm(target_rear_center_global - cv::Point2d(x, y)) > 3.0) {
    //   tracker_.reInit(target_rear_center_global, theta);
    // }
    // std::cout << "target x: " << x << " target y:" << y <<std::endl;
    // std::cout << "left: " << left_rearlight << std::endl;
    // std::cout << "right: " << right_rearlight << std::endl;

    // // get buffer timestamp
    // std::vector<double> buffer_timestamp;
    // tracker_.getBufferTimestamp(buffer_timestamp);
    // std::cout << "buffer t: " << std::endl;
    // for (const auto t : buffer_timestamp) {
    //   std::cout << std::to_string(t) << std::endl;
    // }

    // TODO: publish curr state
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "map";
    transform.child_frame_id = "target";
    transform.transform.translation.x = state.x;
    transform.transform.translation.y = state.y;
    // transform.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, state.theta);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    if (!std::isnan(transform.transform.translation.x) &&
        !std::isnan(transform.transform.translation.y))
      br_.sendTransform(transform);

    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "Pandar40";
    transform.child_frame_id = "target_lidar";
    transform.transform.translation.x = target_pose_lidar.pose.position.x;
    transform.transform.translation.y = target_pose_lidar.pose.position.y;
    // transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = target_pose_lidar.pose.orientation.x;
    transform.transform.rotation.y = target_pose_lidar.pose.orientation.y;
    transform.transform.rotation.z = target_pose_lidar.pose.orientation.z;
    transform.transform.rotation.w = target_pose_lidar.pose.orientation.w;
    if (!std::isnan(transform.transform.translation.x) &&
        !std::isnan(transform.transform.translation.y))
      br_.sendTransform(transform);

    // transform.header.stamp = ros::Time::now();
    // transform.header.frame_id = "map";
    // transform.child_frame_id = "target_left";
    // transform.transform.translation.x = left_rearlight.x;
    // transform.transform.translation.y = left_rearlight.y;
    // // transform.transform.translation.z = 0.0;
    // q.setRPY(0.0, 0.0, theta);
    // transform.transform.rotation.x = q.x();
    // transform.transform.rotation.y = q.y();
    // transform.transform.rotation.z = q.z();
    // transform.transform.rotation.w = q.w();
    // if (!std::isnan(transform.transform.translation.x) &&
    //     !std::isnan(transform.transform.translation.y))
    //   br_.sendTransform(transform);

    // transform.header.stamp = ros::Time::now();
    // transform.header.frame_id = "map";
    // transform.child_frame_id = "target_right";
    // transform.transform.translation.x = right_rearlight.x;
    // transform.transform.translation.y = right_rearlight.y;
    // // transform.transform.translation.z = 0.0;
    // q.setRPY(0.0, 0.0, theta);
    // transform.transform.rotation.x = q.x();
    // transform.transform.rotation.y = q.y();
    // transform.transform.rotation.z = q.z();
    // transform.transform.rotation.w = q.w();
    // if (!std::isnan(transform.transform.translation.x) &&
    //     !std::isnan(transform.transform.translation.y))
    //   br_.sendTransform(transform);

  transform.header.stamp = ros::Time::now();
  transform.header.frame_id = "map";
  transform.child_frame_id = "target_rear_center";
  transform.transform.translation.x = rear_center_proposal.x;
  transform.transform.translation.y = rear_center_proposal.y;
  // transform.transform.translation.z = 0.0;
  q.setRPY(0.0, 0.0, 0.0);
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();
  if (!std::isnan(transform.transform.translation.x) &&
      !std::isnan(transform.transform.translation.y))
    br_.sendTransform(transform);

    // publish left & right rearlight position in camera coordinates
    geometry_msgs::Pose2D left_pose_global, right_pose_global;
    tracker_.calcPartPosition(state,
                              part_based_tracking::VehiclePartType::LEFTREARLIGHT,
                              left_pose_global);
    tracker_.calcPartPosition(
        state, part_based_tracking::VehiclePartType::RIGHTREARLIGHT,
        right_pose_global);
    cv::Point2d left_pos_global{left_pose_global.x, left_pose_global.y},
        right_pos_global{right_pose_global.x, right_pose_global.y};
    cv::Point2d left_pos_cam, right_pos_cam;
    tracker_.transformGlobal2Cam(left_pos_global, left_pos_cam);
    tracker_.transformGlobal2Cam(right_pos_global, right_pos_cam);
    geometry_msgs::PointStamped left_pos_msg, right_pos_msg;
    left_pos_msg.header.stamp = right_pos_msg.header.stamp = ros::Time::now();
    left_pos_msg.point.x = left_pos_cam.x;
    left_pos_msg.point.y = left_pos_cam.y;
    left_pos_msg.point.z = 0.0;
    right_pos_msg.point.x = right_pos_cam.x;
    right_pos_msg.point.y = right_pos_cam.y;
    right_pos_msg.point.z = 0.0;
    pub_left_pos_.publish(left_pos_msg);
    pub_right_pos_.publish(right_pos_msg);

    // do all visualization here
    // visualize filtered state
    if (!std::isnan(state.x) && !std::isnan(state.y))
      visualize_point(cv::Point2d(state.x, state.y), 0.3, 0.3, 1, "r");
    // visualize rearlight and vehicle direction
    std::vector<double> rearlight_directions;
    tracker_.getRearlightDirection(rearlight_directions);
    int id = 2;
    for (const auto &direc : rearlight_directions)
    {
      // std::cout << "rearlight direc: " << direc << std::endl;
      visualize_ray(direc, id++);
    }
    double direction_vh;
    tracker_.getVehicleDirection(direction_vh);
    visualize_ray(direction_vh, 4, "g");
    // std::cout << "vehicle direction: " << direction_vh << std::endl;
    // visualize matched radar point
    cv::Point2d matched_radar_pt;
    tracker_.getMatchedRadarPoint(matched_radar_pt);
    // std::cout << "match: " << matched_radar_pt << std::endl;
    visualize_point(matched_radar_pt, 5);

    ROS_INFO("timer callback done");
  }

  void VehicleTrackingWrapper::visualize_ray(const double angle, const int id,
                                             const std::string &color) const
  {
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "base_link";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "lines";
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.id = id;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.scale.x = 0.1;
    if (color == "r")
    {
      line_list.color.r = 1.0;
    }
    else if (color == "g")
    {
      line_list.color.g = 1.0;
    }
    else
    {
      line_list.color.b = 1.0;
    }
    line_list.color.a = 1.0;
    line_list.pose.orientation.w = 1.0;

    part_based_tracking::SensorPose cam_pose;
    tracker_.getCamExternal(cam_pose);
    geometry_msgs::Point end_point;
    end_point.x = cam_pose.x;
    end_point.y = cam_pose.y;
    end_point.z = 0.0;
    line_list.points.push_back(end_point);
    double x = 30.0;
    double y = std::tan(angle) * (x - cam_pose.x) + cam_pose.y;
    end_point.x = x;
    end_point.y = y;
    line_list.points.push_back(end_point);

    pub_marker_.publish(line_list);
  }

  void VehicleTrackingWrapper::visualize_point(const cv::Point2d &position,
                                               const int id,
                                               const std::string &color) const
  {
    visualization_msgs::Marker point;
    if (id == 1)
    {
      point.header.frame_id = "map";
    }
    else
    {
      point.header.frame_id = "base_link";
    }
    point.header.stamp = ros::Time::now();
    point.ns = "point";
    point.type = visualization_msgs::Marker::SPHERE;
    point.id = id;
    point.action = visualization_msgs::Marker::ADD;
    point.scale.x = 0.5;
    point.scale.y = 0.5;
    point.scale.z = 0.01;

    if (color == "g")
    {
      point.color.g = 1.0;
    }
    else if (color == "r")
    {
      point.color.r = 1.0;
    }
    else
    {
      point.color.b = 1.0;
    }
    point.color.a = 1.0;
    point.pose.orientation.x = point.pose.orientation.y =
        point.pose.orientation.z = point.pose.orientation.w = 0.0;
    point.pose.position.x = position.x;
    point.pose.position.y = position.y;
    // point.pose.position.z = 0.0;

    pub_marker_.publish(point);
  }

  void VehicleTrackingWrapper::visualize_point(const cv::Point2d &position,
                                               const double r_x, const double r_y,
                                               const int id,
                                               const std::string &color) const
  {
    visualization_msgs::Marker point;
    if (id == 1)
    {
      point.header.frame_id = "map";
    }
    else
    {
      point.header.frame_id = "base_link";
    }
    point.header.stamp = ros::Time::now();
    point.ns = "point";
    point.type = visualization_msgs::Marker::SPHERE;
    point.id = id;
    point.action = visualization_msgs::Marker::ADD;
    point.scale.x = r_x;
    point.scale.y = r_y;
    point.scale.z = 0.01;

    if (color == "g")
    {
      point.color.g = 1.0;
    }
    else if (color == "r")
    {
      point.color.r = 1.0;
    }
    else
    {
      point.color.b = 1.0;
    }
    point.color.a = 1.0;
    point.pose.orientation.x = point.pose.orientation.y =
        point.pose.orientation.z = point.pose.orientation.w = 0.0;
    point.pose.position.x = position.x;
    point.pose.position.y = position.y;
    // point.pose.position.z = 0.0;

    pub_marker_.publish(point);
  }

// VehicleTracking
VehicleTracking::VehicleTracking(
    const int num_particles, const double t, const double std_xy,
    const double std_v, const double std_theta, const double std_omega,
    const double measurement_std_xy, const double measurement_std_theta,
    const double measurement_std_v, const double measurement_std_omega,
    const double prediction_std_xy, const double prediction_std_theta,
    const std::unordered_map<part_based_tracking::VehiclePartType, double>
        &measurement_noise_std,
    const std::unordered_map<part_based_tracking::VehiclePartType, cv::Point2d>
        &geometry_model,
    const part_based_tracking::SensorPose &cam_pose, const cv::Mat &K,
    const cv::Mat &D, const cv::Mat &K_new,
    const part_based_tracking::SensorPose &radar_pose,
    const bool delay_compensation, const bool use_target_vis_tracking)
    : tracker_init_(false), tracker_frame_(0), num_particles_(num_particles),
      cam_external_(cam_pose), cam_K_(K), cam_D_(D), cam_K_new_(K_new),
      radar_external_(radar_pose), geometry_model_(geometry_model),
      measurement_std_xy_(measurement_std_xy),
      measurement_std_theta_(measurement_std_theta),
      measurement_std_v_(measurement_std_v),
      measurement_std_omega_(measurement_std_omega),
      prediction_std_xy_(prediction_std_xy),
      prediction_std_theta_(prediction_std_theta),
      delay_compensation_(delay_compensation),
      use_target_vis_tracking_(use_target_vis_tracking) {
  // // initialize particles and filter
  // mrpt::bayes::CParticleFilter::TParticleFilterOptions pf_options;
  // pf_options.adaptiveSampleSize = false;
  // pf_options.PF_algorithm = mrpt::bayes::CParticleFilter::pfStandardProposal;
  // pf_options.resamplingMethod = mrpt::bayes::CParticleFilter::prSystematic;
  // particles_.initializeParticles(num_particles, 7.0, 0.0, 0.5, 0.0, 0.0);
  // particles_.setEgoPose(cv::Point2d(0.0, 0.0), 0.0);
  // particles_.updateAction(0.0, 0.0);
  // pf_.m_options = pf_options;

    tracker_frame_ = 0;

    match_fail_count_ = 0;
    left_rearlight_fail_count_ = right_rearlight_fail_count_ = 0;

    // initialize historical radar point
    last_radar_pt_.x = last_radar_pt_.y = 0.0;

    // initialize front vh bbox
    vh_bbox_.width = 0.0;
    vh_bbox_.height = 0.0;

  init_target_yaw_ = 0.0;

  left_valid_ = right_valid_ = true;

    direction_left_rearlight_ = 0.0;
    direction_right_rearlight_ = 0.0;
    direction_vh_ = 0.0;

    // initialize v2v msg
    front_vh_speed_ = 0.0;
    front_vh_angular_velo_ = 0.0;

  ekf_.Reset();
  ekf_.timeUpdate(ros::Time::now().toSec());

  // parameters for ablation study
  use_rearlight_all_ = true;
  use_rearlight_left_ = true;
  use_rearlight_right_ = true;
  use_radar_ = true;
  use_v2v_ = true;
}

void VehicleTracking::predictUpdate(
    const std::unordered_map<part_based_tracking::VehiclePartType, cv::Rect2d>
        &boxes,
    const double timestamp) {
  // // ROS_INFO("image prediction and update");
  // // mrpt::obs::CSensoryFrame sf;
  // // mrpt::obs::CObservationBearingRangePrecise::Ptr obs_br_precise =
  // //     mrpt::obs::CObservationBearingRangePrecise::Create();
  // // int n = boxes.size();
  // // if (n == 0)
  // //   return;
  // // if (ros::Time::now().toSec() > stop_update_time_)
  // //   return;

  // // obs_br_precise->sensedData.resize(n);
  // // int obs_cnt = 0;
  // // for (auto it = boxes.begin(); it != boxes.end(); ++it, ++obs_cnt) {
  // //   auto center_u = it->second.x + it->second.width / 2;
  // //   auto center_v = it->second.y + it->second.height / 2;
  // //   // TODO: calc x, y in cam frame
  // //   cv::Point2d center_pt{center_u, center_v};
  // //   std::vector<cv::Point2d> distorted_pts{center_pt};
  // //   std::vector<cv::Point2d> undistorted_pts;
  // //   unprojectTo3D(distorted_pts, undistorted_pts);
  // //   auto x = undistorted_pts[0].x;
  // //   obs_br_precise->sensedData[obs_cnt].range = 0.0;
  // //   obs_br_precise->sensedData[obs_cnt].yaw =
  // //       mrpt::math::wrapToPi(std::atan2(-x, 1.0));
  // //   obs_br_precise->sensedData[obs_cnt].landmarkID =
  // //       static_cast<int>(it->first);
  // //   obs_br_precise->sensedData[obs_cnt].timestamp = timestamp;

  // //   // update trakcer record of rearlight direction, for visualization
  // //   if (it->first == part_based_tracking::VehiclePartType::LEFTREARLIGHT) {
  // //     direction_left_rearlight_ =
  // //         obs_br_precise->sensedData[obs_cnt].yaw + cam_external_.yaw;
  // //   }
  // //   if (it->first == part_based_tracking::VehiclePartType::RIGHTREARLIGHT) {
  // //     direction_right_rearlight_ =
  // //         obs_br_precise->sensedData[obs_cnt].yaw + cam_external_.yaw;
  // //   }
  // // }
  // // // // rear center proposal
  // // // obs_br_precise->sensedData.back().range =
  // // //     cv::norm(front_vh_rear_center_proposal_);
  // // // obs_br_precise->sensedData.back().yaw = mrpt::math::wrapToPi(std::atan2(
  // // //     front_vh_rear_center_proposal_.y,
  //
  // front_vh_rear_center_proposal_.x));
  // // // obs_br_precise->sensedData.back().landmarkID =
  // // //     static_cast<int>(part_based_tracking::VehiclePartType::REARCENTER);
  // // // obs_br_precise->sensedData.back().timestamp = timestamp;

  // // //
  // // sf.insert(obs_br_precise);
  // // // pf_.executeOn(particles_, nullptr, &sf);
  // // // return;

  // // ROS_INFO("image pu A");

  // // particles_buffer_.push_back(particles_);
  // // if (particles_buffer_.size() > 50)
  // //   particles_buffer_.pop_front();

    // if (particles_buffer_.size() == 0) {
    //   pf_.executeOn(particles_, nullptr, &sf);
    //   particles_buffer_.push_back(particles_);
    //   if (particles_buffer_.size() > 50)
    //     particles_buffer_.pop_front();
    //   ROS_INFO("image pu done");
    //   return;
    // } else {
    //   std::vector<double> timestamp_err;
    //   for (const auto &p : particles_buffer_) {
    //     auto t = p.getTimestamp();
    //     timestamp_err.emplace_back(std::abs(t - timestamp));
    //   }
    //   int start = 0;
    //   double min_t_err = 1.0e6;
    //   for (int i = 0; i < particles_buffer_.size(); ++i) {
    //     if (timestamp_err[i] < min_t_err) {
    //       start = i;
    //       min_t_err = timestamp_err[i];
    //     }
    //   }

    //   ROS_INFO("image pu B");
    //   bool no_delay = (start == particles_buffer_.size() - 1) &&
    //                   timestamp > particles_buffer_.back().getTimestamp();
    //   if (no_delay) {
    //     ROS_INFO("image pu no delay");
    //     pf_.executeOn(particles_, nullptr, &sf);
    //     particles_buffer_.push_back(particles_);
    //     if (particles_buffer_.size() > 50)
    //       particles_buffer_.pop_front();
    //   } else {
    //     ROS_INFO("image pu delay");
    //     // update the best match step, no prediction
    //     std::cout << "ori obs t: " << std::to_string(timestamp)
    //               << " selected timestamp: "
    //               << std::to_string(particles_buffer_[start].getTimestamp())
    //               << std::endl;
    //     for (int i = 0; i < obs_br_precise->sensedData.size(); ++i) {
    //       ROS_INFO("image pu delay change timestamp");
    //       obs_br_precise->sensedData[i].timestamp =
    //           particles_buffer_[start].getTimestamp();
    //     }
    //     ROS_INFO("image pu delay executeOn");
    //     pf_.executeOn(particles_buffer_[start], nullptr, &sf);
    //     // predict next steps
    //     ROS_INFO("image pu delay update");
    //     ++start;
    //     while (start < particles_buffer_.size()) {
    //       particles_buffer_[start].predictFromParticle(
    //           particles_buffer_[start - 1]);
    //       ++start;
    //     }
    //     particles_.predictFromParticle(particles_buffer_.back());
    //   }
    // }
    // ROS_INFO("image pu done");
  }

void VehicleTracking::predictUpdate(
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &radar,
    const double timestamp) {
  // // ROS_INFO("radar prediction and update");
  // // if (vh_bbox_.width < EPS || vh_bbox_.height < EPS)
  // //   return;
  // // ROS_INFO("radar prediction and update match start");
  // // cv::Point2d radar_pt;
  // // matchRadarPoint(vh_bbox_, radar, radar_pt);
  // // ROS_INFO("radar prediction and update match done");

  // // // if not init, initialize particles according to radar pt
  // // ROS_INFO("tracker init: %d", tracker_init_);
  // // // std::cout << "radar pt: " << radar_pt << std::endl;
  // // if (!tracker_init_ && radar_pt.x > 0) {
  // //   particles_buffer_.clear();
  // //   particles_.initializeParticles(num_particles_, radar_pt.x, radar_pt.y,
  //
  //   0.5,
  // //                                  0.0, 0.0);
  // //   tracker_init_ = true;
  // //   ROS_WARN("re-initialize particles!");
  // // }
  // // // return;

  // // if (ros::Time::now().toSec() > stop_update_time_)
  // //   return;

  // // mrpt::obs::CSensoryFrame sf;
  // // mrpt::obs::CObservationBearingRangePrecise::Ptr obs_br_precise =
  // //     mrpt::obs::CObservationBearingRangePrecise::Create();
  // // obs_br_precise->sensedData.resize(2);
  // // obs_br_precise->sensedData[0].range = cv::norm(radar_pt);
  // // obs_br_precise->sensedData[0].yaw =
  // //     mrpt::math::wrapToPi(std::atan2(radar_pt.y, radar_pt.x));
  // // obs_br_precise->sensedData[0].landmarkID =
  // //     static_cast<int>(part_based_tracking::VehiclePartType::REAR);
  // // obs_br_precise->sensedData[0].timestamp = timestamp;

  // // // rear center proposal
  // // obs_br_precise->sensedData.back().range =
  // //     cv::norm(front_vh_rear_center_proposal_);
  // // obs_br_precise->sensedData.back().yaw = mrpt::math::wrapToPi(std::atan2(
  // //     front_vh_rear_center_proposal_.y, front_vh_rear_center_proposal_.x));
  // // obs_br_precise->sensedData.back().landmarkID =
  // //     static_cast<int>(part_based_tracking::VehiclePartType::REARCENTER);
  // // obs_br_precise->sensedData.back().timestamp = timestamp;

  // // sf.insert(obs_br_precise);
  // // pf_.executeOn(particles_, nullptr, &sf);
  // // return;

  // // particles_buffer_.push_back(particles_);
  // // if (particles_buffer_.size() > 50)
  // //   particles_buffer_.pop_front();

    // if (particles_buffer_.size() == 0) {
    //   pf_.executeOn(particles_, nullptr, &sf);
    //   particles_buffer_.push_back(particles_);
    //   if (particles_buffer_.size() > 50)
    //     particles_buffer_.pop_front();
    //   ROS_INFO("radar pu done");
    //   return;
    // } else {
    //   std::vector<double> timestamp_err;
    //   for (const auto &p : particles_buffer_) {
    //     auto t = p.getTimestamp();
    //     timestamp_err.emplace_back(std::abs(t - timestamp));
    //   }
    //   int start = 0;
    //   double min_t_err = 1.0e6;
    //   for (int i = 0; i < particles_buffer_.size(); ++i) {
    //     if (timestamp_err[i] < min_t_err) {
    //       start = i;
    //       min_t_err = timestamp_err[i];
    //     }
    //   }

    //   ROS_INFO("radar pu B");
    //   bool no_delay = (start == particles_buffer_.size() - 1) &&
    //                   timestamp > particles_buffer_.back().getTimestamp();
    //   if (no_delay) {
    //     ROS_INFO("radar pu no delay");
    //     pf_.executeOn(particles_, nullptr, &sf);
    //     particles_buffer_.push_back(particles_);
    //     if (particles_buffer_.size() > 50)
    //       particles_buffer_.pop_front();
    //   } else {
    //     ROS_INFO("radar pu delay");
    //     // update the best match step, no prediction
    //     std::cout << "ori obs t: " << std::to_string(timestamp)
    //               << " selected timestamp: "
    //               << std::to_string(particles_buffer_[start].getTimestamp())
    //               << std::endl;
    //     for (int i = 0; i < obs_br_precise->sensedData.size(); ++i) {
    //       ROS_INFO("radar pu delay change timestamp");
    //       obs_br_precise->sensedData[i].timestamp =
    //           particles_buffer_[start].getTimestamp();
    //     }
    //     ROS_INFO("radar pu delay executeOn");
    //     pf_.executeOn(particles_buffer_[start], nullptr, &sf);
    //     // predict next steps
    //     ROS_INFO("radar pu delay update");
    //     ++start;
    //     while (start < particles_buffer_.size()) {
    //       particles_buffer_[start].predictFromParticle(
    //           particles_buffer_[start - 1]);
    //       ++start;
    //     }
    //     particles_.predictFromParticle(particles_buffer_.back());
    //   }
    // }
    // ROS_INFO("radar pu done");
  }

void VehicleTracking::matchRadarPoint(
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &radar, cv::Point2d &match,
    const cv::Point2d &sugg) {
  double x_low, x_high, y_low, y_high;
  if (sugg.x > 0.0) {
    x_low = 3.55;
    x_high = sugg.x + 1.0;
    y_low = sugg.y - 2.5;
    y_high = sugg.y + 2.5;
  } else {
    x_low = 3.55;
    x_high = 15.0;
    y_low = -10.0;
    y_high = 10.0;
  }

    // filter points out of ROI
    ROS_INFO("pass through x");
    pcl::PassThrough<pcl::PointXYZINormal> pass_through_x;
    pass_through_x.setInputCloud(radar);
    pass_through_x.setFilterFieldName("x");
    pass_through_x.setFilterLimits(x_low, x_high);
    pass_through_x.setFilterLimitsNegative(false);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr filtered_x(
        new pcl::PointCloud<pcl::PointXYZINormal>);
    pass_through_x.filter(*filtered_x);
    ROS_INFO("pass through x done");

    ROS_INFO("pass through y");
    pcl::PassThrough<pcl::PointXYZINormal> pass_through_y;
    pass_through_x.setInputCloud(filtered_x);
    pass_through_x.setFilterFieldName("y");
    pass_through_x.setFilterLimits(y_low, y_high);
    pass_through_x.setFilterLimitsNegative(false);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr filtered_xy(
        new pcl::PointCloud<pcl::PointXYZINormal>);
    pass_through_x.filter(*filtered_xy);
    ROS_INFO("pass through y done");

  cv::Point2d vh_bbox_center{vh_bbox_.x + vh_bbox_.width / 2,
                             vh_bbox_.y + vh_bbox_.height / 2};
  std::vector<cv::Point2d> distorted_pts{vh_bbox_center};
  std::vector<cv::Point2d> undistorted_pts;
  unprojectTo3D(distorted_pts, undistorted_pts);

    auto vh_direction =
        wrapToPi(std::atan2(-undistorted_pts[0].x, 1.0) + cam_external_.yaw);
    // update vh direction, for visualization
    direction_vh_ = vh_direction;

    // std::cout << "after filter, size: " << (*filtered_xy).size() << std::endl;
    std::vector<std::pair<cv::Point2d, double>> candidates;
    // check with state
    cv::Point2d curr_pos_local;
    if (tracker_frame_ > 50)
    {
      auto X = ekf_.readX(ros::Time::now().toSec(), false);
      geometry_msgs::Pose2D curr_pos_global;
      geometry_msgs::Pose2D rear_center_pos_global;
      curr_pos_global.x = X(0);
      curr_pos_global.y = X(1);
      curr_pos_global.theta = X(2);
      calcPartPosition(curr_pos_global, part_based_tracking::VehiclePartType::REARCENTER,
                       rear_center_pos_global);
      cv::Point2d rear_center_pos_global_cv_point{rear_center_pos_global.x,
                                                  rear_center_pos_global.y};
      transformGlobal2Local(rear_center_pos_global_cv_point, curr_pos_local);
    }
    for (const auto &pt : (*filtered_xy))
    {
      if (pt.intensity < -5.0)
        continue;
      // check with velocity
      auto estimated_velocity_x = ego_velocity_ + pt.normal_x;
      auto estimated_velocity_y =
          ego_angular_velo_ * radar_external_.x + pt.normal_y;
      auto estimated_speed = std::sqrt(std::pow(estimated_velocity_x, 2) +
                                       std::pow(estimated_velocity_y, 2));
      if (std::abs(estimated_speed - ego_velocity_) > 1.5)
        continue;

      auto pt_direction =
          wrapToPi(std::atan2(pt.y - cam_external_.y, pt.x - cam_external_.x) +
                   radar_external_.yaw);
      auto direction_err = std::abs(wrapToPi(vh_direction - pt_direction));

    if (tracker_frame_ < 50) {
      if (direction_err > 10.0 * M_PI / 180.0) {
        continue;
      }
    } else if (rearlight_valid_) {
      if (pt_direction > direction_left_rearlight_ + 10.0 / 180.0 * M_PI ||
          pt_direction < direction_right_rearlight_ - 10.0 / 180.0 * M_PI) {
        continue;
      }
    }
    // else if (vh_bbox_valid_) {
    //   if (pt_direction >
    //           direction_pseudo_left_rearlight_ + 10.0 / 180.0 * M_PI ||
    //       pt_direction <
    //           direction_pseudo_right_rearight_ - 10.0 / 180.0 * M_PI) {
    //     continue;
    //   }
    // }

      cv::Point2d pt_2d = {pt.x, pt.y};
      double dis;
      if (tracker_frame_ > 50)
      {
        dis = cv::norm(pt_2d - curr_pos_local);
      }
      else
      {
        dis = cv::norm(pt_2d);
      }
      candidates.emplace_back(std::make_pair(pt_2d, dis));
      // std::cout << "candidate radar pt: " << pt_2d << std::endl;
    }

    // std::cout << "candidate size: " << candidates.size() << std::endl;
    if (candidates.size() == 0)
    {
      if (last_radar_pt_.x > 0)
      {
        match = last_radar_pt_;
      }
      else
      {
        match.x = -1.0;
        match.y = -1.0;
      }
      return;
    }

    auto candidate_cmp = [](const std::pair<cv::Point2d, double> &a,
                            const std::pair<cv::Point2d, double> &b)
    {
      return a.second <= b.second;
    };
    std::sort(candidates.begin(), candidates.end(), candidate_cmp);

    match.x = candidates.front().first.x;
    match.y = candidates.front().first.y;

    // std::cout << "match radar pt: " << match << std::endl;

    // last radar pt not initialized
    if (cv::norm(last_radar_pt_) < EPS)
    {
      last_radar_pt_ = match;
      return;
    }

    std::vector<std::pair<cv::Point2d, double>> candidates_1;
    for (int i = 0; i < std::min(2, static_cast<int>(candidates.size())); ++i)
    {
      candidates_1.emplace_back(
          std::make_pair(candidates[i].first, cv::norm(candidates[i].first)));
    }
    std::sort(candidates_1.begin(), candidates_1.end(), candidate_cmp);
    match = candidates_1.front().first;

    auto last_curr_dis = cv::norm(curr_pos_local - match);
    if (last_curr_dis > 1.0 && match_fail_count_ <= 10)
    {
      match = last_radar_pt_;
      ++match_fail_count_;
    }
    else
    {
      last_radar_pt_ = match;
      match_fail_count_ = 0;
    }
    // std::cout << "final match radar pt: " << match << std::endl;
  }

} // namespace tracking

int main(int argc, char **argv)
{
  ros::init(argc, argv, "part_based_tracking");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");

  tracking::VehicleTrackingWrapper tracker(node_handle, private_node_handle);
  ros::MultiThreadedSpinner spinner(0);
  spinner.spin();
}
