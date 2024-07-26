#include "pure_pursuit.h"

PurePursuit::PurePursuit() {
  //------成员变量初始化-------------
  path_flag = false;
  current_point_global = 0;
  former_steer_cmd = 0;
  former_reference_distance = 10;
  target_path_global.poses.clear();
  inter_steer_error = 0;
  // stanley_cmd = 0;
  for (int i = 0; i < STEER_ERROR_QUEUE_SIZE; i++) steer_error_queue.push(0);
  //------配置动态更改参数服务-------
  cb = boost::bind(&PurePursuit::configCallback, this, _1, _2);
  dr_srv.setCallback(cb);

  //------参数初始化-----------------
  ros::NodeHandle pnh("~");  //定义私有节点句柄，用于传递参数
  pnh.param("Min_ref_speed_", Min_ref_speed, 12.5);
  pnh.param("Min_preview_distance_", Min_preview_distance, 8.5);
  pnh.param("K_ref_0_20_", K_ref_0_20, 1.0);
  pnh.param("K_ref_20_40_", K_ref_20_40, 0.5);
  pnh.param("K_ref_40_60_", K_ref_40_60, 0.2);
  pnh.param("Kp_error_", Kp_error, 0.2);
  pnh.param("Ki_error_", Ki_error, 0.2);
  pnh.param("Kp_wheel_", Kp_wheel, 945.38);  // 方向盘到16.5 * 180/pi

  //初始化特殊场景下的预描距离
  // pnh.param("pure_pursuit/Default_preview_dis", default_preview_dis_, 8.0);

  pnh.param("filter_param_", filter_param, 0.7);
  pnh.param("wheelbase_", wheelbase, 2.56);
  pnh.param("max_steering_degree", wheel_max, 430.0);  // 方向盘最大角度，单位度
  pnh.param("wheel_zero_", wheel_zero,
            0.0);  //方向盘零位，向左为正，向右为负，单位度

  std::cout << "Min_ref_speed : " << Min_ref_speed << std::endl;
  std::cout << "wheel_max : " << wheel_max << std::endl;

  //------配置订阅和发布的话题-----------
  pub_steer_cmd =
      nh.advertise<cyber_msgs::steercmd>("/rock_can/steer_command", 10);
  pub_reference_pose =
      nh.advertise<geometry_msgs::PoseStamped>("reference_pose", 1);
  pub_target_pose = nh.advertise<geometry_msgs::PoseStamped>("target_pose", 1);
  pub_control_target_point_ = nh.advertise<visualization_msgs::MarkerArray>(
      "/control/preview_point", 1);

  sub_local_trajectory = nh.subscribe(
      "/control/local_trajectory", 5, &PurePursuit::localTrajectoryCallback, this);
  sub_pose = nh.subscribe("/localization/estimation", 1,
                          &PurePursuit::poseCallback, this);
  sub_steer = nh.subscribe("/rock_can/steer_feedback", 5,
                           &PurePursuit::SteerCallback, this);

  printf("%sNode pure_pursuit init done!\n", "\x1B[37m");
  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();
}

PurePursuit::~PurePursuit() {
  printf("\n%sNode pure_pursuit shut down!\n", "\x1B[31m");
  printf("%s", "\x1B[0m");
}

void PurePursuit::localTrajectoryCallback(
    const cyber_msgs::LocalTrajList::ConstPtr &path_in) {

  if (path_in->points.size() == 0) {
    target_path_global.poses.clear();
    current_point_global = 0;
    path_flag = false;
    ROS_WARN("Clear target path now!!!");
    return;
  }

  geometry_msgs::PoseArray target_path;
  target_path.header.stamp = ros::Time::now();
  target_path.header.frame_id = "world";

  geometry_msgs::Pose target_point_;
  for (auto iter = path_in->points.begin(); iter != path_in->points.end();
       iter++) {
    target_point_.position = iter->position;
    target_point_.orientation = iter->orientation;
    target_path.poses.push_back(target_point_);
  }

  path_flag = true;
  current_point_global = 0;
  target_path_global = target_path;
}

void PurePursuit::poseCallback(
    const cyber_msgs::LocalizationEstimate::ConstPtr &pose_in) {
  if (!path_flag) {
    steer_cmd = 0;
    former_steer_cmd = 0;
    former_reference_distance = 0;
    ROS_WARN_THROTTLE(1,"Wait for valid targets!");
    return;
  }

  geometry_msgs::PoseArray target_path;
  int current_point;
  target_path = target_path_global;
  current_point = current_point_global;  //使用局部变量避免冲突调用

  current_pose.pose = pose_in->pose;
  current_yaw = tf::getYaw(pose_in->pose.orientation);
  current_vel = pose_in->velocity.linear.x;

  //计算当前位置对应点
  double temp_dis = sqrt(pow(current_pose.pose.position.y -
                                 target_path.poses[current_point].position.y,
                             2) +
                         pow(current_pose.pose.position.x -
                                 target_path.poses[current_point].position.x,
                             2));
  for (int i = current_point + 1; i < target_path.poses.size(); i++) {
    double next_dis = sqrt(
        pow(current_pose.pose.position.y - target_path.poses[i].position.y, 2) +
        pow(current_pose.pose.position.x - target_path.poses[i].position.x, 2));
    if (next_dis <= temp_dis) {
      current_point = i;
      temp_dis = next_dis;
    } else if (i > current_point + 20)  //对比至少20个点避免初始轨迹不稳定
      break;
  }

  //计算控制参考距离
  double reference_distance;
  //路段预瞄距离根据速度调整
  double current_vel_kmh = current_vel * 3.6;
  if (current_vel_kmh <= Min_ref_speed)
    reference_distance =
        Min_preview_distance + (K_ref_0_20 * Min_ref_speed - Min_preview_distance) /
                                (Min_ref_speed + 0.00001) * current_vel_kmh;
  else if (current_vel_kmh <= 20)
    reference_distance = K_ref_0_20 * current_vel_kmh;
  else if (current_vel_kmh <= 40)
    reference_distance = K_ref_0_20 * 20 + K_ref_20_40 * (current_vel_kmh - 20);
  else if (current_vel_kmh <= 60)
    reference_distance = K_ref_0_20 * 20 + K_ref_20_40 * 20 +
                         K_ref_40_60 * (current_vel_kmh - 40);
  else
    reference_distance = K_ref_0_20 * 20 + K_ref_20_40 * 20 + K_ref_40_60 * 20;

  reference_distance =
      std::max(reference_distance, former_reference_distance - 0.25);

  reference_pose = current_pose;
  //判断使用定位点还是最近点作为参考位置
  if (sqrt(pow(current_pose.pose.position.y -
                   target_path.poses[current_point].position.y,
               2) +
           pow(current_pose.pose.position.x -
                   target_path.poses[current_point].position.x,
               2)) > reference_distance / 10)
    reference_pose.pose = target_path.poses[current_point];
  pub_reference_pose.publish(reference_pose);

  //计算控制目标对应点
  int target_point = current_point;

  for (int i = target_point + 1; i < target_path.poses.size(); i++) {
    double next_dis = std::hypot(
        reference_pose.pose.position.x - target_path.poses[i].position.x,
        reference_pose.pose.position.y - target_path.poses[i].position.y);
    if (next_dis <= reference_distance) {
      target_point = i;
    } else
      break;
  }

  target_pose.header = current_pose.header;
  target_pose.pose = target_path.poses[target_point];
  pub_target_pose.publish(target_pose);
  ShowTargetPoint(target_pose);

  //利用纯跟踪算法计算前轮转角
  double delta_x = target_pose.pose.position.x - current_pose.pose.position.x;
  double delta_y = target_pose.pose.position.y - current_pose.pose.position.y;
  double L = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
  double alpha = atan2(delta_y, delta_x) - current_yaw;  //向左为正
  if (alpha < -M_PI) alpha += 2 * M_PI;
  if (alpha > M_PI) alpha -= 2 * M_PI;
  double R_cmd;
  if (current_point ==
      target_path.poses.size() - 1)  //路径结束, 或者定位失效时保持直行
  {
    R_cmd = 0;
    steer_cmd = 0;
    former_reference_distance = 5;
    path_flag=false;
  } else if (std::fabs(L) < 1e-3 || std::fabs(alpha)<1e-3) {
    R_cmd = 0;
    steer_cmd = 0;
  } else {
    R_cmd = L / (2 * sin(alpha));         //向左为正
    steer_cmd = atan(wheelbase / R_cmd);  //向左为正，WHEELBASE为前后轴距
  }

  steer_cmd =
      filter_param * steer_cmd +
      (1 - filter_param) * former_steer_cmd;  //对所求得的转向控制角进行低通滤波

  former_steer_cmd = steer_cmd;
  former_reference_distance = reference_distance;
  current_point_global = current_point;

  // ROS_INFO(
  //     "Pure Pursuit:\ncurrent_vel_kmh: %f\treference_distance: %f\nL: %f\ta: "
  //     "%f\tsteer_cmd: %f\n",
  //     current_vel_kmh, reference_distance, L, alpha, steer_cmd);
}

void PurePursuit::SteerCallback(
    const cyber_msgs::SteerFeedbackConstPtr &steer_in) {
  if(!path_flag){
    cyber_msgs::steercmd wheel_output;
    wheel_output.enable_auto_steer = true;
    wheel_output.is_updated = true;
    wheel_output.steer_cmd = 0;
    pub_steer_cmd.publish(wheel_output);
    inter_steer_error = 0;
    steer_error_queue.push(0);
    steer_error_queue.pop();
    return;
  }
  steer_feedback = steer_in->SteerAngle / Kp_wheel;
  steer_error = steer_cmd - steer_feedback;

  inter_steer_error =
      inter_steer_error + steer_error -
      steer_error_queue.front();  //计算一段时间内前轮转角误差的积分值
  steer_error_queue.push(steer_error);
  steer_error_queue.pop();

  //将纯跟踪算法的计算值叠加上转向误差PI控制;
  final_steer_cmd = steer_cmd + Kp_error * steer_error +
                    Ki_error * inter_steer_error / STEER_ERROR_QUEUE_SIZE;

  if (final_steer_cmd > wheel_max / Kp_wheel)
    final_steer_cmd = wheel_max / Kp_wheel;
  if (final_steer_cmd < -wheel_max / Kp_wheel)
    final_steer_cmd = -wheel_max / Kp_wheel;  //限幅

  cyber_msgs::steercmd wheel_output;
  wheel_output.enable_auto_steer = true;
  wheel_output.is_updated = true;
  wheel_output.steer_cmd = Kp_wheel * final_steer_cmd + wheel_zero;  //向左为正

  ROS_INFO(
      "Pure Pursuit Control Loop:\nsteer_cmd: %f\nsteer_error: "
      "%f\tinter_steer_error: "
      "%f\nfinal_steer_cmd: %f\twheel_output: %f\n",
      steer_cmd, steer_error, inter_steer_error/STEER_ERROR_QUEUE_SIZE, final_steer_cmd,
      wheel_output.steer_cmd);
  // std::cout << "final_steer_cmd : " << wheel_output.steer_cmd << std::endl;

  pub_steer_cmd.publish(wheel_output);
}

void PurePursuit::configCallback(steer_controller::pursuit_paramConfig &config,
                                 uint32_t level) {
  if (config.update_param_) {
    Min_preview_distance = config.Min_preview_distance_;
    Min_ref_speed = config.Min_ref_speed_;
    K_ref_0_20 = config.K_ref_0_20_;
    K_ref_20_40 = config.K_ref_20_40_;
    K_ref_40_60 = config.K_ref_40_60_;
    Kp_error = config.Kp_error_;
    Ki_error = config.Ki_error_;
    filter_param = config.filter_param_;
    Kp_wheel = config.Kp_wheel_;
    std::cout << "----------reconfigure----------"<<std::endl;
    std::cout << "Kp_error\t" << Kp_error << std::endl;
    std::cout << "Ki_error\t" << Ki_error << std::endl;
    std::cout << "filter_param\t" << filter_param << std::endl;
    std::cout << "Kp_wheel\t" << Kp_wheel << std::endl;
    config.update_param_ = false;
  }
}

void PurePursuit::ShowTargetPoint(
    const geometry_msgs::PoseStamped in_target_point) {
  visualization_msgs::MarkerArray points;
  int point_id = 0;
  visualization_msgs::Marker point;
  point.header.stamp = ros::Time::now();
  point.header.frame_id = "map";
  point.scale.x = 0.3;
  point.scale.y = 0.3;
  point.scale.z = 2.0;
  point.action = visualization_msgs::Marker::ADD;
  point.type = visualization_msgs::Marker::CYLINDER;
  point.lifetime = ros::Duration(0.2);

  point.pose.position.x = in_target_point.pose.position.x;
  point.pose.position.y = in_target_point.pose.position.y;
  point.pose.position.z = 1;
  point.color.b = 0;
  point.color.g = 1;
  point.color.r = 0;
  point.color.a = 1;
  points.markers.emplace_back(point);
  pub_control_target_point_.publish(points);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pure_pursuit");

  PurePursuit PurePursuit_obj;

  return 0;
}
