/**
  *control.cpp
  *brief: vehicle speed and steer control
  *author:Jianlin Zhang
  *date:20180803
  **/

#include "control.h"

namespace Control
{
const double M_PI_180=0.01745329252;  //M_PI/180
const double very_little=1e-3;
double a_,b_,c_,d_,pre_err;

double fun(double x){
  return a_+b_*x+c_*x*x+d_*x*x*x;
}

Control::Control(ros::NodeHandle nh, ros::NodeHandle pnh)
  :speed_k(0.5),
  speed_target(0),
  behind_speed(0),
  x_vehicle(0),
  y_vehicle(0),
  theta_vehicle(0),
  speed_vehicle(0)
{
  pnh.param("wheelbase", wheelbase, 2.56);
  pnh.param("map_frame_id", map_frame_id, std::string("map"));
  pnh.param("preview_distance_base", preview_distance_base, 3.55);//
  pnh.param("preview_time", preview_time, 1.8);//
  pnh.param("min_preview_distance", min_preview_distance, 8.5);
  pnh.param("max_preview_distance", max_preview_distance, 10.5);
  pnh.param("min_preview_distance_for_search", min_preview_distance_for_search, 3.5);
  pnh.param("heading_cost_factor", heading_cost_factor, 0.8);
  pnh.param("speed_cmd_max", speed_cmd_max, 2.0);
  pnh.param<double>("goal_x_set", goal_distance, 6);
  pnh.param("k_front_distance", k_front_distance, 1.0);
  pnh.param("k_behind_distance", k_behind_distance, 0.8);
  pnh.param("c_front_velocity", c_front_velocity, 0.8);
  pnh.param("c_behind_velocity", c_behind_velocity, 0.6);
  pnh.param("k_speed_output", k_speed_output, 200.0);
  pnh.param("k_brake_output", k_brake_output, 2.5);
  pnh.param("zero_brake_acc_threshold", zero_brake_acc_threshold, -0.5);//
  pnh.param("zero_brake_speed_threshold", zero_brake_speed_threshold, 3.0);//
  pnh.param("k_feedforward_acc", k_feedforward_acc, 1.0);//
  pnh.param("k_acc_threshold", k_acc_threshold, 0.0);//
}

double Control::compute_steer(const std::list<TrackPoint>::const_iterator preview_pt) {
  double delta_y = preview_pt->y_ - y_vehicle;
  double delta_x = preview_pt->x_ - x_vehicle;
  real_distance_preview = sqrt(delta_x*delta_x + delta_y*delta_y);
  angle_preview = atan2(delta_y,delta_x) - theta_vehicle;
  double steer_output = 0;

  if(std::abs(angle_preview) < very_little)
  {
    steer_output=0;
    radius_preview = 1e3;
    // ROS_ERROR("FLAG1 debug radius: %f, angle: %f", radius_preview, angle_preview);
  }
  else
  {
    double R = real_distance_preview;
    radius_preview = R / (2 * sin(angle_preview));
    steer_output = atan2(wheelbase,radius_preview);
    if(steer_output >= M_PI/2) steer_output -= M_PI;
    else if(steer_output <= -M_PI/2) steer_output += M_PI;
    // ROS_ERROR("FLAG2 debug radius: %f, angle: %f", radius_preview, angle_preview);
  }

  return steer_output;
}

void Control::compute_steer_cmd_by_search(const std::list<TrackPoint> &map, double front_average_curvature) {
  double steer_output = 0;
  if(map.size()==0)
  {
    ROS_ERROR("No front track!!!");
    real_distance_preview=0;
    angle_preview=0;
    radius_preview=0;
    steer_output=0;
  }
  else if(front_average_curvature < straight_threshold || self_speed > 3.5 || true) // 直道或高速使用纯跟踪
  {
    lateral_type = 0;
    ROS_INFO("lateral control using pure pursuit");
    if(self_speed * 3.6 > 10) preview_distance = self_speed * preview_time + preview_distance_base;
    else preview_distance = 10 / 3.6 * preview_time + preview_distance_base;

    if(preview_distance < min_preview_distance) preview_distance=min_preview_distance;
    else if(preview_distance > max_preview_distance) preview_distance=max_preview_distance;

    std::list<TrackPoint>::const_iterator preview_pt = map.begin();
    while(preview_pt != map.end()) {
      double distance2=(x_vehicle-preview_pt->x_)*(x_vehicle-preview_pt->x_)+(y_vehicle-preview_pt->y_)*(y_vehicle-preview_pt->y_);
      if(distance2>preview_distance*preview_distance) {
        steer_output = compute_steer(preview_pt);
        break;
      }
      ++preview_pt;
    }
    // 一直都没找到就取最后一个
    if(preview_pt == map.end()) {
      --preview_pt;
      steer_output = compute_steer(preview_pt);
    }

    x_preview=preview_pt->x_;
    y_preview=preview_pt->y_;
    preview_point = *preview_pt;
  }
  else // 弯道使用搜索算法
  {
    lateral_type = 1;
    ROS_INFO("lateral control using search");
    // 选取预瞄点并按纯跟踪算法模型先计算大致转角
    std::list<TrackPoint>::const_iterator preview_pt = map.begin();
    while(preview_pt != map.end()) {
      double distance2=(x_vehicle-preview_pt->x_)*(x_vehicle-preview_pt->x_)+(y_vehicle-preview_pt->y_)*(y_vehicle-preview_pt->y_);
      // 设计最低距离预瞄点，防止太近振荡
      if(distance2>min_preview_distance_for_search*min_preview_distance_for_search) {
        // 满足最大转角(28.5角度)以及和当前方向盘转角差90度以内
        steer_output = compute_steer(preview_pt);
        // ROS_WARN("steer: %f, diff: %f", steer_output, fabs(steer_output - steer_vehicle) / M_PI * 180.0 * 16.0);
        if(steer_output < 0.5 && steer_output > -0.5
           && fabs(steer_output - steer_vehicle) / M_PI * 180.0 * 15.5 < 90.0) {
          break;
        }
      }
      ++preview_pt;
    }
    // 一直都没找到就取最后一个
    if(preview_pt == map.end()) --preview_pt;
    x_preview=preview_pt->x_;
    y_preview=preview_pt->y_;
    preview_point = *preview_pt;

    // 计算航向偏差，选取初始搜索范围
    double direct_theta = preview_pt->theta_ - theta_vehicle;
    if(direct_theta > steer_output + M_PI) direct_theta -= 2 * M_PI;
    else if (direct_theta < steer_output - M_PI) direct_theta += 2 * M_PI;

    // 在范围内采样10个输出，使得优化目标横向距离加航向乘系数最小
    int iter_count = 0;
    double steer_min = std::min(direct_theta, steer_output);
    double steer_max = std::max(direct_theta, steer_output);
    // 范围包括转角短时间能打到的范围
    double w_max = 90.0 / 15.5 / 180.0 * M_PI;
    steer_min = std::max(steer_min, steer_vehicle - w_max);
    steer_max = std::min(steer_max, steer_vehicle + w_max);
    double angle_diff = steer_max - steer_min;
    while(fabs(angle_diff) > (M_PI / 180.0) && iter_count < 5 && fabs(steer_min) > very_little && fabs(steer_max) > very_little) {
      const int N = 10;
      std::vector<double> costs(N+1, 0);
      std::vector<double> steers(N+1, 0);
      double cost_min = 1e7;
      int cost_min_index = 0;
      for(int i = 0; i <= N; ++i) {
        steers[i] = steer_min + (i / N) * angle_diff;
        double cost = calc_cost(steers[i]);
        costs[N] = cost;
        if(cost < cost_min) {
          cost_min = cost;
          cost_min_index = i;
        }
      }

      steer_min = steers[cost_min_index];
      steer_output = steer_min;
      if(cost_min_index == 0) {
        steer_max = steers[1];
      }
      else if(cost_min_index == N) {
        steer_max = steers[N - 1];
      }
      else {
        if(costs[cost_min_index - 1] < costs[cost_min_index + 1]) steer_max = steers[cost_min_index - 1];
        else steer_max = steers[cost_min_index + 1];
      }
      angle_diff = steer_max - steer_min;
      iter_count++;
      // ROS_WARN("iter_count: %d, angle_diff: %f", iter_count, angle_diff);
    }
  }

  steer_cmd = steer_output;
  if(steer_cmd>=(M_PI/2)) steer_cmd -= M_PI;
  else if(steer_cmd<=-(M_PI/2)) steer_cmd += M_PI;
  if(map.size()==0) radius_preview = 0;
  else radius_preview = wheelbase / tan(steer_cmd);
}

double Control::calc_cost(double steer_i) {
  double R = wheelbase / tan(steer_i);
  double x_c=x_vehicle-R*sin(theta_vehicle);
  double y_c=y_vehicle+R*cos(theta_vehicle);
  double dis2preview = sqrt((x_c - preview_point.x_) * (x_c - preview_point.x_) + (y_c - preview_point.y_) * (y_c - preview_point.y_));
  double lateral_cost = fabs(dis2preview - fabs(R));
  double future_heading = theta_vehicle + 2 * steer_i;
  if(future_heading > preview_point.theta_ + M_PI) future_heading -= 2 * M_PI;
  else if (future_heading < preview_point.theta_ - M_PI) future_heading += 2 * M_PI;
  double heading_cost = fabs(future_heading - preview_point.theta_);
  return (lateral_cost + heading_cost * heading_cost_factor);
}

void Control::compute_steer_cmd_by_pure_pursuit(const std::list<TrackPoint> &map) {
  if(map.size()==0)
  {
    ROS_ERROR("No front track!!!");
    steer_cmd=0;
  }
  else
  {
    //compute preview distance
    // preview_distance = speed_vehicle * 3.6; //adjust preview distance according to current speed in kmh
    if(std::abs(self_speed)*3.6>10)
    {
      preview_distance=std::abs(self_speed)*1.8;
    }
    else
    {
      preview_distance=5.0;
    }
    // std::cout<<"preview_distance:"<<preview_distance<<std::endl;
    if(preview_distance < min_preview_distance)
    {
      preview_distance=min_preview_distance;
    }
    else if(preview_distance>max_preview_distance)
    {
      preview_distance=max_preview_distance;
    }

    //search preview point on map
    std::list<TrackPoint>::const_iterator preview_pt = map.begin();
    while(preview_pt != map.end()) {
      double distance2=(x_vehicle-preview_pt->x_)*(x_vehicle-preview_pt->x_)+(y_vehicle-preview_pt->y_)*(y_vehicle-preview_pt->y_);
      if(distance2>preview_distance*preview_distance) break;
      ++preview_pt;
    }
    if(preview_pt == map.end()) --preview_pt;
    x_preview=preview_pt->x_;
    y_preview=preview_pt->y_;
    preview_point = *preview_pt;

    //compute steer_cmd by pure pursuit method
    double delta_y = y_preview-y_vehicle;
    double delta_x = x_preview-x_vehicle;
    real_distance_preview = sqrt(delta_x*delta_x + delta_y*delta_y);
    angle_preview = atan2(delta_y,delta_x) - theta_vehicle;

    if(std::abs(angle_preview)<very_little) //go straight
    {
      steer_cmd=0;
    }
    else
    {
      double R=real_distance_preview; //turnning radius used to compute steer_cmd. real_distance
      if(R<min_preview_distance)
      {
        R=min_preview_distance;
      }
      radius_preview=R/(2*sin(angle_preview));
      steer_cmd=atan2(wheelbase,radius_preview);
      if(steer_cmd>=(M_PI/2))
      {
        steer_cmd -= M_PI;
      }
      else if(steer_cmd<=-(M_PI/2))
      {
        steer_cmd += M_PI;
      }
    }
  }
}

void Control::draw_preview_point(ros::Publisher &pt_publisher) {
  visualization_msgs::MarkerArray marker_array_output;
  visualization_msgs::Marker marker_output;
  marker_output.header.frame_id = map_frame_id;
  marker_output.header.stamp = ros::Time::now();
  marker_output.action = visualization_msgs::Marker::ADD;
  marker_output.type = visualization_msgs::Marker::CUBE;
  marker_output.lifetime = ros::Duration(0.1);
  marker_output.color.r = 1.0;
  marker_output.color.g = 1.0;
  marker_output.color.b = 1.0;
  marker_output.color.a = 1.0;
  marker_output.scale.x = 0.25;
  marker_output.scale.y = 0.25;
  marker_output.scale.z = 0.25;

  marker_output.pose.position.x = preview_point.x_;
  marker_output.pose.position.y = preview_point.y_;
  marker_output.pose.position.z = 0;
  marker_output.pose.orientation = tf::createQuaternionMsgFromYaw(preview_point.theta_);
  marker_array_output.markers.emplace_back(marker_output);
  pt_publisher.publish(marker_array_output);
}

double Control::compute_steer_cmd_by_pure_pursuit(const std::list<geometry_msgs::Pose2D> &map,
                                                  const std::list<geometry_msgs::Pose2D>::const_reverse_iterator &goal_index)
//bug here! read origin map, DO NOT copy it,because its iterator was used to clean old points of map!
{

  size_t goal_label=0;
  size_t min_index_label=0;

  //localization on map
  if(map.size()==0)
  {
    min_index=map.crbegin();  //bug here!! it will collaps if min_index was not update in this function
    real_distance_preview=0;
    angle_preview=0;
    radius_preview=0;
    steer_cmd=0;
  }
  else
  {
    {
      auto i=map.crbegin();
      min_index=i;
      double min_dis2=100*100;
      // size_t goal_label=0;
      // size_t min_index_label=0;
      size_t label=0;
      while(i!=map.crend())
      {
        const geometry_msgs::Pose2D &p=*i;
        double delta_dis2=(x_vehicle-p.x)*(x_vehicle-p.x)+(y_vehicle-p.y)*(y_vehicle-p.y);
        if(delta_dis2<min_dis2)
        {
          min_dis2=delta_dis2;
          min_index=i;
          min_index_label=label;
        }
        if(i==goal_index)
        {
          goal_label=label;
        }
        ++i;
        ++label;
      }
      double x_nearest=min_index->x;
      double y_nearest=min_index->y;
    }
    //compute preview distance
    // preview_distance = speed_vehicle * 3.6; //adjust preview distance according to current speed in kmh
    if(std::abs(self_speed)*3.6>10)
    {
      preview_distance=std::abs(self_speed)*1.8;
    }
    else
    {
      preview_distance=5.0;
    }
    // std::cout<<"preview_distance:"<<preview_distance<<std::endl;
    if(preview_distance < min_preview_distance)
    {
      preview_distance=min_preview_distance;
    }
    else if(preview_distance>max_preview_distance)
    {
      preview_distance=max_preview_distance;
    }
    //search preview point on map
    {
      auto i=min_index;
      while(i!=map.crbegin())
      {
        const geometry_msgs::Pose2D &current=*(--i);
        double distance2=(x_vehicle-current.x)*(x_vehicle-current.x)+(y_vehicle-current.y)*(y_vehicle-current.y);
        if(distance2>preview_distance*preview_distance)
        {
          break;
        }
      }
      x_preview=i->x;
      y_preview=i->y;
    }
    //compute steer_cmd by pure pursuit method
    double delta_y = y_preview-y_vehicle;
    double delta_x = x_preview-x_vehicle;
    real_distance_preview = sqrt(delta_x*delta_x + delta_y*delta_y);
    angle_preview = atan2(delta_y,delta_x) - theta_vehicle;

    if(goal_label<min_index_label)
    {
      is_passed=false;
    }
    else if(goal_label == min_index_label)
    {
      if(cos(angle_preview)<0) is_passed=true;
      else is_passed = false;
    }
    else
    {
      is_passed=true;
    }

    if(std::abs(angle_preview)<very_little) //go straight
    {
      steer_cmd=0;
    }
    else
    {
      double R=real_distance_preview; //turnning radius used to compute steer_cmd. real_distance
      if(R<min_preview_distance)
      {
        R=min_preview_distance;
      }
      radius_preview=R/(2*sin(angle_preview));
      steer_cmd=atan2(wheelbase,radius_preview);
      if(steer_cmd>=(M_PI/2))
      {
        steer_cmd -= M_PI;
      }
      else if(steer_cmd<=-(M_PI/2))
      {
        steer_cmd += M_PI;
      }
    }
  }
  return steer_cmd;
}

void Control::compute_speed_cmd(double acc_output, double ego_speed, double speed_output_max,
                                double brake_output_max, double& speed_output, double& brake_output) {
  // speed_output = ego_speed + acc_output * k_speed_output;
  // if(speed_output > speed_output_max) speed_output = speed_output_max;
  // else if(speed_output < 0) speed_output = 0;
  speed_output = acc_output * k_speed_output;
  if (speed_output > speed_output_max) speed_output = speed_output_max;
  else if(speed_output < 0.0) speed_output = 0.0;

  if(acc_output < 0.0 && acc_output > zero_brake_acc_threshold && ego_speed > zero_brake_speed_threshold) brake_output = 0;
  else if(acc_output < 0.0) {
    brake_output = -k_brake_output * acc_output;
  }
  else brake_output = 0;

  if(brake_output < 0) brake_output = 0;
  if(brake_output > brake_output_max) brake_output = brake_output_max;
}

double Control::compute_curvature(const std::list<TrackPoint>::const_iterator first,
                                  const std::list<TrackPoint>::const_iterator second,
                                  const std::list<TrackPoint>::const_iterator third) {
  double curvature = 0;
  if(fabs(first->x_ - second->x_) < very_little && fabs(first->x_ - third->x_) < very_little) {
    return 0;
  }
  else if(fabs(first->y_ - second->y_) < very_little && fabs(first->y_ - third->y_) < very_little) {
    return 0;
  }
  else {
    double dis1,dis2,dis3;
    double cosA,sinA,dis;
    dis1 = sqrt((first->x_ - second->x_)*(first->x_ - second->x_) + (first->y_ - second->y_)*(first->y_ - second->y_));
    dis2 = sqrt((first->x_ - third->x_)*(first->x_ - third->x_) + (first->y_ - third->y_)*(first->y_ - third->y_));
    dis3 = sqrt((second->x_ - third->x_)*(second->x_ - third->x_) + (second->y_ - third->y_)*(second->y_ - third->y_));

    if(dis1 < very_little || dis2 < very_little || dis3 < very_little) return 0;

    dis = dis1*dis1 + dis3*dis3 - dis2*dis2;
    cosA = dis/(2*dis1*dis3);//余弦定理求角度
    if(fabs(cosA) > (1.0- very_little)) return 0;
    sinA = sqrt(1 - cosA*cosA);//求正弦
    if(fabs(sinA) < very_little) curvature = 0;
    else {
      curvature = 0.5*dis2/sinA;//正弦定理求外接圆半径
      curvature = 1.0 / curvature;//半径的倒数是曲率，半径越小曲率越大
    }
    // ROS_ERROR("c: %f, d: %f, sin: %f", curvature, dis2, sinA);
  }
  return curvature;
}

double Control::compute_average_curvature(const std::list<TrackPoint> &map, int val) {
  if(map.size() < 3) return 0;
  int num = map.size() - 2;
  num = num / val;
  if(num <= 0) return 0;
  double curvature_sum = 0;
  std::list<TrackPoint>::const_iterator first = map.begin();
  std::list<TrackPoint>::const_iterator second = first; ++second;
  std::list<TrackPoint>::const_iterator third = second; ++third;
  for(int i = 0; i < num; ++i) {
    curvature_sum += compute_curvature(first, second, third);
    ++first; ++second; ++third;
  }
  return curvature_sum / (double)(num);
}

double Control::compute_average_curvature_by_heading(const std::list<TrackPoint> &map, int val) {
  if(map.size() < 2) return 0;
  int num = (int)(map.size()) - 1;
  num = num / val;
  if(num < 2) return 0;
  double curvature_sum = 0;
  std::list<TrackPoint>::const_iterator first = map.begin();
  std::list<TrackPoint>::const_iterator second = first; ++second;
  for(int i = 0; i < num; ++i) {
    curvature_sum += fabs(first->theta_ - second->theta_);
    ++first; ++second;
  }
  return curvature_sum / (double)(num);
}

double Control::compute_acc_cmd(double front_dis, double front_dis_expect, double front_speed,
            double behind_dis, double behind_dis_expect, double behind_speed, double ego_speed,
            double front_average_curvature, double behind_average_curvature, bool consider_behind) {
  double k_front_curvature = 1.0; //exp(-fabs(front_average_curvature * k_curvature_factor));
  double k_behind_curvature = exp(-fabs(behind_average_curvature * k_curvature_factor));

  // calculate acc_output
  double acc_output = k_front_curvature * k_front_distance*(front_dis - front_dis_expect) + c_front_velocity*(front_speed - ego_speed);
  std::cout << std::setprecision(3) << acc_output << "=" <<k_front_curvature << "*" << k_front_distance << "*(" << front_dis << "-" << front_dis_expect << ")+"
            << c_front_velocity << "*(" << front_speed << "-" << ego_speed << ")" << std::endl;
  if(consider_behind && behind_dis_expect > 0 && behind_dis > 0 && behind_dis < 10) {
    acc_output += k_behind_curvature * k_behind_distance*(behind_dis_expect - behind_dis) + c_behind_velocity*(behind_speed - ego_speed);
    std::cout << std::setprecision(3) << "plus behind: " << k_behind_curvature << "*" << k_behind_distance << "*(" << behind_dis_expect << "-" << behind_dis << ")+"
              << c_behind_velocity << "*(" << behind_speed << "-" << ego_speed << ")" << std::endl;
  }
  std::cout << "--------------------------------------------------------------------------" << std::endl;

  return acc_output;
}

void Control::compute_speed_cmd(const std::list<TrackPoint> &map) {
  if(map.size()==0)
  {
    ROS_ERROR("No front track!!!");
    speed_cmd=0;
  }
  else {
    std::list<TrackPoint>::const_iterator first = map.begin();
    double dx = first->x_ - x_vehicle;
    double dy = first->y_ - y_vehicle;
    double distance = first->curve_distance_ + sqrt(dx * dx + dy * dy);
    //compute speed_cmd
    double dis_error = distance - goal_distance;
    if(dis_error < 0) speed_k = speed_k2;
    else speed_k = speed_k1;

    double speed_add = speed_k * dis_error;
    if(speed_add > 3.0) speed_add = 3.0;
    speed_cmd = speed_target + speed_add;
    std::cout << "cmd:" << speed_cmd << " = " << speed_target << " + " << speed_k << " * " << dis_error << std::endl;
  }
}

double Control::compute_speed_cmd(const std::list<geometry_msgs::Pose2D> &map,
                                  const std::list<geometry_msgs::Pose2D>::const_reverse_iterator &goal_index)
{
  if(map.size()>0)
  {
    //compute speed_cmd
    double dis_error=sqrt((goal_index->x-x_vehicle)*(goal_index->x-x_vehicle)+(goal_index->y-y_vehicle)*(goal_index->y-y_vehicle));
    double feedforward_coe=feedforward_k/(dis_error+feedforward_k);
    if(is_passed)
    {
      dis_error=-dis_error;
      speed_k = speed_k2;
      // ROS_WARN("PASS GOAL!!!");
    }
    else speed_k = speed_k1;

    double speed_add = speed_k * dis_error;
    if(speed_add > 3.0) speed_add = 3.0;
    speed_cmd = speed_target + speed_add;
    std::cout<<"cmd:"<<speed_cmd<<" = "<<speed_target<<" + "<<speed_k<<" * "<<dis_error<<'\t'<<"is_passed:"<<(is_passed?"true":"false")<<std::endl;

    //limit speed_cmd
    if(real_distance_preview>min_preview_distance*sin(angle_preview)) //preview point is out of min turning circle
    {
      double cos_preview=std::abs(cos(angle_preview));
      if(cos_preview<0.1)
      {
        cos_preview=0.1;
      }
      double speed_cmd_limit=speed_cmd_max*cos_preview;
      // std::cout<<"speed_cmd_limit:"<<speed_cmd_limit<<std::endl;
      if(speed_cmd>speed_cmd_limit)
      {
        speed_cmd=speed_cmd_limit;
      }
    }
    else
    {
      speed_cmd=0;
      std::cout<<"arrived in min turning circle, so speed_cmd=0"<<std::endl;
    }
  }
  else
  {
    speed_cmd=0;
  }
  if(speed_cmd<0) speed_cmd = 0;

  return speed_cmd;
}

void Control::compute_speed_cmd_V2V(const double front_dis,
                                    const double front_dis_expect,
                                    const double front_speed,
                                    const double front_acc,
                                    const double ego_speed,
                                    const bool start_mode_flag) {
  double k_d = k_front_distance;
  double k_a = k_feedforward_acc;
  // consider only acc > threshold to avoid noise
  if (fabs(front_acc) < k_acc_threshold) {
    k_a = 0.0;
  }
  // ignore dis_error for faster start
  if (start_mode_flag) {
    k_d = 0.0;
  }
  speed_cmd = front_speed + k_d * (front_dis - front_dis_expect) +
              k_a * front_acc;
  if(speed_cmd <= 0) speed_cmd = 0;
  std::cout << std::setprecision(3) << speed_cmd << "=" << front_speed << "+"
            << k_d << "*(" << front_dis << "-" << front_dis_expect
            << ")+" << k_a << "*" << front_acc << std::endl;
}

std::list<geometry_msgs::Pose2D>::const_reverse_iterator Control::get_min_index(void) //get min_index to delete old poins on map
{
  return min_index;
}

void Control::draw_pure_pursuit(ros::Publisher &draw_publisher)
{
  if(fabs(radius_preview) > 1e6) return;
  visualization_msgs::Marker::Ptr line_list(new visualization_msgs::Marker);
  //initialize line list;
  line_list->header.frame_id = map_frame_id;
  line_list->header.stamp = ros::Time::now();
  line_list->ns = map_frame_id;
  line_list->action = visualization_msgs::Marker::ADD;
  line_list->pose.orientation.w = 1.0;
  line_list->id = 1;
  line_list->type = visualization_msgs::Marker::LINE_LIST;
  if(lateral_type == 0) {
    line_list->color.r = 0.0;
    line_list->color.b = 1.0;
  }
  else if(lateral_type == 1) {
    line_list->color.r = 1.0;
    line_list->color.b = 0.0;
  }

  line_list->color.g = 0.0;
  line_list->color.a = 1.0;
  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_list->scale.x = 0.05;
  line_list->scale.y = 0.05;
  line_list->scale.z = 0.05;
  double x_c=x_vehicle-radius_preview*sin(theta_vehicle); //turning circle center
  double y_c=y_vehicle+radius_preview*cos(theta_vehicle); //turning circle center
  double theta=atan2((x_vehicle-x_c)*(y_preview-y_c)-(y_vehicle-y_c)*(x_preview-x_c),(y_vehicle-y_c)*(y_preview-y_c)+(x_vehicle-x_c)*(x_preview-x_c));
  Vehicle::draw_arc(line_list,x_c,y_c,x_vehicle,y_vehicle,0,theta,8);
  draw_publisher.publish(line_list);
} //void Control::draw_pure_pursuit(ros::Publisher &draw_publisher)

}
