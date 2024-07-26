/**
  *mapping.cpp
  *brief: Mapping via target pose(gps or lidar)
  *author:Jianlin Zhang
  *date:20180422
  **/

#include "mapping.h"
#include "common/math/math_utils.h"

using namespace std;
using namespace cyberc3::common;

namespace Mapping
{
  const double M_PI_180=0.01745329252;  //M_PI/180
  const double very_little=0.05;
Mapping::Mapping(ros::NodeHandle nh, ros::NodeHandle pnh)
  :is_goal_setted(false),time_offset(ros::Time::now().toSec())
{
  pnh.param("map_frame_id", map_frame_id, std::string("map"));
  pnh.param("max_track_size", max_track_size, 3000);
  pnh.param<double>("goal_x_set", goal_x_set, 10);
  pnh.param<double>("goal_y_set", goal_y_set, 0);
  set_goal(goal_x_set,goal_y_set);
}

bool Mapping::check_point_passed(std::list<TrackPoint>::const_iterator cp, const geometry_msgs::Pose2D& local_pose) {
  double dx_local = cp->x_ - local_pose.x;
  double dy_local = cp->y_ - local_pose.y;
  double angle = atan2(dy_local,dx_local) - local_pose.theta;
  double distance = sqrt(dx_local * dx_local + dy_local * dy_local);
  if(cos(angle) < 0 && distance > k_passed_reverse_distance) return true;
  else return false;
}

double Mapping::dis2local(std::list<TrackPoint>::const_iterator cp, const geometry_msgs::Pose2D& local_pose) {
  double dx_local = cp->x_ - local_pose.x;
  double dy_local = cp->y_ - local_pose.y;
  double distance = sqrt(dx_local * dx_local + dy_local * dy_local);
  return distance;
}

void Mapping::update_front_track(
    const geometry_msgs::Pose2D::ConstPtr &front_pose,
    geometry_msgs::Pose2D local_pose) {
  if (front_track_raw.empty()) {
    front_track_raw.push_back(
        TrackPoint(local_pose.x, local_pose.y, local_pose.theta));
    closest_point = front_track_raw.begin();
    // 初始化距离太远时中间补点
    curve_distance_max = dis2local(closest_point, *front_pose);
    if (curve_distance_max > 2.0) {
      int nums = (int)(dis2local(closest_point, *front_pose) / 0.1);
      for (int i = 1; i < nums; ++i) {
        TrackPoint insert_point;
        double ratio = (double)(i) / (double)(nums);
        insert_point.x_ =
            closest_point->x_ + ratio * (front_pose->x - closest_point->x_);
        insert_point.y_ =
            closest_point->y_ + ratio * (front_pose->y - closest_point->y_);
        insert_point.theta_ = front_pose->theta;
        insert_point.curve_distance_ = 0.1 * i;
        front_track_raw.push_back(insert_point);
      }
    }
    start_s = curve_distance_max;
    return;
  }
  // 如果与上一个点距离太近则忽略,生成在后面的也忽略
  const double d_x = front_pose->x - front_track_raw.back().x_;
  const double d_y = front_pose->y - front_track_raw.back().y_;
  const double d_s = sqrt(pow(d_x, 2) + pow(d_y, 2));
  const double dot = d_x * cos(front_track_raw.back().theta_) +
                     d_y * sin(front_track_raw.back().theta_);
  double current_t = ros::Time::now().toSec() - time_offset;
  if ((d_s > very_little && dot > 0) || d_s > 0.50) {
    double kDeltaS = 0.1;
    int k_interpolation_num = std::ceil(d_s / kDeltaS);
    auto last_point = front_track_raw.back();
    const double d_theta =
        math::NormalizeAngle(front_pose->theta - last_point.theta_);
    const double d_t = current_t - last_point.t_;
    for (int i = 1; i <= k_interpolation_num; ++i) {
      TrackPoint insert_point;
      double ratio = (double)i / (double)k_interpolation_num;
      insert_point.x_ = last_point.x_ + ratio * d_x;
      insert_point.y_ = last_point.y_ + ratio * d_y;
      insert_point.theta_ = last_point.theta_ + ratio * d_theta;
      insert_point.curve_distance_ = last_point.curve_distance_ + ratio * d_s;
      insert_point.t_ = last_point.t_ + ratio * d_t;
      front_track_raw.push_back(insert_point);
    }
    // front_track_raw.push_back(
    //     TrackPoint(front_pose->x, front_pose->y, front_pose->theta,
    //                front_track_raw.back().curve_distance_ + ds));
  }
  curve_distance_max =
      front_track_raw.back().curve_distance_ - closest_point->curve_distance_;
}

void Mapping::remove_passed_points(const geometry_msgs::Pose2D &local_pose) {
  // 删除已经经过的点
  auto clean_point = front_track_raw.begin();
  while (!front_track_raw.empty() && clean_point != front_track_raw.end()) {
    if (check_point_passed(clean_point, local_pose)) {
      ++clean_point;
      front_track_raw.pop_front();
    } else
      break;
  }

  // search closest_point
  double min_dis = 1e9;
  for (auto temp_point = front_track_raw.begin();
       temp_point != front_track_raw.end(); ++temp_point) {
    double dis = dis2local(temp_point, local_pose);
    if (dis < min_dis) {
      min_dis = dis;
      closest_point = temp_point;
    } else if (temp_point->curve_distance_ - closest_point->curve_distance_ >
               2.0) // search extra 2m
      break;
  }

}

void Mapping::update_behind_track(const geometry_msgs::Pose2D::ConstPtr &behind_pose, geometry_msgs::Pose2D local_pose) {
  // 如果与上一个点距离太近则忽略
  if(behind_track.empty() || (abs(local_pose.x - behind_track.back().x_) > very_little
                              || abs(local_pose.y - behind_track.back().y_) > very_little)) {
    if(behind_track.empty()) behind_track.push_back(TrackPoint(local_pose.x, local_pose.y, local_pose.theta, 0.0));
    else {
      // 生成在后面的也忽略
      double dx = local_pose.x - behind_track.back().x_;
      double dy = local_pose.y - behind_track.back().y_;
      double dot = dx * cos(behind_track.back().theta_) + dy * sin(behind_track.back().theta_);
      if(dot > 0) behind_track.push_back(TrackPoint(local_pose.x, local_pose.y, local_pose.theta, 0.0));
    }
  }

  // 删除已经经过的点
  std::list<TrackPoint>::iterator clean_point = behind_track.begin();
  while(!behind_track.empty() && clean_point != behind_track.end()) {
    if(check_point_passed(clean_point, *behind_pose)) {
      ++clean_point;
      behind_track.pop_front();
    }
    else {
      break;
    }
  }
  closest_point = behind_track.begin();

  // 初始化距离太远时中间补点
  if(dis2local(closest_point, *behind_pose) > 2.0) {
    int nums = (int)(dis2local(closest_point, *behind_pose) / 0.05);
    for(int i=1; i < nums; ++i) {
      TrackPoint insert_point;
      double ratio = (double)(i) / (double)(nums);
      insert_point.x_ = closest_point->x_ + ratio * (behind_pose->x - closest_point->x_);
      insert_point.y_ = closest_point->y_ + ratio * (behind_pose->y - closest_point->y_);
      insert_point.theta_ = closest_point->theta_;
      behind_track.push_front(insert_point);
    }
  }

  closest_point = behind_track.begin();

  // 更新轨迹点距离
  std::list<TrackPoint>::iterator curr_point = behind_track.end();
  --curr_point;
  double curve_distance = 0.0;
  while(curr_point != behind_track.begin()) {
    // 更新轨迹点距离
    curr_point->curve_distance_ = curve_distance;
    std::list<TrackPoint>::iterator last_point = curr_point;
    --curr_point;
    double dx = curr_point->x_ - last_point->x_;
    double dy = curr_point->y_ - last_point->y_;
    curve_distance += sqrt(dx * dx + dy * dy);
  }
  curr_point->curve_distance_ = curve_distance;
  behind_curve_distance_max = curve_distance;
}

void Mapping::update_target_track_from_lidar(const geometry_msgs::Pose2D::ConstPtr &tp)
{
  // static geometry_msgs::Pose2D::Ptr tp_old;
  static bool is_initialized(false);
  if(is_initialized)
  {
    // std::cout<<"add "<<target_track.size()<<" element"<<std::endl;
    geometry_msgs::Pose2D::Ptr mid(new geometry_msgs::Pose2D);
    geometry_msgs::Pose2D last(*(target_track.crbegin()));
    mid->x=(last.x+tp->x)/2;
    mid->y=(last.y+tp->y)/2;
    mid->theta=(last.theta+tp->theta)/2;
    update_target_track(mid);
    // tp_old=tp;
  }
  if(target_track.size()==0)
  {
    // std::cout<<"add first element"<<std::endl;
    update_target_track(tp);
  }
  // else if((target_track.size()==1)&&(!is_initialized))  //bug here!! is_initialized will not change when switch from gps mode to lidar_mode
  else if((target_track.size()>=1)&&(!is_initialized))
  {
    // std::cout<<"init"<<std::endl;
    is_initialized=true;
    // tp_old=tp;
  }
} //void Mapping::update_target_track_from_lidar(const geometry_msgs::Pose2D::ConstPtr &tp)

void Mapping::update_target_track(const geometry_msgs::Pose2D::ConstPtr &tp)
{
  if(target_track.size()==0)
  {
    if((std::abs(tp->x)>very_little)||(std::abs(tp->y)>very_little))  //it is the first point
    {
      target_track.push_back(*tp);
    }
  }
  else
  {
    auto tp_old=target_track.back();
    if((std::abs(tp->x-tp_old.x)>very_little)||(std::abs(tp->y-tp_old.y)>very_little))  //it is a new point
    {
      target_track.push_back(*tp);
      if(target_track.size()>max_track_size)
      {
        for(size_t i=0;i<max_track_size/2;++i)
        {
          target_track.pop_front();
        }
      }
    }
  }
} //void Mapping::update_target_track(const geometry_msgs::Pose2D::ConstPtr &tp)

void Mapping::update_map(double local_x, double local_y, double local_theta)
{
  if((target_track.size()>0)&&is_goal_setted)
  {
    //push back new point
    geometry_msgs::Pose2D &goal_new=target_track.back();
    goal_new.x-=goal_y*sin(goal_new.theta);
    goal_new.y+=goal_y*cos(goal_new.theta);

    geometry_msgs::Pose2D goal_old;
    //geometry_msgs::Pose2D &goal_old=map.back();
    if(map.size() > 0) goal_old=map.back();
    else
    {
      goal_old.x = local_x;
      goal_old.y = local_y;
      goal_old.theta = local_theta;
    }

    if((std::abs(goal_new.x-goal_old.x)>very_little)||(std::abs(goal_new.y-goal_old.y)>very_little))  //it is a new point
    {
      if((std::abs(goal_new.x-goal_old.x)>1.0)||(std::abs(goal_new.y-goal_old.y)>1.0))
      {
        for(double i=0;i<1.0;i+=0.05)
        {
          geometry_msgs::Pose2D goal_insert;
          goal_insert.x=(1-i)*goal_old.x+i*goal_new.x;
          goal_insert.y=(1-i)*goal_old.y+i*goal_new.y;
          //std::cout<<"pushing("<<goal_insert.x<<','<<goal_insert.y<<')'<<std::endl;
          map.push_back(goal_insert);
        }
      }
      map.push_back(goal_new);
      //if map length is too long, cut a half of it
      if(map.size()>max_track_size)
      {
        for(size_t i=0;i<max_track_size/2;++i)
        {
          map.pop_front();
        }
      }
      //find goal point index
      auto it=map.crbegin();
      double distance=0;
      geometry_msgs::Pose2D current=*it;

      //std::cout<<"goal_x:"<<goal_x<<std::endl;
      if(std::abs(goal_x) < 1)
      {
        std::cout<<"goal_x is changed to 0"<<std::endl;
        goal_index=it;
      }
      else
      {
        while(++it!=map.crend())
        {
          const geometry_msgs::Pose2D &next=*it;
          double c=cos(current.theta);
          double s=sin(current.theta);
          if(std::abs(c)>std::abs(s))
          {
            distance+=std::abs((next.x-current.x)/c);
          }
          else
          {
            distance+=std::abs((next.y-current.y)/s);
          }
          if(distance>std::abs(goal_x))
          {
            goal_index=it;
            break;
          }
          current=next;
        }
        if(distance<std::abs(goal_x)) //map length is too short
        {
          std::cout<<"map is too short, so goal is set to end"<<std::endl;
          goal_index=map.crend();
        }
      }
    }
  }
  else
  {
    std::cout<<"no track size:"<<target_track.size()<<std::endl;
  }
}

std::list<geometry_msgs::Pose2D>::const_reverse_iterator Mapping::get_goal_index(void)
{
  return goal_index;
}

void Mapping::clear_old_map(std::list<geometry_msgs::Pose2D>::const_reverse_iterator &min_index)
{
  if((min_index!=map.crend())&&(min_index!=map.crbegin())&&(goal_index!=map.crend()))
  {
    auto min_index_next=++min_index;
    auto goal_index_next=goal_index;
    ++goal_index_next;
    while((map.crend()!=min_index_next)&&(map.crend()!=goal_index_next))
    {
      map.pop_front();
    }
  }
}

void Mapping::clear_map()
{
  map.clear();
}

void Mapping::clear_target_track()
{
  target_track.clear();
}

void Mapping::set_goal(const double &x, const double &y)
{
  goal_x=x;
  goal_y=y;
  if(!is_goal_setted)
  {
    is_goal_setted=true;
  }
}

void Mapping::draw_target_track(ros::Publisher &track_draw_publisher)
{
  geometry_msgs::PoseArray::Ptr output(new geometry_msgs::PoseArray());
  for(geometry_msgs::Pose2D &pose:target_track)
  {
    geometry_msgs::Pose p;
    p.position.x=pose.x;
    p.position.y=pose.y;
    p.orientation.w=cos(pose.theta/2);
    p.orientation.x=0;
    p.orientation.y=0;
    p.orientation.z=sin(pose.theta/2);
    output->poses.push_back(p);
  }
  output->header.frame_id = map_frame_id;
  output->header.stamp = ros::Time::now();
  track_draw_publisher.publish(output);
}

void Mapping::draw_map(ros::Publisher &track_draw_publisher)
{
  visualization_msgs::Marker::Ptr line_list(new visualization_msgs::Marker);
  //initialize line list;
  line_list->header.frame_id = map_frame_id;
  line_list->header.stamp = ros::Time::now();
  line_list->ns = map_frame_id;
  line_list->action = visualization_msgs::Marker::ADD;
  line_list->pose.orientation.w = 1.0;
  line_list->id = 1;
  line_list->type = visualization_msgs::Marker::LINE_STRIP;
  line_list->color.r = 0.5;
  line_list->color.g = 1.0;
  line_list->color.b = 1.0;
  line_list->color.a = 1.0;
  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_list->scale.x = 0.05;
  line_list->scale.y = 0.05;
  line_list->scale.z = 0.05;
  for(geometry_msgs::Pose2D &pose:map)
  {
    geometry_msgs::Point p;
    p.x=pose.x;
    p.y=pose.y;
    p.z=0;
    // std::cout<<'('<<p.x<<','<<p.y<<')'<<std::endl;
    line_list->points.push_back(p);
  }
  track_draw_publisher.publish(line_list);
}

void Mapping::draw_front_track(ros::Publisher &pose_draw_publisher, ros::Publisher &marker_draw_publisher) {
  geometry_msgs::PoseArray pose_output;
  for(TrackPoint &point : front_track_raw)
  {
    geometry_msgs::Pose p;
    p.position.x = point.x_;
    p.position.y = point.y_;
    p.position.z = 0;
    p.orientation = tf::createQuaternionMsgFromYaw(point.theta_);
    pose_output.poses.push_back(p);
  }
  pose_output.header.frame_id = map_frame_id;
  pose_output.header.stamp = ros::Time::now();
  pose_draw_publisher.publish(pose_output);

  visualization_msgs::MarkerArray marker_array_output;
  visualization_msgs::Marker marker_output;
  marker_output.header.frame_id = map_frame_id;
  marker_output.header.stamp = ros::Time::now();
  marker_output.action = visualization_msgs::Marker::ADD;
  marker_output.type = visualization_msgs::Marker::CUBE;
  marker_output.lifetime = ros::Duration(0.1);

  int index = 0;
  for(TrackPoint &point : front_track_raw)
  {
    if(index == 0) {
      marker_output.color.r = 1.0;
      marker_output.color.g = 0.0;
      marker_output.color.b = 0.0;
      marker_output.color.a = 1.0;
      marker_output.scale.x = 0.1;
      marker_output.scale.y = 0.1;
      marker_output.scale.z = 0.1;
    }
    else if(index == front_track_raw.size() - 1) {
      marker_output.color.r = 0.0;
      marker_output.color.g = 0.0;
      marker_output.color.b = 1.0;
      marker_output.color.a = 1.0;
      marker_output.scale.x = 0.1;
      marker_output.scale.y = 0.1;
      marker_output.scale.z = 0.1;
    }
    else {
      marker_output.color.r = 0.0;
      marker_output.color.g = 1.0;
      marker_output.color.b = 0.0;
      marker_output.color.a = 1.0;
      marker_output.scale.x = 0.02;
      marker_output.scale.y = 0.02;
      marker_output.scale.z = 0.02;
    }

    marker_output.id = index++;
    marker_output.pose.position.x = point.x_;
    marker_output.pose.position.y = point.y_;
    marker_output.pose.position.z = 0;
    marker_output.pose.orientation = tf::createQuaternionMsgFromYaw(point.theta_);
    marker_array_output.markers.emplace_back(marker_output);
  }
  marker_draw_publisher.publish(marker_array_output);
}

// void Mapping::draw_front_track(ros::Publisher &pose_draw_publisher) {
//   geometry_msgs::PoseArray pose_output;
//   for (TrackPoint &point : front_track_raw) {
//     geometry_msgs::Pose p;
//     p.position.x = point.x_;
//     p.position.y = point.y_;
//     p.position.z = 0;
//     p.orientation = tf::createQuaternionMsgFromYaw(point.theta_);
//     pose_output.poses.push_back(p);
//   }
//   pose_output.header.frame_id = map_frame_id;
//   pose_output.header.stamp = ros::Time::now();
//   pose_draw_publisher.publish(pose_output);
// }

void Mapping::draw_track(std::list<TrackPoint> &track,
                         ros::Publisher &pose_draw_publisher) {
  geometry_msgs::PoseArray pose_output;
  for (TrackPoint &point : track) {
    geometry_msgs::Pose p;
    p.position.x = point.x_;
    p.position.y = point.y_;
    p.position.z = 0;
    p.orientation = tf::createQuaternionMsgFromYaw(point.theta_);
    pose_output.poses.push_back(p);
  }
  pose_output.header.frame_id = map_frame_id;
  pose_output.header.stamp = ros::Time::now();
  pose_draw_publisher.publish(pose_output);
}

void Mapping::draw_track(DiscretizedPath &track,
                         ros::Publisher &pose_draw_publisher) {
  geometry_msgs::PoseArray pose_output;
  for (PathPoint &point : track) {
    geometry_msgs::Pose p;
    p.position.x = point.x();
    p.position.y = point.y();
    p.position.z = 0;
    p.orientation = tf::createQuaternionMsgFromYaw(point.theta());
    pose_output.poses.push_back(p);
  }
  pose_output.header.frame_id = map_frame_id;
  pose_output.header.stamp = ros::Time::now();
  pose_draw_publisher.publish(pose_output);
}

void Mapping::draw_behind_track(ros::Publisher &pose_draw_publisher) {
  geometry_msgs::PoseArray pose_output;
  for(TrackPoint &point : behind_track)
  {
    geometry_msgs::Pose p;
    p.position.x = point.x_;
    p.position.y = point.y_;
    p.position.z = 0;
    p.orientation = tf::createQuaternionMsgFromYaw(point.theta_);
    pose_output.poses.push_back(p);
  }
  pose_output.header.frame_id = map_frame_id;
  pose_output.header.stamp = ros::Time::now();
  pose_draw_publisher.publish(pose_output);
}

bool Mapping::convert_trajectory(const DiscretizedPath &path,
                                 cyber_msgs::LocalTrajList &traj_msg_out) {
  if (!path.empty()) {
    double kDeltaS = 0.02;
    double path_length = path.Length();
    double delta_s = path_length / std::ceil(path_length / kDeltaS);
    path_length += delta_s * 1.0e-6;
    traj_msg_out.points.clear();
    for (double s = path.front().s(); s < path.back().s(); s += delta_s) {
      cyber_msgs::LocalTrajPoint point;
      const auto point2d = path.Evaluate(s);
      point.position.x = point2d.x();
      point.position.y = point2d.y();
      point.s = point2d.s();
      point.theta = point2d.theta();
      point.kappa = point2d.kappa();
      point.kappa_prime = point2d.dkappa();
      point.orientation = tf::createQuaternionMsgFromYaw(point2d.theta());
      traj_msg_out.points.push_back(std::move(point));
    }
    traj_msg_out.header.frame_id = map_frame_id;
    traj_msg_out.header.stamp = ros::Time::now();
    return true;
  }
  return false;
}

bool Mapping::convert_trajectory(const std::list<TrackPoint> &track,
                                 cyber_msgs::LocalTrajList &traj_msg_out) {
  if (!track.empty()) {
    DiscretizedPath path;
    for (auto iter = track.begin(); iter != track.end(); ++iter) {
      PathPoint path_point;
      path_point.set_x(iter->x_);
      path_point.set_y(iter->y_);
      path_point.set_theta(iter->theta_);
      path_point.set_s(iter->curve_distance_);
      path_point.set_kappa(iter->kappa_);
      path.push_back(std::move(path_point));
    }
    return Mapping::convert_trajectory(path, traj_msg_out);
  }
  return false;
}

TrackPoint Mapping::query_nearest_point_by_relative_time(const double t) {
  auto func_com = [](const TrackPoint &point, const double time) {
    return point.t_ < time;
  };
  auto it_low = std::lower_bound(front_track_raw.begin(), front_track_raw.end(),
                                 closest_point->t_ + t, func_com);
  if (it_low == front_track_raw.begin()) {
    return front_track_raw.front();
  }

  if (it_low == front_track_raw.end()) {
    return front_track_raw.back();
  }

  auto it_lower = std::prev(it_low);
  if (it_low->t_ - (closest_point->t_ + t) <
      (closest_point->t_ + t) - it_lower->t_) {
    return *it_low;
  }
  return *it_lower;
}

TrackPoint Mapping::query_target_point_by_relative_time(const double t) {
  auto func_com = [](const double time, const TrackPoint &point) {
    return point.t_ < time;
  };
  auto it_low =
      std::upper_bound(front_track_raw.rbegin(), front_track_raw.rend(),
                       front_track_raw.back().t_ - t, func_com);
  if (it_low == front_track_raw.rbegin()) {
    return front_track_raw.back();
  }

  if (it_low == front_track_raw.rend()) {
    return front_track_raw.front();
  }

  auto it_lower = std::prev(it_low);
  if ((front_track_raw.back().t_ - t) - it_low->t_ <
      it_lower->t_ - (front_track_raw.back().t_ - t)) {
    return *it_low;
  }
  return *it_lower;
  return *it_low;
}
}
