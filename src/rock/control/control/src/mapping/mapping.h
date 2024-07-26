/**
  *mapping.h
  *brief: Mapping via target pose(gps or lidar)
  *author:Jianlin Zhang
  *date:20180730
  **/

#ifndef MAPPING_H
#define MAPPING_H
#define _USE_MATH_DEFINES //use constant number defined in math.h
#include <cmath>
#include <list>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "common.h"
#include "track_smoother.h"
#include "discretized_path.h"
#include "cyber_msgs/LocalTrajList.h"

namespace Mapping
{
  using namespace cyberc3::planning;
class Mapping
{
public:
  Mapping(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~Mapping(){};
  void update_target_track_from_lidar(const geometry_msgs::Pose2D::ConstPtr &tp);
  void update_target_track(const geometry_msgs::Pose2D::ConstPtr &tp);
  void update_front_track(const geometry_msgs::Pose2D::ConstPtr &front_pose, geometry_msgs::Pose2D local_pose);
  void remove_passed_points(const geometry_msgs::Pose2D &local_pose);
  void update_behind_track(const geometry_msgs::Pose2D::ConstPtr &behind_pose, geometry_msgs::Pose2D local_pose);
  void update_map(double local_x, double local_y, double local_theta);
  void clear_old_map(std::list<geometry_msgs::Pose2D>::const_reverse_iterator &min_index);
  std::list<geometry_msgs::Pose2D>::const_reverse_iterator get_goal_index(void);
  void set_goal(const double &x, const double &y);
  void draw_target_track(ros::Publisher &track_draw_publisher);
  void draw_map(ros::Publisher &track_draw_publisher);
  void draw_front_track(ros::Publisher &pose_draw_publisher, ros::Publisher &marker_draw_publisher);
  // void draw_front_track(ros::Publisher &pose_draw_publisher);
  void draw_track(std::list<TrackPoint> &track,
                  ros::Publisher &pose_draw_publisher);
  void draw_track(DiscretizedPath &track,
                  ros::Publisher &pose_draw_publisher);
  void draw_behind_track(ros::Publisher &pose_draw_publisher);
  std::list<geometry_msgs::Pose2D> map;  //track of expect position
  std::list<geometry_msgs::Pose2D> target_track;  //track of target vehicle
  std::list<TrackPoint> front_track_raw;  //track of front vehicle
  DiscretizedPath front_track_smoothed;  //track of front vehicle
  std::list<TrackPoint> behind_track;  //track of behind vehicle
  std::list<TrackPoint>::iterator closest_point;
  double curve_distance_max;
  double behind_curve_distance_max;
  double start_s = 0.0;
  void clear_map();
  void clear_target_track();
  // convert track to trajectory for publishing
  bool convert_trajectory(const DiscretizedPath &path,
                          cyber_msgs::LocalTrajList &traj_msg_out);
  bool convert_trajectory(const std::list<TrackPoint> &track,
                          cyber_msgs::LocalTrajList &traj_msg_out);
  void reset_time_offset() { time_offset = ros::Time::now().toSec(); };
  TrackPoint query_nearest_point_by_relative_time(const double t);
  TrackPoint query_target_point_by_relative_time(const double t);

private:
  std::string map_frame_id;
  int max_track_size; //if track size is too large, cut a half of it
  double k_passed_reverse_distance = 0.0; //reserve certain passed points for smoother calculation
  double time_offset;
  double goal_x;
  double goal_y;
  double goal_x_set;
  double goal_y_set;
  bool is_goal_setted;
  bool check_point_passed(std::list<TrackPoint>::const_iterator cp, const geometry_msgs::Pose2D& local_pose);
  double dis2local(std::list<TrackPoint>::const_iterator cp, const geometry_msgs::Pose2D& local_pose);
  std::list<geometry_msgs::Pose2D>::const_reverse_iterator goal_index; //refer to current goal point on map
};
}
#endif
