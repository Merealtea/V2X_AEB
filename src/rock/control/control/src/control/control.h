/**
  *control.h
  *brief: vehicle speed and steer control
  *author:Jianlin Zhang
  *date:20180817
  **/

#ifndef CONTROL_H
#define CONTROL_H
#define _USE_MATH_DEFINES //use constant number defined in math.h
#include <iomanip>
#include <cmath>
#include <queue>
#include <vector>
#include <list>
#include <eigen3/Eigen/Geometry>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "common.h"
#include "draw.h"

namespace Control
{
class Control
{
public:
  //cmd output
  double steer_cmd;
  double speed_cmd;
  int brake_cmd;

  //control parameters
  double speed_k;
  double speed_k1;
  double speed_k2;
  // double min_dis_thres; //if min_dis < thres, steer_cmd will not change
  // double speed_cmd_thres; //if speed_cmd < speed_cmd_thres, steer will not change
  double preview_distance;
  double preview_distance_base;
  double preview_time;
  double speed_target; // speed of target
  double behind_speed; // speed of behind vehicle
  double self_speed;
  double feedforward_k; //k for speed feedforward control. the greater k will lead to more feedback
  double zero_brake_acc_threshold;
  double zero_brake_speed_threshold;

  //vehicle status
  double x_vehicle; //x of vehicle in the world coordinate
  double y_vehicle; //y of vehicle in the world coordinate
  double theta_vehicle; //theta of vehicle in the world coordinate
  double speed_vehicle; //speed of vehicle
  double wheelbase;      //wheel base
  double steer_vehicle;

  double straight_threshold = 0.035;
  double k_curvature_factor = 5.0;
  double min_preview_distance_for_search = 3.5;
  double heading_cost_factor = 0.8;
  double k_front_distance = 1.0;
  double k_behind_distance = 0.8;
  double c_front_velocity = 0.8;
  double c_behind_velocity = 0.6;
  double k_speed_output = 1.0;
  double k_brake_output = 5.0;
  double k_feedforward_acc = 1.0;  //gain for V2V acc information
  double k_acc_threshold = 0.0; //min threshold for V2V acc information

  Control(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~Control(){};
  double compute_steer_cmd_by_pure_pursuit(const std::list<geometry_msgs::Pose2D> &map,
                                          const std::list<geometry_msgs::Pose2D>::const_reverse_iterator &goal_index);
  void compute_steer_cmd_by_pure_pursuit(const std::list<TrackPoint> &map);
  void compute_steer_cmd_by_search(const std::list<TrackPoint> &map, double front_average_curvature);
  void draw_preview_point(ros::Publisher &pt_publisher);
  void draw_pure_pursuit(ros::Publisher &draw_publisher);
  double compute_speed_cmd(const std::list<geometry_msgs::Pose2D> &map,
                          const std::list<geometry_msgs::Pose2D>::const_reverse_iterator &goal_index);
  void compute_speed_cmd(const std::list<TrackPoint> &map);
  double compute_average_curvature(const std::list<TrackPoint> &map, int val);
  double compute_average_curvature_by_heading(const std::list<TrackPoint> &map, int val);
  double compute_curvature(const std::list<TrackPoint>::const_iterator first,
                           const std::list<TrackPoint>::const_iterator second,
                           const std::list<TrackPoint>::const_iterator third);
  double compute_acc_cmd(double front_dis, double front_dis_expect, double front_speed,
                       double behind_dis, double behind_dis_expect, double behind_speed, double ego_speed,
                       double front_average_curvature, double behind_average_curvature, bool consider_behind);
  void compute_speed_cmd(double acc_output, double ego_speed, double speed_output_max,
                         double brake_output_max, double& speed_output, double& brake_output);
  void compute_speed_cmd_V2V(const double front_dis,
                             const double front_dis_expect,
                             const double front_speed, const double front_acc,
                             const double ego_speed,
                             const bool start_mode_flag);

  std::list<geometry_msgs::Pose2D>::const_reverse_iterator get_min_index(void); //get min_index to delete old poins on map
private:
  std::string map_frame_id;
  double min_preview_distance;
  double max_preview_distance;
  double x_preview;
  double y_preview;
  int lateral_type = 0;
  bool is_passed; //whether is vehicle passed goal
  double radius_preview;  //expect turnning radius
  double angle_preview; //angle between vehicle direction and preview point
  double real_distance_preview; //distance between vehicle and preview point, used to feedback speed_cmd, so use new R to compute steer_cmd
  std::list<geometry_msgs::Pose2D>::const_reverse_iterator min_index; //refer to min distance between current pose and map
  std::list<geometry_msgs::Pose2D>::const_reverse_iterator min_index_lc;
  double speed_cmd_max; //unit: m/s
  TrackPoint preview_point;
  double goal_distance;
  double compute_steer(const std::list<TrackPoint>::const_iterator preview_pt);
  double calc_cost(double steer_i);
}; //class Control
} //namespace Control
#endif
