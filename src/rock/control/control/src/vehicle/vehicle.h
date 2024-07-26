/**
  *vehicle.h
  *brief:
  *author:Jianlin Zhang
  *date:20170608
  **/

#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>

namespace Vehicle
{
/*──────────────────────────────────────────────────────────
description:
the coordinate of vehicle is in the middle of rear wheels.
X axis is forward. Y axis is leftward. And Z axis is upward.
It just look like this:
                            
                     ┌─────────────┐
                    ┌┤             ├┐
                    ||             ||
                    └┤  /¯¯\       ├┘
                     |  \__/       |
                     |      ↑x     |
                     |      |      |
                     |      |      |
                    ┌┤      |      ├┐
                 y ←||──────┼      ||
                    └┤             ├┘
                     └─────────────┘
───────────────────────────────────────────────────────────*/
class Vehicle
{
public:
  //vehicle's stable parameter
  double wheelbase;      //wheel base
  double steer_ratio;
  //vehicle's status
  double x;
  double y;
  double theta;

  double speed; //the real speed of vehicle,estimate from encoder and other sensors
  double steer; //steer angle
  double mileage; //unit: m
  
  //control command send to vehicle
  double steer_cmd; //unit: degree, the respect angle of front wheel
  double speed_cmd; //unit: m/s
  double speed_cmd_max; //unit: m/s
  double brake_cmd;

  Vehicle(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~Vehicle() {}
  void update_track(void);
  void draw_track(ros::Publisher &track_draw_publisher);
  void clear_track();
  void reset_local_pose();

private:
  std::string vehicle_frame_id;
  std::string map_frame_id;

  std::list<geometry_msgs::Pose2D> track;  //track of vehicle
  int max_track_size; //if track size is too large, cut a half of it
};
}
#endif
