/**
  *draw.h
  *brief:common draw functions define
  *author:Jianlin Zhang
  *date:20171213
  **/

#ifndef DRAW_H
#define DRAW_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace Vehicle
{
void draw_cuboid(visualization_msgs::Marker::Ptr line_list,
                 double p1_x, double p1_y, double p1_z,
                 double p7_x, double p7_y, double p7_z);
void draw_plane(visualization_msgs::Marker::Ptr plane,
                double a, double b, double c, double d,
                double x_min, double x_max,
                double y_min, double y_max);
void draw_square(visualization_msgs::Marker::Ptr line_list,
                 double p1_x, double p1_y, double p1_z,
                 double p3_x, double p3_y, double p3_z);
void draw_arc(visualization_msgs::Marker::Ptr line_list,
              double center_x, double center_y,
              double p_x, double p_y,
              double height, double theta, int segment);
void draw_arrow(visualization_msgs::Marker::Ptr arrows,
                double a, double b, double c,
                double start_x, double start_y, double start_z,
                double norm);
void draw_grid(visualization_msgs::Marker::Ptr line_list,
               double x_min,double x_max,
               double y_min,double y_max,
               size_t row,size_t column);
} //namespace vehicle

#endif