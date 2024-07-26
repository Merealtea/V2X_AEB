/**
  *ground_filter.h
  *brief: filter ground in Lidar's point cloud
  *author:Jianlin Zhang
  *date:20180819
  **/

#ifndef GROUND_FILTER_H
#define GROUND_FILTER_H
#define _USE_MATH_DEFINES //use constant number defined in math.h
#include <cmath>
#include <ros/ros.h>
#include "velodyne.h"
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

namespace Lidar
{
typedef pcl::PointXYZIR PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
class Ground_filter
{
  public:
    int window_width;
    double cov_thres;
    double grid_length;
    double grid_width;

    Ground_filter(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~Ground_filter(){};
    void remove_local_ground(const PointCloudT::Ptr &pcl_in, PointCloudT::Ptr &pcl_out);
};
}
#endif