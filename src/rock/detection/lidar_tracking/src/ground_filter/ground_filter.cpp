/**
  *segment.cpp
  *brief: segment of Lidar's point pcl_xyz
  *author:Jianlin Zhang
  *date:20180819
  **/

#include "ground_filter.h"

namespace Lidar
{
Ground_filter::Ground_filter(ros::NodeHandle nh, ros::NodeHandle pnh)
  :window_width(3),
  cov_thres(0.1),
  grid_length(0.1),
  grid_width(0.1)
{
  
}

void Ground_filter::remove_local_ground(const PointCloudT::Ptr &pcl_in, PointCloudT::Ptr &pcl_out)
{
  if(pcl_in->width>0)
  {
    float x_min=100.0;
    float y_min=100.0;
    float x_max=-100.0;
    float y_max=-100.0;
    for(const VPoint &p:pcl_in->points)
    {
      x_max=std::max(p.x,x_max);
      y_max=std::max(p.y,y_max);
      x_min=std::min(p.x,x_min);
      y_min=std::min(p.y,y_min);
    }
    if((x_max>x_min)&&(y_max>y_min))
    {
      const size_t grid_cols=ceil((x_max-x_min)/grid_length);
      const size_t grid_rows=ceil((y_max-y_min)/grid_width);
      PointCloudT pcl_ground(*pcl_in);
      double height[grid_cols][grid_rows];
      double height2[grid_cols][grid_rows];
      bool is_obstacle[grid_cols][grid_rows];
      size_t points_count[grid_cols][grid_rows];
      // ROS_INFO("initialize all array:%zu,%zu",grid_cols,grid_rows);
      for(size_t i=0;i<grid_cols;++i)
      {
        for(size_t j=0;j<grid_rows;++j)
        {
          height[i][j]=0;
          height2[i][j]=0;
          points_count[i][j]=0;
          is_obstacle[i][j]=false;
        }
      }
      // ROS_INFO("statistic height");  
      for(VPoint &p:pcl_ground.points)
      {
        size_t j=floor((p.y-y_min)/grid_width);
        p.intensity=j;
        p.ring=(floor((p.x-x_min)/grid_length));
        height[p.ring][j]+=p.z;
        height2[p.ring][j]+=p.z*p.z;
        ++points_count[p.ring][j];
      }
      // ROS_INFO("judging");  
      for(size_t i=0;i<grid_cols;++i)
      {
        for(size_t j=0;j<grid_rows;++j)
        {
          if(points_count[i][j]>3)
          {
            if((height2[i][j]*points_count[i][j]-height[i][j]*height[i][j]>cov_thres*cov_thres*points_count[i][j]*points_count[i][j]))
            {
              is_obstacle[i][j]=true;
            }
          }
        }
      }
      // ROS_INFO("collecting points");  
      for(const VPoint &p:pcl_ground.points)
      {
        size_t i=static_cast<size_t>(p.ring);
        size_t j=static_cast<size_t>(p.intensity);
        if(p.z>0.5)
        {
          pcl_out->push_back(p);
        }else if((i<grid_cols)&&(j<grid_rows)&&(is_obstacle[i][j]))
        {
          pcl_out->push_back(p);
        }
      }
      pcl_out->height=1;
    }
  }
}

}
