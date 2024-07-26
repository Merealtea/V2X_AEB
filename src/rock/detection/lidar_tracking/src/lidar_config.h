/**
  *lidar_config.h
  *brief: base configuration of lidar
  *author:Jianlin Zhang
  *date:20180817
  **/

#ifndef LIDAR_CONFIG_H
#define LIDAR_CONFIG_H
#define _USE_MATH_DEFINES //use constant number defined in math.h
#include <cmath>
#include <ros/ros.h>
#include "velodyne.h"

namespace Lidar
{
struct Config
{
  double theta_max;
  double theta_min;
  double range_max;
  double range_min;
  bool is_pcl_orderd;  //if pcl is ordered, height will NOT be 1
}; //struct Config

inline bool isfinite(const VPoint &p)
{
  return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);
} //inline bool isfinite(const VPoint &p)

inline void select_by_theta(const VPointCloud::ConstPtr &pcl_in,VPointCloud::Ptr pcl_out,const Config &config)
{
  pcl_out->header.frame_id=pcl_in->header.frame_id;
  if(config.is_pcl_orderd)
  {
    size_t j_min=static_cast<size_t>(config.theta_min*pcl_in->width/360);
    size_t j_max=static_cast<size_t>(config.theta_max*pcl_in->width/360);
    for(size_t i=0;i<pcl_in->height;++i)
    {
      for(size_t j=j_min;j<j_max;++j)
      {
        pcl_out->points.push_back(pcl_in->at(j,i));
      }
    }
    pcl_out->width=j_max-j_min;
    pcl_out->height=pcl_in->height;
  }
  else
  {
    double theta_min=config.theta_min*M_PI/180;
    double theta_max=config.theta_max*M_PI/180;
    for(const VPoint &p:pcl_in->points)
    {
      if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z))
      {
        double theta=atan2(p.x,p.y); //unit: degree
        if(theta<0)
        {
          theta+=M_PI*2;
        }
        if((theta>theta_min)&&(theta<theta_max))
        {
          pcl_out->points.push_back(p);
        }
      }
    }
    pcl_out->width=pcl_in->points.size();
    pcl_out->height=1;
  }
} //inline void select_by_theta(const VPointCloud::ConstPtr &pcl_in,VPointCloud::Ptr pcl_out,const Config &config)

inline void select_by_range(const VPointCloud::ConstPtr &pcl_in,VPointCloud::Ptr pcl_out,const Config &config)
{
  pcl_out->header.frame_id=pcl_in->header.frame_id;
  if(config.is_pcl_orderd)
  {
    VPoint p_nan;
    p_nan.x=NAN;
    p_nan.y=NAN;
    p_nan.z=NAN;
    for(size_t i=0;i<pcl_in->height;++i)
    {
      for(size_t j=0;pcl_in->width;++j)
      {
        if(isfinite(pcl_in->at(j,i)))
        {
          double range2=pcl_in->at(j,i).x*pcl_in->at(j,i).x+pcl_in->at(j,i).y*pcl_in->at(j,i).y;
          if((range2>config.range_min*config.range_min)&&(range2<config.range_max*config.range_max))
          {
            pcl_out->points.push_back(pcl_in->at(j,i));
          }
          else
          {
            pcl_out->points.push_back(p_nan);
          }
        }
      }
    }
    pcl_out->width=pcl_in->width;
    pcl_out->height=pcl_in->height;
  }
  else
  {
    for(const VPoint &p:pcl_in->points)
    {
      if(isfinite(p))
      {
        double range2=p.x*p.x+p.y*p.y;
        if((range2>config.range_min*config.range_min)&&(range2<config.range_max*config.range_max))
        {
          pcl_out->points.push_back(p);
        }
      }
    }
    pcl_out->width=pcl_in->points.size();
    pcl_out->height=1;
  }
} //inline void select_by_theta(const VPointCloud::ConstPtr &pcl_in,VPointCloud::Ptr pcl_out,const Config &config)

inline void select_by_config(const VPointCloud::ConstPtr &pcl_in,VPointCloud::Ptr pcl_out,const Config &config)
{
  pcl_out->header.frame_id=pcl_in->header.frame_id;
  if(config.is_pcl_orderd)
  {
    VPoint p_nan;
    p_nan.x=NAN;
    p_nan.y=NAN;
    p_nan.z=NAN;
    size_t j_min=static_cast<size_t>(config.theta_min*pcl_in->width/360);
    size_t j_max=static_cast<size_t>(config.theta_max*pcl_in->width/360);
    for(size_t i=0;i<pcl_in->height;++i)
    {
      for(size_t j=j_min;j<j_max;++j)
      {
        if(isfinite(pcl_in->at(j,i)))
        {
          double range2=pcl_in->at(j,i).x*pcl_in->at(j,i).x+pcl_in->at(j,i).y*pcl_in->at(j,i).y;
          if((range2>config.range_min*config.range_min)&&(range2<config.range_max*config.range_max))
          {
            pcl_out->points.push_back(pcl_in->at(j,i));
          }
          else
          {
            pcl_out->points.push_back(p_nan);
          }
        }
        else
        {
          pcl_out->points.push_back(pcl_in->at(j,i));
        }
      }
    }
    pcl_out->width=j_max-j_min;
    pcl_out->height=pcl_in->height;
  }
  else
  {
    double theta_min=config.theta_min*M_PI/180;
    double theta_max=config.theta_max*M_PI/180;
    for(const VPoint &p:pcl_in->points)
    {
      if(isfinite(p))
      {
        double range2=p.x*p.x+p.y*p.y;
        if((range2>config.range_min*config.range_min)&&(range2<config.range_max*config.range_max))
        {
          double theta=atan2(p.x,p.y); //unit: degree
          if(theta<0)
          {
            theta+=M_PI*2;
          }
          if((theta>theta_min)&&(theta<theta_max))
          {
            pcl_out->points.push_back(p);
          }
        }
      }
    }
    pcl_out->width=pcl_in->points.size();
    pcl_out->height=1;
  }
} //inline void select_by_config(const VPointCloud::ConstPtr &pcl_in,VPointCloud::Ptr pcl_out,const Config &config)

} //namespace Lidar
#endif