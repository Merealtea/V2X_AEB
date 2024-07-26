/**
  *common.h
  *brief: common structs
  *author:Chen Xiaofeng
  *date:20201021
  **/

#ifndef COMMON_H
#define COMMON_H

struct TrackPoint
{
    double x_;
    double y_;
    double theta_;
    double curve_distance_;
    double kappa_;
    double t_; //relative time from begining of tracking
    TrackPoint(double x = 0.0, double y = 0.0, double theta = 0.0, double curve_distance = 0.0, double kappa = 0.0,double t = 0.0) :
        x_(x), y_(y), theta_(theta), curve_distance_(curve_distance), kappa_(kappa), t_(t){}
};


#endif
