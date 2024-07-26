/**
  *map_frame.h
  *brief:transform between gps and map
  *author:Chen Xiaofeng
  *date:20191028
  **/

#ifndef MAP_FRAME_H
#define MAP_FRAME_H
#include "sensor_msgs/NavSatFix.h"
#include <cmath>

// const double OFFSET_X = 11583000; // for SJTU
// const double OFFSET_Y = 3116000;
const double OFFSET_X = 11453000; // for Changshu
const double OFFSET_Y = 3160000; // for Changshu
const double EARTH_RAD_EQ = 1000 * 6378.137; //unit: m
// const double SCALE = cos(31.03 * M_PI / 180.0); // for SJTU
const double SCALE = cos(31.59 * M_PI / 180.0); // for Changshu

class MapFrame{
public:
    double x;//east
    double y;//north

    MapFrame(const double x_in=0.0,
             const double y_in=0.0):
        x(x_in),
        y(y_in){}

    MapFrame(const MapFrame &pos):
        x(pos.x),
        y(pos.y){}

    ~MapFrame(){}

    sensor_msgs::NavSatFix MapFrame2GPS();

    void GPS2MapFrame(const sensor_msgs::NavSatFix &gps);
    double deadReckoning(const double &travel_distance,const double &yaw_before, const double &turn_radian);
    double calcDistance(const MapFrame &another);
};

#endif
