/**
  *map_frame.cpp
  *brief:transform between gps and map
  *author:Chen Xiaofeng
  *date:20191028
  **/

#include "map_frame.h"
sensor_msgs::NavSatFix MapFrame::MapFrame2GPS(){
    double lat = 360.0 / M_PI * atan(exp((y+OFFSET_Y)/(SCALE * EARTH_RAD_EQ)))-90.0;
    double lon = 180.0 * (x + OFFSET_X) / (SCALE * EARTH_RAD_EQ * M_PI);
    sensor_msgs::NavSatFix gps;
    gps.latitude = lat;
    gps.longitude = lon;
    return gps;
}
void MapFrame::GPS2MapFrame(const sensor_msgs::NavSatFix &gps){
    x = SCALE * EARTH_RAD_EQ * gps.longitude * M_PI / 180.0 - OFFSET_X;
    y = SCALE * EARTH_RAD_EQ * log(tan((90.0 + gps.latitude) * (M_PI / 360.0))) - OFFSET_Y;
}
double MapFrame::deadReckoning(const double &travel_distance,const double &yaw_before, const double &turn_radian){
    double yaw = yaw_before + turn_radian/2;
    x += travel_distance * cos(yaw);
    y += travel_distance * sin(yaw);
    yaw += turn_radian/2;
    return yaw;
}
double MapFrame::calcDistance(const MapFrame &another){
    return sqrt(pow(x - another.x,2) + pow(y - another.y,2));
}
