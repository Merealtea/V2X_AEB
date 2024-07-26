/**
  *top.h
  *brief:main of node
  *author:Chen Xiaofeng
  *date:20191028
  **/

#include "top.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization_filter");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    Top top(nh,nh_priv);
    ros::spin();
    return 0;
}