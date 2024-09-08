/**
  *main.cpp
  *brief:main of node
  *author:Jianlin Zhang
  *date:20180817
  **/

#include "speed_controller_test.h"

int main(int argc, char ** argv)
{
  ros::init(argc,argv,"rock_control_node");
  ros::NodeHandle node_handle;
  ros::NodeHandle privete_node_handle("~");

  SpeedControllerTest speed_controller_test(&node_handle);

  ros::spin();
  return 0;
}
