/**
  *main.cpp
  *brief:main of node
  *author:Jianlin Zhang
  *date:20180817
  **/

#include "top.h"
#include "common/utils/find_path.h"

int main(int argc, char ** argv)
{
  ros::init(argc,argv,"vehicle_node");
  ros::NodeHandle node_handle;
  ros::NodeHandle privete_node_handle("~");

  google::InitGoogleLogging("platoon_control");

  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  FLAGS_log_prefix = true;
  FLAGS_logbufsecs = 0;
  FLAGS_max_log_size = 1024;
  FLAGS_log_dir = expand_catkin_ws("/log/");

  Top::Top top(node_handle,privete_node_handle);
  ros::spin();
  return 0;
}
