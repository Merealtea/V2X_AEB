/**
	*vehicle.cpp
	*brief: update vehicle status and control vehicle
	*author:Jianlin Zhang
	*date:20170608
	**/

#include "vehicle.h"

namespace Vehicle
{
Vehicle::Vehicle(ros::NodeHandle nh, ros::NodeHandle pnh)
{
  pnh.param("wheelbase", wheelbase, 2.56);
  pnh.param("steer_ratio", steer_ratio, 15.5);

  pnh.param("speed_cmd_max", speed_cmd_max, 3000.0);
  pnh.param("max_track_size", max_track_size, 3000);

  pnh.param("vehicle_frame_id", vehicle_frame_id, std::string("vehicle"));
  pnh.param("map_frame_id", map_frame_id, std::string("map"));
}

void Vehicle::update_track(void)
{
  geometry_msgs::Pose2D p;
  p.x=x;
  p.y=y;
  p.theta=theta;
  track.push_back(p);
  if(track.size()>max_track_size)
  {
    for(size_t i=0;i<max_track_size/2;++i)
    {
      track.pop_front();
    }
  }
}

void Vehicle::clear_track()
{
  track.clear();
}

void Vehicle::reset_local_pose()
{
  x = 0;
  y = 0;
  theta = 0;
}

void Vehicle::draw_track(ros::Publisher &track_draw_publisher)
{
  if(track.size()>0)
  {
    geometry_msgs::PoseArray::Ptr output(new geometry_msgs::PoseArray());
    for(geometry_msgs::Pose2D &pose:track)
    {
      geometry_msgs::Pose p;
      p.position.x=pose.x;
      p.position.y=pose.y;
      p.orientation.w=cos(pose.theta/2);
      p.orientation.x=0;
      p.orientation.y=0;
      p.orientation.z=sin(pose.theta/2);
      output->poses.push_back(p);
    }
    output->header.frame_id = map_frame_id;
    output->header.stamp = ros::Time::now();
    track_draw_publisher.publish(output);
  }
}

}
