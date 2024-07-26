/**
  *top.h
  *brief:top layer of software
  *author:Jianlin Zhang
  *date:20171031
  **/

#define _USE_MATH_DEFINES //use constant number defined in math.h
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>
#include "vehicle.h"
#include "mapping.h"
#include "control/ParamConfig.h"
#include "control.h"
#include "track_smoother.h"
#include "common/log.h"
#include "common/math/filters/digital_filter.h"
#include "common/math/filters/digital_filter_coefficients.h"

#include <cyber_msgs/SpeedFeedback.h>
#include <cyber_msgs/SteerFeedback.h>
#include <cyber_msgs/speedcmd.h>
#include <cyber_msgs/steercmd.h>
#include <cyber_msgs/brakecmd.h>
#include <cyber_msgs/V2VPacket.h>
#include <cyber_msgs/LocalizationEstimate.h>
#include <cyber_msgs/LocalTrajList.h>
#include <cyber_msgs/PlatoonControlDebug.h>

namespace Top
{
class Top
{
public:
  Top(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle);
  ~Top(){};

private:
  // for self localization
  bool flag_enable_smooth = true;
  double self_speed = 0.0;
  double t_localization = 0.0;
  bool init_localization = false;
  double t_v2v = 0.0;
  bool init_v2v = false;

  bool target_lost=true;
  bool behind_lost=true;
  double wheelbase = 2.56;
  double speed_cmd_min = 0.0;
  double brake_cmd_max = 4.0;
  double front_distance = 0.0;
  double front_distance_expect = 0.0;
  double front_distance_expect_new = 0.0;
  double behind_distance_expect = 0.0;
  double front_speed = 0.0;
  double front_speed_V2V = 0.0;
  double behind_speed = 0.0;
  double target_acc = 0.0;
  bool v2v_updated = false;
  bool consider_behind = false;
  bool start_mode_flag = true;

  double dis_base = 4.0;
  double dis_speed_factor = 0.25 ;
  double dis_max = 8.0;
  double dis_min = 4.0;
  double dis_offset = 3.55; //base_link to front offset for distace calcualtion

  double k_acc_filter = 0.1; //Filter coef for V2V target acc data
  double k_resmooth_threshold = 2.0; //distance to last smoothed track end before another smooth
  double last_smooth_s = 0.0; //index of smoothed track ending
  double v2v_delay = 0.0;
  double stop_time_count = ros::Time::now().toSec();

  Vehicle::Vehicle vehicle;
  Mapping::Mapping mapping;
  Control::Control control;

  std::string vehicle_frame_id;
  std::string map_frame_id;
  std::string target_frame_id;
  std::string behind_frame_id;
  tf::TransformBroadcaster tf_broadcaster;

  ros::Timer timer;

  ros::Subscriber imu_subscriber;
  ros::Subscriber steer_subscriber;
  ros::Subscriber target_pose_lidar_subscriber; //receive target pose from lidar tracking node
  ros::Subscriber target_local_pose_subscriber; //receive target relative pose from tracking node
  ros::Subscriber behind_pose_lidar_subscriber;
  ros::Subscriber target_speed_subscriber; //receive target current speed
  ros::Subscriber estimated_speed_subscriber; //receive target estimated speed
  ros::Subscriber behind_speed_subscriber; //receive behind estimated speed
  ros::Subscriber self_speed_subscriber; //receive self current speed
  ros::Subscriber target_lost_subscriber; //receive target lost
  ros::Subscriber behind_lost_subscriber; //receive behind lost
  ros::Subscriber rematch_subscriber;
  ros::Subscriber target_v2v_subscriber;
  ros::Subscriber localization_subscriber;

  ros::Publisher local_pose_publisher;
  ros::Publisher localization_publisher;
  ros::Publisher front_traj_publisher; //publish front trajectory for control
  ros::Publisher vehicle_track_draw_publisher; //draw vehicle track in rviz with line list
  ros::Publisher map_draw_publisher; //draw goal point track in rviz with line strip
  ros::Publisher target_track_draw_publisher;
  ros::Publisher front_track_draw_publisher;
  ros::Publisher front_track_smoothed_draw_publisher;
  ros::Publisher front_track_interpolated_draw_publisher;
  ros::Publisher behind_track_draw_publisher;
  ros::Publisher preview_draw_publisher;
  ros::Publisher pure_pursuit_draw_publisher; //draw pure pursuit result circle
  ros::Publisher steer_publisher;
  ros::Publisher speed_command_publisher;
  ros::Publisher brake_command_publisher;
  ros::Publisher ego_speed_publisher;
  ros::Publisher front_distance_publisher;
  ros::Publisher curve_distance_publisher;
  ros::Publisher behind_distance_publisher;
  ros::Publisher front_distance_expect_publisher;
  ros::Publisher behind_distance_expect_publisher;
  ros::Publisher steercmd_display_publisher;
  ros::Publisher speedcmd_display_publisher;
  ros::Publisher brakecmd_display_publisher;
  ros::Publisher control_debug_publisher;

  dynamic_reconfigure::Server<control::ParamConfig> server;

  cyberc3::common::math::DigitalFilter v2v_acc_filter_;

  void config_callback(control::ParamConfig &config, uint32_t level);
  void timer_callback(const ros::TimerEvent &event);
  void steer_callback(const cyber_msgs::SteerFeedback::ConstPtr &rec);
  void target_pose_lidar_callback(const geometry_msgs::Pose2D::ConstPtr &rec);
  void target_local_pose_callback(const geometry_msgs::Pose2D::ConstPtr &rec);
  void behind_pose_lidar_callback(const geometry_msgs::Pose2D::ConstPtr &rec);
  void estimated_speed_callback(const std_msgs::Float32::ConstPtr &rec);
  void behind_speed_callback(const std_msgs::Float32::ConstPtr &rec);
  void self_speed_callback(const cyber_msgs::SpeedFeedback::ConstPtr &rec);
  void target_lost_callback(const std_msgs::Bool::ConstPtr &rec);
  void behind_lost_callback(const std_msgs::Bool::ConstPtr &rec);
  void RematchCallback(const std_msgs::Bool::ConstPtr &rec);
  void target_v2v_callback(const cyber_msgs::V2VPacket::ConstPtr &rec);
  void localization_callback(const cyber_msgs::LocalizationEstimate::ConstPtr &rec);
  double calc_distance_expect(double curr_speed);
  double calc_distance_expect_by_time(double relative_time);
  void check_start_mode_flag(const double current_speed, const double target_s,
                             const double current_s);
};
}
