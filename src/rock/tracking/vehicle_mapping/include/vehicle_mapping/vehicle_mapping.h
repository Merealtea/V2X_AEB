#pragma once

#include <deque>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <cyber_msgs/SpeedFeedback.h>
#include <cyber_msgs/SteerFeedback.h>
#include <cyber_msgs/V2VPacket.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
// #include <sensor_msgs/NavSatFix.h>

namespace tracking {
class VehicleMapping {
public:
  VehicleMapping(ros::NodeHandle node_handle,
                 ros::NodeHandle private_node_handle);
private:
    ros::NodeHandle nh_, pnh_;
    ros::Timer timer_;
    ros::Subscriber sub_speed_;
    ros::Subscriber sub_steer_;
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_imu_wit_;
    // ros::Subscriber sub_gps_wit_;
    ros::Subscriber sub_v2v_;
    ros::Subscriber sub_leader_wit_imu_;
    ros::Publisher pub_track_vis_;
    ros::Publisher pub_target_track_vis_;
    ros::Publisher pub_orientation_debug_;
    ros::Publisher pub_front_speed_;

    tf2_ros::TransformBroadcaster br_;

    double timestamp_;

    geometry_msgs::Pose2D pose_;
    bool ego_init_;
    double init_yaw_;
    double last_yaw_;
    double curr_speed_;
    double curr_angular_velo_;
    double curr_angular_velo_wit_;

    geometry_msgs::Pose2D target_pose_;
    double target_curr_speed_;
    double target_curr_angular_velo_;

    std::deque<geometry_msgs::Pose2D> ego_pose_buffer_;
    std::deque<geometry_msgs::Pose2D> target_pose_buffer_;

    void speedCallback(const cyber_msgs::SpeedFeedback::ConstPtr &msg);
    void steerCallback(const cyber_msgs::SteerFeedback::ConstPtr &msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void V2VCallback(const cyber_msgs::V2VPacketConstPtr &msg);
    void imuWitCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void leaderImuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    // void gpsWitCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);
    void mapping(const ros::TimerEvent &event);
};
} // namespace tracking
