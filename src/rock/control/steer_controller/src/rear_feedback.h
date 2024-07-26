#ifndef _CONTROL_H
#define _CONTROL_H

#include <ros/ros.h>
#include <queue>
#include <vector>

#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#include <cyber_msgs/LocalTrajList.h>
#include <cyber_msgs/LocalTrajPoint.h>
#include <cyber_msgs/LocalizationEstimate.h>
#include <tf/transform_datatypes.h>

#define STEER_ERROR_QUEUE_SIZE 100

#define WHEELBASE 2.5
#define WHEEL_MAX 550
#define WHEEL_ZERO 0

#define LITTLE 1E-14 

class RearFeedback{
public:
    RearFeedback();
    ~RearFeedback();

private:
//---------定义成员变量----------------
    struct TrajPoint
    {
        geometry_msgs::Pose point;
        int mode;
        TrajPoint(geometry_msgs::Pose point_in, int mode_in): point(point_in), mode(mode_in) {}
    };
    bool path_flag;

    int temp_point;
    double current_steer_angle;
    double temp_steer_error;
    double inter_steer_error;
    double former_steer_cmd;
    double temp_steer_cmd;
    double final_steer_cmd;
    std_msgs::Float64 wheel_output;
    std::queue <double> steer_error_queue;

    double ref_distance;
    double k_angle;
    double k_error;
    double Kp_error;
    double Ki_error;
    double filter_param;
    double Kp_wheel;

    double current_yaw;
    double current_ang;
    double current_vel;
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::PoseStamped reference_pose;
    geometry_msgs::PoseStamped target_pose;
    geometry_msgs::PoseArray target_path;

    cyber_msgs::LocalTrajList target_trajectory;

//--------定义订阅者和发布者-----------
    ros::NodeHandle nh;
    ros::Publisher stanley_pub_steer_cmd;
//    ros::Publisher pub_current_pose;
    ros::Publisher stanley_pub_reference_pose;
    ros::Publisher stanley_pub_target_pose;
    ros::Publisher stanley_pub_target_path ;
    ros::Subscriber sub_target_path;
    ros::Subscriber sub_local_trajectory;
    ros::Subscriber sub_target;
    ros::Subscriber sub_pose;

//---------定义成员函数----------------
    void localTrajectoryCallback(const cyber_msgs::LocalTrajList::ConstPtr& path_in);
    void poseCallback(const cyber_msgs::LocalizationEstimate::ConstPtr& pose_in);
};

#endif
