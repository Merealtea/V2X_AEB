#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

class Listener {
public:
  Listener(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh), last_yaw_(0.0) {
    sub_imu_ = nh_.subscribe("/leader/wit/imu", 2, &Listener::imuCallback, this);
    pub_pose_ = nh_.advertise<std_msgs::Float32>("/imu/yaw", 2);
    pub_pose_x_ = nh_.advertise<std_msgs::Float32>("/imu/x", 2);
    pub_pose_y_ = nh_.advertise<std_msgs::Float32>("/imu/y", 2);
    pub_pose_z_ = nh_.advertise<std_msgs::Float32>("/imu/z", 2);
    pub_pose_w_ = nh_.advertise<std_msgs::Float32>("/imu/w", 2);
    pub_pose_diff_ = nh_.advertise<std_msgs::Float32>("/imu/diff", 2);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_imu_;
  ros::Publisher pub_pose_;
  ros::Publisher pub_pose_x_;
  ros::Publisher pub_pose_y_;
  ros::Publisher pub_pose_z_;
  ros::Publisher pub_pose_w_;
  ros::Publisher pub_pose_diff_;
  double last_yaw_;

  void imuCallback(const sensor_msgs::ImuConstPtr &msg) {
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y,
                      msg->orientation.z, msg->orientation.w);
    std_msgs::Float32 x, y, z, w;
    x.data = msg->orientation.x;
    y.data = msg->orientation.y;
    z.data = msg->orientation.z;
    w.data = msg->orientation.w;
    pub_pose_x_.publish(x);
    pub_pose_y_.publish(y);
    pub_pose_z_.publish(z);
    pub_pose_w_.publish(w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    std_msgs::Float32 yaw_msg, pitch_msg, roll_msg;
    yaw_msg.data = yaw;
    pub_pose_.publish(yaw_msg);


    std_msgs::Float32 yaw_diff_msg;
    yaw_diff_msg.data = yaw - last_yaw_;
    pub_pose_diff_.publish(yaw_diff_msg);

    last_yaw_ = yaw;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");
  Listener listener(node_handle, private_node_handle);
  ros::spin();
}