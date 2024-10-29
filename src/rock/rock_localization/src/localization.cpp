#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <hycan_msgs/Localization.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "common/wgs84_to_utm.h"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, sensor_msgs::Imu> MySyncPolicy;
                                                         

class VehicleLocalization
{
public:
    VehicleLocalization() 
    {
        gps_sub.subscribe(nh_, "/Inertial/gps/fix", 1);
        imu_sub.subscribe(nh_, "/Inertial/imu/data", 1);

        sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), gps_sub, imu_sub));
        sync_->registerCallback(boost::bind(&VehicleLocalization::localization, this, _1, _2));

        // 发布自定义消息
        pub_ = nh_.advertise<hycan_msgs::Localization>("rock_utm_localization", 1);
    }

    void localization(const sensor_msgs::NavSatFix::ConstPtr& gps_msg,
                                 const sensor_msgs::Imu::ConstPtr& imu_msg) {
        
        double vhx, vhy;

        // Get postition
        char* zone = nullptr;
        LLtoUTM(gps_msg->latitude, gps_msg->longitude, vhy, vhx, zone);
        ROS_INFO("UTM Coordinates: Easting %f, Northing %f", vhx, vhy);
        
        // Get Orientation
        double q0 = imu_msg->orientation.w;
        double q1 = imu_msg->orientation.x;
        double q2 = imu_msg->orientation.y;
        double q3 = imu_msg->orientation.z;

        // Calculate yaw from quaternion
        double imu_yaw = atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));
        imu_yaw = imu_yaw - M_PI * 50 / 64; // Correct for IMU orientation
        
        hycan_msgs::Localization localization_msg;
        localization_msg.utm_x = vhx;
        localization_msg.utm_y = vhy;
        localization_msg.heading = imu_yaw;

        // Set header
        localization_msg.header = gps_msg->header;
        pub_.publish(localization_msg);
    }


private:
    ros::NodeHandle nh_;
    boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub;
    
    ros::Publisher pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rock_image_processor");
    VehicleLocalization ip;
    ros::spin();
    return 0;
}