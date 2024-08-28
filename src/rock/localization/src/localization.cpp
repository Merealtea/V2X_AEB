#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <hycan_msgs/Localization.h>
#include "common/wgs84_to_utm.h"

class VehicleLocalization
{
public:
    VehicleLocalization() 
    {
        gps_sub = nh_.subscribe("/Inertial/gps/fix", 10, &VehicleLocalization::gps_callback, this);
        imu_sub = nh_.subscribe("/Inertial/imu/data", 10, &VehicleLocalization::imu_callback, this);

        // 发布自定义消息
        pub_ = nh_.advertise<hycan_msgs::Localization>("rock_utm_localization", 1);
    }

    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
        double q0 = msg->orientation.w;
        double q1 = msg->orientation.x;
        double q2 = msg->orientation.y;
        double q3 = msg->orientation.z;
        // Calculate yaw from quaternion
        imu_yaw = atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));
    }

    void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        if (last_gps_valid) {
            calculate_heading(last_gps, *msg);
            convert_to_utm(*msg);
            hycan_msgs::Localization localization_msg;

            localization_msg.utm_x = vhx;
            localization_msg.utm_y = vhy;
            localization_msg.heading = corrected_heading;

            localization_msg.header = msg.get()->header; 
            pub_.publish(localization_msg);
        }
        last_gps = *msg;
        last_gps_valid = true;
    }

    void convert_to_utm(const sensor_msgs::NavSatFix& gps) {
        char* zone = nullptr;
        LLtoUTM(gps.latitude, gps.longitude, vhx, vhy, zone);
        ROS_INFO("UTM Coordinates: Easting %f, Northing %f", vhx, vhy);
    }

    void calculate_heading(const sensor_msgs::NavSatFix& last_gps, const sensor_msgs::NavSatFix& current_gps) {
        double lon1 = last_gps.longitude * RADIANS_PER_DEGREE;
        double lat1 = last_gps.latitude * RADIANS_PER_DEGREE;
        double lon2 = current_gps.longitude * RADIANS_PER_DEGREE;
        double lat2 = current_gps.latitude * RADIANS_PER_DEGREE;
        
        double dLon = lon2 - lon1;
        double y = sin(dLon) * cos(lat2);
        double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
        double gps_heading = atan2(y, x);

        double corrected_heading = gps_heading + imu_yaw;
        // Convert radians to degrees
        corrected_heading = corrected_heading * DEGREES_PER_RADIAN;

        ROS_INFO("Corrected Heading: %f degrees", corrected_heading);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber gps_sub, imu_sub;
    bool last_gps_valid = false;
    sensor_msgs::NavSatFix last_gps;

    double imu_yaw = 0.0;
    double corrected_heading = 0.0;
    double vhx = 0.0, vhy = 0.0;
    
    ros::Publisher pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rock_image_processor");
    VehicleLocalization ip;
    ros::spin();
    return 0;
}