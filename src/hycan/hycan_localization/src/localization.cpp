#include <ros/ros.h>
#include <cyber_msgs/Heading.h>
#include <sensor_msgs/NavSatFix.h>
#include <hycan_msgs/Localization.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "common/wgs84_to_utm.h"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, cyber_msgs::Heading> MySyncPolicy;

class VehicleLocalization
{
public:
    VehicleLocalization() 
    {
        // 订阅GPS和Heaing
        gps_sub.subscribe(nh_, "/strong/fix", 1);
        heading_sub.subscribe(nh_, "/strong/heading", 1);

        // 同步策略
        sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), gps_sub, heading_sub));
        sync_->registerCallback(boost::bind(&VehicleLocalization::localization_callback, this, _1, _2));

        // 发布自定义消息
        pub_ = nh_.advertise<hycan_msgs::Localization>("hycan_utm_localization", 1);
    }

    void localization_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg,
                                 const cyber_msgs::Heading::ConstPtr& heading_msg) {
        double st = ros::Time::now().toSec();
        convert_to_utm(*gps_msg);
        double delta_lon = (gps_msg->longitude - central_meridian) * RADIANS_PER_DEGREE;
        double convergence_angle = delta_lon * sin(gps_msg->latitude * RADIANS_PER_DEGREE);
        // convergence_angle *= DEGREES_PER_RADIAN;

        double corrected_heading = heading_msg->data + M_PI;// + convergence_angle;
        ROS_INFO("Corrected Heading: %f radians", corrected_heading);

        hycan_msgs::Localization localization_msg;
        localization_msg.utm_x = vhx;
        localization_msg.utm_y = vhy;
        localization_msg.heading = corrected_heading;

        localization_msg.header = gps_msg.get()->header;
        pub_.publish(localization_msg);
        ROS_INFO("Published UTM Localization, Process Time: %6f", (ros::Time::now().toSec() - st));
    }

    void convert_to_utm(const sensor_msgs::NavSatFix& gps) {
        char* zone = nullptr;
        LLtoUTM(gps.latitude, gps.longitude, vhy, vhx, zone);
        ROS_INFO("UTM Coordinates: Easting %f, Northing %f", vhx, vhy);
    }

private:
    ros::NodeHandle nh_;
    boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub;
    message_filters::Subscriber<cyber_msgs::Heading> heading_sub;

    double vhx, vhy;
    const double central_meridian = 120;
    ros::Publisher pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hycan_image_processor");
    VehicleLocalization ip;
    ros::spin();
    return 0;
}