#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <opencv2/highgui/highgui.hpp>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensing_driver");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/driver/fisheye/front", 1);

    cv::VideoCapture cap(1);
    ROS_INFO("b");
    if (!cap.isOpened())
    {
        ROS_ERROR("Open camera failed!");
        return -1;
    }
    cv::Mat frame;
    sensor_msgs::ImagePtr img_msg;
    std_msgs::Header header;

    ros::Rate loop_rate(30);
    while (nh.ok())
    {
        ROS_INFO("c");
        cap >> frame;
        ROS_INFO("D");
        header.stamp = ros::Time::now();
        if (!frame.empty())
        {
            ROS_INFO("E");
            img_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
            ROS_INFO("F");
            pub.publish(img_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}