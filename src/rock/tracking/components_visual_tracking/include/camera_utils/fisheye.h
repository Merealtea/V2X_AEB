// #ifndef FISHEYE_H
// #define FISHEYE_H
#pragma once

#include <iostream>

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui.hpp>

class Fisheye
{
    public:
        Fisheye(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle);
    private:
        cv::Mat K;
        cv::Mat D;
}

// #endif