// #ifndef ROI_GENERATION_H
// #define ROI_GENERATION_H
#pragma once

#include <iostream>
#include <math.h>
#include <vector>

#include <XmlRpcValue.h>

#include <ros/ros.h>
#include <cyber_msgs/Object.h>
#include <cyber_msgs/Box2D.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PointStamped.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

// #include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

// 产生前车在图像（鱼眼或普通前视相机）的ROI
// 接收YOLOX + DeepSort，毫米波雷达点云；若YOLOX + DeepSort 给出的ROI存在，则返回该ROI；否则返回融合雷达后的ROI。

class ROIGenerator
{
    public:
        ROIGenerator(ros::NodeHandle nh, ros::NodeHandle pnh);
        void update_ROI2D(const cyber_msgs::Box2D &image_roi,
            bool image_roi_valid,
            const pcl::PointCloud<pcl::PointXYZINormal> &curr_radar_pc,
            cyber_msgs::Box2D &final_roi);      // 更新并返回图像中的ROI
    
    private:
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener listener;
        // parameters
        int img_width, img_height;
        cv::Mat K, D;
        bool is_fisheye;
        double cam_vh_x, cam_vh_y;

        cyber_msgs::Object curr_front;  // 前车当前相对位置（姿态）

        void map_to_image(const cyber_msgs::Object &vh_pose, cyber_msgs::Box2D &final_roi);    // 将3D ROI映射到图像上
        std::vector<double> map_imgpt_to_ray(const std::vector<cv::Point2d> &points);
        pcl::PointCloud<pcl::PointXYZINormal> filter_curr_pc(const pcl::PointCloud<pcl::PointXYZINormal> &curr_radar_pc);
};


// #endif