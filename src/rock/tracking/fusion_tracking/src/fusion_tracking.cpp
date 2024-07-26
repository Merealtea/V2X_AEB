#include <iostream>
#include <math.h>
#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_ros/impl/transforms.hpp>

#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <cyber_msgs/Object.h>
#include <cyber_msgs/ObjectArray.h>

struct PixelCoord
{
    int u;
    int v;
    
    PixelCoord(int u_init, int v_init): u(u_init), v(v_init){}
    PixelCoord(): u(), v(){}
};

struct Point
{
    float x;
    float y;
    
    Point(float x_init, float y_init): x(x_init), y(y_init){}
    Point(): x(), y(){}
};

class FusionTracker
{
    private:
        ros::NodeHandle nh_;
        ros::Subscriber sub_radar;
        ros::Subscriber sub_image;
        // ros::Subscriber sub_tracked_image;
        ros::Subscriber sub_relative_pose;
        ros::Subscriber sub_front_local_speed;
        ros::Subscriber sub_rematch;
        ros::Publisher pub_;
        ros::Publisher marker_pub;
        ros::Publisher filtered_pc_pub;
        // image_transport::Publisher tmp_pub;
        ros::Timer timer;

        tf::TransformListener radar2vehicle_tf_listener;

        cyber_msgs::ObjectArray curr_obj_array;
        pcl::PointCloud<pcl::PointXYZINormal> curr_radar_pc;
        pcl::PointCloud<pcl::PointXYZINormal> filtered_radar_pc;
        cv::Mat tracked_image;
        geometry_msgs::Pose2D curr_front_local_pose;
        geometry_msgs::Vector3 curr_front_local_velo;
        bool is_obj_array_existed = false;
        bool is_radar_cluster_existed = false;
        bool is_tracked_image_existed = false;
        int counter = 0;
        cyber_msgs::Object last_match;

        // parameters
        const float x_c_r = 3.55 - 2.05, y_c_r = 0.40;
        const int dim_1 = 1200, dim_2 = 1920;
        Eigen::MatrixXd intrinsics_matrix = Eigen::MatrixXd(3, 3);
        Eigen::VectorXd distortion_coeffs = Eigen::VectorXd(5);

        void fusion();
        cv::Mat undistort_image(const cv::Mat &obj_mask);
        Eigen::Vector3d map_to_camera_cood(const Eigen::Vector3d &pix_coord);
        void radar_filter();    // filter with ROI and speed
        
        void visualize_image_cone(const float &k);
        void visualize_fusion_result(const float &x, const float &y);

    public:
        FusionTracker();
        void radar_callback(const sensor_msgs::PointCloud2ConstPtr &msg);
        void image_callback(const cyber_msgs::ObjectArray &msg);
        // void tracked_image_callback(const sensor_msgs::ImageConstPtr &msg);
        void front_local_pose_callback(const geometry_msgs::Pose2D &msg);
        void front_local_velo_callback(const geometry_msgs::Vector3 &msg);
        void rematch_callback(const std_msgs::Bool &msg);
        void timer_callback(const ros::TimerEvent& timer_event);
        // void publish_fusion_tracking();
};

FusionTracker::FusionTracker()
{
    sub_radar = nh_.subscribe("/radar_pc", 1, &FusionTracker::radar_callback, this);
    sub_image = nh_.subscribe("/visual_track", 1, &FusionTracker::image_callback, this);
    // sub_tracked_image = nh_.subscribe("/visual_tracking/tracked_image", 1, &FusionTracker::tracked_image_callback, this);
    sub_relative_pose = nh_.subscribe("/tracking/front_vehicle/local_pose", 1, &FusionTracker::front_local_pose_callback, this);
    sub_front_local_speed = nh_.subscribe("/tracking/front_vehicle/local_velo", 1, &FusionTracker::front_local_velo_callback, this);
    sub_rematch = nh_.subscribe("/xboxone/rematch", 1, &FusionTracker::rematch_callback, this);
    pub_ = nh_.advertise<cyber_msgs::Object>("/fusion_tracking/object", 1);
    marker_pub = nh_.advertise<visualization_msgs::Marker>("/fusion_tracking/cone", 1);
    filtered_pc_pub = nh_.advertise<sensor_msgs::PointCloud2>("/tracking/filtered_radar_pc", 1);

    image_transport::ImageTransport image_trans(nh_);
    // tmp_pub = image_trans.advertise("/undistort", 1);

    timer = nh_.createTimer(ros::Duration(0.1), &FusionTracker::timer_callback, this);

    last_match.pose.position.x = last_match.pose.position.y = last_match.pose.position.z = 0.0;
    last_match.velocity.linear.x = last_match.velocity.linear.y = last_match.velocity.linear.z = 0.0;

    intrinsics_matrix << 1201.6, 0.0, 953.94, 
        0.0, 1211.80, 636.9038, 
        0.0, 0.0, 1.0;
    distortion_coeffs << -0.1430, 0.0383, 0.0027, -0.0012, 0.0;
    ros::spin();
}

void FusionTracker::radar_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    sensor_msgs::PointCloud2 transformed_msg;
    pcl_ros::transformPointCloud("base_link", *msg, transformed_msg, radar2vehicle_tf_listener);
    pcl::fromROSMsg(transformed_msg, curr_radar_pc);
    is_radar_cluster_existed = true;
}

void FusionTracker::image_callback(const cyber_msgs::ObjectArray &msg)
{
    curr_obj_array = msg;
    is_obj_array_existed = true;
}

// void FusionTracker::tracked_image_callback(const sensor_msgs::ImageConstPtr &msg)
// {
//     cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::TYPE_8UC3);
//     tracked_image = cv_ptr -> image;
//     is_tracked_image_existed = true;
// }

void FusionTracker::front_local_pose_callback(const geometry_msgs::Pose2D &msg)
{
    curr_front_local_pose = msg;
    // ROS_INFO("local pose callback !!!!!!!");
}

void FusionTracker::front_local_velo_callback(const geometry_msgs::Vector3 &msg)
{
    curr_front_local_velo = msg;
    // ROS_INFO("local velo callback !!!!!!!");
}

void FusionTracker::rematch_callback(const std_msgs::Bool &msg)
{
    if (msg.data)
    {
        ROS_WARN("-------------Fusion Tracking Rematch-------------");
        counter = 0;
    }
}

void FusionTracker::fusion()
{   
    // std::cout << "fusion" << std::endl;
    ros::Time start_time = ros::Time::now();
    if ((not is_obj_array_existed) or (not is_radar_cluster_existed)) return;

    cv::Mat obj_mask = cv::Mat::zeros(dim_1, dim_2, CV_8UC1);
    for (const auto &object:curr_obj_array.objects)
    {
        int u = object.pose.position.x; // column index
        int v = object.pose.position.y; // row index
        // std::cout << u << " " << v << std::endl;
        obj_mask.at<uint8_t>(v, u) = 255;
        // for (int ii = -5; ii < 5; ii++)
        //     for (int jj = -5; jj < 5; jj++)
        //         obj_mask.at<uint8_t>(v + ii, u + jj) = 255;
    }

    // cv::Mat undistort_tracked_image = FusionTracker::undistort_image(tracked_image);
    cv::Mat undistort_obj_mask = FusionTracker::undistort_image(obj_mask);

    cv::Mat non_zero_pos;
    cv::findNonZero(undistort_obj_mask, non_zero_pos);
    std::vector<PixelCoord> non_zero_pixel_list;
    for (int i = 0; i < non_zero_pos.total(); i++) non_zero_pixel_list.push_back(PixelCoord(non_zero_pos.at<cv::Point>(i).x, non_zero_pos.at<cv::Point>(i).y));
    //filter
    // std::cout << "filter" << std::endl;
    std::vector<PixelCoord> filtered_pixel_list;
    for (auto& pix:non_zero_pixel_list)
    {
        if (filtered_pixel_list.size() == 0) filtered_pixel_list.push_back(PixelCoord(pix.u, pix.v));
        else
        {
            bool flag = true;
            for (auto filtered_pix:filtered_pixel_list)
            {
                if (abs(pix.u - filtered_pix.u) < 3 and abs(pix.v - filtered_pix.v) < 3)
                {
                    flag = false;
                    break;
                }
            }
            if (flag) filtered_pixel_list.push_back(PixelCoord(pix.u, pix.v));
        }
    }
    // sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", undistort_obj_mask).toImageMsg();
    // tmp_pub.publish(img_msg);

    // map to camera coord
    // std::cout << "map " << std::endl;
    std::vector<Eigen::Vector2d> obj_cam_coord_list;
    for (auto& pix:filtered_pixel_list)
    {
        Eigen::Vector3d pix_coord {{(float)pix.u}, {(float)pix.v}, {1.0}};
        Eigen::Vector3d cam_coord = FusionTracker::map_to_camera_cood(pix_coord);
        // std::cout << cam_coord << std::endl;
        // cam_coord(0) = -cam_coord(0);   // align with radar x axis orientation
        obj_cam_coord_list.push_back(cam_coord.head(2));
    }

    // fusion
    // std::cout << "match " << std::endl;
    if (counter >= 10) FusionTracker::radar_filter();
    for (auto& coord: obj_cam_coord_list)
    {
        std::vector<pcl::PointXYZINormal> point_candidates;
        float k = -coord(0); // slope, z_c = 1.0, so k = -x
        FusionTracker::visualize_image_cone(k);
        // std::cout << k << std::endl;
        // ROS_INFO("counter: %d", counter);
        pcl::PointCloud<pcl::PointXYZINormal> pc_to_tracked = (counter >= 10)? filtered_radar_pc : curr_radar_pc;
        ROS_INFO("pc size: %d", pc_to_tracked.size());
        for (pcl::PointCloud<pcl::PointXYZINormal>::const_iterator it = pc_to_tracked.begin(); it != pc_to_tracked.end(); it++)
        {
            if (abs(it -> y - y_c_r - k * (it -> x - x_c_r)) / (sqrt(pow(k, 2) + 1.0)) < 2.0 and it -> intensity > -10.0)
            {
                point_candidates.push_back(*it);
            }
        }
        //find the best point
        // std::cout << point_candidates.size() << std::endl;
        float min_l1_dis = 1000.0;
        int min_idx = -1;
        for (int i = 0; i < point_candidates.size(); i++)
        {
            if (fabs(point_candidates[i].x) + fabs(point_candidates[i].y) < min_l1_dis)
            {
                min_l1_dis = fabs(point_candidates[i].x) + fabs(point_candidates[i].y);
                min_idx = i;
            }
        }
        // std::cout << min_idx << std::endl;
        cyber_msgs::Object match;
        if (min_idx != -1)
        {
            match.pose.position.x = point_candidates[min_idx].x;
            match.pose.position.y = point_candidates[min_idx].y;
            match.velocity.linear.x = point_candidates[min_idx].normal_x;
            match.velocity.linear.y = point_candidates[min_idx].normal_y;

            if (match.pose.position.x > 30.0 or fabs(match.pose.position.y) > 15.0)
                match = last_match;
            // amplitude limit filter
            double speed = sqrt(pow(match.velocity.linear.x, 2) + pow(match.velocity.linear.y, 2));
            if (counter >= 10)
                if (sqrt(pow(match.pose.position.x - last_match.pose.position.x, 2) + pow(match.pose.position.y - last_match.pose.position.y, 2)) > 0.12 * speed)
                {
                    match.pose.position.x = 0.5 * (match.pose.position.x + last_match.pose.position.x);
                    match.pose.position.y = 0.5 * (match.pose.position.y + last_match.pose.position.y);
                    match.velocity.linear.x = 0.5 * (match.velocity.linear.x + match.velocity.linear.x);
                    match.velocity.linear.y = 0.5 * (match.velocity.linear.y + match.velocity.linear.y);
                }
            FusionTracker::visualize_fusion_result(match.pose.position.x, match.pose.position.y);

            last_match = match;
            ROS_INFO("Match object position x: %f , y: %f", match.pose.position.x, match.pose.position.y);
            if (counter < 10) counter++;
        }
        else
        {
            match = last_match;
            // match.velocity.linear.x = match.velocity.linear.y = 0.0;    // 无目标时，使用上一帧前车位置，但速度置为0，以防止KalmanFilter CV模型的滤波结果持续向某个方向偏移
            FusionTracker::visualize_fusion_result(match.pose.position.x, match.pose.position.y);
            ROS_INFO("No match, last object position x: %f , y: %f", match.pose.position.x, match.pose.position.y);
        }
        match.pose.position.z = k;  // keep slope for kalman filter
        pub_.publish(match);
        ros::Time end_time = ros::Time::now();
        ros::Duration duration = end_time - start_time;
        ROS_INFO("Matching costs %f sec", duration.toSec());
    }

}

cv::Mat FusionTracker::undistort_image(const cv::Mat &to_be_map)
{
    cv::Mat undistort_image;
    cv::Mat intrinsics_matrix_cv;
    cv::eigen2cv(intrinsics_matrix, intrinsics_matrix_cv);
    cv::Mat distortion_coeffs_cv;
    cv::eigen2cv(distortion_coeffs, distortion_coeffs_cv);
    cv::undistort(to_be_map, undistort_image, intrinsics_matrix_cv,
        distortion_coeffs_cv);
    
    return undistort_image;
}

Eigen::Vector3d FusionTracker::map_to_camera_cood(const Eigen::Vector3d &pix_coord)
{
    double z_c = 1.0;
    Eigen::Vector3d cam_coord = z_c * intrinsics_matrix.inverse() * pix_coord;
    return cam_coord;
}

void FusionTracker::radar_filter()
{
    filtered_radar_pc.clear();
    ROS_INFO("roi_x: %f, roi_y: %f", curr_front_local_pose.x, curr_front_local_pose.y);
    for (pcl::PointCloud<pcl::PointXYZINormal>::const_iterator it = curr_radar_pc.begin(); it != curr_radar_pc.end(); it++)
    {
        if (sqrt(pow(it -> x - curr_front_local_pose.x, 2) + pow(it -> y - curr_front_local_pose.y, 2)) < 3.5)
        {
            float gamma = std::atan2(it -> y, it -> x);
            if (sqrt(pow(curr_front_local_velo.x - it -> normal_x, 2) + pow(curr_front_local_velo.y - it -> normal_y, 2)) < 2.0)
                filtered_radar_pc.push_back(*it);
        }
    }
    sensor_msgs::PointCloud2 filtered_radar_pc_msg;
    pcl::toROSMsg(filtered_radar_pc, filtered_radar_pc_msg);
    filtered_radar_pc_msg.header.frame_id = "base_link";
    filtered_radar_pc_msg.header.stamp = ros::Time::now();
    filtered_pc_pub.publish(filtered_radar_pc_msg);
}

void FusionTracker::timer_callback(const ros::TimerEvent& timer_event)
{
    fusion();
}

void FusionTracker::visualize_image_cone(const float &k)
{
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "base_link";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "lines";
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.id = 1;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.scale.x = 0.1;
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
    line_list.pose.orientation.w = 1.0;

    geometry_msgs::Point end_point;
    end_point.x = x_c_r;
    end_point.y = y_c_r;
    end_point.z = 0.0;
    line_list.points.push_back(end_point);
    float x_ = 30.0;
    float y_ = k * x_ + y_c_r - k * x_c_r;
    end_point.x = x_;
    end_point.y = y_;
    line_list.points.push_back(end_point);

    marker_pub.publish(line_list);
}

void FusionTracker::visualize_fusion_result(const float &x, const float &y)
{
    visualization_msgs::Marker point;
    point.header.frame_id = "base_link";
    point.header.stamp = ros::Time::now();
    point.ns = "point";
    point.type = visualization_msgs::Marker::SPHERE;
    point.id = 2;
    point.action = visualization_msgs::Marker::ADD;
    point.scale.x = 1.0;
    point.scale.y = 1.0;
    point.scale.z = 1.0;
    point.color.g = 1.0;
    point.color.a = 1.0;
    point.pose.orientation.x = point.pose.orientation.y = point.pose.orientation.z = point.pose.orientation.w = 0.0;
    point.pose.position.x = x;
    point.pose.position.y = y;
    point.pose.position.z = 0.0;

    marker_pub.publish(point);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "fusion_tracking_node");
    // ros::NodeHandle nh;
    FusionTracker fusion_tracker;

    // ros::spin();

    return 0;
}