#include "roi_generation/roi_generation.h"

ROIGenerator::ROIGenerator(ros::NodeHandle nh, ros::NodeHandle pnh): listener(tf_buffer)
{
    // load camera intrinsic params
    pnh.param<bool>("is_fisheye", is_fisheye, true);
    std::vector<double> K_vec;
    std::vector<double> D_vec;
    pnh.getParam("cam_K", K_vec);
    pnh.getParam("cam_D", D_vec);
    K = cv::Mat(K_vec); K.resize(3);
    D = cv::Mat(D_vec);
    if (is_fisheye) D.resize(4);
    else D.resize(5);

    pnh.param<int>("image_width", img_width, 1920);
    pnh.param<int>("image_height", img_height, 1200);
    
    pnh.param<double>("cam_vh_x", cam_vh_x, 3.55);
    pnh.param<double>("cam_vh_y", cam_vh_y, 0.0);
}

void ROIGenerator::update_ROI2D(const cyber_msgs::Box2D &image_roi,
    bool image_roi_valid,
    const pcl::PointCloud<pcl::PointXYZINormal> &curr_radar_pc, // radar in base_link frame
    cyber_msgs::Box2D &final_roi)
{
    if (image_roi_valid) final_roi = image_roi;

    // update front vehicle curr position
    // 无论image_roi是否有效，都要更新当前前车相对位置
    auto filtered_radar_pc = filter_curr_pc(curr_radar_pc);
    if (image_roi_valid)
    {
        int roi_center_x = image_roi.center_y;  // dim 1
        int roi_center_y = image_roi.center_x;  // dim 0
        std::vector<cv::Point2d> points = {cv::Point2d(roi_center_x, roi_center_y)};
        auto ray = map_imgpt_to_ray(points);
        auto k = std::tan(ray[0]);
        
        std::vector<pcl::PointXYZINormal> candidates;
        for (auto pt:filtered_radar_pc)
        {
            if (std::abs(pt.y - cam_vh_y - k * (pt.x - cam_vh_x)) / (sqrt(pow(k, 2) + 1.0)) < 2.0)
                candidates.emplace_back(pt);
        }
        double min_l1_dis = 1000.0;
        int min_idx = -1;
        for (int i = 0; i < candidates.size(); i++)
        {
            auto l1_dis = std::abs(candidates[i].x) + std::abs(candidates[i].y);
            if (l1_dis < min_l1_dis)
            {
                min_l1_dis = l1_dis;
                min_idx = i;
            }
        }
        if (candidates.size() > 0)
        {
            auto front_vh_pt = candidates[min_idx];
            if (front_vh_pt.x > curr_front.pose.position.x &&
                std::sqrt(std::pow(front_vh_pt.x - curr_front.pose.position.x, 2) + std::pow(front_vh_pt.y - curr_front.pose.position.y, 2)) > 1.5) {}
            else
            {
                curr_front.pose.position.x = front_vh_pt.x;
                curr_front.pose.position.y = front_vh_pt.y;
            }
        }
    }
    else
    {
        double min_dis = 1000.0;
        int min_idx = -1;
        for (int i = 0; i < filtered_radar_pc.size(); i++)
        {
            auto dis = std::sqrt(std::pow(curr_front.pose.position.x - filtered_radar_pc[i].x, 2) + 
                std::pow(curr_front.pose.position.y - filtered_radar_pc[i].y, 2));
            if (dis < min_dis)
            {
                min_dis = dis;
                min_idx = i;
            }
        }
        if (min_dis != -1)
        {
            curr_front.pose.position.x = filtered_radar_pc[min_idx].x;
            curr_front.pose.position.y = filtered_radar_pc[min_idx].y;
        }
    }

    // calculate image ROI according to current front vehicle 3D position estimation
    map_to_image(curr_front, final_roi);
}

std::vector<double> ROIGenerator::map_imgpt_to_ray(const std::vector<cv::Point2d> &points)
{
    // map image point to direction in cam coordinate (rad)
    // fisheye / front cam
    if (!points.empty()) return {0.0};

    std::vector<cv::Point2d> points_undistorted;
    cv::fisheye::undistortPoints(points, points_undistorted, K, D);

    std::vector<double> ray(points_undistorted.size(), 0.0);
    for (int i = 0; i < ray.size(); i++)
    {
        ray[i] = std::atan2(1.0, -points_undistorted[i].x);
    }

    return ray;
}

void ROIGenerator::map_to_image(const cyber_msgs::Object &vh_pose, cyber_msgs::Box2D &final_roi)
{
    // map object to image roi

    // convert object from base_link to cam
    // geometry_msgs::TransformStamped vh2cam = tf_buffer.lookupTransform("camera", "base_link", ros::Time::now());
    geometry_msgs::TransformStamped vh2cam = tf_buffer.lookupTransform("camera", "base_link", ros::Time(0.0));
    geometry_msgs::PointStamped curr_vh_estimation_high, curr_vh_estimation_low, curr_vh_estimation_lt;
    
    curr_vh_estimation_low.header.frame_id = "camera";
    curr_vh_estimation_low.header.stamp = ros::Time::now(); // ros::Time();
    curr_vh_estimation_low.point.x = vh_pose.pose.position.x;
    curr_vh_estimation_low.point.y = vh_pose.pose.position.y;
    curr_vh_estimation_high.header.frame_id = "camera";
    curr_vh_estimation_high.header.stamp = ros::Time::now(); // ros::Time();
    curr_vh_estimation_high.point.x = vh_pose.pose.position.x;
    curr_vh_estimation_high.point.y = vh_pose.pose.position.y;
    curr_vh_estimation_high.point.z = 1.90;
    curr_vh_estimation_lt.header.frame_id = "camera";
    curr_vh_estimation_lt.header.stamp = ros::Time::now(); // ros::Time();
    curr_vh_estimation_lt.point.x = vh_pose.pose.position.x;
    curr_vh_estimation_lt.point.y = vh_pose.pose.position.y - 0.8; // half of width
    curr_vh_estimation_lt.point.z = 1.90;

    geometry_msgs::PointStamped vh_low_in_cam = tf_buffer.transform(curr_vh_estimation_low, "camera");
    geometry_msgs::PointStamped vh_high_in_cam = tf_buffer.transform(curr_vh_estimation_high, "camera");
    geometry_msgs::PointStamped vh_lt_in_cam = tf_buffer.transform(curr_vh_estimation_lt, "camera");

    // convert to uv coordinates
    std::vector<cv::Point2d> points_normalized = 
    {
        cv::Point2d(-vh_low_in_cam.point.y / vh_low_in_cam.point.x, -vh_low_in_cam.point.z / vh_low_in_cam.point.x), 
        cv::Point2d(-vh_high_in_cam.point.y / vh_high_in_cam.point.x, -vh_high_in_cam.point.z / vh_high_in_cam.point.x), 
        cv::Point2d(-vh_lt_in_cam.point.y / vh_lt_in_cam.point.x, -vh_lt_in_cam.point.z / vh_lt_in_cam.point.x)
    };
    std::vector<cv::Point2d> projected_points;
    cv::fisheye::projectPoints(
        points_normalized, 
        projected_points, 
        cv::Mat::zeros(3, 1, CV_64F), 
        cv::Mat::zeros(3, 1, CV_64F), 
        K, 
        D
        );
    final_roi.center_x = (int)(projected_points[0].y + projected_points[1].y) / 2;
    final_roi.center_y = (int)(projected_points[0].x + projected_points[1].x) / 2;
    final_roi.height = std::abs(projected_points[0].y - projected_points[1].y);
    final_roi.width = 2 * std::abs(projected_points[0].x - projected_points[2].x);
}