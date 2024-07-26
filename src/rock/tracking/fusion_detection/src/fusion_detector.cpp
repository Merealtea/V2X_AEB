#include <iostream>
#include <math.h>
#include <vector>
#include <deque>
#include <string>
#include <Eigen/Eigen>

#include <ros/ros.h>
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
#include <cyber_msgs/Box2D.h>
#include <cyber_msgs/Box2DArray.h>
#include <cyber_msgs/SteerFeedback.h>
#include <cyber_msgs/SpeedFeedback.h>
#include <cyber_msgs/V2VPacket.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_ros/impl/transforms.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

typedef std::vector<float> FLOAT_VECTOR;
typedef std::vector<std::vector<float>> FLOAT_VECTOR_2D;

#define EPS 1e-3


// TODO: judge curr yaw sign according to last and curr (x, y)
// TODO: leave more margin for match between rearlights and radar

struct Ray
{
    double stamp;
    float theta;

    Ray(double stamp, float theta): stamp(stamp), theta(theta) {};
};

class LowPassFilter
{
    public:
        LowPassFilter(float th_1, float th_2, float base_k): th_1(th_1), th_2(th_2), base_k(base_k), k(base_k) {};
        float filter(float old_value, float new_value)
        {
            new_flag = new_value > old_value;
            if (new_flag == old_flag)
            {
                if (std::abs(new_value - old_value) > th_1)
                    num_x += 1.0;
                if (num_x > th_2)
                    k += 0.2;
            }
            else
            {
                num_x = 0.0;
                k = base_k;
                old_flag = new_flag;
            }
            if (k > 0.95) k = 0.95;

            return (1 - k) * old_value + k * new_value;
        }
    
    private:
        const float th_1;
        const float th_2;
        const float base_k;

        bool new_flag = true;
        bool old_flag = true;
        float k;
        float num_x = 0.0;
};

class FusionDetector
{
    public:
        FusionDetector(ros::NodeHandle nh_);
    private:
        // ros
        ros::NodeHandle nh;
        ros::Subscriber sub_radar;
        ros::Subscriber sub_rearlight;
        ros::Subscriber sub_steer_feedback;
        ros::Subscriber sub_speed_feedback;
        ros::Subscriber sub_v2v;
        ros::Publisher pub_detection;
        ros::Publisher pub_vis;
        ros::Timer timer;
        tf::TransformListener radar2vehicle_tf_listener;
        tf::TransformBroadcaster br;
        tf::Transform transform;

        cyber_msgs::Box2DArray curr_rearlights;
        bool is_rearlights_init;
        pcl::PointCloud<pcl::PointXYZINormal> curr_radar;
        double curr_radar_stamp;
        pcl::PointXYZINormal last_radar_pt;
        bool is_last_radar_pt_init;
        bool is_radar_init;

        float curr_speed = 0.0; // m/s
        FLOAT_VECTOR ego_global_pose = {0.0, 0.0, 0.0}; // (x, y, yaw)
        double t_imu = 0.0;
        bool init_imu = false;
        std::deque<FLOAT_VECTOR> past_front_vh_loc;

        float yaw;

        float curr_ego_omege;
        float curr_front_omega;
        float last_front_omega;

        float yaw_search_range = 2.0 / 180 * M_PI;
        float is_search_init;

        Eigen::MatrixXf intrinsics_matrix = Eigen::Matrix3f(3, 3);
        Eigen::VectorXf distortion_coeffs = Eigen::VectorXf(5);

        float vh_width = 1.36;
        float x_radar2vh = 3.55;
        float y_radar2vh = -0.01;
        float x_cam2vh = 1.15;
        float y_cam2vh = 0.05;

        float radar_rcs_threshold = -3.0;

        // filter
        LowPassFilter yaw_filter = LowPassFilter(0.07, 4.0, 0.2);
        LowPassFilter front_omega_filter = LowPassFilter(0.003, 4.0, 0.2);


        void radar_callback(const sensor_msgs::PointCloud2ConstPtr &msg);
        void rearlight_callback(const cyber_msgs::Box2DArray &msg);
        void steer_feedback_callback(const cyber_msgs::SteerFeedback &msg);
        void speed_feedback_callback(const cyber_msgs::SpeedFeedback &msg);
        void v2v_callback(const cyber_msgs::V2VPacket &msg);
        void timer_callback(const ros::TimerEvent &timer_event);
        void visualize_rearlight(float theta, int id);
        void visualize_points(float x, float y, float yaw=0.0, int id=3, char color='g', const std::string &shape="SPHERE");
        FLOAT_VECTOR_2D search_yaw_one_step(float left, float right, float slope_1, float slope_2, const FLOAT_VECTOR &radar_pt);
        FLOAT_VECTOR_2D search_yaw(float max_yaw, float min_yaw, float left_ray, float right_ray, const FLOAT_VECTOR &radar_pt);

        // helper
        static std::vector<cv::Point2d> unproject(const std::vector<cv::Point2d> &points,
            // const FLOAT_VECTOR &Z,
            const cv::Mat &intrinsic,
            const cv::Mat &distortion);

        static float clip_angle(float angle)    // clip angle in (-PI, PI]
        {
            if (angle <= -M_PI)
                angle += 2 * M_PI;
            else if (angle > M_PI)
                angle -= 2 * M_PI;
            
            return angle;
        }

        static FLOAT_VECTOR calc_2lines_intersection(const FLOAT_VECTOR &pt_1, float slope_1, const FLOAT_VECTOR &pt_2, float slope_2)
        {
            // calculate intersection of 2 lines in the form of point-slope
            // assert (abs(slope_1 - slope_2) > 1e-2);
            FLOAT_VECTOR intersection(2);
            float k_1 = tan(slope_1), k_2 = tan(slope_2);
            intersection[0] = (-pt_1[1] + pt_2[1] + k_1 * pt_1[0] - k_2 * pt_2[0]) / (k_1 - k_2);
            intersection[1] = (k_1 * pt_2[1] - k_2 * pt_1[1] + k_1 * k_2 * (pt_1[0] - pt_2[0])) / (k_1 - k_2);

            return intersection;
        }

        static float calc_dis(float x_1, float y_1, float x_2, float y_2)
        {
            return sqrt(pow(x_1 - x_2, 2) + pow(y_1 - y_2, 2));
        }
};

FusionDetector::FusionDetector(ros::NodeHandle nh_): nh(nh_)
{
    sub_radar = nh.subscribe("/radar_pc", 1, &FusionDetector::radar_callback, this);
    sub_rearlight = nh.subscribe("/tracking/detection/rearlights", 1, &FusionDetector::rearlight_callback, this);
    sub_steer_feedback = nh.subscribe("/rock_can/steer_feedback", 1, &FusionDetector::steer_feedback_callback, this);
    sub_speed_feedback = nh.subscribe("/rock_can/speed_feedback", 1, &FusionDetector::speed_feedback_callback, this);
    sub_v2v = nh.subscribe("/V2V/leader", 1, &FusionDetector::v2v_callback, this);
    pub_detection = nh.advertise<cyber_msgs::Object>("/tracking/detection/fusion", 1);
    pub_vis = nh.advertise<visualization_msgs::Marker>("tracking/fusion/vis", 1);
    timer = nh.createTimer(ros::Duration(0.1), &FusionDetector::timer_callback, this);

    is_radar_init = is_rearlights_init = false;
    is_last_radar_pt_init = false;
    is_search_init = false;

    float yaw = 0.0;

    curr_ego_omege = curr_front_omega = last_front_omega = 0.0;

    intrinsics_matrix << 1201.6, 0.0, 953.94, 
        0.0, 1211.80, 636.9038, 
        0.0, 0.0, 1.0;
    distortion_coeffs << -0.1430, 0.0383, 0.0027, -0.0012, 0.0;
}

void FusionDetector::radar_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    sensor_msgs::PointCloud2 transformed_msg;
    pcl_ros::transformPointCloud("base_link", *msg, transformed_msg, radar2vehicle_tf_listener);
    pcl::fromROSMsg(transformed_msg, curr_radar);
    curr_radar_stamp = msg -> header.stamp.toSec();
    is_radar_init = true;
}

void FusionDetector::rearlight_callback(const cyber_msgs::Box2DArray &msg)
{
    curr_rearlights = msg;
    is_rearlights_init = true;
}

void FusionDetector::steer_feedback_callback(const cyber_msgs::SteerFeedback &msg)
{
    if (!init_imu)
    {
        t_imu = ros::Time::now().toSec();
        init_imu = true;
        return;
    }

    float curr_steer_angle = msg.SteerAngle / 180.0 * M_PI / 15.5; // front wheel angle in rad

    double dt = ros::Time::now().toSec() - t_imu;
    t_imu = ros::Time::now().toSec();

    curr_ego_omege = curr_speed * tan(curr_steer_angle) / 2.56;
    float delta_theta = curr_speed * dt * tan(curr_steer_angle) / 2.56;
    if(curr_speed > 0.04)
    {
        if(delta_theta < 0.01 && delta_theta > -0.01)
        {
            ego_global_pose[0] += curr_speed * cos(ego_global_pose[2]) * dt;
            ego_global_pose[1] += curr_speed * sin(ego_global_pose[2]) * dt;
            ego_global_pose[2] += delta_theta;
        }
        else
        {
            ego_global_pose[0] += curr_speed*dt/delta_theta*(sin(ego_global_pose[2]+delta_theta)-sin(ego_global_pose[2]));
            ego_global_pose[1] += curr_speed*dt/delta_theta*(cos(ego_global_pose[2]) - cos(ego_global_pose[2]+delta_theta));
            ego_global_pose[2] += delta_theta;
        }
        ego_global_pose[2] = clip_angle(ego_global_pose[2]);
    }
}

void FusionDetector::speed_feedback_callback(const cyber_msgs::SpeedFeedback &msg)
{
    curr_speed = 0.01 * msg.speed_cms;
}

void FusionDetector::v2v_callback(const cyber_msgs::V2VPacket &msg)
{
    last_front_omega = curr_front_omega;
    curr_front_omega = front_omega_filter.filter(last_front_omega, msg.angular_velo_z);
}

std::vector<cv::Point2d> FusionDetector::unproject(const std::vector<cv::Point2d> &points,
            // const FLOAT_VECTOR &Z,
            const cv::Mat &intrinsic,
            const cv::Mat &distortion)
{
    float f_x = intrinsic.at<float>(0, 0);
    float f_y = intrinsic.at<float>(1, 1);
    float c_x = intrinsic.at<float>(0, 2);
    float c_y = intrinsic.at<float>(1, 2);

    // project to normalized camera planar (z = 1.0)
    std::vector<cv::Point2d> points_undistorted;
    // assert (Z.size() == 1 || Z.size() == points.size());
    if (!points.empty())
    {
        cv::undistortPoints(points, points_undistorted, intrinsic, distortion, cv::noArray());
    }
    
    // // project to camera coordinate
    // std::vector<cv::Point3d> result;
    // result.reserve(points.size());
    // for (int idx = 0; idx < points_undistorted.size(); idx++)
    // {
    //     const float z = Z.size() == 1 ? Z[0]: Z[idx];
    //     result.push_back(cv::Point3d((points_undistorted.x - c_y) / f_x * z, (points_undistorted.y - c_y) / f_y * z, z));
    // }
    
    return points_undistorted;
}

void FusionDetector::timer_callback(const ros::TimerEvent &timer_event)
{
    auto ros_time = ros::Time::now();
    auto delay = ros_time.toSec() - curr_rearlights.header.stamp.toSec();
    ROS_WARN("delay: %f sec", delay);
    if (!is_radar_init || !is_rearlights_init)
        return;

    // map rearlight detection from image to vehicle frame
    cv::Point2d left_topleft = cv::Point2d(curr_rearlights.boxes[0].center_y - 0.5 * curr_rearlights.boxes[0].width, 
        curr_rearlights.boxes[0].center_x - 0.5 * curr_rearlights.boxes[0].height);
    cv::Point2d right_topright = cv::Point2d(curr_rearlights.boxes[1].center_y + 0.5 * curr_rearlights.boxes[1].width, 
        curr_rearlights.boxes[1].center_x - 0.5 * curr_rearlights.boxes[1].height);
    std::vector<cv::Point2d> points = {left_topleft, right_topright};
    cv::Mat intrinsic_cv(3, 3, CV_32F), distortion_cv(5, 1, CV_32F);
    cv::eigen2cv(intrinsics_matrix, intrinsic_cv);
    cv::eigen2cv(distortion_coeffs, distortion_cv);
    std::vector<cv::Point2d> points_normalized = unproject(points, intrinsic_cv, distortion_cv);

    // ray
    // front -- x, left -- y
    float left_ray = clip_angle(atan2(-points_normalized[0].x, 1.0)) + 2.0 / 180 * M_PI; // rad
    float right_ray = clip_angle(atan2(-points_normalized[1].x, 1.0)) + 2.0 / 180 * M_PI;

    left_ray += (0.1 * curr_front_omega - curr_ego_omege) * (delay + 0.20);
    right_ray += (0.1 * curr_front_omega - curr_ego_omege) * (delay + 0.20);

    // past_left_ray.emplace_back(Ray(curr_rearlights.header.stamp.toSec(), left_ray));
    // if (past_left_ray.size() > 3) past_left_ray.pop_front();
    // past_right_ray.emplace_back(Ray(curr_rearlights.header.stamp.toSec(), right_ray));
    // if (past_right_ray.size() > 3) past_right_ray.pop_front();
    // if (past_left_ray.size() == 3)
    // {
    //     auto omega = (past_left_ray[2].theta - past_left_ray[1].theta) / (past_left_ray[2].stamp - past_left_ray[1].theta);
    //     auto dt = ros_time.toSec() - past_left_ray[2].stamp + 0.05;
    //     left_ray = left_ray + omega * dt;
    // }
    // if (past_right_ray.size() == 3)
    // {
    //     auto omega = (past_right_ray[2].theta - past_right_ray[1].theta) / (past_right_ray[2].stamp - past_right_ray[1].theta);
    //     auto dt = ros_time.toSec() - past_right_ray[2].stamp + 0.05;
    //     right_ray = right_ray + omega * dt;
    // }

    visualize_rearlight(left_ray, 1);
    visualize_rearlight(right_ray, 2);
    assert (left_ray >= right_ray);

    // match radar points
    std::vector<pcl::PointXYZINormal> matched_points;
    for (pcl::PointCloud<pcl::PointXYZINormal>::const_iterator it = curr_radar.begin(); it != curr_radar.end(); it++)
    {
        float angle = clip_angle(atan2(it -> y - y_cam2vh, it -> x - x_cam2vh));
        if (angle <= left_ray + 12.0 / 180 * M_PI && angle >= right_ray - 12.0 / 180 * M_PI && it -> intensity >= radar_rcs_threshold
            && calc_dis(it -> x, it -> y, 0.0, 0.0) < 30.0)
            matched_points.emplace_back(*it);
    }
    if (matched_points.size() == 0 && is_last_radar_pt_init) matched_points.emplace_back(last_radar_pt);
    ROS_INFO("num of matched points: %d", matched_points.size());

    float min_dis = 100.0;
    int min_idx = 0;
    for (int i = 0; i < matched_points.size(); i++)
    {
        float curr_dis = sqrt(pow(matched_points[i].x, 2) + pow(matched_points[i].y, 2));
        if (curr_dis < min_dis)
        {
            min_dis = curr_dis;
            min_idx = i;
        }
    }
    auto radar_pt_front_vh = matched_points[min_idx];
    if (is_last_radar_pt_init)
    {
        auto tmp = radar_pt_front_vh;
        float dis_1 = calc_dis(radar_pt_front_vh.x, radar_pt_front_vh.y, 0.0, 0.0);
        float dis_2 = calc_dis(last_radar_pt.x, last_radar_pt.y, 0.0, 0.0);
        if (dis_1 - dis_2 > 1.0) radar_pt_front_vh = last_radar_pt;
        last_radar_pt = tmp;
    }
    else
    {
        last_radar_pt = radar_pt_front_vh;
        is_last_radar_pt_init = true;
    }
    visualize_points(radar_pt_front_vh.x, radar_pt_front_vh.y);
    
    // search for best fusion
    float last_yaw = 0.0;
    if (past_front_vh_loc.size() == 2)
    {
        last_yaw = atan2(past_front_vh_loc[1][1] - past_front_vh_loc[0][1], past_front_vh_loc[1][0] - past_front_vh_loc[0][0]) - ego_global_pose[2];
        last_yaw = clip_angle(last_yaw);
    }
    if (last_yaw > M_PI / 2 || last_yaw < -M_PI / 2)
    {
        last_yaw = 0.0;
        past_front_vh_loc.clear();
        is_search_init = false;
    }
    // if (past_front_vh_loc.size() > 0 && calc_dis(radar_pt_front_vh.x, radar_pt_front_vh.y, past_front_vh_loc.back()[0], past_front_vh_loc.back()[1]) > 1.0)
    // {
    //     last_yaw = 0.0;
    //     past_front_vh_loc.clear();
    //     is_search_init = false;
    // }
    float max_yaw = 0.0, min_yaw = 0.0;
    if (is_search_init)
    {
        max_yaw = clip_angle(last_yaw + yaw_search_range);
        min_yaw = clip_angle(last_yaw - yaw_search_range);
    }
    else
    {
        max_yaw = clip_angle(last_yaw + 60.0 / 180.0 * M_PI);
        min_yaw = clip_angle(last_yaw - 60.0 / 180.0 * M_PI);
        is_search_init = true;
    }
    FLOAT_VECTOR_2D search_intervals = search_yaw(max_yaw, min_yaw, left_ray, right_ray, {radar_pt_front_vh.x, radar_pt_front_vh.y});
    if (search_intervals.size() == 0)
    {
        max_yaw = clip_angle(last_yaw + 20.0 / 180.0 * M_PI);
        min_yaw = clip_angle(last_yaw - 20.0 / 180.0 * M_PI);
    }
    search_intervals = search_yaw(max_yaw, min_yaw, left_ray, right_ray, {radar_pt_front_vh.x, radar_pt_front_vh.y});

    ROS_INFO("num of yaw candidates: %d", search_intervals.size());
    if (search_intervals.size() == 0) return;

    float tmp = M_PI;
    int idx = 0;
    for (int i = 0; i < search_intervals.size(); i++)
    {
        if (abs(search_intervals[i][0] - last_yaw) < tmp)
        {
            tmp = abs(search_intervals[i][0] - last_yaw);
            idx = i;
        }
    }
    float curr_yaw = search_intervals[idx][0];
    // last_yaw = curr_yaw;
    FLOAT_VECTOR left_corner = calc_2lines_intersection({x_cam2vh, y_cam2vh}, left_ray, {radar_pt_front_vh.x, radar_pt_front_vh.y}, curr_yaw + 0.5 * M_PI);
    FLOAT_VECTOR right_corner = calc_2lines_intersection({x_cam2vh, y_cam2vh}, right_ray, {radar_pt_front_vh.x, radar_pt_front_vh.y}, curr_yaw + 0.5 * M_PI);
    FLOAT_VECTOR center = {0.5 * (left_corner[0] + right_corner[0]), 0.5 * (left_corner[1] + right_corner[1])};
    
    // record front vehicle location
    past_front_vh_loc.push_back({ego_global_pose[0] + center[0] * cos(ego_global_pose[2]) - center[1] * sin(ego_global_pose[2]),
        ego_global_pose[1] + center[0] * sin(ego_global_pose[2]) + center[1] * cos(ego_global_pose[2])});
    if (past_front_vh_loc.size() > 3) past_front_vh_loc.pop_front();
    
    if (past_front_vh_loc.size() == 3)
    {
        curr_yaw = atan2(past_front_vh_loc[2][1] - past_front_vh_loc[0][1], past_front_vh_loc[2][0] - past_front_vh_loc[0][0]) - ego_global_pose[2];
    }
    curr_yaw = clip_angle(curr_yaw);
    curr_yaw = yaw_filter.filter(yaw, curr_yaw);
    yaw = curr_yaw;

    // publish transform
    transform.setOrigin(tf::Vector3(center[0], center[1], 0.0));
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, curr_yaw);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "target"));
    std::cout << "send tf" << std::endl;

    visualize_points(center[0], center[1], 0.0, 4, 'r', "SPHERE");
    visualize_points(center[0], center[1], curr_yaw, 5, 'b', "ARROW");
    ROS_INFO("front vh yaw: %f, x: %f, y: %f", curr_yaw, center[0], center[1]);
}

FLOAT_VECTOR_2D FusionDetector::search_yaw_one_step(float left, float right, float slope_1, float slope_2, const FLOAT_VECTOR &radar_pt)
{
    float step = abs(left - right) / 10.0;
    if (step > 4.0 / 180.0 * M_PI)
    {
        step = 4.0 / 180.0 * M_PI;
    }

    FLOAT_VECTOR_2D ans;
    // std::cout << "boundary: " << left << " " << right << std::endl;
    FLOAT_VECTOR dis_buffer;
    for (int i = 0; i < (int)((left - right) / step) + 1; i++)
    {
        float l = left - step * i;
        float r = std::max(l - step, right);

        FLOAT_VECTOR l_intersection_1 = calc_2lines_intersection({x_cam2vh, y_cam2vh}, slope_1, radar_pt, l + 0.5 * M_PI);
        FLOAT_VECTOR l_intersection_2 = calc_2lines_intersection({x_cam2vh, y_cam2vh}, slope_2, radar_pt, l + 0.5 * M_PI);
        float l_dis = calc_dis(l_intersection_1[0], l_intersection_1[1], l_intersection_2[0], l_intersection_2[1]);

        FLOAT_VECTOR r_intersection_1 = calc_2lines_intersection({x_cam2vh, y_cam2vh}, slope_1, radar_pt, r + 0.5 * M_PI);
        FLOAT_VECTOR r_intersection_2 = calc_2lines_intersection({x_cam2vh, y_cam2vh}, slope_2, radar_pt, r + 0.5 * M_PI);
        float r_dis = calc_dis(r_intersection_1[0], r_intersection_1[1], r_intersection_2[0], r_intersection_2[1]);

        // std::cout << l_dis << "\t" << r_dis << "\t" << l << "\t" << r << std::endl;
        if ((l_dis - vh_width) * (r_dis - vh_width) < 0.0) ans.emplace_back(FLOAT_VECTOR({l, r}));
        dis_buffer.emplace_back((l_dis + r_dis) / 2);
    }
    if (ans.size() != 0) return ans;
    else
    {
        float min_err = 1e6;
        int min_idx = 0;
        for (int i = 0; i < dis_buffer.size(); i++)
        {
            if (abs(dis_buffer[i]) - vh_width < min_err)
            {
                min_err = abs(dis_buffer[i] - vh_width);
                min_idx = i;
            }
        }
        if (min_err < 0.20)
        {
            ans.emplace_back(FLOAT_VECTOR({left - step * min_idx, left - step * (min_idx + 1)}));
        }
    }
    return ans;
}

FLOAT_VECTOR_2D FusionDetector::search_yaw(float max_yaw, float min_yaw, float left_ray, float right_ray, const FLOAT_VECTOR &radar_pt)
{
    FLOAT_VECTOR_2D search_intervals = search_yaw_one_step(max_yaw, min_yaw, left_ray, right_ray, radar_pt);
    for (int i = 0; i < search_intervals.size(); i++)
    {
        while (abs(search_intervals[i][0] - search_intervals[i][1]) > EPS)
        {
            FLOAT_VECTOR_2D tmp = search_yaw_one_step(search_intervals[i][0], search_intervals[i][1], left_ray, right_ray, radar_pt);
            search_intervals[i] = tmp[0];
        }
    }

    return search_intervals;
}

void FusionDetector::visualize_rearlight(float theta, int id)
{
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "base_link";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "lines";
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.id = id;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.scale.x = 0.1;
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
    line_list.pose.orientation.w = 1.0;

    geometry_msgs::Point end_point;
    end_point.x = x_cam2vh;
    end_point.y = y_cam2vh;
    end_point.z = 0.0;
    line_list.points.push_back(end_point);
    float x_ = 30.0;
    float y_ = tan(theta) * (x_ - x_cam2vh) + y_cam2vh;
    end_point.x = x_;
    end_point.y = y_;
    line_list.points.push_back(end_point);

    pub_vis.publish(line_list);
}

void FusionDetector::visualize_points(float x, float y, float yaw, int id, char color, const std::string &shape)
{
    visualization_msgs::Marker point;
    point.header.frame_id = "base_link";
    point.header.stamp = ros::Time::now();
    point.ns = "vis";
    point.action = visualization_msgs::Marker::ADD;
    point.color.a = 1.0;

    if (shape == "ARROW")
    {
        point.scale.x = 0.05;
        point.scale.y = 0.5;
        point.scale.z = 0.5;
    }
    else
    {
        point.scale.x = 0.6;
        point.scale.y = 0.6;
        point.scale.z = 0.6;
    }

    if (shape == "SPHERE")
        point.type = visualization_msgs::Marker::SPHERE;
    else if (shape == "ARROW")
        point.type = visualization_msgs::Marker::ARROW;
    else
        point.type = visualization_msgs::Marker::SPHERE;
    
    point.id = id;
    
    switch (color)
    {
        case 'g':
        {
            point.color.g = 1.0;
            break;
        }
        case 'r':
        {
            point.color.r = 1.0;
            break;
        }
        case 'b':
        {
            point.color.b = 1.0;
            break;
        }
        default:
        {
            point.color.g = 1.0;
            break;
        }
    }

    if (shape == "ARROW")
    {
        // TODO: locate arrow position and orientation with start and end point
        // start, end points: geometry_msgs/Point
        geometry_msgs::Point start_point, end_point;
        start_point.x = x;
        start_point.y = y;
        start_point.z = 0.0;
        float arrow_length = 3.0;
        end_point.x = x + arrow_length * cos(yaw);
        end_point.y = y + arrow_length * sin(yaw);
        end_point.z = 0.0;
        point.points.emplace_back(start_point);
        point.points.emplace_back(end_point);
    }
    else
    {
        point.pose.position.x = x;
        point.pose.position.y = y;
        point.pose.position.z = 0.0;
    }
    
    pub_vis.publish(point);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "fusion_detection");

    ros::NodeHandle nh;
    FusionDetector fusion_detector(nh);
    ros::spin();

    return 0;
}
