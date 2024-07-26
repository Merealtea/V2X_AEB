#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


bool enforce_velocity_similarity(const pcl::PointXYZINormal& point_a, const pcl::PointXYZINormal& point_b, float squared_distance)
{
    return std::abs(point_a.normal_x - point_b.normal_x) < 1.5;
}

class RadarDetector
{
    public:
        RadarDetector();
        void radar_detector_callback(const sensor_msgs::PointCloud2ConstPtr &msg);
    
    private:
        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        ros::Publisher pub_1;
        image_transport::Publisher pub;

        float dist_long_max = 60.0;
        float dist_long_min = 0.0;
        float dist_long_res = 0.2;
        float dist_lat_max = 20.0;
        float dist_lat_min = -20.0;
        float dist_lat_res = 0.2;

        int guard_cells = 10;
        int train_cells = 5;
        float fa = 1e-1;
        int cfar_units = 1 + 2 * guard_cells + 2 * train_cells;
        int half_cfar_uints = (int)(cfar_units / 2) + 1;

        cv::Mat ca_cfar_detect(const cv::Mat radar_bev);
};

RadarDetector::RadarDetector()
{
    sub_ = nh_.subscribe("/radar_pc", 1, &RadarDetector::radar_detector_callback, this);
    image_transport::ImageTransport image_trans(nh_);
    pub = image_trans.advertise("/radar_bev", 1);

    pub_1 = nh_.advertise<sensor_msgs::PointCloud2>("/radar_cluster", 1);
    
    ros::NodeHandle nh_priv_("~");
    ros::spin();
    
}

void RadarDetector::radar_detector_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    // std::cout << "receive pc" << std::endl;
    // convert pointcloud to image
    ros::Time time_1 = ros::Time::now();
    pcl::PointCloud<pcl::PointXYZINormal> pc;
    // std::cout << "start cvt pc" << std::endl;
    pcl::fromROSMsg(*msg, pc);
    // std::cout << "pc cvted" << std::endl;
    int dim_1 = ceil((dist_long_max - dist_long_min) / dist_long_res);
    int dim_2 = ceil((dist_lat_max - dist_lat_min) / dist_lat_res);
    cv::Mat bev_map = cv::Mat::zeros(dim_1, dim_2, CV_64FC1);
    float x, y, intensity;
    int coord_x, coord_y;
    int count = 0;
    // std::cout << "generate bev" << std::endl;
    for (pcl::PointCloud<pcl::PointXYZINormal>::const_iterator it = pc.begin(); it != pc.end(); it++)
    {
        x = it -> x;
        y = it -> y;
        if (x < 0 or x > 50.0 or y < -15.0 or y > 15.0) continue;
        coord_x = (int)((x - dist_long_min) / dist_long_res);
        coord_y = (int)((y - dist_lat_min) / dist_lat_res);
        intensity = it -> intensity;
        bev_map.at<double>(coord_x, coord_y) = intensity;
    }
    // std::cout << "bev generated" << std::endl;

    // CA-CFAR
    cv::Mat mask = ca_cfar_detect(bev_map);
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", mask).toImageMsg();
    pub.publish(img_msg);
    ros::Time time_2 = ros::Time::now();
    
    // mask pointcloud
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr filtered_pc (new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointXYZINormal tmp_point;
    for (pcl::PointCloud<pcl::PointXYZINormal>::const_iterator it = pc.begin(); it != pc.end(); it++)
    {
        x = it -> x;
        y = it -> y;
        if (x < 0 or x > 50.0 or y < -15.0 or y > 15.0) continue;
        coord_x = (int)((x - dist_long_min) / dist_long_res);
        coord_y = (int)((y - dist_lat_min) / dist_lat_res);

        if (mask.at<uchar>(coord_x, coord_y) == 255)
        {
            tmp_point.x = x;
            tmp_point.y = y;
            tmp_point.z = 0.0;
            tmp_point.intensity = it -> intensity;
            tmp_point.normal_x = it -> normal_x;
            tmp_point.normal_y = 0.0;

            (*filtered_pc).push_back(tmp_point);
        }
    }

    // clustering
    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZINormal>);
    pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters), small_clusters (new pcl::IndicesClusters), large_clusters (new pcl::IndicesClusters);
    pcl::IndicesClustersPtr selected_clusters(new pcl::IndicesClusters);
    tree -> setInputCloud(filtered_pc);

    pcl::ConditionalEuclideanClustering<pcl::PointXYZINormal> cec(true);
    cec.setInputCloud(filtered_pc);
    cec.setInputCloud(filtered_pc);
    cec.setConditionFunction(&enforce_velocity_similarity);
    cec.setClusterTolerance(2.0);
    cec.setMinClusterSize(2);
    cec.setMaxClusterSize(20);
    cec.segment(*clusters);
    cec.getRemovedClusters(small_clusters, large_clusters);

    selected_clusters = clusters;
    pcl::PointCloud<pcl::PointXYZINormal> clustered_pc;
    pcl::PointXYZINormal cluster_point;
    for (int i = 0; i < selected_clusters -> size(); i++)
    {
        float x = 0.0, y = 0.0, intensity = 0.0, v = 0.0;
        for (int j = 0; j < (*selected_clusters)[i].indices.size(); j++)
        {
            tmp_point = filtered_pc -> points[(*selected_clusters)[i].indices[j]];
            x += tmp_point.x;
            y += tmp_point.y;
            intensity += tmp_point.intensity;
            v += tmp_point.normal_x;
        }
        x /= (*selected_clusters)[i].indices.size();
        y /= (*selected_clusters)[i].indices.size();
        intensity /= (*selected_clusters)[i].indices.size();
        v /= (*selected_clusters)[i].indices.size();
        cluster_point.x = x;
        cluster_point.y = y;
        cluster_point.z = 0.0;
        cluster_point.intensity = intensity;
        cluster_point.normal_x = v;
        clustered_pc.push_back(cluster_point);
    }

    sensor_msgs::PointCloud2 radar_pc_msg;
    pcl::toROSMsg(clustered_pc, radar_pc_msg);
    radar_pc_msg.header.frame_id = "vehicle";
    radar_pc_msg.header.stamp = ros::Time::now();
    pub_1.publish(radar_pc_msg);

    ROS_INFO("Num of clusters: %d", clustered_pc.size());

    clustered_pc.clear();

}

cv::Mat RadarDetector::ca_cfar_detect(const cv::Mat radar_bev)
{
    int center_cell_x, center_cell_y;
    float average;
    int count;
    int dim_1 = ceil((dist_long_max - dist_long_min) / dist_long_res);
    int dim_2 = ceil((dist_lat_max - dist_lat_min) / dist_lat_res);
    cv::Mat mask = cv::Mat::zeros(dim_1, dim_2, CV_8UC1);
    for (int i = 0; i < radar_bev.rows - cfar_units; i++)
        for (int j = 0; j < radar_bev.cols - cfar_units; j++)
        {
            center_cell_x = i + train_cells + guard_cells;
            center_cell_y = j + train_cells + guard_cells;
            if ((radar_bev.at<double>(center_cell_x, center_cell_y) > 1e-8) or (radar_bev.at<double>(center_cell_x, center_cell_y) < -1e-8))
            {
                mask.at<uchar>(center_cell_x, center_cell_y) = 0;
            }
            if ((radar_bev.at<double>(center_cell_x, center_cell_y) < 1e-8) and (radar_bev.at<double>(center_cell_x, center_cell_y) > -1e-8))
                continue;
            if (radar_bev.at<double>(center_cell_x, center_cell_y) < 0.0)
                continue;

            average = 0.0;
            count = 0;
            for (int k = 0; k < cfar_units; k++)
                for (int l = 0; l < cfar_units; l++)
                {
                    if ((k >= train_cells) and (k < (cfar_units - train_cells)) and (l >= train_cells) and (l < (cfar_units - train_cells)))
                        continue;
                    average += radar_bev.at<double>(i + k, j + l);
                    if (radar_bev.at<double>(i + k, j + l) < -1e-8 or radar_bev.at<double>(i + k, j + l) > 1e-8)
                        count++;
                }
            if (count > 0)
            {
                average /= count;
                if (radar_bev.at<double>(center_cell_x, center_cell_y) > (average * count * (pow(fa, (-1.0 / count)) - 1.0)))
                {
                    mask.at<uchar>(center_cell_x, center_cell_y) = 255;
                    // float x = dist_long_min + center_cell_x * dist_long_res;
                    // float y = dist_lat_min + center_cell_y * dist_lat_res;
                    // std::cout << "x: " << x << "  y: " << y << std::endl;
                    // for (int ii = -1; ii < 1; ii++)
                    //     for (int jj = -1; jj < 1; jj++)
                    //         mask.at<uchar>(center_cell_x + ii, center_cell_y + jj) = 255;
                }
            }
        }
    return mask;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "radar_detector_node");
    ros::NodeHandle nh;
    RadarDetector detector;
    ros::spin();

    return 0;
}