#include <iostream>
#include <math.h>
#include <vector>
#include <unordered_map>
#include <deque>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <cyber_msgs/canframe.h>


#define height 0.0

struct RadarParsedMsg
{
    int id;
    float dist_long;
    float dist_lat;
    float v_long;
    float v_lat;
    int dyn_prop;
    float rcs;
};

class RadarParser
{
    public:
        RadarParser();
        void radar_parser_callback(const cyber_msgs::canframe &can_msg);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        ros::Publisher pub_;

        bool with_quality = false;

        int id_ = -1;
        int len_;
        unsigned char data_[8];

        std::unordered_map<int, RadarParsedMsg> radar_list;
        pcl::PointCloud<pcl::PointXYZINormal> radar_frame;
        pcl::PointXYZINormal point;
        // std::vector<int> track_ids;

        void publish_pointcloud();
};

RadarParser::RadarParser()
{
    sub_ = nh_.subscribe("/radar_can_frame", 1, &RadarParser::radar_parser_callback, this);
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/radar_pc", 10);

    radar_frame.clear();

    ros::NodeHandle nh_priv_("~");
    ros::spin();
}

void RadarParser::radar_parser_callback(const cyber_msgs::canframe &can_msg)
{
    id_ = can_msg.id;
    len_ = can_msg.len;

    for (int i = 0; i < len_; i++) data_[i] = can_msg.data[i];

    RadarParsedMsg parsed_msg;
    switch (id_)
    {
        case 0x701: // cluster
        {
            ROS_INFO("cluster general");
            parsed_msg.id = data_[0];
            parsed_msg.rcs = data_[7] * 0.5 - 64.0;
            parsed_msg.dist_long = ((data_[1] << 5) | (data_[2] >> 3)) * 0.2 - 500.0;
            parsed_msg.dist_lat = (((data_[2] & 0x3) << 8) | data_[3])  * 0.2 - 102.3;
            parsed_msg.v_long = ((data_[4] << 2) | (data_[5] >> 6)) * 0.25 - 128.0;
            parsed_msg.v_lat = (((data_[5] & 0x3f) << 3) | (data_[6] >> 5)) * 0.25 - 64.0;
            parsed_msg.dyn_prop = data_[6] & 0x7;

            radar_list.emplace(parsed_msg.id, parsed_msg);
            if (!with_quality)
            {
                point.x = parsed_msg.dist_long;
                point.y = parsed_msg.dist_lat;
                point.z = height;
                point.intensity = parsed_msg.rcs;
                point.normal_x = parsed_msg.v_long;
                point.normal_y = parsed_msg.v_lat;
                radar_frame.push_back(point);
            }
            break;
        }
        case 0x702: // cluster quality
        {
            ROS_INFO("cluster quality");
            with_quality = true;
            int id = data_[0];
            unsigned int Pdh0 = data_[3] & 0x7;
            unsigned int ambig_state = data_[4] & 0x7;
            unsigned int invalid_state = data_[4] >> 3;

            bool keep = true;
            // keep = keep && (invalid_state == 0x00 || invalid_state == 0x04 || invalid_state == 0x08 || invalid_state == 0x09
            //     || invalid_state == 0x0B || invalid_state == 0x0F || invalid_state == 0x10 || invalid_state == 0x11);
            keep = keep && (ambig_state == 0x2 || ambig_state == 0x03 || ambig_state == 0x04);
            // keep = keep && (ambig_state == 0x1 || ambig_state == 0x2);
            if (keep)
            {
                auto search = radar_list.find(id);
                if (search != radar_list.end())
                {
                    parsed_msg = search -> second;
                    point.x = parsed_msg.dist_long;
                    point.y = parsed_msg.dist_lat;
                    point.z = height;
                    point.intensity = parsed_msg.rcs;
                    point.normal_x = parsed_msg.v_long;
                    point.normal_y = parsed_msg.v_lat;
                    radar_frame.push_back(point);
                }
            }
            break;
        }
        case 0x600: // cluster head
        {
            publish_pointcloud();
            ROS_INFO("cluster head");
            break;
        }
        case 0x60B: // object
        {
            ROS_INFO("object general");
            parsed_msg.id = data_[0];
            parsed_msg.rcs = data_[7] * 0.5 - 64.0;
            parsed_msg.dist_long = ((data_[1] << 5) | (data_[2] >> 3)) * 0.2 - 500.0;
            parsed_msg.dist_lat = (((data_[2] & 0x7) << 8) | data_[3])  * 0.2 - 204.6;
            parsed_msg.v_long = ((data_[4] << 2) | (data_[5] >> 6)) * 0.25 - 128.0;
            parsed_msg.v_lat = (((data_[5] & 0x3f) << 3) | (data_[6] >> 5)) * 0.25 - 64.0;
            parsed_msg.dyn_prop = data_[6] & 0x7;

            point.x = parsed_msg.dist_long;
            point.y = parsed_msg.dist_lat;
            point.z = height;
            point.intensity = parsed_msg.rcs;
            point.normal_x = parsed_msg.v_long;
            point.normal_y = parsed_msg.v_lat;
            radar_frame.push_back(point);
            break;
        }
        case 0x60C: // object quality
        {
            ROS_INFO("object quality");
            break;
        }
        case 0x60A: // object head
        {
            publish_pointcloud();
            ROS_INFO("object head");
            break;
        }
        default:
            break;
    }
}

void RadarParser::publish_pointcloud()
{
    if (radar_frame.size() > 0)
    {
        sensor_msgs::PointCloud2 radar_pc_msg;
        pcl::toROSMsg(radar_frame, radar_pc_msg);
        radar_pc_msg.header.frame_id = "radar";
        radar_pc_msg.header.stamp = ros::Time::now();

        pub_.publish(radar_pc_msg);
        ROS_INFO("Parsed radar pointcloud size: %d ", radar_frame.size());
        radar_frame.clear();
        radar_list.clear();
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "radar_can_decoder");
    ros::NodeHandle nh;
    RadarParser parser;
    ros::spin();

    return 0;
}
