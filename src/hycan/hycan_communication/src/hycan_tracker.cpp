#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <vector>

#include "hycan_msgs/DetectionResults.h"
#include "hycan_msgs/Box3D.h"
#include "tracker.hpp"

using namespace std;


class HycanTracker
{
public:
    HycanTracker() 
    {
        box_sub = nh_.subscribe("hycan_detection_results", 1, &HycanTracker::DetectionCallback, this);
        track_pub = nh_.advertise<hycan_msgs::DetectionResults>("hycan_track_results", 1);

    }

    void DetectionCallback(const hycan_msgs::DetectionResults::ConstPtr &msg)
    {
        ros::Time st_time = ros::Time::now();

        // 转换检测结果到世界坐标系
        float heading = msg->localization.heading;
        float utm_x = msg->localization.utm_x;
        float utm_y = msg->localization.utm_y;

        // 生成转换矩阵
        Eigen::Matrix3f tf_matrix;
        tf_matrix << cos(heading), -sin(heading), utm_x,
                     sin(heading), cos(heading), utm_y,
                     0, 0, 1;

        // 读取检测结果
        std::vector<hycan_msgs::Box3D> boxes = msg->box3d_array;

        ROS_INFO("Receive %d detection results", boxes.size());

        // 保存检测结果
        std::vector<BoundingBox> boundingBoxes;

        for (int i = 0; i < msg->num_boxes; i++)
        {
            hycan_msgs::Box3D box = boxes[i];

            Eigen::Vector3f position(box.center_x, box.center_y, 1);
            Eigen::Vector3f size(box.width, box.length, box.height);

            // 转换到世界坐标系
            position = tf_matrix * position;
            position(2) = box.center_z;

            BoundingBox bbox(position, size, box.heading + heading);

            boundingBoxes.push_back(bbox);
        }

        std::vector<BoundingBox> bbox_predicted;
        SORT.predict(bbox_predicted, msg->image_stamp.toSec());
        ROS_INFO("Predict %d trackers", bbox_predicted.size());

        // 目标关联
        std::vector<int> assignment_predict(bbox_predicted.size(), -1);
        std::vector<int> assignment_observe(boundingBoxes.size(), -1);

        DataAssociation(boundingBoxes, bbox_predicted, assignment_predict, assignment_observe);

        // 更新卡尔曼滤波器
        SORT.update(boundingBoxes, assignment_predict);


        // 使用birth&Death Manager来处理新出现的目标和消失的目标
        SORT.birthAndDeath(boundingBoxes, assignment_observe, assignment_predict, msg->image_stamp.toSec());

        ROS_INFO("There are %d trackers after Birth and death", SORT.trackers.size());
        // 输出id结果
        hycan_msgs::DetectionResults track_res;
        track_res.frame_idx = msg->frame_idx;
        track_res.image_stamp = msg->image_stamp;
        track_res.localization = msg->localization;

        for(int i = 0; i < assignment_predict.size(); i++) {
            if(assignment_predict[i] != -1) {
                hycan_msgs::Box3D box;
                box.center_x = boundingBoxes[i].postion(0);
                box.center_y = boundingBoxes[i].postion(1);
                box.center_z = boundingBoxes[i].postion(2);
                box.width = boundingBoxes[i].size(0);
                box.length = boundingBoxes[i].size(1);
                box.height = boundingBoxes[i].size(2);
                box.heading = boundingBoxes[i].theta;
                box.id = SORT.trackers[i].tracker_id;
                box.speed_x = SORT.trackers[i].velocity_estimate(0);
                box.speed_y = SORT.trackers[i].velocity_estimate(1);
                box.speed_angle = SORT.trackers[i].velocity_estimate(2);
                
                track_res.box3d_array.push_back(box);
            }
        }

        track_res.num_boxes = track_res.box3d_array.size();

        track_pub.publish(track_res);
        ROS_INFO("FPS in tracker is %f", 1.0 / (ros::Time::now() - st_time).toSec());
        ROS_INFO("Send %d tracking results", track_res.box3d_array.size());
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber box_sub;
    ros::Publisher track_pub;
    MOT3D SORT;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hycan_detection_tracker");
    HycanTracker SORT_tracker;
    ros::spin();
    return 0;
}
