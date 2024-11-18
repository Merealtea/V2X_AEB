//
// Created by runrunxin on 24-3-19.
//

#ifndef SRC_TRACKER_HPP
#define SRC_TRACKER_HPP


#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include "Hungarian.h"
#include <cmath>


class BoundingBox {
public:
    Eigen::Vector3f position;
    Eigen::Vector3f size;
    double theta;
    Eigen::Vector3f velocity;
    double score = 0;

    float min_width = 0.4, min_length = 0.2, min_height = 1.5;
    float max_width = 0.6, max_length = 0.3, max_height = 2.0;

    Eigen::Vector3f scale_bbox(Eigen::Vector3f &size) {
        if(size(0) > size(1)) {
            size(0) = std::max(min_width, std::min(max_width, float(size(0))));
            size(1) = std::max(min_length, std::min(max_length, float(size(1))));
        }
        else {
            size(0) = std::max(min_length, std::min(max_length, float(size(0))));
            size(1) = std::max(min_width, std::min(max_width, float(size(1))));
        }
        size(2) = std::max(min_height, std::min(max_height, float(size(2))));
        return size;
    }

    BoundingBox(Eigen::Vector3f position, Eigen::Vector3f size, 
                double theta, double score = 0,
                Eigen::Vector3f velocity = Eigen::Vector3f::Zero()) {
        this->position = position;
        this->size = scale_bbox(size);
        this->theta = theta;
        this->velocity = velocity;
        this->score = score;
    }
};

class Tracker {
public:
    // tracker id
    int tracker_id = 0;
    int time_since_update = 0;
    int hit_count = 0;

    bool comfirmed = false;

    Eigen::Vector3f position_estimate = Eigen::Vector3f::Zero();
    Eigen::Vector3f size_estimate = Eigen::Vector3f::Zero();
    double theta_estimate = 0;
    Eigen::Vector3f velocity_estimate = Eigen::Vector3f::Zero();

    double last_stamp = 0;
    double time_diff = 0;
    
    // define a 3D Kalman Filter
    cv::KalmanFilter *kf;

    void init(int id, double stamp);

    void predict(Eigen::Vector3f &position, Eigen::Vector3f &size, double &theta,
                    Eigen::Vector3f &velocity, double stamp);
    void update(Eigen::Vector3f position, Eigen::Vector3f size,
                 double theta, double score);
};

class MOT3D {
public:
    MOT3D() {};
    ~MOT3D() {};

    int max_age = 10;
    int min_hits = 5;
    int id_count = 0;

    // 定义Tracker列表
    std::vector<Tracker> trackers;

    void predict(std::vector<BoundingBox> &bbox_predicted, double stamp);
    void update(std::vector<BoundingBox> &bbox_observed, std::vector<int> assignment_predict);

    void birthAndDeath(std::vector<BoundingBox> &bbox_observed, std::vector<int> assignment_observe, 
                        std::vector<int> assignment_predict, double stamp);
};


// 目标关联
// 输入：观测到的bbox，预测的bbox
// 输出：关联结果assignment_predict：预测的bbox匹配上的观测索引，若预测没匹配上，则assignment_predict[i] = 0;
//      关联结果assignment_observe：观测的bbox匹配上的预测索引，若观测j没匹配上，则assignment_observe[j] = 0;
void DataAssociation(std::vector<BoundingBox> bbox_observed, std::vector<BoundingBox> bbox_predicted, std::vector<int> &assignment_predict,std::vector<int> &assignment_observe);
#endif //SRC_TRACKER_HPP
