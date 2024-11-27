//
// Created by runrunxin on 24-3-19.
//

#include "tracker.hpp"


void Tracker::init(int id, double stamp) {
    tracker_id = id;

    int state_dim = 10;
    int measure_dim = 7;
    int control_dim = 0;
    last_stamp = stamp;

    kf = new cv::KalmanFilter(state_dim, measure_dim, control_dim);

    // initialize Kalman Filter

    // 定义状态转移矩阵F
    // state x dimension 10: x, y, z, theta, l, w, h, dx, dy, dtheta
    // constant velocity model: x' = x + dx, y' = y + dy, theta' = theta + dtheta
    // while all others (z, l, w, h, dx, dy, dtheta) remain the same
    // state transition matrix, dim_x * dim_x
    cv::setIdentity(kf->transitionMatrix);
    kf->transitionMatrix.at<float>(0, 7) = 1.0;
    kf->transitionMatrix.at<float>(1, 8) = 1.0;
    kf->transitionMatrix.at<float>(3, 9) = 1.0;

    // 定义测量矩阵H
    // measurement matrix, dim_z * dim_x, the first 7 dimensions of the measurement correspond to the state
    kf->measurementMatrix = (cv::Mat_<float>(7, 10) <<
            1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 1, 0, 0, 0);

    // 定义过程噪声协方差矩阵Q
    cv::setIdentity(kf->processNoiseCov, cv::Scalar::all(1));
    kf->processNoiseCov.at<float>(7, 7) *= 1;
    kf->processNoiseCov.at<float>(8, 8) *= 1;
    kf->processNoiseCov.at<float>(9, 9) *= 1;

    // 定义测量噪声协方差矩阵R
    cv::setIdentity(kf->measurementNoiseCov);
    kf->measurementNoiseCov *= 0.5;
    kf->measurementNoiseCov.at<float>(3, 3) = 0.5;

    cv::setIdentity(kf->errorCovPost, cv::Scalar::all(0.1));

    // 定义初始状态
    kf->statePost = cv::Mat::zeros(10, 1, CV_32F);
}


void Tracker::predict(Eigen::Vector3f &position, Eigen::Vector3f &size, 
                        double &theta, Eigen::Vector3f &velocity, double stamp) {
    cv::Mat prediction = kf->predict();

    position(0) = prediction.at<float>(0);
    position(1) = prediction.at<float>(1);
    position(2) = prediction.at<float>(2);

    theta = prediction.at<float>(3);
    size(0) = prediction.at<float>(4);
    size(1) = prediction.at<float>(5);
    size(2) = prediction.at<float>(6);

    time_diff = stamp - last_stamp;
    velocity(0) = prediction.at<float>(7) * time_diff;
    velocity(1) = prediction.at<float>(8) * time_diff;
    velocity(2) = prediction.at<float>(9) * time_diff;
    last_stamp = stamp;
}

double within_range(double theta) {
    // Make sure the orientation is within a proper range

    if (theta >= M_PI) {
        theta -= 2 * M_PI;  // Make the theta still in the range
    }
    if (theta < -M_PI) {
        theta += 2 * M_PI;
    }

    return theta;
}

void orientation_correction(double &theta_update, double theta) {
    // Update orientation in propagated tracks and detected boxes so that they are within 90 degrees

    // Make the theta still in the range
    theta_update = within_range(theta_update);
    theta = within_range(theta);

    // If the angle of two theta is not acute angle, then make it acute
    if (std::abs(theta - theta_update) > M_PI / 2.0 && std::abs(theta - theta_update) < M_PI * 3 / 2.0) {
        theta_update += M_PI;
        theta_update = within_range(theta_update);
    }

    // Now the angle is acute: < 90 or > 270, convert the case of > 270 to < 90
    if (std::abs(theta - theta_update) >= M_PI * 3 / 2.0) {
        if (theta > 0) {
            theta_update += M_PI * 2;
        } else {
            theta_update -= M_PI * 2;
        }
    }
}

void Tracker::update(Eigen::Vector3f position, 
                        Eigen::Vector3f size, 
                            double theta, double score) {
    double theta_measure = theta;
    cv::Mat prediction = kf->predict();

    double theta_pre = prediction.at<float>(3);

    double var_noise = 3 * exp(-score);

    kf->measurementNoiseCov = cv::Mat::eye(7, 7, CV_32F) * var_noise;
    kf->measurementNoiseCov.at<float>(3, 3) = 1;

    cv::Mat measurement(7, 1, CV_32F);
    measurement.at<float>(0) = position(0);
    measurement.at<float>(1) = position(1);
    measurement.at<float>(2) = position(2);

    measurement.at<float>(3) = theta_measure;
    measurement.at<float>(4) = size(0);
    measurement.at<float>(5) = size(1);
    measurement.at<float>(6) = size(2);

    cv::Mat estimated = kf->correct(measurement);

    // kf->statePost.at<float>(0) = position(0);
    // kf->statePost.at<float>(1) = position(1);

    // update the state
    position_estimate(0) = estimated.at<float>(0);
    position_estimate(1) = estimated.at<float>(1);
    position_estimate(2) = estimated.at<float>(2);

    theta_estimate = estimated.at<float>(3);
    size_estimate(0) = estimated.at<float>(4);
    size_estimate(1) = estimated.at<float>(5);
    size_estimate(2) = estimated.at<float>(6);

    velocity_estimate(0) = estimated.at<float>(7) * time_diff;
    velocity_estimate(1) = estimated.at<float>(8) * time_diff;
    velocity_estimate(2) = estimated.at<float>(9) * time_diff;
}

void MOT3D::predict(std::vector<BoundingBox> &bbox_predicted, double stamp) {
    if(trackers.size() == 0)
        return;
    for(int i = 0; i < trackers.size(); i++) {
        Eigen::Vector3f position, size, velocity;
        double theta;
        trackers[i].predict(position, size, theta, velocity, stamp);
        BoundingBox bbox_pre(position, size, theta, 0,velocity);

        bbox_predicted.emplace_back(bbox_pre);
    }
}

void MOT3D::update(std::vector<BoundingBox> &bbox_observed, std::vector<int> assignment_predict) {
    for(int i = 0; i < assignment_predict.size(); i++) {
        if(assignment_predict[i] == -1)
            continue;
        trackers[i].update(bbox_observed[assignment_predict[i]].position, 
                        bbox_observed[assignment_predict[i]].size, 
                        bbox_observed[assignment_predict[i]].theta,
                        bbox_observed[assignment_predict[i]].score);
    }
}

void MOT3D::birthAndDeath(std::vector<BoundingBox> &bbox_observed, 
                            std::vector<int> assignment_observe, 
                                std::vector<int> assignment_predict,
                                    double stamp) {
    // 把所有没有匹配上的观测作为新的tracker
    for(int i = 0; i < assignment_observe.size(); i++) {
        if(assignment_observe[i] == -1) {
            Tracker tracker;

            tracker.init(id_count++, stamp);
            tracker.hit_count ++;
            tracker.time_since_update = 0;

            Eigen::Vector3f position, size;
            double theta;
            double score;
            position = bbox_observed[i].position;
            size = bbox_observed[i].size;
            theta = bbox_observed[i].theta;
            score = bbox_observed[i].score;

            // x, y, z, theta, l, w, h, dx, dy, dz
            cv::Mat initial_state(10, 1, CV_32F);
            initial_state.at<float>(0) = position(0);
            initial_state.at<float>(1) = position(1);
            initial_state.at<float>(2) = position(2);
            initial_state.at<float>(3) = theta;
            initial_state.at<float>(4) = size(0);
            initial_state.at<float>(5) = size(1);
            initial_state.at<float>(6) = size(2);
            initial_state.at<float>(7) = 0;
            initial_state.at<float>(8) = 0;
            initial_state.at<float>(9) = 0;

            tracker.kf->statePost = initial_state;

            trackers.push_back(tracker);
            trackers[trackers.size() - 1].update(position, size, theta, score);
            std::cout << "\033[33m" << "birth a new tracker: id " << id_count << "\033[0m" << std::endl;
//            std::cout << "position: " << position.transpose() << " estimated: " << trackers[trackers.size() - 1].position_estimate.transpose() << std::endl;
        }
    }
    for(int i = 0; i < assignment_predict.size(); i++) {
        if(assignment_predict[i] == -1) {
            trackers[i].time_since_update ++;
        } else {
            trackers[i].time_since_update = 0;
            trackers[i].hit_count ++;
            if (trackers[i].hit_count > min_hits) 
                trackers[i].comfirmed = true;
        }
        if(trackers[i].time_since_update > max_age) {
            trackers.erase(trackers.begin() + i);
            assignment_predict.erase(assignment_predict.begin() + i);
        }
    }
}

void DataAssociation(std::vector<BoundingBox> bbox_observed, std::vector<BoundingBox> bbox_predicted, std::vector<int> &assignment_predict,std::vector<int> &assignment_observe) {
    // calculate the distance between observed and predicted bbox
    int n_observed = bbox_observed.size();
    int n_predicted = bbox_predicted.size();

    // calculate the distance between observed and predicted bbox
    std::vector<std::vector<double>> cost_matrix(n_predicted, std::vector<double>(n_observed, 1000.0));

//    std::vector<std::vector<double>> cost_matrix_sum(n_predicted, std::vector<double>(n_observed, 1000.0));

    for (int i = 0; i < n_predicted; i++) {
        for (int j = 0; j < n_observed; j++) {
            Eigen::Vector3f diff = bbox_predicted[i].position - bbox_observed[j].position;
            Eigen::Vector3f diff_size = bbox_predicted[i].size - bbox_observed[j].size;
            float distance = diff.norm() + 0.5 * diff_size.norm();
            cost_matrix[i][j] = distance;
        }
    }


    if (n_predicted == 0 || n_observed == 0) {
        return;
    }

    // solve the assignment problem
    HungarianAlgorithm hungarian;
    vector<int> Assignment;
    double cost = hungarian.Solve(cost_matrix, Assignment);

    double distance_threshold = 1;

    for (int i = 0; i < n_predicted; i++) {
        if(Assignment[i] == -1)
            continue;
        if(cost_matrix[i][Assignment[i]] < distance_threshold) {
            std::cout << "predicted " << i << " is matched to " << Assignment[i] << " with distance " << cost_matrix[i][Assignment[i]] << std::endl;
            assignment_predict[i] = Assignment[i];
            if(Assignment[i] < n_observed)
                assignment_observe[Assignment[i]] = i;
        }
    }
}
