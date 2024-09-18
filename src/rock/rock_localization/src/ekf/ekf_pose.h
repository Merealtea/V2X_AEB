/*
 * @Author: your name
 * @Date: 2021-07-24 12:15:33
 * @LastEditTime: 2021-07-27 17:56:40
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /pipeline/src/ekf/ekf_pose.h
 */
/**
  *ekf_pose.h
  *brief:ekf fusion of gps and slam and imu
  *author:Chen Xiaofeng
  *date:20191028
  **/

#ifndef ALL_EKF_POSE_H
#define ALL_EKF_POSE_H
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <iomanip>
#include <chrono>


using namespace std;

class All_EKF_Pose {
public:
    All_EKF_Pose(){
        gps_init_flag = false;
        vel_init_flag = false;

        X = Eigen::VectorXd::Zero(5);
        P = Eigen::MatrixXd::Zero(5,5);
         auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	    //转为字符串
        if(data_record)
        {
            std::string package_path = ros::package::getPath("localization_pipeline");
            //std::string config_file_path = package_path + ",map";
            std::stringstream ss;
            ss << std::put_time(std::localtime(&t), "%F-%H-%M-%S");
            std::string str = package_path+"/data/data"+ss.str()+".txt";
            cout<<"data file path:"<< str;
            data_file.open(str);
        }
    }
    ~All_EKF_Pose(){
        data_file.close();
    }

    void gpsStateUpdate(Eigen::Vector3d &Z_gps, const Eigen::Matrix3d &R_gps, const double time);
    void slamStateUpdate(Eigen::Vector3d &Z_slam, const Eigen::Matrix3d &R_slam, const double time);
    void velStateUpdate(const Eigen::Vector2d &Z_vel, const Eigen::Matrix2d &R_vel, const double time, const bool ang_only=false);
    Eigen::VectorXd readX(double time,bool prediction);
    Eigen::MatrixXd readP(double time,bool prediction);
    void timeUpdate(double time);
    void resetState();

private:
    bool gps_init_flag;
    bool vel_init_flag;
    double time_now;
    Eigen::VectorXd X;//x1:pos_x,x2:pos_y,x3:yaw,x4:v;x5:w
    Eigen::MatrixXd P;
    std::ofstream data_file;
    bool data_record = false;
    void statePrediction(double dt);
    void constrainRadian(double &x);
};
#endif
