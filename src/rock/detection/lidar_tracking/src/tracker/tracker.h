/******************************************************************************
 * Copyright 2019, Ezekiel. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef TRACKER_TRACKER_H_
#define TRACKER_TRACKER_H_

#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPosePDFGaussian.h>

#include <ros/ros.h>
#include <mrpt_bridge/mrpt_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <cmath>
#include <vector>
#include <limits>

namespace tracker {
template <typename PointT>
class Tracker {
  using PointCloudT = typename pcl::PointCloud<PointT>;
  using PointCloudPtr = typename PointCloudT::Ptr;
  using PointCloudConstPtr = typename PointCloudT::ConstPtr;
  using LineModel = typename pcl::SampleConsensusModelLine<PointT>;
  using LineModelPtr = typename LineModel::Ptr;
  using RosPose = geometry_msgs::Pose2D;
  using MrptPose = mrpt::poses::CPose2D;

 public:
  Tracker():
    target_pose_delta_(0, 0, 0),
    goodness_old_(1),
    goodness_thres_(0.7),
    roi_length_(3.6),
    roi_width_(4.2),
    filter_cnt(0),
    filter_cnt_up(50),
    lost_cnt(23),
    target_lost(true),
    vehicle_pose_(0, 0, 0) {}
  virtual ~Tracker() {}
  RosPose::Ptr LocateTarget(const PointCloudPtr &in) {
    if (!in->empty()) {
      goodness_old_ = LocateTargetByIcp(in);

      // 等待滤波器收敛几秒
      if(filter_cnt < filter_cnt_up)
      {
        target_pose_delta_ = target_icp_pose_ - target_icp_pose_old_;
        target_icp_pose_old_ = target_icp_pose_;
        if ((std::abs(target_pose_delta_.phi()) > M_PI / 5) ||
            (std::hypot(target_pose_delta_.x(), target_pose_delta_.y()) > 1.0)) {
          target_pose_delta_ = MrptPose(0, 0, 0);
        }
      }
    } else {
      ROS_WARN("No points in, target by guess!!!");
      // 等待滤波器收敛几秒
      if(filter_cnt < filter_cnt_up)
      {
        target_icp_pose_ = target_icp_pose_old_ + target_pose_delta_;
        target_icp_pose_old_ = target_icp_pose_;
      }
      else
      {
        target_icp_pose_ = init_guess_pose_;
      }
      if(lost_cnt < 30) lost_cnt++;
      if(lost_cnt >= 30)
      {
        target_lost = true;
        ROS_ERROR("Target lost, please rematch!!!");
      }
      else if(lost_cnt < 20) target_lost = false;
    }
    if(filter_cnt < filter_cnt_up) init_guess_pose_ = target_icp_pose_ + target_pose_delta_;
    // printf("goodness:%.2f\n", goodness_old_);
    return PoseMrpt2Ros(vehicle_pose_ + target_icp_pose_);
  }

  RosPose::Ptr LocateTargetBehind(const PointCloudPtr &in) {
    if (!in->empty()) {
      // goodness_old_ = LocateTargetByIcpBehind(in);
      LocateTargetByIcpBehind(in);
    } 
    
    return PoseMrpt2Ros(vehicle_pose_ + target_icp_pose_behind);
  }

  void SetInitGuessPoseInMap(const RosPose::Ptr &rec) {
    // 等待滤波器收敛几秒
    if(filter_cnt < filter_cnt_up)
    {
      filter_cnt++;
      ROS_WARN("Waiting filter!");
    }
    else init_guess_pose_ = PoseRos2Mrpt(rec) - vehicle_pose_;
  }

  void SetInitGuessPoseInMapBehind(const RosPose::Ptr &rec) {
    // 等待滤波器收敛几秒
    if(filter_cnt < filter_cnt_up)
    {
      filter_cnt++;
      ROS_WARN("Waiting filter!");
    }
    else init_guess_pose_behind = PoseRos2Mrpt(rec) - vehicle_pose_;
  }

  void ResetInitGuessPose() {
    init_guess_pose_ = MrptPose(goal_x_set, 0, 0);
    target_icp_pose_ = MrptPose(goal_x_set, 0, 0);
    target_icp_pose_old_ = MrptPose(goal_x_set, 0, 0);
  }

  void ResetInitGuessPoseBehind() {
    init_guess_pose_behind = MrptPose(goal_x_set_behind, 0, 0);
    target_icp_pose_behind = MrptPose(goal_x_set_behind, 0, 0);
    target_icp_pose_old_behind = MrptPose(goal_x_set_behind, 0, 0);
  }

  void SetVehiclePose(const RosPose::ConstPtr &rec) {
    vehicle_pose_ = PoseRos2Mrpt(rec);
  }

  RosPose GetRelativePose(const RosPose::Ptr &rec) {
    MrptPose pose_temp = PoseRos2Mrpt(rec) - vehicle_pose_;
    RosPose relative_pose;
    relative_pose.x = pose_temp.x();
    relative_pose.y = pose_temp.y();
    relative_pose.theta = pose_temp.phi();
    return relative_pose;
  }

  void SetVehicleModels(const std::string &path, const std::vector<std::string> &filenames) {
    for (const std::string &name : filenames) {
      const std::string fullname(path + name);
      mrpt::maps::CSimplePointsMap model;
      model.loadFromPlyFile(fullname);
      vehicle_models_.push_back(model);
    }
  }

  void SetBehindVehicleModels(const std::string &path, const std::vector<std::string> &filenames) {
    for (const std::string &name : filenames) {
      const std::string fullname(path + name);
      mrpt::maps::CSimplePointsMap model;
      model.loadFromPlyFile(fullname);
      behind_vehicle_models_.push_back(model);
    }
  }

  void SetGoodnessThres(double thres) {
    goodness_thres_ = thres;
  }

  void SetRoi(double length, double width) {
    roi_length_ = length;
    roi_width_ = width;
  }

  PointCloudPtr ExtractPointsInRoi(const PointCloudPtr &in) {
    const double x = init_guess_pose_.x();
    const double y = init_guess_pose_.y();
    const double cos_theta = init_guess_pose_.phi_cos();
    const double sin_theta = init_guess_pose_.phi_sin();
    PointCloudPtr out(new PointCloudT);
    for (const PointT &p : in->points) {
      if(p.z < 0.0 || p.z > 2.0) continue;
      const double dis_x =
        std::abs((p.x - x) * sin_theta - (p.y - y) * cos_theta);
      const double dis_y =
        std::abs((p.x - x) * cos_theta + (p.y - y) * sin_theta);
      if ((dis_x < roi_length_ / 2) && (dis_y < roi_width_ / 2)) {
        out->push_back(p);
      }
    }
    return out;
  }

  PointCloudPtr ExtractPointsInRoiBehind(const PointCloudPtr &in) {
    const double x = init_guess_pose_behind.x();
    const double y = init_guess_pose_behind.y();
    const double cos_theta = init_guess_pose_behind.phi_cos();
    const double sin_theta = init_guess_pose_behind.phi_sin();
    PointCloudPtr out(new PointCloudT);
    for (const PointT &p : in->points) {
      const double dis_x =
        std::abs((p.x - x) * sin_theta - (p.y - y) * cos_theta);
      const double dis_y =
        std::abs((p.x - x) * cos_theta + (p.y - y) * sin_theta);
      if ((dis_x < roi_length_ / 2) && (dis_y < roi_width_ / 2)) {
        out->push_back(p);
      }
    }
    return out;
  }

  void GetParameters(const ros::NodeHandle & nh,
                     const ros::NodeHandle & pnh) {
    std::string filepath;
    pnh.param("template_filepath", filepath, std::string("~"));
    if (filepath.back() != '/') {
      filepath.push_back('/');
    }

    std::vector<std::string> filenames;
    pnh.param("template_file_list", filenames, {"vehicle_model.ply"});
    SetVehicleModels(filepath, filenames);

    double goodness_thres;
    pnh.param("goodness_thres", goodness_thres, 0.7);
    SetGoodnessThres(goodness_thres);

    pnh.param("goal_x_set", goal_x_set, 7.0);
    init_guess_pose_ = MrptPose(goal_x_set, 0, 0);
    target_icp_pose_ = MrptPose(goal_x_set, 0, 0);
    target_icp_pose_old_ = MrptPose(goal_x_set, 0, 0);
  }

  void GetBehindParameters(const ros::NodeHandle & nh,
                     const ros::NodeHandle & pnh) {
    std::string filepath;
    pnh.param("template_filepath", filepath, std::string("~"));
    if (filepath.back() != '/') {
      filepath.push_back('/');
    }

    std::vector<std::string> filenames;
    pnh.param("behind_template_file_list", filenames, {"vehicle_model.ply"});
    SetVehicleModels(filepath, filenames);

    double goodness_thres;
    pnh.param("goodness_thres", goodness_thres, 0.7);
    SetGoodnessThres(goodness_thres);

    pnh.param("goal_x_set_behind", goal_x_set, 7.0);
    init_guess_pose_ = MrptPose(goal_x_set, 0, 0);
    target_icp_pose_ = MrptPose(goal_x_set, 0, 0);
    target_icp_pose_old_ = MrptPose(goal_x_set, 0, 0);
  }

  bool GetTargetLost(){
    return target_lost;
  }

  // void rematch(const std_msgs::Float64::ConstPtr &msg){
  //   init_guess_pose_ = MrptPose(msg->data, 0, 0);
  //   target_icp_pose_ = MrptPose(msg->data, 0, 0);
  //   target_icp_pose_old_ = MrptPose(msg->data, 0, 0);
  // }

 private:
  RosPose::Ptr PoseMrpt2Ros(const MrptPose &in) const {
    RosPose::Ptr out(new RosPose);
    out->x = in.x();
    out->y = in.y();
    out->theta = in.phi();
    return out;
  }
  MrptPose PoseRos2Mrpt(const RosPose::Ptr &in) const {
    return MrptPose(in->x, in->y, in->theta);
  }
  MrptPose PoseRos2Mrpt(const RosPose::ConstPtr &in) const {
    return MrptPose(in->x, in->y, in->theta);
  }
  double LocateTargetByIcp(const PointCloudPtr &in) {
    //convert pcl to mrpt format
    sensor_msgs::PointCloud2 point_cloud2_msg;
    pcl::toROSMsg(*in, point_cloud2_msg);
    mrpt::maps::CSimplePointsMap converted;
    mrpt_bridge::copy(point_cloud2_msg, converted);
    mrpt::maps::CSimplePointsMap raw_points;
    raw_points.insertionOptions.minDistBetweenLaserPoints = 0.05;
    raw_points.insertionOptions.fuseWithExisting = true;
    raw_points.insertionOptions.addToExistingPointsMap = true;
    raw_points.enableFilterByHeight(true);
    raw_points.setHeightFilterLevels(-1, 1.8);
    converted.extractPoints(mrpt::math::TPoint3D(60, 60, 3),
                            mrpt::math::TPoint3D(-60, -60, -3),
                            &raw_points);

    // printf("init_guess_pose_=(%.2f,%.2f,%.2f)\n",
    //        init_guess_pose_.x(),
    //        init_guess_pose_.y(),
    //        init_guess_pose_.phi());
    mrpt::poses::CPosePDFGaussian init_guess_pdf(init_guess_pose_);
    float running_time;
    mrpt::slam::CICP::TReturnInfo info;
    mrpt::poses::CPosePDFPtr pdf;
    int model_cnt = -1;
    for (const auto &model : vehicle_models_) {
      pdf = icp_.AlignPDF(&raw_points,  //reference
                          &model, //to be aligned
                          init_guess_pdf, //initial guess
                          &running_time,
                          (void*)&info);
      model_cnt++;
      if (info.goodness > goodness_thres_) {
        // std::cout << "match model: " << model_cnt << std::endl;
        break;
      }
    }
    ROS_INFO("Match goodness %f ", info.goodness);
    mrpt::math::CMatrixDouble33 matching_cov;
    pdf->getCovarianceAndMean(matching_cov, target_icp_pose_);
    if (info.goodness < goodness_thres_) {
      target_icp_pose_ = init_guess_pose_;
      ROS_WARN("Can not matching, target by guess!!!");
      if(lost_cnt < 30) lost_cnt++;
    }
    else
    {
      if(lost_cnt > 0) lost_cnt--;
    }

    if(lost_cnt >= 30)
    {
      target_lost = true;
      ROS_ERROR("Target lost, please rematch!!!");
    }
    else if(lost_cnt < 20) target_lost = false;

    // printf("target_icp_pose_=(%.2f,%.2f,%.2f)\n",
    //        target_icp_pose_.x(),
    //        target_icp_pose_.y(),
    //        target_icp_pose_.phi());
    return info.goodness;
  }

  double LocateTargetByIcpBehind(const PointCloudPtr &in) {
    //convert pcl to mrpt format
    sensor_msgs::PointCloud2 point_cloud2_msg;
    pcl::toROSMsg(*in, point_cloud2_msg);
    mrpt::maps::CSimplePointsMap converted;
    mrpt_bridge::copy(point_cloud2_msg, converted);
    mrpt::maps::CSimplePointsMap raw_points;
    raw_points.insertionOptions.minDistBetweenLaserPoints = 0.05;
    raw_points.insertionOptions.fuseWithExisting = true;
    raw_points.insertionOptions.addToExistingPointsMap = true;
    raw_points.enableFilterByHeight(true);
    raw_points.setHeightFilterLevels(-1, 1.8);
    converted.extractPoints(mrpt::math::TPoint3D(60, 60, 3),
                            mrpt::math::TPoint3D(-60, -60, -3),
                            &raw_points);

    printf("init_guess_pose_behind_=(%.2f,%.2f,%.2f)\n",
           init_guess_pose_behind.x(),
           init_guess_pose_behind.y(),
           init_guess_pose_behind.phi());
    mrpt::poses::CPosePDFGaussian init_guess_pdf(init_guess_pose_behind);
    float running_time;
    mrpt::slam::CICP::TReturnInfo info;
    mrpt::poses::CPosePDFPtr pdf;
    int model_cnt = -1;
    for (const auto &model : behind_vehicle_models_) {
      pdf = icp_.AlignPDF(&raw_points,  //reference
                          &model, //to be aligned
                          init_guess_pdf, //initial guess
                          &running_time,
                          (void*)&info);
      model_cnt++;
      std::cout << "match model: " << model_cnt << " goodness: " << info.goodness << std::endl;
    }
    mrpt::math::CMatrixDouble33 matching_cov;
    pdf->getCovarianceAndMean(matching_cov, target_icp_pose_behind);
    if (info.goodness < goodness_thres_) {
      target_icp_pose_behind = init_guess_pose_behind;
      ROS_WARN("Can not matching, target by guess!!!");
    }

    printf("target_icp_pose_behind=(%.2f,%.2f,%.2f)\n",
           target_icp_pose_behind.x(),
           target_icp_pose_behind.y(),
           target_icp_pose_behind.phi());
    return info.goodness;
  }

  MrptPose init_guess_pose_;
  MrptPose target_icp_pose_;
  MrptPose target_icp_pose_old_;
  MrptPose target_pose_delta_;
  double goal_x_set;

  MrptPose init_guess_pose_behind;
  MrptPose target_icp_pose_behind;
  MrptPose target_icp_pose_old_behind;
  MrptPose target_pose_delta_behind;
  double goal_x_set_behind;

  // ICP algorithm parameters
  mrpt::slam::CICP icp_;
  std::vector<mrpt::maps::CSimplePointsMap> vehicle_models_;
  std::vector<mrpt::maps::CSimplePointsMap> behind_vehicle_models_;
  double goodness_old_;
  double goodness_thres_;

  // Region of interesting
  double roi_length_;
  double roi_width_;
  int filter_cnt;
  int filter_cnt_up;
  std::string name_;
  int lost_cnt;
  bool target_lost;
  MrptPose vehicle_pose_;
};
}

#endif
