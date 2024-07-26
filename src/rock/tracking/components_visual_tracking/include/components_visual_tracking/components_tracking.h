// #ifndef COMPONENTS_TRACKING_H
// #define COMPONENTS_TRACKING_H
#pragma once

#include <iostream>

#include <cv_bridge/cv_bridge.h>
#include <cyber_msgs/Box2D.h>
#include <cyber_msgs/Box2DArray.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

#include "custom_kcf/custom_kcf.h"
#include "custom_kcf/vehicle_kcf.h"

#include "FDSST/fdssttracker.hpp"

// pointer to visual detection result
std::vector<cyber_msgs::Box2D> *visual_dets_pt = nullptr;
// pointer to vehicle tracking result
cv::Rect2d *vehicle_bbox_pt = nullptr;
// pointer to rearlight tracking result
cv::Rect2d *left_rearlight_bbox_pt = nullptr,
           *right_rearlight_bbox_pt = nullptr;
// pointer to vehicle image speed
cv::Point2f *velo_vehicle_pt = nullptr;
// night flag
bool is_night = false;
bool rematch_flag = false;

namespace tracking {
namespace visual_tracking {
class ComponentsTracking {
public:
  ComponentsTracking();
  ~ComponentsTracking() {
    visual_dets_pt = nullptr;
    vehicle_bbox_pt = nullptr;
    left_rearlight_bbox_pt = right_rearlight_bbox_pt = nullptr;
    velo_vehicle_pt = nullptr;
  }

private:
  std::vector<cyber_msgs::Box2D> vis_det_;
  cv::Rect2d bbox_vh_, bbox_left_, bbox_right_;
  bool bbox_init_;
  bool tracker_init_;
  cv::Ptr<cv::Tracker> tracker_vh_, tracker_left_, tracker_right_;
  FDSSTTracker f_tracker_left_, f_tracker_right_;
  cv::Point2f velo_vehicle_; // vehicle velocity in image

public:
  bool is_init() const {return tracker_init_ && bbox_init_;}
  void detect(const cv::Mat &image, cv::Rect2d &bbox_vh, cv::Rect2d &bbox_left,
              cv::Rect2d &bbox_right, const bool update_vh=false);
  bool init(const cv::Mat &image);
  void rematch(const cv::Mat &image, const bool is_night, cv::Rect2d &new_bbox_left, cv::Rect2d &new_bbox_right) {
    std::vector<cv::Rect2d> proposals;
    bool succ = rearlight_proposal::searchInitProposal(image, bbox_vh_,
                                                       is_night, proposals);
    if (succ) {
      bbox_left_ = proposals[0];
      bbox_right_ = proposals[1];
      new_bbox_left = proposals[0];
      new_bbox_right = proposals[1];
      ROS_WARN("rematch success");
      return;
    }
    ROS_WARN("rematch fail");
    new_bbox_left = cv::Rect2d();
    new_bbox_right = cv::Rect2d();
  }

  void set_init(const cv::Rect2d &bbox_vh, const cv::Rect2d &bbox_left,
                const cv::Rect2d &right) {
    bbox_vh_ = bbox_vh;
    bbox_left_ = bbox_left;
    bbox_right_ = right;
    bbox_init_ = true;
  }
  bool is_init() { return tracker_init_; }
  void update_vis_det(const std::vector<cyber_msgs::Box2D> &vis_det) {
    // ROS_WARN("update vis_det");
    vis_det_ = vis_det;
    visual_dets_pt = get_vis_det_pt();
  }
  std::vector<cyber_msgs::Box2D> *get_vis_det_pt() {
    if (vis_det_.size() == 0)
      return nullptr;
    else
      return &vis_det_;
  }

private:
  bool match_vehicle_detection(const cv::Rect2d &roi, cv::Rect2d &proposal);
};

class ComponentsTrackingWrapper {

public:
  ComponentsTrackingWrapper(ros::NodeHandle node_handle,
                            ros::NodeHandle private_node_handle);
  ~ComponentsTrackingWrapper() = default;

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_image_;
  ros::Subscriber sub_visual_det_;
  ros::Subscriber sub_vis_rematch_;
  ros::Publisher pub_rearlight_;
  ros::Publisher pub_vh_;
  ros::Timer timer_;

  cv::Mat image_;
  std_msgs::Header img_header_;

  ComponentsTracking tracker;

  void image_callback(const sensor_msgs::CompressedImageConstPtr &image_in);
  void visual_det_callback(const cyber_msgs::Box2DArrayConstPtr &det_in);
  void timer_callback(const ros::TimerEvent &event);
  void rematch_callback(const std_msgs::Bool::ConstPtr &msg);
};
} // namespace visual_tracking

} // namespace tracking

// #endif