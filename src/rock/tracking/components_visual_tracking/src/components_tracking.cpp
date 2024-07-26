#include "components_visual_tracking/components_tracking.h"
#include <algorithm>
#include <memory>

using namespace tracking::visual_tracking;

ComponentsTrackingWrapper::ComponentsTrackingWrapper(
    ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
    : nh_(node_handle), pnh_(private_node_handle) {
  sub_image_ = nh_.subscribe("/driver/fisheye/front/compressed", 2,
                             &ComponentsTrackingWrapper::image_callback, this);
  sub_visual_det_ =
      nh_.subscribe("/tracking/visual_detection", 2,
                    &ComponentsTrackingWrapper::visual_det_callback, this);
  sub_vis_rematch_ =
      nh_.subscribe("/tracking/components_tracking/rematch", 2,
                    &ComponentsTrackingWrapper::rematch_callback, this);
  pub_rearlight_ = nh_.advertise<cyber_msgs::Box2DArray>(
      "/tracking/components_tracking/rearlight", 2);
  pub_vh_ =
      nh_.advertise<cyber_msgs::Box2D>("/tracking/components_tracking/vh", 2);
  timer_ = nh_.createTimer(ros::Duration(0.1),
                           &ComponentsTrackingWrapper::timer_callback, this);

  tracker = ComponentsTracking();

  // night flag
  pnh_.param("/is_night", is_night, false);

  // tracking init params, for offline
  double vh_1_x, vh_1_y, vh_2_x, vh_2_y;
  double left_1_x, left_1_y, left_2_x, left_2_y;
  double right_1_x, right_1_y, right_2_x, right_2_y;
  bool use_init_param;
  pnh_.param("/use_init_param", use_init_param, false);
  if (use_init_param) {
    pnh_.param("/vh_1_x", vh_1_x, -1.0);
    pnh_.param("/vh_1_y", vh_1_y, -1.0);
    pnh_.param("/vh_2_x", vh_2_x, -1.0);
    pnh_.param("/vh_2_y", vh_2_y, -1.0);
    pnh_.param("/left_1_x", left_1_x, -1.0);
    pnh_.param("/left_1_y", left_1_y, -1.0);
    pnh_.param("/left_2_x", left_2_x, -1.0);
    pnh_.param("/left_2_y", left_2_y, -1.0);
    pnh_.param("/right_1_x", right_1_x, -1.0);
    pnh_.param("/right_1_y", right_1_y, -1.0);
    pnh_.param("/right_2_x", right_2_x, -1.0);
    pnh_.param("/right_2_y", right_2_y, -1.0);

    cv::Rect2d bbox_vh{cv::Point2d(vh_1_x, vh_1_y),
                       cv::Point2d(vh_2_x, vh_2_y)};
    cv::Rect2d bbox_left{cv::Point2d(left_1_x, left_1_y),
                         cv::Point2d(left_2_x, left_2_y)};
    cv::Rect2d bbox_right{cv::Point2d(right_1_x, right_1_y),
                          cv::Point2d(right_2_x, right_2_y)};
    tracker.set_init(bbox_vh, bbox_left, bbox_right);
  }
}

void ComponentsTrackingWrapper::image_callback(
    const sensor_msgs::CompressedImageConstPtr &image_in) {
  image_ = cv::imdecode(cv::Mat(image_in->data), 1);
  img_header_ = image_in->header;

  // cv::imwrite("/home/wuhr/Workspace/cyberc3_platooning/init_images/init.jpg",
  //             image_);
  // ROS_WARN("save image");
}

void ComponentsTrackingWrapper::visual_det_callback(
    const cyber_msgs::Box2DArrayConstPtr &det_in) {
  std::vector<cyber_msgs::Box2D> vis_det;
  for (const auto obj : det_in->boxes) {
    vis_det.emplace_back(obj);
  }
  tracker.update_vis_det(vis_det);
  cv::Rect2d bbox_vh, bbox_left, bbox_right;
  tracker.detect(image_, bbox_vh, bbox_left, bbox_right, true);
  // visual_dets_pt = tracker.get_vis_det_pt();
  // ROS_INFO("det received");
}

void ComponentsTrackingWrapper::timer_callback(const ros::TimerEvent &event) {
  if (image_.empty())
    return;

  auto header = img_header_;
  cv::Rect2d bbox_vh, bbox_left, bbox_right;
  tracker.detect(image_, bbox_vh, bbox_left, bbox_right);
  // if (((bbox_left & bbox_vh) == cv::Rect2d() ||
  //      (bbox_right & bbox_vh) == cv::Rect2d()) &&
  //     tracker.is_init()) {
  //   tracker.rematch(image_, is_night, bbox_left, bbox_right);
  //   rematch_flag = true;
  // }

  cyber_msgs::Box2D bbox_vh_msg, bbox_left_msg, bbox_right_msg;
  cyber_msgs::Box2DArray rearlight_bboxes_msg;

  bbox_vh_msg.header = header;
  bbox_vh_msg.center_x = static_cast<int>(bbox_vh.y + bbox_vh.height / 2);
  bbox_vh_msg.center_y = static_cast<int>(bbox_vh.x + bbox_vh.width / 2);
  bbox_vh_msg.width = static_cast<int>(bbox_vh.width);
  bbox_vh_msg.height = static_cast<int>(bbox_vh.height);
  pub_vh_.publish(bbox_vh_msg);

  // rearlight_bboxes_msg.header = header;
  // bbox_left_msg.header = header;
  // bbox_left_msg.center_x = static_cast<int>(bbox_left.y + bbox_left.height / 2);
  // bbox_left_msg.center_y = static_cast<int>(bbox_left.x + bbox_left.width / 2);
  // bbox_left_msg.width = static_cast<int>(bbox_left.width);
  // bbox_left_msg.height = static_cast<int>(bbox_left.height);
  // bbox_left_msg.score = 0.1;
  // bbox_right_msg.header = header;
  // bbox_right_msg.center_x =
  //     static_cast<int>(bbox_right.y + bbox_right.height / 2);
  // bbox_right_msg.center_y =
  //     static_cast<int>(bbox_right.x + bbox_right.width / 2);
  // bbox_right_msg.width = static_cast<int>(bbox_right.width);
  // bbox_right_msg.height = static_cast<int>(bbox_right.height);
  // bbox_right_msg.score = 0.1;
  // rearlight_bboxes_msg.boxes.emplace_back(bbox_left_msg);
  // rearlight_bboxes_msg.boxes.emplace_back(bbox_right_msg);
  // pub_rearlight_.publish(rearlight_bboxes_msg);
}

void ComponentsTrackingWrapper::rematch_callback(
    const std_msgs::Bool::ConstPtr &msg) {
  return;
  if (msg->data) {
    cv::Rect2d new_bbox_left, new_bbox_right;
    tracker.rematch(image_, is_night, new_bbox_left, new_bbox_right);
    rematch_flag = true;
  }
}

//============================================================================
ComponentsTracking::ComponentsTracking()
    : f_tracker_left_(true, false, true, true),
      f_tracker_right_(true, false, true, true) {
  cv::VehicleTrackerKCF::Params param_vh;
  param_vh.desc_pca = cv::TrackerKCF::GRAY | cv::TrackerKCF::CN;
  cv::CustomTrackerKCF::Params param_rearlight;
  param_rearlight.desc_pca = cv::TrackerKCF::GRAY | cv::TrackerKCF::CN;
  // param.desc_npca = 0;
  // param.compress_feature = true;
  // param.compressed_size = 2;
  tracker_vh_ = cv::VehicleTrackerKCF::create(param_vh);
  tracker_left_ = cv::CustomTrackerKCF::create(param_rearlight);
  tracker_right_ = cv::CustomTrackerKCF::create(param_rearlight);
  tracker_init_ = false;
  bbox_init_ = false;

  velo_vehicle_ = {0.0, 0.0};
}

bool ComponentsTracking::init(const cv::Mat &image) {
  if (!tracker_init_) {
    if (vis_det_.size() == 0)
      return false;

    ROS_WARN("init!!!!!!!!!!!!!");

    if (!bbox_init_) {
      auto bbox_vh_init = cv::selectROI("vehicle", image);
      // match best visual detection as initial vehicle bbox
      while (!match_vehicle_detection(bbox_vh_init, bbox_vh_)) {
      }

      // bbox_left_ = cv::selectROI("tracker_left", image);
      // if (bbox_left_.width == 0 || bbox_left_.height == 0)
      //   return false;
      // left_rearlight_bbox_pt = &bbox_left_;
      // bbox_right_ = cv::selectROI("tracker_right", image);
      // if (bbox_right_.width == 0 || bbox_right_.height == 0)
      //   return false;
      // right_rearlight_bbox_pt = &bbox_right_;
    }

    // std::cout << "vh_x: " << bbox_vh_.x << " vh_y: " << bbox_vh_.y
    //           << " vh_width: " << bbox_vh_.width
    //           << " vh_height: " << bbox_vh_.height << std::endl;
    // std::cout << "left_x: " << bbox_left_.x << " left_y: " << bbox_left_.y
    //           << " left_width: " << bbox_left_.width
    //           << " left_height: " << bbox_left_.height << std::endl;
    // std::cout << "right_x: " << bbox_right_.x << " right_y: " << bbox_right_.y
    //           << " right_width: " << bbox_right_.width
    //           << " right_height: " << bbox_right_.height << std::endl;

    tracker_vh_->init(image.clone(), bbox_vh_);
    vehicle_bbox_pt = &bbox_vh_;

    // // init rearlight tracker
    // cv::Mat masked_image;
    // cv::Mat roi_mask = cv::Mat::zeros(image.size(), CV_8UC1);
    // roi_mask(bbox_vh_).setTo(255);
    // cv::bitwise_and(image, image, masked_image, roi_mask);
    // tracker_left_->init(masked_image.clone(), bbox_left_);
    // tracker_right_->init(masked_image.clone(), bbox_right_);

    // cv::Mat gray;
    // cv::cvtColor(masked_image, gray, CV_BGR2GRAY);
    // f_tracker_left_.init(bbox_left_, gray);
    // f_tracker_right_.init(bbox_right_, gray);

    tracker_init_ = true;
    return true;
  }
  return true;
}

void ComponentsTracking::detect(const cv::Mat &image, cv::Rect2d &bbox_vh,
                                cv::Rect2d &bbox_left, cv::Rect2d &bbox_right,
                                const bool update_vh) {
  if (!tracker_init_)
    init(image);
  if (!tracker_init_)
    return;

  auto t_1 = ros::WallTime::now();

  if (update_vh) {
    ROS_INFO("update vehicle");
    auto bbox_vh_cp = bbox_vh_;
    tracker_vh_->update(image.clone(), bbox_vh_);
    return;
  }

  // ROS_INFO("update rearlight");
  // cv::Mat masked_image;
  // cv::Mat roi_mask = cv::Mat::zeros(image.size(), CV_8UC1);
  // roi_mask(bbox_vh_).setTo(255);
  // cv::bitwise_and(image, image, masked_image, roi_mask);
  // std::cout << "curr left: " << bbox_left_ << std::endl;
  // std::cout << "curr right: " << bbox_right_ << std::endl;
  // if (rematch_flag) {
  //   ROS_WARN("rematch tracking");
  // }
  // // tracker_left_->update(masked_image.clone(), bbox_left_);
  // // tracker_right_->update(masked_image.clone(), bbox_right_);
  // rematch_flag = false;
  // // std::cout << "final left: " << bbox_left_ << std::endl;
  // // std::cout << "final right: " << bbox_right_ << std::endl;

  // // update with fDSST
  // cv::Mat gray;
  // cv::cvtColor(masked_image, gray, CV_BGR2GRAY);
  // bbox_left_ = f_tracker_left_.update(gray);
  // bbox_right_ = f_tracker_right_.update(gray);



  // bbox_left = bbox_left_;
  // bbox_right = bbox_right_;
  // // ROS_INFO("roi_x: %.1f, roi_y: %.1f, roi_w: %.1f, roi_h: %.1f", roi.x,
  // // roi.y,
  // //          roi.width, roi.height);

  // auto t_2 = ros::Time::now();
  // ROS_INFO("cost %.4f sec", t_2.toSec() - t_1.toSec());

  // cv::Mat img_show = image.clone();
  // cv::rectangle(img_show, bbox_vh_, cv::Scalar(0, 255, 128), 2, 1);
  // cv::rectangle(img_show, bbox_left_, cv::Scalar(0, 255, 0), 2, 1);
  // cv::rectangle(img_show, bbox_right_, cv::Scalar(0, 255, 0), 2, 1);

  // std::string img_name = "/media/wuhr/data/platoon_dataset/2022_10_20/"
  //                        "evaluation/2d/our_method/vis/";
  // auto ros_time = ros::Time::now().toSec();
  // img_name += std::to_string(ros_time);
  // img_name += ".jpg";
  // cv::imwrite(img_name, img_show);

  // cv::imshow("tracker", img_show);
  // cv::waitKey(1);
  // ROS_INFO("plot");
}

bool ComponentsTracking::match_vehicle_detection(const cv::Rect2d &roi,
                                                 cv::Rect2d &proposal) {
  if (vis_det_.size() == 0) {
    ROS_WARN("No visual detection received yet!");
    return false;
  }

  auto center_x = roi.x + roi.width / 2;
  auto center_y = roi.y + roi.height / 2;

  std::vector<std::pair<int, float>> candidates;
  for (int i = 0; i < vis_det_.size(); ++i) {
    const auto obj = vis_det_[i];
    auto obj_min_x = obj.center_y - obj.width / 2;
    auto obj_max_x = obj.center_y + obj.width / 2;
    auto obj_min_y = obj.center_x - obj.height / 2;
    auto obj_max_y = obj.center_x + obj.height / 2;
    if (center_x >= obj_min_x && center_x <= obj_max_x &&
        center_y >= obj_min_y && center_y <= obj_max_y)
      candidates.emplace_back(std::make_pair(i, obj.score));
  }
  auto cmp = [](const std::pair<int, float> &a,
                const std::pair<int, float> &b) {
    return a.second >= b.second;
  };
  sort(candidates.begin(), candidates.end(), cmp);
  auto best_idx = candidates.front().first;
  proposal.x = vis_det_[best_idx].center_y - vis_det_[best_idx].width / 2;
  proposal.y = vis_det_[best_idx].center_x - vis_det_[best_idx].height / 2;
  proposal.width = vis_det_[best_idx].width;
  proposal.height = vis_det_[best_idx].height;

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "components_tracking");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");

  ComponentsTrackingWrapper tracker(node_handle, private_node_handle);
  ros::spin();
  // ros::MultiThreadedSpinner spinner(2);
  // spinner.spin();
}
