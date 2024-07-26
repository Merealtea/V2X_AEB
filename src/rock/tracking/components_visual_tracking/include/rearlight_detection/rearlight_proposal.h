#pragma once

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <tuple>
#include <vector>

namespace tracking {
namespace rearlight_proposal {

bool searchInitProposal(const cv::Mat &image, const cv::Rect2d &bounding_box,
                        const bool is_night,
                        std::vector<cv::Rect2d> &proposals) {
  // apply roi
  cv::Mat roi_mask = cv::Mat::zeros(image.size(), CV_8UC1);
  roi_mask(bounding_box).setTo(255);
  cv::Mat img_roi;
  cv::bitwise_and(image, image, img_roi, roi_mask);

  // get hsv
  cv::Mat img_hsv;
  cv::cvtColor(img_roi, img_hsv, CV_BGR2HSV);

  // segmentation
  cv::Mat red_mask;
  if (is_night) {
    cv::Mat red_mask_1, red_mask_2, white_mask;
    cv::inRange(img_hsv, cv::Scalar(0, 50, 150), cv::Scalar(10, 255, 255),
                red_mask_1);
    cv::inRange(img_hsv, cv::Scalar(170, 50, 150), cv::Scalar(180, 255, 255),
                red_mask_2);
    cv::bitwise_or(red_mask_1, red_mask_2, red_mask);
  } else {
    cv::Mat red_mask_1, red_mask_2;
    cv::inRange(img_hsv, cv::Scalar(0, 70, 20), cv::Scalar(10, 255, 255),
                red_mask_1);
    cv::inRange(img_hsv, cv::Scalar(170, 70, 20), cv::Scalar(180, 255, 255),
                red_mask_2);
    cv::bitwise_or(red_mask_1, red_mask_2, red_mask);
  }

  // morphology
  auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
  cv::morphologyEx(red_mask, red_mask, CV_MOP_CLOSE, kernel, cv::Point(-1, -1),
                   2);
  cv::morphologyEx(red_mask, red_mask, CV_MOP_OPEN, kernel);

  // connected component extraction
  cv::Mat labels, stats, centroids;
  int nccomps =
      cv::connectedComponentsWithStats(red_mask, labels, stats, centroids);
  // extract possible ROIs from connected components
  std::vector<cv::Rect2d> prop_candidates;
  for (int i = 1; i < nccomps; ++i) // dismiss background 0
  {
    auto area = stats.at<int>(i, cv::CC_STAT_AREA);
    if (area < 50 || area > 10000)
      continue;
    auto ccomp_x = stats.at<int>(i, cv::CC_STAT_LEFT);
    auto ccomp_y = stats.at<int>(i, cv::CC_STAT_TOP);
    auto width = stats.at<int>(i, cv::CC_STAT_WIDTH);
    auto height = stats.at<int>(i, cv::CC_STAT_HEIGHT);

    std::cout << "ccomp x: " << ccomp_x << " y: " << ccomp_y << " w: " << width
              << " h: " << height << std::endl;

    cv::Rect2d ccomp_box = cv::Rect2d(ccomp_x, ccomp_y, width, height);
    prop_candidates.emplace_back(ccomp_box);
  }
  // find best left & right
  std::vector<std::tuple<int, int, double>> left_right_matches;
  for (int i = 0; i < prop_candidates.size(); ++i) {
    for (int j = i + 1; j < prop_candidates.size(); ++j) {
      auto box_1_y = prop_candidates[i].y + prop_candidates[i].height / 2;
      auto box_2_y = prop_candidates[j].y + prop_candidates[j].height / 2;
      auto height_diff = std::abs(box_1_y - box_2_y);
      left_right_matches.emplace_back(std::make_tuple(i, j, height_diff));
    }
  }
  auto match_cmp = [](const std::tuple<int, int, double> &a,
                      const std::tuple<int, int, double> &b) {
    return std::get<2>(a) <= std::get<2>(b);
  };
  std::sort(left_right_matches.begin(), left_right_matches.end(), match_cmp);
  int best_pair_idx = -1;
  double max_pair_dis = -1e6;
  // std::cout << "num of pair: " << left_right_matches.size() << std::endl;
  for (int i = 0; i < std::min(3, static_cast<int>(left_right_matches.size()));
       ++i) {
    auto prop_1 = prop_candidates[std::get<0>(left_right_matches[i])];
    auto prop_2 = prop_candidates[std::get<1>(left_right_matches[i])];
    auto prop_1_x = prop_1.x + prop_1.width / 2;
    auto prop_2_x = prop_2.x + prop_2.width / 2;
    auto pair_dis = std::abs(prop_1_x - prop_2_x);
    if (pair_dis > max_pair_dis) {
      max_pair_dis = pair_dis;
      best_pair_idx = i;
    }
  }
  if (best_pair_idx == -1)
    return false;
  else {
    std::cout << "prop 1 idx: "
              << std::get<0>(left_right_matches[best_pair_idx])
              << " prop 2 idx: "
              << std::get<1>(left_right_matches[best_pair_idx]) << std::endl;
    auto prop_1 =
        prop_candidates[std::get<0>(left_right_matches[best_pair_idx])];
    auto prop_2 =
        prop_candidates[std::get<1>(left_right_matches[best_pair_idx])];
    if (prop_1.x + prop_1.width / 2 <= prop_2.x + prop_2.width / 2) {
      proposals.emplace_back(prop_1);
      proposals.emplace_back(prop_2);
    } else {
      proposals.emplace_back(prop_2);
      proposals.emplace_back(prop_1);
    }
    return true;
  }
}

void getRearlightProposal(const cv::Mat &image, const cv::Rect2d &bounding_box,
                          const bool is_night,
                          const std::vector<std::pair<int, int>> &thresh,
                          std::vector<cv::Rect2d> &proposals) {
  // image: new frame
  // bounding_box: last frame rearlight state

  // ROS_INFO("propose start");
  // apply roi
  cv::Rect2d roi = bounding_box;
  auto center_x = roi.x + roi.width / 2;
  auto center_y = roi.y + roi.height / 2;
  roi.width *= 3.0;
  roi.height *= 2.5;
  roi.x = center_x - roi.width / 2;
  roi.y = center_y - roi.height / 2;
  cv::Mat roi_mask = cv::Mat::zeros(image.size(), CV_8UC1);
  roi_mask(roi).setTo(255);
  cv::Mat img_roi;
  cv::bitwise_and(image, image, img_roi, roi_mask);
  // get HSV
  cv::Mat img_hsv;
  cv::cvtColor(img_roi, img_hsv, CV_BGR2HSV);

  // ROS_INFO("propose A");
  // daytime or nighttime
  // cv::Scalar feat_sum = cv::sum(img_hsv); // shape: (3,)
  // auto avg_brightness = feat_sum[2] / (img_hsv.rows * img_hsv.cols);
  // bool is_night =
  //     avg_brightness < 80.0; // TODO: threshold for night, to be tuned
  int max_thresh_num = is_night ? 4 : 2;

  // ROS_WARN("night: %d", is_night);

  // ROS_INFO("propose B");
  // segment red area
  cv::Mat final_mask = cv::Mat::zeros(image.size(), CV_8UC1);
  for (int i = 0; i < std::min(static_cast<int>(thresh.size()), max_thresh_num);
       ++i) {
    cv::Mat red_mask;
    cv::Mat red_mask_1;
    cv::Mat red_mask_2;
    int th_s, th_v;
    if (is_night) {
      th_s = std::max(thresh[i].first, 10);
      th_v = std::max(thresh[i].second, 140);
    } else {
      th_s = std::max(thresh[i].first, 50);
      th_v = std::max(thresh[i].second, 20);
    }
    std::cout << "final_s = " << th_s << " final_v = " << th_v << std::endl;
    cv::inRange(img_hsv, cv::Scalar(0, th_s, th_v), cv::Scalar(10, 255, 255),
                red_mask_1);
    cv::inRange(img_hsv, cv::Scalar(170, th_s, th_v), cv::Scalar(180, 255, 255),
                red_mask_2);
    cv::bitwise_or(red_mask_1, red_mask_2, red_mask);
    cv::bitwise_or(final_mask, red_mask, final_mask);

    if (is_night) {
      // cv::Mat supp_mask_1, supp_mask_2;
      // cv::inRange(img_hsv, cv::Scalar(0, th_s, 100), cv::Scalar(15, 255,
      // 255),
      //             supp_mask_1);
      // cv::inRange(img_hsv, cv::Scalar(165, th_s, 100),
      //             cv::Scalar(180, 255, 255), supp_mask_2);
      // cv::bitwise_or(final_mask, supp_mask_1, final_mask);
      // cv::bitwise_or(final_mask, supp_mask_2, final_mask);
    } else {
      cv::Mat supp_mask_1, supp_mask_2;
      cv::inRange(img_hsv, cv::Scalar(0, th_s, 20), cv::Scalar(15, 255, 255),
                  supp_mask_1);
      cv::inRange(img_hsv, cv::Scalar(165, th_s, 20), cv::Scalar(180, 255, 255),
                  supp_mask_2);
      cv::bitwise_or(final_mask, supp_mask_1, final_mask);
      cv::bitwise_or(final_mask, supp_mask_2, final_mask);
    }
  }

  // cv::Mat white_mask, white_mask_1, white_mask_2;
  // cv::inRange(img_hsv, cv::Scalar(0, 0, 150), cv::Scalar(180, 70, 255),
  //             white_mask_1);
  // cv::inRange(img_hsv, cv::Scalar(0, 0, 150), cv::Scalar(180, 70, 255),
  //             white_mask_2);
  // cv::bitwise_or(white_mask_1, white_mask_2, white_mask);
  // cv::Mat img_white = image.clone();
  // img_white.setTo(cv::Scalar(255, 0, 255), white_mask);
  // cv::imshow("white", img_white);
  // cv::waitKey(1);
  // cv::bitwise_or(final_mask, white_mask, final_mask);

  // cv::Mat r_mask, r_mask_1, r_mask_2;
  // cv::inRange(img_hsv, cv::Scalar(0, 20, 150), cv::Scalar(10, 255, 255),
  // r_mask_1); cv::inRange(img_hsv, cv::Scalar(170, 20, 150), cv::Scalar(180,
  // 255, 255), r_mask_2); cv::bitwise_or(r_mask_1, r_mask_2, r_mask); cv::Mat
  // img_red = image.clone(); img_red.setTo(cv::Scalar(255, 0, 0), r_mask);
  // cv::imshow("red", img_red);
  // cv::waitKey(1);

  // cv::Mat red_mask;
  // cv::Mat red_mask_1;
  // cv::Mat red_mask_2;
  // cv::Mat supp_mask_1;
  // cv::Mat supp_mask_2;
  // int th_s, th_v;
  // if (is_night) {
  //   th_s = std::max(thresh_s, 50);
  //   th_v = std::max(thresh_v, 150);
  // } else {
  //   th_s = std::max(thresh_s, 20);
  //   th_v = std::max(thresh_v, 20);
  // }
  // std::cout << "final_s = " << th_s << " final_v = " << th_v << std::endl;
  // cv::inRange(img_hsv, cv::Scalar(0, th_s, th_v), cv::Scalar(10, 255, 255),
  //             red_mask_1);
  // cv::inRange(img_hsv, cv::Scalar(170, th_s, th_v), cv::Scalar(180, 255,
  // 255),
  //             red_mask_2);
  // cv::bitwise_or(red_mask_1, red_mask_2, red_mask);
  // // cv::inRange(img_hsv, cv::Scalar(0, 50, th_v), cv::Scalar(10, 255, 255),
  // //             supp_mask_1);
  // // cv::inRange(img_hsv, cv::Scalar(170, 50, th_v), cv::Scalar(180, 255,
  // 255),
  // //             supp_mask_2);
  // cv::bitwise_or(red_mask, supp_mask_1, red_mask);
  // cv::bitwise_or(red_mask, supp_mask_2, red_mask);
  // if (is_night) {
  //   cv::Mat white_mask_1;
  //   cv::Mat white_mask_2;
  //   cv::inRange(img_hsv, cv::Scalar(0, 0, 180), cv::Scalar(15, 30, 255),
  //               white_mask_1);
  //   cv::inRange(img_hsv, cv::Scalar(165, 0, 180), cv::Scalar(180, 30, 255),
  //               white_mask_2);
  //   cv::bitwise_or(red_mask, white_mask_1, red_mask);
  //   cv::bitwise_or(red_mask, white_mask_2, red_mask);
  // }

  // ROS_INFO("propose C");
  // morphology
  auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
  cv::morphologyEx(final_mask, final_mask, CV_MOP_OPEN, kernel);
  cv::morphologyEx(final_mask, final_mask, CV_MOP_CLOSE, kernel,
                   cv::Point(-1, -1), 2);

  // ROS_INFO("propose C2");
  // cv::Mat img_show;
  // cv::bitwise_and(image, image, img_show, final_mask);
  // cv::imshow("red_mask", img_show);
  // cv::waitKey(5);

  // ROS_INFO("propose C3");
  // connected component extraction
  cv::Mat labels, stats, centroids;
  int nccomps =
      cv::connectedComponentsWithStats(final_mask, labels, stats, centroids);
  // ROS_INFO("propose C4");
  // extract possible ROIs from connected components
  std::vector<std::pair<cv::Rect2d, double>> roi_candidates;
  auto curr_roi_x = bounding_box.x;
  auto curr_roi_y = bounding_box.y;
  for (int i = 1; i < nccomps; ++i) // dismiss background 0
  {
    auto area = stats.at<int>(i, cv::CC_STAT_AREA);
    if (area < 20 || area > 10000)
      continue;
    auto ccomp_x = stats.at<int>(i, cv::CC_STAT_LEFT);
    auto ccomp_y = stats.at<int>(i, cv::CC_STAT_TOP);
    auto width = stats.at<int>(i, cv::CC_STAT_WIDTH);
    auto height = stats.at<int>(i, cv::CC_STAT_HEIGHT);

    double dis = std::sqrt(std::pow(ccomp_x - curr_roi_x, 2) +
                           std::pow(ccomp_y - curr_roi_y, 2));
    cv::Rect2d ccomp_box = cv::Rect2d(ccomp_x, ccomp_y, width, height);
    roi_candidates.push_back(std::make_pair(std::move(ccomp_box), dis));
  }
  // ROS_INFO("propose C5");
  std::sort(roi_candidates.begin(), roi_candidates.end(),
            [](const std::pair<cv::Rect2d, double> &a,
               const std::pair<cv::Rect2d, double> &b) {
              return a.second <= b.second;
            });
  // ROS_INFO("propose D");
  cv::Mat img_vis = image.clone();
  for (int i = 0; i < std::min(static_cast<int>(roi_candidates.size()), 2);
       ++i) {
    cv::Rect2d box = roi_candidates[i].first;
    auto center_x = box.x + box.width / 2;
    auto center_y = box.y + box.height / 2;
    box.width *= 1.1;
    box.height *= 1.1;
    box.x = center_x - box.width / 2;
    box.y = center_y - box.height / 2;

    proposals.push_back(box);
    // proposals.emplace_back(roi_candidates[i].first);
    // cv::rectangle(img_vis, box, cv::Scalar(0, 255, 255), 2, 0);
  }
  // cv::imshow("red_vis", img_vis);
  // cv::waitKey(1);
  std::cout << "propose!" << std::endl;
  return;
}

void getRearlightProposal(const cv::Mat &image, const cv::Rect2d &bounding_box,
                          const bool is_night,
                          std::vector<cv::Rect2d> &proposals) {
  // image: new frame
  // bounding_box: last frame rearlight state

  // get HSV
  cv::Mat img_hsv;
  cv::cvtColor(image, img_hsv, CV_BGR2HSV);

  // // daytime or nighttime
  // cv::Scalar feat_sum = cv::sum(img_hsv); // shape: (3,)
  // auto avg_brightness = feat_sum[2] / (img_hsv.rows * img_hsv.cols);

  // ROS_WARN("night: %d", is_night);

  // segment red area
  cv::Mat red_mask;
  if (is_night) {
    cv::Mat red_mask_1;
    cv::Mat red_mask_2;
    cv::Mat white_mask;
    cv::Mat tmp_mask;
    cv::inRange(img_hsv, cv::Scalar(0, 50, 150), cv::Scalar(10, 255, 255),
                red_mask_1);
    cv::inRange(img_hsv, cv::Scalar(170, 50, 150), cv::Scalar(180, 255, 255),
                red_mask_2);
    cv::bitwise_or(red_mask_1, red_mask_2, red_mask);
  } else {
    cv::Mat red_mask_1;
    cv::Mat red_mask_2;
    cv::inRange(img_hsv, cv::Scalar(0, 70, 20), cv::Scalar(10, 255, 255),
                red_mask_1);
    cv::inRange(img_hsv, cv::Scalar(170, 70, 20), cv::Scalar(180, 255, 255),
                red_mask_2);
    cv::bitwise_or(red_mask_1, red_mask_2, red_mask);
  }
  // ROS_WARN("A");

  // morphology
  auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
  cv::morphologyEx(red_mask, red_mask, CV_MOP_CLOSE, kernel, cv::Point(-1, -1),
                   2);
  cv::morphologyEx(red_mask, red_mask, CV_MOP_OPEN, kernel);

  // ROS_WARN("B");
  // apply roi
  cv::Mat roi_red_mask;
  cv::Rect2d roi = bounding_box;
  auto center_x = roi.x + roi.width / 2;
  auto center_y = roi.y + roi.height / 2;
  roi.width *= 3.0;
  roi.height *= 3.0;
  roi.x = center_x - roi.width / 2;
  roi.y = center_y - roi.height / 2;
  cv::Mat roi_mask = cv::Mat::zeros(red_mask.size(), CV_8UC1);
  roi_mask(roi).setTo(255);
  cv::bitwise_and(red_mask, red_mask, roi_red_mask, roi_mask);

  // ROS_WARN("C");
  // cv::Mat img_show;
  // cv::bitwise_and(image, image, img_show, roi_red_mask);
  // ROS_WARN("C1");
  // cv::imshow("red_mask", img_show);
  // cv::waitKey(5);

  // ROS_WARN("D");
  // connected component extraction
  cv::Mat labels, stats, centroids;
  int nccomps =
      cv::connectedComponentsWithStats(roi_red_mask, labels, stats, centroids);
  // extract possible ROIs from connected components
  std::vector<std::pair<cv::Rect2d, double>> roi_candidates;
  auto curr_roi_x = bounding_box.x;
  auto curr_roi_y = bounding_box.y;
  for (int i = 1; i < nccomps; ++i) // dismiss background 0
  {
    auto area = stats.at<int>(i, cv::CC_STAT_AREA);
    if (area < 50 || area > 5000)
      continue;
    auto ccomp_x = stats.at<int>(i, cv::CC_STAT_LEFT);
    auto ccomp_y = stats.at<int>(i, cv::CC_STAT_TOP);
    auto width = stats.at<int>(i, cv::CC_STAT_WIDTH);
    auto height = stats.at<int>(i, cv::CC_STAT_HEIGHT);

    double dis = std::sqrt(std::pow(ccomp_x - curr_roi_x, 2) +
                           std::pow(ccomp_y - curr_roi_y, 2));
    cv::Rect2d ccomp_box = cv::Rect2d(ccomp_x, ccomp_y, width, height);
    roi_candidates.push_back(std::make_pair(std::move(ccomp_box), dis));
  }
  std::sort(roi_candidates.begin(), roi_candidates.end(),
            [](const std::pair<cv::Rect2d, double> &a,
               const std::pair<cv::Rect2d, double> &b) {
              return a.second <= b.second;
            });
  for (int i = 0; i < std::min(static_cast<int>(roi_candidates.size()), 2);
       ++i) {
    proposals.emplace_back(roi_candidates[i].first);
  }
  // ROS_WARN("E");
  return;
}

class RearlightProposalGenerator {
public:
  RearlightProposalGenerator() : is_night(false) {}
  ~RearlightProposalGenerator() = default;

  void updateFromTracker(const cv::Rect2d &left_box,
                         const cv::Rect2d &right_box, const cv::Mat &left_mat,
                         const cv::Mat &right_mat) {
    last_left_bbox_ = left_box;
    last_right_bbox_ = right_box;
    last_left_mat_ = left_mat;
    last_right_mat_ = right_mat;
  }

  void getLeftProposal(cv::Rect2d *bbox) { bbox = &curr_left_bbox_; }

  void getRightProposal(cv::Rect2d *bbox) { bbox = &curr_right_bbox_; }

  void updateProposals(const cv::Mat &image, const cv::Mat &mat,
                       const cv::Rect2d &vehicle_roi);

private:
  bool is_night;
  cv::Rect2d last_left_bbox_, last_right_bbox_;
  cv::Mat last_left_mat_, last_right_mat_;
  cv::Rect2d curr_left_bbox_, curr_right_bbox_;
};

} // namespace rearlight_proposal
} // namespace tracking