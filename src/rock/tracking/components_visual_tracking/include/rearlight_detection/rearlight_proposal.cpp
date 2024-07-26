#include "rearlight_detection/rearlight_proposal.h"

using namespace tracking::rearlight_proposal;

void RearlightProposalGenerator::updateProposals(
    const cv::Mat &image, const cv::Mat &mat, const cv::Rect2d &vehicle_roi) {
  // mat: image in roi
  auto img_roi = image(vehicle_roi);

  // segment mask according to HSV values
  cv::Mat img_hsv;
  cv::cvtColor(img_roi, img_hsv, CV_BGR2HSV);

  cv::Mat red_mask, red_mask_1, red_mask_2;
  cv::inRange(img_hsv, cv::Scalar(0, 10, 20), cv::Scalar(15, 255, 255),
              red_mask_1);
  cv::inRange(img_hsv, cv::Scalar(165, 10, 20), cv::Scalar(180, 255, 255),
              red_mask_2);
  cv::bitwise_or(red_mask_1, red_mask_2, red_mask);

  auto kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  cv::morphologyEx(red_mask, red_mask, CV_MOP_CLOSE, kernel);

  cv::Mat img_masked;
  cv::bitwise_and(img_roi, img_roi, img_masked, red_mask);
  cv::Mat img_masked_hsv;
  cv::cvtColor(img_masked, img_masked_hsv, CV_BGR2HSV);

  // extract clustering samples
  cv::Rect2d left_expanded_bbox = last_left_bbox_,
             right_expanded_bbox = last_right_bbox_;
  left_expanded_bbox.x -= vehicle_roi.x;
  left_expanded_bbox.y -= vehicle_roi.y;
  right_expanded_bbox.x -= vehicle_roi.x;
  right_expanded_bbox.y -= vehicle_roi.y;

  auto left_center_x = left_expanded_bbox.x + left_expanded_bbox.width / 2;
  auto left_center_y = left_expanded_bbox.y + left_expanded_bbox.height / 2;
  left_expanded_bbox.width *= 2;
  left_expanded_bbox.height *= 2;
  left_expanded_bbox.x = left_center_x - left_expanded_bbox.width / 2;
  left_expanded_bbox.y = left_center_y - left_expanded_bbox.height / 2;
  auto right_center_x = right_expanded_bbox.x + right_expanded_bbox.width / 2;
  auto right_center_y = right_expanded_bbox.y + right_expanded_bbox.height / 2;
  right_expanded_bbox.width *= 2;
  right_expanded_bbox.height *= 2;
  right_expanded_bbox.x = right_center_x - right_expanded_bbox.width / 2;
  right_expanded_bbox.y = right_center_y - right_expanded_bbox.height / 2;

  auto img_left_roi = img_masked(left_expanded_bbox);
  auto img_right_roi = img_masked(right_expanded_bbox);

  // left
  cv::Mat img_left_roi_ycrcb;
  cv::cvtColor(img_left_roi, img_left_roi_ycrcb, CV_BGR2YCrCb);
  img_left_roi_ycrcb.convertTo(img_left_roi_ycrcb, CV_32F);
  std::vector<cv::Mat> chann_left_roi_ycrcb;
  cv::split(img_left_roi_ycrcb, chann_left_roi_ycrcb);
  cv::Mat left_weighting_chann;
  if (is_night)
    left_weighting_chann =
        0.5 * chann_left_roi_ycrcb[1] + 0.5 * chann_left_roi_ycrcb[0];
  else
    left_weighting_chann =
        0.9 * chann_left_roi_ycrcb[1] + 0.1 * chann_left_roi_ycrcb[0];
  cv::Mat left_col_avg, left_row_avg;
  cv::reduce(left_weighting_chann, left_col_avg, 0, CV_REDUCE_AVG, CV_32F);
  cv::reduce(left_weighting_chann, left_col_avg, 1, CV_REDUCE_AVG, CV_32F);
  std::vector<float> left_col_avg_vec(left_col_avg.cols * left_col_avg.rows *
                                      left_col_avg.channels()),
      left_row_avg_vec(left_row_avg.cols * left_row_avg.rows *
                       left_row_avg.channels());
  if (!left_col_avg.isContinuous())
    left_col_avg = left_col_avg.clone();
  if (!left_row_avg.isContinuous())
    left_row_avg = left_row_avg.clone();
  left_col_avg_vec = left_col_avg;
  left_row_avg_vec = left_row_avg;
  std::vector<std::pair<int, float>> left_col_avg_idx_vec, left_row_avg_idx_vec;
  for (int i = 0; i < left_col_avg_vec.size(); ++i) {
    left_col_avg_idx_vec.emplace_back(std::make_pair(i, left_col_avg_vec[i]));
  }
  for (int i = 0; i < left_row_avg_vec.size(); ++i) {
    left_row_avg_idx_vec.emplace_back(std::make_pair(i, left_row_avg_vec[i]));
  }

  auto left_response_cmp = [](const std::pair<int, float> &a,
                              const std::pair<int, float> &b) {
    return a.second >= b.second;
  };
  std::sort(left_col_avg_idx_vec.begin(), left_col_avg_idx_vec.end(),
       left_response_cmp);
  std::sort(left_row_avg_idx_vec.begin(), left_row_avg_idx_vec.end(),
       left_response_cmp);

  auto left_best_candidate_pt = cv::Point2i(left_col_avg_idx_vec.front().first,
                                            left_row_avg_idx_vec.front().first);

//   std::vector<cv::Point2i> left_candidate_pts;
//   for (int i = 0;
//        i < std::min(static_cast<int>(left_col_avg_idx_vec.size()), 2); ++i) {
//     for (int j = 0;
//          j < std::min(static_cast<int>(left_row_avg_idx_vec.size()), 2); ++j) {
//       left_candidate_pts.emplace_back(cv::Point2i(
//           left_col_avg_idx_vec[i].first, left_row_avg_idx_vec[j].first));
//     }
//   }
//   cv::Point2i left_roi_center(static_cast<int>(left_expanded_bbox.width / 2),
//                               static_cast<int>(left_expanded_bbox.height / 2));
//   auto left_candidate_cmp = [&left_roi_center](const cv::Point2i &a,
//                                                const cv::Point2i &b) {
//     auto dis_a = cv::norm(left_roi_center - a);
//     auto dis_b = cv::norm(left_roi_center - b);
//   };
//   std::sort(left_candidate_pts.begin(), left_candidate_pts.end(), left_candidate_cmp);
//   auto left_best_candidate_pt = left_candidate_pts.front();
  

  //   // clustering analysis
  //   // left
  //   std::vector<cv::Mat> left_sample_bgr;
  //   cv::split(img_left_roi, left_sample_bgr);
  //   int n_sample_left = img_left_roi.cols * img_right_roi.rows;
  //   cv::Mat left_samples(n_sample_left, 3, CV_8U);
  //   for (int i = 0; i < 3; ++i) {
  //     left_sample_bgr[i].reshape(1,
  //     n_sample_left).copyTo(left_samples.col(i));
  //   }
  //   left_samples.convertTo(left_samples, CV_32F);

  //   cv::Mat left_labels, left_centers;
  //   double left_compactness =
  //       cv::kmeans(left_samples, 8, left_labels,
  //                  cv::TermCriteria(
  //                      cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
  //                      10, 1.0),
  //                  3, cv::KMEANS_PP_CENTERS, left_centers);

  //   auto left_labels_reshaped = left_labels.reshape(0, img_left_roi.rows);
  //   left_labels_reshaped.convertTo(left_labels_reshaped, CV_8U);
}
