// #ifndef CUSTOM_KCF_H
// #define CUSTOM_KCF_H
#pragma once

#include <cmath>
#include <complex>
#include <numeric>
#include <utility>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <opencv2/tracking/tracking.hpp>

#include "custom_kcf/featureColorName.hpp"
#include "rearlight_detection/rearlight_proposal.h"

// ref:
// https://docs.opencv.org/3.4/d2/dff/classcv_1_1TrackerKCF.html#a2a97c5b832651524b8e782496b933d45
// ref: https://docs.opencv.org/3.4/d2/d0a/tutorial_introduction_to_tracker.html
// ref:
// https://docs.opencv.org/4.x/d7/d60/classcv_1_1SIFT.html#a94ee0141f77675822e574bbd2c079811
namespace cv {
class CustomTrackerKCF : public Tracker {
public:
  /**
* \brief Feature type to be used in the tracking grayscale, colornames,
compressed color-names
* The modes available now:
-   "GRAY" -- Use grayscale values as the feature
-   "CN" -- Color-names feature
*/
  enum MODE { GRAY = (1 << 0), CN = (1 << 1), CUSTOM = (1 << 2) };

  struct CV_EXPORTS Params {
    /**
     * \brief Constructor
     */
    Params();

    /**
     * \brief Read parameters from a file
     */
    void read(const FileNode & /*fn*/);

    /**
     * \brief Write parameters to a file
     */
    void write(FileStorage & /*fs*/) const;

    float detect_thresh;       //!<  detection confidence threshold
    float sigma;               //!<  gaussian kernel bandwidth
    float lambda;              //!<  regularization
    float interp_factor;       //!<  linear interpolation factor for adaptation
    float output_sigma_factor; //!<  spatial bandwidth (proportional to target)
    float pca_learning_rate;   //!<  compression learning rate
    bool resize; //!<  activate the resize feature to improve the processing
                 //!<  speed
    bool split_coeff; //!<  split the training coefficients into two matrices
    bool wrap_kernel; //!<  wrap around the kernel values
    bool
        compress_feature; //!<  activate the pca method to compress the features
    int max_patch_size;   //!<  threshold for the ROI size
    int compressed_size;  //!<  feature size after compression
    int desc_pca;         //!<  compressed descriptors of TrackerKCF::MODE
    int desc_npca;        //!<  non-compressed descriptors of TrackerKCF::MODE
  };

  virtual void setFeatureExtractor(void (*)(const Mat, const Rect, Mat &),
                                   bool pca_func = false) = 0;

  /** @brief Constructor
@param parameters KCF parameters TrackerKCF::Params
*/
  static Ptr<CustomTrackerKCF>
  create(const CustomTrackerKCF::Params &parameters);

  CV_WRAP static Ptr<CustomTrackerKCF> create();

  virtual ~CustomTrackerKCF() CV_OVERRIDE {}
};
} /* namespace cv */

namespace cv {
/**
 * \brief Implementation of TrackerModel for KCF algorithm
 */
class CustomTrackerKCFModel : public TrackerModel {
public:
  CustomTrackerKCFModel(CustomTrackerKCF::Params /*params*/) {}
  ~CustomTrackerKCFModel() {}

protected:
  void modelEstimationImpl(const std::vector<Mat> & /*responses*/) CV_OVERRIDE {
  }
  void modelUpdateImpl() CV_OVERRIDE {}
};
} /* namespace cv */

// pointer to visual detection result
extern std::vector<cyber_msgs::Box2D> *visual_dets_pt;
// pointer to vehicle tracking result
extern cv::Rect2d *vehicle_bbox_pt;
// pointer to rearlight tracking result
extern cv::Rect2d *left_rearlight_bbox_pt, *right_rearlight_bbox_pt;
// pointer to vehicle image speed
extern cv::Point2f *velo_vehicle_pt;
// night flag
extern bool is_night;
// rematch flag
extern bool rematch_flag;

namespace cv {

/*
 * Prototype
 */
class CustomTrackerKCFImpl : public CustomTrackerKCF {
public:
  CustomTrackerKCFImpl(
      const CustomTrackerKCF::Params &parameters = CustomTrackerKCF::Params());
  void read(const FileNode & /*fn*/) CV_OVERRIDE;
  void write(FileStorage & /*fs*/) const CV_OVERRIDE;
  void setFeatureExtractor(void (*f)(const Mat, const Rect, Mat &),
                           bool pca_func = false) CV_OVERRIDE;

protected:
  /*
   * basic functions and vars
   */
  bool initImpl(const Mat & /*image*/, const Rect2d &boundingBox) CV_OVERRIDE;
  bool updateImpl(const Mat &image, Rect2d &boundingBox) CV_OVERRIDE;

  CustomTrackerKCF::Params params;

  /*
   * KCF functions and vars
   */
  // my custom functions
  void detect(const Mat &image, const Rect2d &proposal, double &minVal,
              double &maxVal, Point &minLoc, Point &maxLoc);
  void analyzeProposals(const Mat &image, const Rect2d &bounding_box);
  void fitProposalSegmentation(const Mat &image, const Rect2d &bounding_box);
  void adjustThreshold(const Mat &image, const Rect2d &bounding_box);

  void createHanningWindow(OutputArray dest, const cv::Size winSize,
                           const int type) const;
  void inline fft2(const Mat src, std::vector<Mat> &dest,
                   std::vector<Mat> &layers_data) const;
  void inline fft2(const Mat src, Mat &dest) const;
  void inline ifft2(const Mat src, Mat &dest) const;
  void inline pixelWiseMult(const std::vector<Mat> src1,
                            const std::vector<Mat> src2, std::vector<Mat> &dest,
                            const int flags, const bool conjB = false) const;
  void inline sumChannels(std::vector<Mat> src, Mat &dest) const;
  void inline updateProjectionMatrix(const Mat src, Mat &old_cov,
                                     Mat &proj_matrix, float pca_rate,
                                     int compressed_sz,
                                     std::vector<Mat> &layers_pca,
                                     std::vector<Scalar> &average, Mat pca_data,
                                     Mat new_cov, Mat w, Mat u, Mat v);
  void inline compress(const Mat proj_matrix, const Mat src, Mat &dest,
                       Mat &data, Mat &compressed) const;
  bool getSubWindow(const Mat img, const Rect roi, Mat &feat, Mat &patch,
                    CustomTrackerKCF::MODE desc = GRAY) const;
  bool getSubWindow(const Mat img, const Rect roi, Mat &feat,
                    void (*f)(const Mat, const Rect, Mat &)) const;
  void extractCN(Mat patch_data, Mat &cnFeatures) const;
  void denseGaussKernel(const float sigma, const Mat, const Mat y_data,
                        Mat &k_data, std::vector<Mat> &layers_data,
                        std::vector<Mat> &xf_data, std::vector<Mat> &yf_data,
                        std::vector<Mat> xyf_v, Mat xy, Mat xyf) const;
  void calcResponse(const Mat alphaf_data, const Mat kf_data,
                    Mat &response_data, Mat &spec_data) const;
  void calcResponse(const Mat alphaf_data, const Mat alphaf_den_data,
                    const Mat kf_data, Mat &response_data, Mat &spec_data,
                    Mat &spec2_data) const;

  void shiftRows(Mat &mat) const;
  void shiftRows(Mat &mat, int n) const;
  void shiftCols(Mat &mat, int n) const;
#ifdef HAVE_OPENCL
  bool inline oclTransposeMM(const Mat src, float alpha, UMat &dst);
#endif

private:
  // my personal vars
  int cluster_num;
  int thresh_num;
  Size temp_size;
  double update_alpha;
  std::vector<std::pair<int, int>> proposal_thresh;

  float output_sigma;
  Rect2d roi;
  Mat hann;    // hann window filter
  Mat hann_cn; // 10 dimensional hann-window filter for CN features,

  Mat y, yf;                      // training response and its FFT
  Mat x;                          // observation and its FFT
  Mat k, kf;                      // dense gaussian kernel and its FFT
  Mat kf_lambda;                  // kf+lambda
  Mat new_alphaf, alphaf;         // training coefficients
  Mat new_alphaf_den, alphaf_den; // for splitted training coefficients
  Mat z;                          // model
  Mat response;                   // detection result
  Mat old_cov_mtx, proj_mtx;      // for feature compression

  // pre-defined Mat variables for optimization of private functions
  Mat spec, spec2;
  std::vector<Mat> layers;
  std::vector<Mat> vxf, vyf, vxyf;
  Mat xy_data, xyf_data;
  Mat data_temp, compress_data;
  std::vector<Mat> layers_pca_data;
  std::vector<Scalar> average_data;
  Mat img_Patch;

  // // predefined scale searching ratio
  // std::vector<double> scales = {0.90, 0.95, 1.0, 1.05, 1.10};

  // storage for the extracted features, KRLS model, KRLS compressed model
  Mat X[2], Z[2], Zc[2];

  // storage of the extracted features
  std::vector<Mat> features_pca;
  std::vector<Mat> features_npca;
  std::vector<MODE> descriptors_pca;
  std::vector<MODE> descriptors_npca;

  // optimization variables for updateProjectionMatrix
  Mat data_pca, new_covar, w_data, u_data, vt_data;

  // custom feature extractor
  bool use_custom_extractor_pca;
  bool use_custom_extractor_npca;
  std::vector<void (*)(const Mat img, const Rect roi, Mat &output)>
      extractor_pca;
  std::vector<void (*)(const Mat img, const Rect roi, Mat &output)>
      extractor_npca;

  bool resizeImage; // resize the image whenever needed and the patch size is
                    // large

#ifdef HAVE_OPENCL
  ocl::Kernel transpose_mm_ker; // OCL kernel to compute transpose matrix
                                // multiply matrix.
#endif

  int frame;
};

/*
 * Constructor
 */
Ptr<CustomTrackerKCF>
CustomTrackerKCF::create(const CustomTrackerKCF::Params &parameters) {
  return Ptr<CustomTrackerKCFImpl>(new CustomTrackerKCFImpl(parameters));
}
Ptr<CustomTrackerKCF> CustomTrackerKCF::create() {
  return Ptr<CustomTrackerKCFImpl>(new CustomTrackerKCFImpl());
}
CustomTrackerKCFImpl::CustomTrackerKCFImpl(
    const CustomTrackerKCF::Params &parameters)
    : params(parameters) {
  isInit = false;
  resizeImage = false;
  use_custom_extractor_pca = false;
  use_custom_extractor_npca = false;

#ifdef HAVE_OPENCL
  // For update proj matrix's multiplication
  if (ocl::useOpenCL()) {
    cv::String err;
    ocl::ProgramSource tmmSrc = ocl::tracking::tmm_oclsrc;
    ocl::Program tmmProg(tmmSrc, String(), err);
    transpose_mm_ker.create("tmm", tmmProg);
  }
#endif
}

void CustomTrackerKCFImpl::read(const cv::FileNode &fn) { params.read(fn); }

void CustomTrackerKCFImpl::write(cv::FileStorage &fs) const {
  params.write(fs);
}

/*
 * Initialization:
 * - creating hann window filter
 * - ROI padding
 * - creating a gaussian response for the training ground-truth
 * - perform FFT to the gaussian response
 */
bool CustomTrackerKCFImpl::initImpl(const Mat &image,
                                    const Rect2d &boundingBox) {
  ROS_WARN(
      "init_roi_x: %.1f, init_roi_y: %.1f, init_roi_w: %.1f, init_roi_h: %.1f",
      boundingBox.x, boundingBox.y, boundingBox.width, boundingBox.height);
  frame = 0;
  update_alpha = 0.1;
  cluster_num = 7;
  thresh_num = 4;
  proposal_thresh = std::vector<std::pair<int, int>>(thresh_num);

  roi.x = cvRound(boundingBox.x);
  roi.y = cvRound(boundingBox.y);
  roi.width = cvRound(boundingBox.width);
  roi.height = cvRound(boundingBox.height);

  // calclulate output sigma
  output_sigma = std::sqrt(static_cast<float>(roi.width * roi.height)) *
                 params.output_sigma_factor;
  output_sigma = -0.5f / (output_sigma * output_sigma);

  // resize the ROI whenever needed
  if (params.resize && roi.width * roi.height > params.max_patch_size) {
    resizeImage = true;
    roi.x /= 2.0;
    roi.y /= 2.0;
    roi.width /= 2.0;
    roi.height /= 2.0;
  }

  // add padding to the roi
  roi.x -= roi.width / 2;
  roi.y -= roi.height / 2;
  roi.width *= 2;
  roi.height *= 2;

  temp_size.width = roi.width;
  temp_size.height = roi.height;

  ROS_WARN("after init, init_roi_x: %.1f, init_roi_y: %.1f, init_roi_w: %.1f, "
           "init_roi_h: %.1f",
           roi.x, roi.y, roi.width, roi.height);

  // initialize the hann window filter
  createHanningWindow(hann, roi.size(), CV_32F);

  // hann window filter for CN feature
  Mat _layer[] = {hann, hann, hann, hann, hann, hann, hann, hann, hann, hann};
  merge(_layer, 10, hann_cn);

  // create gaussian response
  y = Mat::zeros((int)roi.height, (int)roi.width, CV_32F);
  for (int i = 0; i < int(roi.height); i++) {
    for (int j = 0; j < int(roi.width); j++) {
      y.at<float>(i, j) = static_cast<float>(
          (i - roi.height / 2 + 1) * (i - roi.height / 2 + 1) +
          (j - roi.width / 2 + 1) * (j - roi.width / 2 + 1));
    }
  }

  y *= (float)output_sigma;
  cv::exp(y, y);

  // perform fourier transfor to the gaussian response
  fft2(y, yf);

  if (image.channels() == 1) { // disable CN for grayscale images
    params.desc_pca &= ~(CN);
    params.desc_npca &= ~(CN);
  }
  model = Ptr<CustomTrackerKCFModel>(new CustomTrackerKCFModel(params));

  // record the non-compressed descriptors
  if ((params.desc_npca & GRAY) == GRAY)
    descriptors_npca.push_back(GRAY);
  if ((params.desc_npca & CN) == CN)
    descriptors_npca.push_back(CN);
  if (use_custom_extractor_npca)
    descriptors_npca.push_back(CUSTOM);
  features_npca.resize(descriptors_npca.size());

  // record the compressed descriptors
  if ((params.desc_pca & GRAY) == GRAY)
    descriptors_pca.push_back(GRAY);
  if ((params.desc_pca & CN) == CN)
    descriptors_pca.push_back(CN);
  if (use_custom_extractor_pca)
    descriptors_pca.push_back(CUSTOM);
  features_pca.resize(descriptors_pca.size());

  // accept only the available descriptor modes
  CV_Assert((params.desc_pca & GRAY) == GRAY ||
            (params.desc_npca & GRAY) == GRAY || (params.desc_pca & CN) == CN ||
            (params.desc_npca & CN) == CN || use_custom_extractor_pca ||
            use_custom_extractor_npca);

  // return true only if roi has intersection with the image
  if ((roi & Rect2d(0, 0, resizeImage ? image.cols / 2 : image.cols,
                    resizeImage ? image.rows / 2 : image.rows)) == Rect2d())
    return false;

  return true;
}

/*
 * Main part of the KCF algorithm
 */
bool CustomTrackerKCFImpl::updateImpl(const Mat &image, Rect2d &boundingBox) {
  double minVal, maxVal; // min-max response
  Point minLoc, maxLoc;  // min-max location

  Mat img = image.clone();
  // check the channels of the input image, grayscale is preferred
  CV_Assert(img.channels() == 1 || img.channels() == 3);

  // resize the image whenever needed
  if (resizeImage)
    resize(img, img, Size(img.cols / 2, img.rows / 2), 0, 0,
           INTER_LINEAR_EXACT);

  // analyze proposals
  // ROS_INFO("analyze proposals");
  auto t_1 = ros::WallTime::now().toSec();
  analyzeProposals(image.clone(), boundingBox); // input un-resized image
  auto t_2 = ros::WallTime::now().toSec();
  std::cout << "analyzeProposals cost " << t_2 - t_1 << " sec" << std::endl;

  // detection part
  // ROS_INFO("final detect frame: %d", frame);
  if (frame > 0) {
    // extract and pre-process the patch
    // get non compressed descriptors
    for (unsigned i = 0; i < descriptors_npca.size() - extractor_npca.size();
         i++) {
      if (!getSubWindow(img, roi, features_npca[i], img_Patch,
                        descriptors_npca[i])) {
        // ROS_INFO("extract fail 1");
        return false;
      }
    }
    // get non-compressed custom descriptors
    for (unsigned i = 0, j = (unsigned)(descriptors_npca.size() -
                                        extractor_npca.size());
         i < extractor_npca.size(); i++, j++) {
      if (!getSubWindow(img, roi, features_npca[j], extractor_npca[i])) {
        // ROS_INFO("extract fail 2");
        return false;
      }
    }
    if (features_npca.size() > 0)
      merge(features_npca, X[1]);

    // get compressed descriptors
    for (unsigned i = 0; i < descriptors_pca.size() - extractor_pca.size();
         i++) {
      if (!getSubWindow(img, roi, features_pca[i], img_Patch,
                        descriptors_pca[i])) {
        // ROS_INFO("extract fail 3");
        return false;
      }
    }
    // get compressed custom descriptors
    for (unsigned i = 0,
                  j = (unsigned)(descriptors_pca.size() - extractor_pca.size());
         i < extractor_pca.size(); i++, j++) {
      if (!getSubWindow(img, roi, features_pca[j], extractor_pca[i])) {
        // ROS_INFO("extract fail 4");
        return false;
      }
    }
    if (features_pca.size() > 0)
      merge(features_pca, X[0]);

    // compress the features and the KRSL model
    if (params.desc_pca != 0) {
      compress(proj_mtx, X[0], X[0], data_temp, compress_data);
      compress(proj_mtx, Z[0], Zc[0], data_temp, compress_data);
    }

    // copy the compressed KRLS model
    Zc[1] = Z[1];

    // merge all features
    if (features_npca.size() == 0) {
      x = X[0];
      z = Zc[0];
    } else if (features_pca.size() == 0) {
      x = X[1];
      z = Z[1];
    } else {
      merge(X, 2, x);
      merge(Zc, 2, z);
    }

    // compute the gaussian kernel
    denseGaussKernel(params.sigma, x, z, k, layers, vxf, vyf, vxyf, xy_data,
                     xyf_data);

    // compute the fourier transform of the kernel
    fft2(k, kf);
    if (frame == 1)
      spec2 = Mat_<Vec2f>(kf.rows, kf.cols);

    // calculate filter response
    if (params.split_coeff)
      calcResponse(alphaf, alphaf_den, kf, response, spec, spec2);
    else
      calcResponse(alphaf, kf, response, spec);

    // extract the maximum response
    minMaxLoc(response, &minVal, &maxVal, &minLoc, &maxLoc);
    // imshow("response", response);
    // waitKey(5);

    if (maxVal < params.detect_thresh) {
      // ROS_INFO("final detect fail");
      return false;
    }

    Point max_loc_roi;
    max_loc_roi.x =
        (maxLoc.x - temp_size.width / 2 + 1) * roi.width / temp_size.width;
    max_loc_roi.y =
        (maxLoc.y - temp_size.height / 2 + 1) * roi.height / temp_size.height;
    roi.x += max_loc_roi.x;
    roi.y += max_loc_roi.y;
    // ROS_INFO("min_val: %.4f, max_val: %.4f", minVal, maxVal);
    // ROS_INFO("max_loc x: %d, max_loc y: %d", maxLoc.x, maxLoc.y);
    // ROS_INFO("delta x: %d, delta y: %d", max_loc_roi.x, max_loc_roi.y);
    std::cout << "delta x: " << max_loc_roi.x << " "
              << "delta y: " << max_loc_roi.y << std::endl;
  }

  // auto img_show = img.clone();
  // rectangle(img_show, roi, Scalar(255, 0, 0), 2, 1);
  // imshow("roi", img_show);
  // waitKey(5);

  // update the bounding box
  boundingBox.x = (resizeImage ? roi.x * 2 : roi.x) +
                  (resizeImage ? roi.width * 2 : roi.width) / 4;
  boundingBox.y = (resizeImage ? roi.y * 2 : roi.y) +
                  (resizeImage ? roi.height * 2 : roi.height) / 4;
  boundingBox.width = (resizeImage ? roi.width * 2 : roi.width) / 2;
  boundingBox.height = (resizeImage ? roi.height * 2 : roi.height) / 2;

  // fit proposal segmentation parameters
  auto t_3 = ros::WallTime::now().toSec();
  adjustThreshold(image, boundingBox);
  auto t_4 = ros::WallTime::now().toSec();
  std::cout << "adjustThreshold cost " << t_4 - t_3 << " sec" << std::endl;

  // ROS_INFO("update bbox");

  // extract the patch for learning purpose
  // get non compressed descriptors
  for (unsigned i = 0; i < descriptors_npca.size() - extractor_npca.size();
       i++) {
    if (!getSubWindow(img, roi, features_npca[i], img_Patch,
                      descriptors_npca[i]))
      return false;
  }
  // get non-compressed custom descriptors
  for (unsigned i = 0,
                j = (unsigned)(descriptors_npca.size() - extractor_npca.size());
       i < extractor_npca.size(); i++, j++) {
    if (!getSubWindow(img, roi, features_npca[j], extractor_npca[i]))
      return false;
  }
  if (features_npca.size() > 0)
    merge(features_npca, X[1]);

  // get compressed descriptors
  for (unsigned i = 0; i < descriptors_pca.size() - extractor_pca.size(); i++) {
    if (!getSubWindow(img, roi, features_pca[i], img_Patch, descriptors_pca[i]))
      return false;
  }
  // get compressed custom descriptors
  for (unsigned i = 0,
                j = (unsigned)(descriptors_pca.size() - extractor_pca.size());
       i < extractor_pca.size(); i++, j++) {
    if (!getSubWindow(img, roi, features_pca[j], extractor_pca[i]))
      return false;
  }
  if (features_pca.size() > 0)
    merge(features_pca, X[0]);

  // update the training data
  if (frame == 0) {
    Z[0] = X[0].clone();
    Z[1] = X[1].clone();
  } else {
    Z[0] = (1.0 - params.interp_factor) * Z[0] + params.interp_factor * X[0];
    Z[1] = (1.0 - params.interp_factor) * Z[1] + params.interp_factor * X[1];
  }

  if (params.desc_pca != 0 || use_custom_extractor_pca) {
    // initialize the vector of Mat variables
    if (frame == 0) {
      layers_pca_data.resize(Z[0].channels());
      average_data.resize(Z[0].channels());
    }

    // feature compression
    updateProjectionMatrix(Z[0], old_cov_mtx, proj_mtx,
                           params.pca_learning_rate, params.compressed_size,
                           layers_pca_data, average_data, data_pca, new_covar,
                           w_data, u_data, vt_data);
    compress(proj_mtx, X[0], X[0], data_temp, compress_data);
  }

  // merge all features
  if (features_npca.size() == 0)
    x = X[0];
  else if (features_pca.size() == 0)
    x = X[1];
  else
    merge(X, 2, x);

  // initialize some required Mat variables
  if (frame == 0) {
    layers.resize(x.channels());
    vxf.resize(x.channels());
    vyf.resize(x.channels());
    vxyf.resize(vyf.size());
    new_alphaf = Mat_<Vec2f>(yf.rows, yf.cols);
  }

  // Kernel Regularized Least-Squares, calculate alphas
  denseGaussKernel(params.sigma, x, x, k, layers, vxf, vyf, vxyf, xy_data,
                   xyf_data);

  // compute the fourier transform of the kernel and add a small value
  fft2(k, kf);
  kf_lambda = kf + params.lambda;

  float den;
  if (params.split_coeff) {
    mulSpectrums(yf, kf, new_alphaf, 0);
    mulSpectrums(kf, kf_lambda, new_alphaf_den, 0);
  } else {
    for (int i = 0; i < yf.rows; i++) {
      for (int j = 0; j < yf.cols; j++) {
        den = 1.0f /
              (kf_lambda.at<Vec2f>(i, j)[0] * kf_lambda.at<Vec2f>(i, j)[0] +
               kf_lambda.at<Vec2f>(i, j)[1] * kf_lambda.at<Vec2f>(i, j)[1]);

        new_alphaf.at<Vec2f>(i, j)[0] =
            (yf.at<Vec2f>(i, j)[0] * kf_lambda.at<Vec2f>(i, j)[0] +
             yf.at<Vec2f>(i, j)[1] * kf_lambda.at<Vec2f>(i, j)[1]) *
            den;
        new_alphaf.at<Vec2f>(i, j)[1] =
            (yf.at<Vec2f>(i, j)[1] * kf_lambda.at<Vec2f>(i, j)[0] -
             yf.at<Vec2f>(i, j)[0] * kf_lambda.at<Vec2f>(i, j)[1]) *
            den;
      }
    }
  }

  // update the RLS model
  if (frame == 0) {
    alphaf = new_alphaf.clone();
    if (params.split_coeff)
      alphaf_den = new_alphaf_den.clone();
  } else {
    alphaf = (1.0 - params.interp_factor) * alphaf +
             params.interp_factor * new_alphaf;
    if (params.split_coeff)
      alphaf_den = (1.0 - params.interp_factor) * alphaf_den +
                   params.interp_factor * new_alphaf_den;
  }

  frame++;
  return true;
}

/*-------------------------------------
|  implementation of the KCF functions
|-------------------------------------*/

void CustomTrackerKCFImpl::detect(const Mat &image, const Rect2d &proposal,
                                  double &minVal, double &maxVal, Point &minLoc,
                                  Point &maxLoc) {
  // inference proposals
  // image: raw image
  // proposal: proposal extracted from raw image

  // double minVal, maxVal; // min-max response
  // Point minLoc, maxLoc;  // min-max location
  minVal = -1.0;
  maxVal = -1.0;

  Mat img = image.clone();
  // check the channels of the input image, grayscale is preferred
  CV_Assert(img.channels() == 1 || img.channels() == 3);

  // resize the image whenever needed
  if (resizeImage)
    resize(img, img, Size(img.cols / 2, img.rows / 2), 0, 0,
           INTER_LINEAR_EXACT);

  // detection part
  if (frame > 0) {
    // ROS_INFO("proposal detect frame: %d", frame);
    // extract and pre-process the patch
    // get non compressed descriptors
    for (unsigned i = 0; i < descriptors_npca.size() - extractor_npca.size();
         i++) {
      if (!getSubWindow(img, proposal, features_npca[i], img_Patch,
                        descriptors_npca[i]))
        return;
    }
    // get non-compressed custom descriptors
    for (unsigned i = 0, j = (unsigned)(descriptors_npca.size() -
                                        extractor_npca.size());
         i < extractor_npca.size(); i++, j++) {
      if (!getSubWindow(img, proposal, features_npca[j], extractor_npca[i]))
        return;
    }
    if (features_npca.size() > 0)
      merge(features_npca, X[1]);

    // get compressed descriptors
    for (unsigned i = 0; i < descriptors_pca.size() - extractor_pca.size();
         i++) {
      if (!getSubWindow(img, proposal, features_pca[i], img_Patch,
                        descriptors_pca[i]))
        return;
    }
    // get compressed custom descriptors
    for (unsigned i = 0,
                  j = (unsigned)(descriptors_pca.size() - extractor_pca.size());
         i < extractor_pca.size(); i++, j++) {
      if (!getSubWindow(img, proposal, features_pca[j], extractor_pca[i]))
        return;
    }
    if (features_pca.size() > 0)
      merge(features_pca, X[0]);

    // ROS_INFO("proposal detect extract");

    // compress the features and the KRSL model
    if (params.desc_pca != 0) {
      compress(proj_mtx, X[0], X[0], data_temp, compress_data);
      compress(proj_mtx, Z[0], Zc[0], data_temp, compress_data);
    }

    // copy the compressed KRLS model
    Zc[1] = Z[1];

    // merge all features
    if (features_npca.size() == 0) {
      x = X[0];
      z = Zc[0];
    } else if (features_pca.size() == 0) {
      x = X[1];
      z = Z[1];
    } else {
      merge(X, 2, x);
      merge(Zc, 2, z);
    }

    // compute the gaussian kernel
    denseGaussKernel(params.sigma, x, z, k, layers, vxf, vyf, vxyf, xy_data,
                     xyf_data);

    // compute the fourier transform of the kernel
    fft2(k, kf);
    if (frame == 1)
      spec2 = Mat_<Vec2f>(kf.rows, kf.cols);

    // calculate filter response
    if (params.split_coeff)
      calcResponse(alphaf, alphaf_den, kf, response, spec, spec2);
    else
      calcResponse(alphaf, kf, response, spec);

    // extract the maximum response
    minMaxLoc(response, &minVal, &maxVal, &minLoc, &maxLoc);
    // if (maxVal < params.detect_thresh)
    // {
    //   return;
    // }
    // ROS_INFO("proposal max response: %f", maxVal);
  }
}

// !!!! TODO: select ROI according to HSV segmentation, vheicle ROI and motion
// estimation
// ********************* update bounding box scale *********************
void CustomTrackerKCFImpl::analyzeProposals(const Mat &image,
                                            const Rect2d &bounding_box) {
  std::vector<Rect2d> proposals;
  std::vector<Rect2d> proposals_padding;
  if (rematch_flag) {
    auto bounding_box_padding = bounding_box;
    bounding_box_padding.x -= bounding_box_padding.width / 2;
    bounding_box_padding.y -= bounding_box_padding.height / 2;
    bounding_box_padding.width *= 2;
    bounding_box_padding.height *= 2;

    roi.x = (resizeImage ? bounding_box_padding.x / 2 : bounding_box_padding.x);
    roi.y = (resizeImage ? bounding_box_padding.y / 2 : bounding_box_padding.y);
    roi.width = (resizeImage ? bounding_box_padding.width / 2
                             : bounding_box_padding.width);
    roi.height = (resizeImage ? bounding_box_padding.height / 2
                              : bounding_box_padding.height);
    return;
  }

  // get proposals
  auto t_1 = ros::WallTime::now().toSec();
  if (frame >= 0 || is_night)
    // nighttime
    tracking::rearlight_proposal::getRearlightProposal(image, bounding_box,
                                                       is_night, proposals);
  else {
    // daytime
    tracking::rearlight_proposal::getRearlightProposal(
        image, bounding_box, is_night, proposal_thresh, proposals);
  }
  auto t_2 = ros::WallTime::now().toSec();
  std::cout << "get proposals cost " << t_2 - t_1 << " sec" << std::endl;
  // ROS_INFO("analyze A");

  if (proposals.size() == 0)
    return;

  // ROS_INFO("analyze B");
  // analyze proposals
  auto t_5 = ros::WallTime::now().toSec();
  std::vector<double> proposals_min_val;
  std::vector<double> proposals_max_val;
  std::vector<Point> proposals_min_loc;
  std::vector<Point> proposals_max_loc;
  for (const auto &proposal : proposals) {
    double curr_proposal_min_val, curr_proposal_max_val;
    Point curr_proposal_min_loc, curr_proposal_max_loc;
    // add padding to proposals
    auto proposal_padding = proposal;
    proposal_padding.x -= proposal_padding.width / 2;
    proposal_padding.y -= proposal_padding.height / 2;
    proposal_padding.width *= 2;
    proposal_padding.height *= 2;
    proposals_padding.emplace_back(proposal_padding);

    // check with vehicle bbox
    auto intersec = (*vehicle_bbox_pt) & proposal;
    if (intersec.area() < 0.75 * proposal.area()) {
      proposals_min_val.emplace_back(-1.0);
      proposals_max_val.emplace_back(-1.0);
      continue;
    }
    if (proposal.width > 0.4 * (vehicle_bbox_pt->width) ||
        proposal.height > 0.2 * (vehicle_bbox_pt->height)) {
      // ROS_INFO("aaaaaaaaaaaaa");
      proposals_min_val.emplace_back(-1.0);
      // ROS_INFO("bbbbbbbbbbbbb");
      proposals_max_val.emplace_back(-1.0);
      // ROS_INFO("ccccccccccccc");
      continue;
    }

    auto inference_proposal = proposal_padding;
    if (resizeImage) {
      inference_proposal.x /= 2;
      inference_proposal.y /= 2;
      inference_proposal.width /= 2;
      inference_proposal.height /= 2;
    }
    detect(image, inference_proposal, curr_proposal_min_val,
           curr_proposal_max_val, curr_proposal_min_loc, curr_proposal_max_loc);
    proposals_min_val.emplace_back(curr_proposal_min_val);
    proposals_max_val.emplace_back(curr_proposal_max_val);
    proposals_min_loc.emplace_back(curr_proposal_min_loc);
    proposals_max_loc.emplace_back(curr_proposal_max_loc);
  }
  auto t_6 = ros::WallTime::now().toSec();
  std::cout << "check prop cost " << t_6 - t_5 << " sec" << std::endl;

  // // analyze roi
  // auto t_3 = ros::WallTime::now().toSec();
  // double roi_min_val, roi_max_val;
  // Point roi_min_loc, roi_max_loc;
  // auto img_for_roi = image.clone();
  // detect(img_for_roi, roi, roi_min_val, roi_max_val, roi_min_loc,
  // roi_max_loc); auto t_4 = ros::WallTime::now().toSec(); std::cout << "detect
  // once cost " << t_4 - t_3 << " sec" << std::endl;

  // ROS_INFO("analyze D");
  // find best proposal
  auto t_7 = ros::WallTime::now().toSec();
  int best_proposal_idx = -1;
  for (int i = 0; i < proposals.size(); ++i) {
    // ROS_INFO("analyze E");
    // matching gate
    Point roi_center(roi.x + roi.width / 2, roi.y + roi.height / 2);
    // ROS_INFO("analyze e");
    Point proposal_center(
        proposals_padding[i].x + proposals_padding[i].width / 2,
        proposals_padding[i].y + proposals_padding[i].height / 2);
    // ROS_INFO("analyze EE");
    auto dis = norm(roi_center - proposal_center);
    std::cout << "dis = " << dis << std::endl;
    if (dis > 45)
      continue;
    if (frame < 10 || proposals_max_val[i] >= 0.2)
    // if (false)
    // if (i == 0)
    {
      // ROS_INFO("fit ROI");
      // ROS_INFO("p_x: %.1f, p_y: %.1f", proposals_padding[i].x,
      // proposals_padding[i].y); ROS_INFO("before fit, roi_x: %.1f, roi_y:
      // %.1f", roi.x, roi.y);
      // ROS_INFO("analyze F");
      // rectangle(img, proposals[i], Scalar(0, 255, 0), 2, 0);
      best_proposal_idx = i;
      // rectangle(img, proposals_padding[i], Scalar(0, 255, 0), 2, 0);
      roi.width = (1 - update_alpha) * roi.width +
                  update_alpha * (resizeImage ? proposals_padding[i].width / 2
                                              : proposals_padding[i].width);
      roi.height = (1 - update_alpha) * roi.height +
                   update_alpha * (resizeImage ? proposals_padding[i].height / 2
                                               : proposals_padding[i].height);

      // ROS_INFO("analyze G");
      if (proposals_max_val[i] >= 0.6) {
        auto large_update_alpha = std::max(8 * update_alpha, 1.0);
        roi.x = (1 - large_update_alpha) * roi.x +
                large_update_alpha * (resizeImage ? proposals_padding[i].x / 2
                                                  : proposals_padding[i].x);
        roi.y = (1 - large_update_alpha) * roi.y +
                large_update_alpha * (resizeImage ? proposals_padding[i].y / 2
                                                  : proposals_padding[i].y);
      } else {
        roi.x = (1 - update_alpha) * roi.x +
                update_alpha * (resizeImage ? proposals_padding[i].x / 2
                                            : proposals_padding[i].x);
        roi.y = (1 - update_alpha) * roi.y +
                update_alpha * (resizeImage ? proposals_padding[i].y / 2
                                            : proposals_padding[i].y);
      }

      // ROS_INFO("after fit, roi_x: %.1f, roi_y: %.1f", roi.x, roi.y);
      break;
    }
  }
  auto t_8 = ros::WallTime::now().toSec();
  std::cout << "fit roi cost " << t_8 - t_7 << " sec" << std::endl;
  // ROS_INFO("analyze I");

  // auto t_9 = ros::WallTime::now().toSec();
  // for (int i = 0; i < proposals.size(); ++i) {
  //   if (i == best_proposal_idx)
  //     continue;
  //   rectangle(img, proposals[i], Scalar(0, 0, 255), 2, 0);
  // }
  // imshow("proposals", img);
  // waitKey(5);
  // auto t_10 = ros::WallTime::now().toSec();
  // std::cout << "show cost " << t_10 - t_9 << " sec" << std::endl;
  // ROS_INFO("analyze J");
}

void CustomTrackerKCFImpl::fitProposalSegmentation(const Mat &image,
                                                   const Rect2d &bounding_box) {
  // extract patch
  Rect region = bounding_box;
  Point center;
  center.x = region.x + region.width / 2;
  center.y = region.y + region.height / 2;
  region.width /= 2;
  region.height /= 2;
  region.x = center.x - region.width / 2;
  region.y = center.y - region.height / 2;

  if ((roi & Rect2d(0, 0, image.cols, image.rows)) == Rect2d())
    return;

  if (bounding_box.x < 0) {
    region.x = 0;
    region.width += bounding_box.x;
  }
  if (bounding_box.y < 0) {
    region.y = 0;
    region.height += bounding_box.y;
  }
  if (bounding_box.x + bounding_box.width > image.cols)
    region.width = image.cols - bounding_box.x;
  if (bounding_box.y + bounding_box.height > image.rows)
    region.height = image.rows - bounding_box.y;
  if (region.width > image.cols)
    region.width = image.cols;
  if (region.height > image.rows)
    region.height = image.rows;

  if (region.empty())
    return;

  auto analysis_roi = image(region).clone();
  Mat analysis_hsv;
  cvtColor(analysis_roi, analysis_hsv, CV_BGR2HSV);

  // auto cp_img = image.clone();
  // rectangle(cp_img, region, Scalar(0, 0, 255), 2, 0);
  // imshow("fit", cp_img);
  // waitKey(5);

  double max_val[3];
  double min_val[3];
  Point max_loc[3];
  Point min_loc[3];
  Mat hsv_channels[3];
  split(analysis_hsv, hsv_channels);
  for (int i = 0; i < 3; ++i) {
    minMaxLoc(hsv_channels[i], &min_val[i], &max_val[i], &min_loc[i],
              &max_loc[i]);
  }
}

void CustomTrackerKCFImpl::adjustThreshold(const Mat &image,
                                           const Rect2d &bounding_box) {
  const int K = cluster_num;
  // ROS_INFO("adjust A");
  // prepare kmeans data
  auto image_roi = image(bounding_box);
  std::vector<Mat> image_bgr;
  split(image_roi, image_bgr);
  int n = image_roi.cols * image_roi.rows;
  Mat points(n, 3, CV_8U);
  for (int i = 0; i < 3; ++i) {
    image_bgr[i].reshape(1, n).copyTo(points.col(i));
  }
  points.convertTo(points, CV_32F);

  Mat labels;
  Mat centers; // (num_cluster, dim)
  double compactness =
      kmeans(points, K, labels,
             TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 10, 1.0), 3,
             KMEANS_PP_CENTERS, centers);

  // analyze kmeans
  // std::cout << "center = \n" << centers << std::endl;
  std::vector<int> best_cluster_candidates;
  for (int cluster = 0; cluster < K; ++cluster) {
    // Vec3b cluster_bgr_center = centers.row(cluster);
    // if (cluster_bgr_center.val[2] > cluster_bgr_center.val[0] &&
    //     cluster_bgr_center.val[2] > cluster_bgr_center.val[1])
    best_cluster_candidates.emplace_back(cluster);
  }

  // extract last frame averaged bgr and hsv of rearlight part
  // TODO: gaussian weighting average, to stress center part
  Rect2d sampling_roi = bounding_box;
  auto center_x = sampling_roi.x + sampling_roi.width / 2;
  auto center_y = sampling_roi.y + sampling_roi.height / 2;
  sampling_roi.width /= 2.0;
  sampling_roi.height /= 2.0;
  sampling_roi.x = center_x - sampling_roi.width / 2;
  sampling_roi.y = center_y - sampling_roi.height / 2;

  Mat image_sampling_roi = image(sampling_roi);
  Mat image_sampling_roi_hsv;
  cvtColor(image_sampling_roi, image_sampling_roi_hsv, CV_BGR2HSV);
  Scalar mean_hsv = mean(image_sampling_roi_hsv);
  Point3f mean_hsv_pt;
  mean_hsv_pt.x = mean_hsv.val[0];
  mean_hsv_pt.y = mean_hsv.val[1];
  mean_hsv_pt.z = mean_hsv.val[2];
  Point2f mean_sv_pt;
  mean_sv_pt.x = mean_hsv.val[1];
  mean_sv_pt.y = mean_hsv.val[2];
  // std::cout << "mean hsv = " << mean_hsv_pt << std::endl;

  Scalar mean_bgr = mean(image_roi);
  Point3f mean_bgr_pt;
  mean_bgr_pt.x = mean_bgr.val[0];
  mean_bgr_pt.y = mean_bgr.val[1];
  mean_bgr_pt.z = mean_bgr.val[2];
  // std::cout << "mean bgr = " << mean_bgr_pt << std::endl;

  // ROS_INFO("adjust B");
  // find best cluster accoding to bgr and hsv distance with average
  auto labels_reshaped = labels.reshape(0, image_roi.rows);
  labels_reshaped.convertTo(labels_reshaped, CV_8U);
  std::vector<std::pair<float, int>> final_candidates;
  std::unordered_map<int, Mat> final_candidate_masks;
  std::unordered_map<int, Mat> final_candidats_hsv;
  for (const auto cluster : best_cluster_candidates) {
    Mat cluster_bgr(1, 1, CV_8UC3);
    for (int channel = 0; channel < 3; ++channel) {
      cluster_bgr.at<Vec3b>(0, 0)[channel] =
          centers.at<float>(cluster, channel);
    }
    Mat cluster_hsv;
    cvtColor(cluster_bgr, cluster_hsv, CV_BGR2HSV);
    // std::cout << "hsv = " << cluster_hsv << std::endl;

    Point3f cluster_bgr_pt;
    cluster_bgr_pt.x = centers.at<float>(cluster, 0);
    cluster_bgr_pt.y = centers.at<float>(cluster, 1);
    cluster_bgr_pt.z = centers.at<float>(cluster, 2);

    Point2f cluster_sv_pt;
    cluster_sv_pt.x = cluster_hsv.at<Vec3b>(0, 0)[1];
    cluster_sv_pt.y = cluster_hsv.at<Vec3b>(0, 0)[2];

    auto bgr_dis = norm(cluster_bgr_pt - mean_bgr_pt);
    auto sv_dis = norm(cluster_sv_pt - mean_sv_pt);
    auto total_dis = bgr_dis + sv_dis;

    // ROS_INFO("adjust C");
    final_candidates.emplace_back(std::make_pair(total_dis, cluster));

    // ROS_INFO("adjust D");
    Mat cluster_mask;
    inRange(labels_reshaped, cv::Scalar(cluster), cv::Scalar(cluster),
            cluster_mask);
    final_candidate_masks[cluster] = cluster_mask;
    final_candidats_hsv[cluster] = cluster_hsv;
    // std::cout << "bgr dis = " << bgr_dis << std::endl;
    // std::cout << "sv dis = " << sv_dis << std::endl;
    // std::cout << "total dis = " << total_dis << std::endl;
  }
  // ROS_INFO("adjust E");
  auto cmp_candidates = [](const std::pair<float, int> &a,
                           const std::pair<float, int> &b) {
    return a.first <= b.first;
  };
  // ROS_INFO("adjust F");
  std::sort(final_candidates.begin(), final_candidates.end(), cmp_candidates);
  // ROS_INFO("adjust G");
  // analyze N best clusters
  for (int i = 0; i < std::min(static_cast<int>(best_cluster_candidates.size()),
                               thresh_num);
       ++i) {
    Mat image_roi_hsv;
    cvtColor(image(bounding_box), image_roi_hsv, CV_BGR2HSV);
    Scalar cluster_mean, cluster_std;
    meanStdDev(image_roi_hsv, cluster_mean, cluster_std,
               final_candidate_masks[final_candidates[i].second]);
    auto std_s = cluster_std.val[1];
    auto std_v = cluster_std.val[2];
    auto curr_thresh_s =
        final_candidats_hsv[final_candidates[i].second].at<Vec3b>(0, 0)[1] -
        0.1 * std_s;
    auto curr_thresh_v =
        final_candidats_hsv[final_candidates[i].second].at<Vec3b>(0, 0)[2] -
        0.1 * std_v;
    proposal_thresh[i] = std::make_pair(curr_thresh_s, curr_thresh_v);
  }
  // ROS_INFO("adjust H");
}

/*
 * hann window filter
 */
void CustomTrackerKCFImpl::createHanningWindow(OutputArray dest,
                                               const cv::Size winSize,
                                               const int type) const {
  CV_Assert(type == CV_32FC1 || type == CV_64FC1);

  dest.create(winSize, type);
  Mat dst = dest.getMat();

  int rows = dst.rows, cols = dst.cols;

  AutoBuffer<float> _wc(cols);
  float *const wc = _wc.data();

  const float coeff0 = 2.0f * (float)CV_PI / (cols - 1);
  const float coeff1 = 2.0f * (float)CV_PI / (rows - 1);
  for (int j = 0; j < cols; j++)
    wc[j] = 0.5f * (1.0f - cos(coeff0 * j));

  if (dst.depth() == CV_32F) {
    for (int i = 0; i < rows; i++) {
      float *dstData = dst.ptr<float>(i);
      float wr = 0.5f * (1.0f - cos(coeff1 * i));
      for (int j = 0; j < cols; j++)
        dstData[j] = (float)(wr * wc[j]);
    }
  } else {
    for (int i = 0; i < rows; i++) {
      double *dstData = dst.ptr<double>(i);
      double wr = 0.5f * (1.0f - cos(coeff1 * i));
      for (int j = 0; j < cols; j++)
        dstData[j] = wr * wc[j];
    }
  }

  // perform batch sqrt for SSE performance gains
  // cv::sqrt(dst, dst); //matlab do not use the square rooted version
}

/*
 * simplification of fourier transform function in opencv
 */
void inline CustomTrackerKCFImpl::fft2(const Mat src, Mat &dest) const {
  dft(src, dest, DFT_COMPLEX_OUTPUT);
}

void inline CustomTrackerKCFImpl::fft2(const Mat src, std::vector<Mat> &dest,
                                       std::vector<Mat> &layers_data) const {
  split(src, layers_data);

  for (int i = 0; i < src.channels(); i++) {
    dft(layers_data[i], dest[i], DFT_COMPLEX_OUTPUT);
  }
}

/*
 * simplification of inverse fourier transform function in opencv
 */
void inline CustomTrackerKCFImpl::ifft2(const Mat src, Mat &dest) const {
  idft(src, dest, DFT_SCALE + DFT_REAL_OUTPUT);
}

/*
 * Point-wise multiplication of two Multichannel Mat data
 */
void inline CustomTrackerKCFImpl::pixelWiseMult(const std::vector<Mat> src1,
                                                const std::vector<Mat> src2,
                                                std::vector<Mat> &dest,
                                                const int flags,
                                                const bool conjB) const {
  for (unsigned i = 0; i < src1.size(); i++) {
    mulSpectrums(src1[i], src2[i], dest[i], flags, conjB);
  }
}

/*
 * Combines all channels in a multi-channels Mat data into a single channel
 */
void inline CustomTrackerKCFImpl::sumChannels(std::vector<Mat> src,
                                              Mat &dest) const {
  dest = src[0].clone();
  for (unsigned i = 1; i < src.size(); i++) {
    dest += src[i];
  }
}

#ifdef HAVE_OPENCL
bool inline CustomTrackerKCFImpl::oclTransposeMM(const Mat src, float alpha,
                                                 UMat &dst) {
  // Current kernel only support matrix's rows is multiple of 4.
  // And if one line is less than 512KB, CPU will likely be faster.
  if (transpose_mm_ker.empty() || src.rows % 4 != 0 ||
      (src.rows * 10) < (1024 * 1024 / 4))
    return false;

  Size s(src.rows, src.cols);
  const Mat tmp = src.t();
  const UMat uSrc = tmp.getUMat(ACCESS_READ);
  transpose_mm_ker.args(ocl::KernelArg::PtrReadOnly(uSrc), (int)uSrc.rows,
                        (int)uSrc.cols, alpha,
                        ocl::KernelArg::PtrWriteOnly(dst));
  size_t globSize[2] = {static_cast<size_t>(src.cols * 64),
                        static_cast<size_t>(src.cols)};
  size_t localSize[2] = {64, 1};
  if (!transpose_mm_ker.run(2, globSize, localSize, true))
    return false;
  return true;
}
#endif

/*
 * obtains the projection matrix using PCA
 */
void inline CustomTrackerKCFImpl::updateProjectionMatrix(
    const Mat src, Mat &old_cov, Mat &proj_matrix, float pca_rate,
    int compressed_sz, std::vector<Mat> &layers_pca,
    std::vector<Scalar> &average, Mat pca_data, Mat new_cov, Mat w, Mat u,
    Mat vt) {
  CV_Assert(compressed_sz <= src.channels());

  split(src, layers_pca);

  for (int i = 0; i < src.channels(); i++) {
    average[i] = mean(layers_pca[i]);
    layers_pca[i] -= average[i];
  }

  // calc covariance matrix
  merge(layers_pca, pca_data);
  pca_data = pca_data.reshape(1, src.rows * src.cols);

#ifdef HAVE_OPENCL
  bool oclSucceed = false;
  Size s(pca_data.cols, pca_data.cols);
  UMat result(s, pca_data.type());
  if (oclTransposeMM(pca_data, 1.0f / (float)(src.rows * src.cols - 1),
                     result)) {
    if (old_cov.rows == 0)
      old_cov = result.getMat(ACCESS_READ).clone();
    SVD::compute((1.0 - pca_rate) * old_cov +
                     pca_rate * result.getMat(ACCESS_READ),
                 w, u, vt);
    oclSucceed = true;
  }
#define TMM_VERIFICATION 0

  if (oclSucceed == false || TMM_VERIFICATION) {
    new_cov =
        1.0f / (float)(src.rows * src.cols - 1) * (pca_data.t() * pca_data);
#if TMM_VERIFICATION
    for (int i = 0; i < new_cov.rows; i++)
      for (int j = 0; j < new_cov.cols; j++)
        if (abs(new_cov.at<float>(i, j) -
                result.getMat(ACCESS_RW).at<float>(i, j)) >
            abs(new_cov.at<float>(i, j)) * 1e-3)
          printf("error @ i %d j %d got %G expected %G \n", i, j,
                 result.getMat(ACCESS_RW).at<float>(i, j),
                 new_cov.at<float>(i, j));
#endif
    if (old_cov.rows == 0)
      old_cov = new_cov.clone();
    SVD::compute((1.0f - pca_rate) * old_cov + pca_rate * new_cov, w, u, vt);
  }
#else
  new_cov = 1.0 / (float)(src.rows * src.cols - 1) * (pca_data.t() * pca_data);
  if (old_cov.rows == 0)
    old_cov = new_cov.clone();

  // calc PCA
  SVD::compute((1.0 - pca_rate) * old_cov + pca_rate * new_cov, w, u, vt);
#endif
  // extract the projection matrix
  proj_matrix = u(Rect(0, 0, compressed_sz, src.channels())).clone();
  Mat proj_vars = Mat::eye(compressed_sz, compressed_sz, proj_matrix.type());
  for (int i = 0; i < compressed_sz; i++) {
    proj_vars.at<float>(i, i) = w.at<float>(i);
  }

  // update the covariance matrix
  old_cov = (1.0 - pca_rate) * old_cov +
            pca_rate * proj_matrix * proj_vars * proj_matrix.t();
}

/*
 * compress the features
 */
void inline CustomTrackerKCFImpl::compress(const Mat proj_matrix, const Mat src,
                                           Mat &dest, Mat &data,
                                           Mat &compressed) const {
  data = src.reshape(1, src.rows * src.cols);
  compressed = data * proj_matrix;
  dest = compressed.reshape(proj_matrix.cols, src.rows).clone();
}

/*
 * obtain the patch and apply hann window filter to it
 */
bool CustomTrackerKCFImpl::getSubWindow(const Mat img, const Rect _roi,
                                        Mat &feat, Mat &patch,
                                        CustomTrackerKCF::MODE desc) const {

  Rect region = _roi;

  // return false if roi is outside the image
  if ((roi & Rect2d(0, 0, img.cols, img.rows)) == Rect2d())
    return false;

  // extract patch inside the image
  if (_roi.x < 0) {
    region.x = 0;
    region.width += _roi.x;
  }
  if (_roi.y < 0) {
    region.y = 0;
    region.height += _roi.y;
  }
  if (_roi.x + _roi.width > img.cols)
    region.width = img.cols - _roi.x;
  if (_roi.y + _roi.height > img.rows)
    region.height = img.rows - _roi.y;
  if (region.width > img.cols)
    region.width = img.cols;
  if (region.height > img.rows)
    region.height = img.rows;

  // return false if region is empty
  if (region.empty())
    return false;

  patch = img(region).clone();

  // add some padding to compensate when the patch is outside image border
  int addTop, addBottom, addLeft, addRight;
  addTop = region.y - _roi.y;
  addBottom =
      (_roi.height + _roi.y > img.rows ? _roi.height + _roi.y - img.rows : 0);
  addLeft = region.x - _roi.x;
  addRight =
      (_roi.width + _roi.x > img.cols ? _roi.width + _roi.x - img.cols : 0);

  copyMakeBorder(patch, patch, addTop, addBottom, addLeft, addRight,
                 BORDER_REPLICATE);

  if (patch.rows == 0 || patch.cols == 0)
    return false;

  resize(patch, patch, temp_size, 0, 0, INTER_LINEAR_EXACT);

  // extract the desired descriptors
  switch (desc) {
  case CN:
    CV_Assert(img.channels() == 3);
    extractCN(patch, feat);
    feat = feat.mul(hann_cn); // hann window filter
    break;
  default: // GRAY
    if (img.channels() > 1)
      cvtColor(patch, feat, COLOR_BGR2GRAY);
    else
      feat = patch;
    // feat.convertTo(feat,CV_32F);
    feat.convertTo(feat, CV_32F, 1.0 / 255.0, -0.5);
    // feat=feat/255.0-0.5; // normalize to range -0.5 .. 0.5
    feat = feat.mul(hann); // hann window filter
    break;
  }

  return true;
}

/*
 * get feature using external function
 */
bool CustomTrackerKCFImpl::getSubWindow(const Mat img, const Rect _roi,
                                        Mat &feat,
                                        void (*f)(const Mat, const Rect,
                                                  Mat &)) const {

  // return false if roi is outside the image
  if ((_roi.x + _roi.width < 0) || (_roi.y + _roi.height < 0) ||
      (_roi.x >= img.cols) || (_roi.y >= img.rows))
    return false;

  f(img, _roi, feat);

  if (_roi.width != feat.cols || _roi.height != feat.rows) {
    printf("error in customized function of features extractor!\n");
    printf("Rules: roi.width==feat.cols && roi.height = feat.rows \n");
  }

  Mat hann_win;
  std::vector<Mat> _layers;

  for (int i = 0; i < feat.channels(); i++)
    _layers.push_back(hann);

  merge(_layers, hann_win);

  feat = feat.mul(hann_win); // hann window filter

  return true;
}

/* Convert BGR to ColorNames
 */
void CustomTrackerKCFImpl::extractCN(Mat patch_data, Mat &cnFeatures) const {
  Vec3b &pixel = patch_data.at<Vec3b>(0, 0);
  unsigned index;

  if (cnFeatures.type() != CV_32FC(10))
    cnFeatures = Mat::zeros(patch_data.rows, patch_data.cols, CV_32FC(10));

  for (int i = 0; i < patch_data.rows; i++) {
    for (int j = 0; j < patch_data.cols; j++) {
      pixel = patch_data.at<Vec3b>(i, j);
      index = (unsigned)(floor((float)pixel[2] / 8) +
                         32 * floor((float)pixel[1] / 8) +
                         32 * 32 * floor((float)pixel[0] / 8));

      // copy the values
      for (int _k = 0; _k < 10; _k++) {
        cnFeatures.at<Vec<float, 10>>(i, j)[_k] = ColorNames[index][_k];
      }
    }
  }
}

/*
 *  dense gauss kernel function
 */
void CustomTrackerKCFImpl::denseGaussKernel(
    const float sigma, const Mat x_data, const Mat y_data, Mat &k_data,
    std::vector<Mat> &layers_data, std::vector<Mat> &xf_data,
    std::vector<Mat> &yf_data, std::vector<Mat> xyf_v, Mat xy, Mat xyf) const {
  double normX, normY;

  fft2(x_data, xf_data, layers_data);
  fft2(y_data, yf_data, layers_data);

  normX = norm(x_data);
  normX *= normX;
  normY = norm(y_data);
  normY *= normY;

  pixelWiseMult(xf_data, yf_data, xyf_v, 0, true);
  sumChannels(xyf_v, xyf);
  ifft2(xyf, xyf);

  if (params.wrap_kernel) {
    shiftRows(xyf, x_data.rows / 2);
    shiftCols(xyf, x_data.cols / 2);
  }

  //(xx + yy - 2 * xy) / numel(x)
  xy = (normX + normY - 2 * xyf) /
       (x_data.rows * x_data.cols * x_data.channels());

  // TODO: check wether we really need thresholding or not
  // threshold(xy,xy,0.0,0.0,THRESH_TOZERO);//max(0, (xx + yy - 2 * xy) /
  // numel(x))
  for (int i = 0; i < xy.rows; i++) {
    for (int j = 0; j < xy.cols; j++) {
      if (xy.at<float>(i, j) < 0.0)
        xy.at<float>(i, j) = 0.0;
    }
  }

  float sig = -1.0f / (sigma * sigma);
  xy = sig * xy;
  exp(xy, k_data);
}

/* CIRCULAR SHIFT Function
 * http://stackoverflow.com/questions/10420454/shift-like-matlab-function-rows-or-columns-of-a-matrix-in-opencv
 */
// circular shift one row from up to down
void CustomTrackerKCFImpl::shiftRows(Mat &mat) const {

  Mat temp;
  Mat m;
  int _k = (mat.rows - 1);
  mat.row(_k).copyTo(temp);
  for (; _k > 0; _k--) {
    m = mat.row(_k);
    mat.row(_k - 1).copyTo(m);
  }
  m = mat.row(0);
  temp.copyTo(m);
}

// circular shift n rows from up to down if n > 0, -n rows from down to up if n
// < 0
void CustomTrackerKCFImpl::shiftRows(Mat &mat, int n) const {
  if (n < 0) {
    n = -n;
    flip(mat, mat, 0);
    for (int _k = 0; _k < n; _k++) {
      shiftRows(mat);
    }
    flip(mat, mat, 0);
  } else {
    for (int _k = 0; _k < n; _k++) {
      shiftRows(mat);
    }
  }
}

// circular shift n columns from left to right if n > 0, -n columns from right
// to left if n < 0
void CustomTrackerKCFImpl::shiftCols(Mat &mat, int n) const {
  if (n < 0) {
    n = -n;
    flip(mat, mat, 1);
    transpose(mat, mat);
    shiftRows(mat, n);
    transpose(mat, mat);
    flip(mat, mat, 1);
  } else {
    transpose(mat, mat);
    shiftRows(mat, n);
    transpose(mat, mat);
  }
}

/*
 * calculate the detection response
 */
void CustomTrackerKCFImpl::calcResponse(const Mat alphaf_data,
                                        const Mat kf_data, Mat &response_data,
                                        Mat &spec_data) const {
  // alpha f--> 2channels ; k --> 1 channel;
  mulSpectrums(alphaf_data, kf_data, spec_data, 0, false);
  ifft2(spec_data, response_data);
}

/*
 * calculate the detection response for splitted form
 */
void CustomTrackerKCFImpl::calcResponse(const Mat alphaf_data,
                                        const Mat _alphaf_den,
                                        const Mat kf_data, Mat &response_data,
                                        Mat &spec_data, Mat &spec2_data) const {

  mulSpectrums(alphaf_data, kf_data, spec_data, 0, false);

  // z=(a+bi)/(c+di)=[(ac+bd)+i(bc-ad)]/(c^2+d^2)
  float den;
  for (int i = 0; i < kf_data.rows; i++) {
    for (int j = 0; j < kf_data.cols; j++) {
      den = 1.0f /
            (_alphaf_den.at<Vec2f>(i, j)[0] * _alphaf_den.at<Vec2f>(i, j)[0] +
             _alphaf_den.at<Vec2f>(i, j)[1] * _alphaf_den.at<Vec2f>(i, j)[1]);
      spec2_data.at<Vec2f>(i, j)[0] =
          (spec_data.at<Vec2f>(i, j)[0] * _alphaf_den.at<Vec2f>(i, j)[0] +
           spec_data.at<Vec2f>(i, j)[1] * _alphaf_den.at<Vec2f>(i, j)[1]) *
          den;
      spec2_data.at<Vec2f>(i, j)[1] =
          (spec_data.at<Vec2f>(i, j)[1] * _alphaf_den.at<Vec2f>(i, j)[0] -
           spec_data.at<Vec2f>(i, j)[0] * _alphaf_den.at<Vec2f>(i, j)[1]) *
          den;
    }
  }

  ifft2(spec2_data, response_data);
}

void CustomTrackerKCFImpl::setFeatureExtractor(void (*f)(const Mat, const Rect,
                                                         Mat &),
                                               bool pca_func) {
  if (pca_func) {
    extractor_pca.push_back(f);
    use_custom_extractor_pca = true;
  } else {
    extractor_npca.push_back(f);
    use_custom_extractor_npca = true;
  }
}
/*----------------------------------------------------------------------*/

/*
 * Parameters
 */
CustomTrackerKCF::Params::Params() {
  detect_thresh = 0.01f;
  sigma = 0.2f;
  lambda = 0.0001f;
  interp_factor = 0.075f;
  output_sigma_factor = 1.0f / 16.0f;
  resize = true;
  max_patch_size = 80 * 80;
  split_coeff = true;
  wrap_kernel = false;
  desc_npca = GRAY;
  desc_pca = CN;

  // feature compression
  compress_feature = true;
  compressed_size = 2;
  pca_learning_rate = 0.15f;
}

void CustomTrackerKCF::Params::read(const cv::FileNode &fn) {
  *this = CustomTrackerKCF::Params();

  if (!fn["detect_thresh"].empty())
    fn["detect_thresh"] >> detect_thresh;

  if (!fn["sigma"].empty())
    fn["sigma"] >> sigma;

  if (!fn["lambda"].empty())
    fn["lambda"] >> lambda;

  if (!fn["interp_factor"].empty())
    fn["interp_factor"] >> interp_factor;

  if (!fn["output_sigma_factor"].empty())
    fn["output_sigma_factor"] >> output_sigma_factor;

  if (!fn["resize"].empty())
    fn["resize"] >> resize;

  if (!fn["max_patch_size"].empty())
    fn["max_patch_size"] >> max_patch_size;

  if (!fn["split_coeff"].empty())
    fn["split_coeff"] >> split_coeff;

  if (!fn["wrap_kernel"].empty())
    fn["wrap_kernel"] >> wrap_kernel;

  if (!fn["desc_npca"].empty())
    fn["desc_npca"] >> desc_npca;

  if (!fn["desc_pca"].empty())
    fn["desc_pca"] >> desc_pca;

  if (!fn["compress_feature"].empty())
    fn["compress_feature"] >> compress_feature;

  if (!fn["compressed_size"].empty())
    fn["compressed_size"] >> compressed_size;

  if (!fn["pca_learning_rate"].empty())
    fn["pca_learning_rate"] >> pca_learning_rate;
}

void CustomTrackerKCF::Params::write(cv::FileStorage &fs) const {
  fs << "detect_thresh" << detect_thresh;
  fs << "sigma" << sigma;
  fs << "lambda" << lambda;
  fs << "interp_factor" << interp_factor;
  fs << "output_sigma_factor" << output_sigma_factor;
  fs << "resize" << resize;
  fs << "max_patch_size" << max_patch_size;
  fs << "split_coeff" << split_coeff;
  fs << "wrap_kernel" << wrap_kernel;
  fs << "desc_npca" << desc_npca;
  fs << "desc_pca" << desc_pca;
  fs << "compress_feature" << compress_feature;
  fs << "compressed_size" << compressed_size;
  fs << "pca_learning_rate" << pca_learning_rate;
}
} /* namespace cv */

// #endif