#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

namespace cyberc3 {
namespace planning {

/**
 * @brief Templated Berzier Curve
 */
template <typename PointType>
class BezierCurve {
 public:
  /**
   * @brief default constructor
   */
  BezierCurve() = default;

  /**
   * @brief constructor with input control points
   *
   * @param pts is the control points which are inputed
   */

  explicit BezierCurve(const std::vector<PointType> &pts) {
    SetControlPoints(pts);
  }

  /**
   * @brief get point from bezier according to t
   *
   * @param t is a param that in (0, 1)
   *
   * @return the point at t from bezier
   */
  PointType operator()(const double t) const {
    PointType ans;
    if (pts_.size() > 1) {
      if (t < 0) return pts_.front();
      if (t > 1) return pts_.back();
      for (int i = 0; i < num_; i++) {
        ans = ans + pts_[i] * std::pow(1 - t, num_ - 1 - i) * std::pow(t, i) *
                        (factorial_[num_ - 1] / factorial_[i] /
                         factorial_[num_ - 1 - i]);
      }
    }
    return ans;
  }

  /**
   * @brief reset the control points
   *
   * @param pts is the control points which are inputed
   *
   * @return whether we reset successfully
   */
  bool Reset(const std::vector<PointType> &pts) {
    return SetControlPoints(pts);
  }

 private:
  bool SetControlPoints(const std::vector<PointType> &pts) {
    pts_.clear();
    factorial_.clear();
    num_ = pts.size();
    if (num_ < 2) return false;

    pts_.resize(num_);
    pts_.assign(pts.begin(), pts.end());

    factorial_.resize(num_ + 1);
    factorial_[0] = 1;

    for (int i = 1; i <= num_; i++) {
      factorial_[i] = i * factorial_[i - 1];
    }
    return true;
  }
  std::vector<PointType> pts_;
  std::vector<double> factorial_;
  int num_ = 0;
};
}  // namespace planning
}  // namespace cyberc3
