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

#ifndef FILTERS_BOX_ROI_H_
#define FILTERS_BOX_ROI_H_

#include <pcl/point_cloud.h>

#include <vector>

namespace filters {
template<typename PointT>
class BoxRoi {
  using PointCloudT = typename pcl::PointCloud<PointT>;
  using PointCloudPtr = typename PointCloudT::Ptr;
  using PointCloudConstPtr = typename PointCloudT::ConstPtr;
 public:
  using Ptr = boost::shared_ptr<BoxRoi<PointT> >;
  using ConstPtr = boost::shared_ptr<const BoxRoi<PointT> >;
  BoxRoi() :
    box_min_x_(0), box_min_y_(-10.0), box_min_z_(0),
    box_max_x_(1000.0), box_max_y_(10.0), box_max_z_(3.0) {}
  virtual ~BoxRoi() {}
  PointCloudPtr Filter(const PointCloudPtr &in) {
    if ((in != nullptr) && (!in->empty())) {
      PointCloudPtr pcl_output(new PointCloudT);
      for (const PointT &p : in->points) {
        if ((p.x < box_max_x_) && (p.x > box_min_x_) &&
            (p.y < box_max_y_) && (p.y > box_min_y_) &&
            (p.z < box_max_z_) && (p.z > box_min_z_)) {
          pcl_output->push_back(p);
        }
      }
      pcl_output->header = in->header;
      pcl_output->width = pcl_output->size();
      return pcl_output;
    } else {
      return in;
    }
  }
  void SetBox(float min_x, float min_y, float min_z,
              float max_x, float max_y, float max_z) {
    box_min_x_ = min_x;
    box_min_y_ = min_y;
    box_min_z_ = min_z;
    box_max_x_ = max_x;
    box_max_y_ = max_y;
    box_max_z_ = max_z;
  }

 private:
  float box_min_x_;
  float box_min_y_;
  float box_min_z_;
  float box_max_x_;
  float box_max_y_;
  float box_max_z_;
};
}  // namespace filters
#endif  // FILTERS_BOX_ROI_H_
