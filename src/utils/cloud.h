// Copyright (C) 2020  I. Bogoslavskyi, C. Stachniss
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

#ifndef SRC_UTILS_CLOUD_H_
#define SRC_UTILS_CLOUD_H_

#include <Eigen/Core>
#include <Eigen/StdVector>

#if PCL_FOUND
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#endif  // PCL_FOUND

#include <algorithm>
#include <list>
#include <memory>
#include <vector>

#include "projections/cloud_projection.h"
#include "projections/ring_projection.h"
#include "projections/spherical_projection.h"
#include "utils/pose.h"
#include "utils/useful_typedefs.h"

namespace depth_clustering {

/**
 * @brief      A class that stores a vector of RichPoints
 * @details    A utility class  for storing points. If PCL is available has ways
 *             of converting to and from pcl. Also knows how to generate a
 *             projection from its points and can be generated from an image.
 */
class Cloud {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = shared_ptr<Cloud>;
  using ConstPtr = shared_ptr<const Cloud>;

  Cloud() : _points{}, _pose{}, _sensor_pose{} {}
  explicit Cloud(const Cloud& cloud);
  explicit Cloud(const Pose& pose) : _pose(pose), _sensor_pose() {}

  virtual ~Cloud() {}

  inline const RichPoint::AlignedVector& points() const { return _points; }

  inline Pose& pose() { return _pose; }
  inline const Pose& pose() const { return _pose; }

  inline Pose& sensor_pose() { return _sensor_pose; }
  inline const Pose& sensor_pose() const { return _sensor_pose; }

  inline void push_back(const RichPoint& point) { _points.push_back(point); }
  inline size_t size() const { return _points.size(); }
  inline bool empty() const { return _points.empty(); }
  inline void reserve(size_t size) { _points.reserve(size); }

  inline RichPoint& operator[](int idx) { return _points[idx]; }
  inline const RichPoint& operator[](int idx) const { return _points[idx]; }

  inline RichPoint& at(int idx) { return _points[idx]; }
  inline const RichPoint& at(int idx) const { return _points[idx]; }

  inline void Resize(size_t new_size) { _points.resize(new_size); }
  inline void SetPose(const Pose& pose) { _pose = pose; }

  inline const typename CloudProjection::ConstPtr projection_ptr() const {
    return _projection;
  }

  inline typename CloudProjection::Ptr projection_ptr() { return _projection; }

  std::list<const RichPoint*> PointsProjectedToPixel(int row, int col) const;

  void TransformInPlace(const Pose& pose);
  Cloud::Ptr Transform(const Pose& pose) const;

  void SetProjectionPtr(typename CloudProjection::Ptr proj_ptr);

  void InitProjection(const ProjectionParams& params);

  static Cloud::Ptr FromImage(const cv::Mat& image,
                              const ProjectionParams& params);

// PCL specific part
#if PCL_FOUND
  typename pcl::PointCloud<pcl::PointXYZL>::Ptr ToPcl() const;

  template <class PointT>
  static Cloud::Ptr FromPcl(const pcl::PointCloud<PointT>& pcl_cloud) {
    Cloud cloud;
    for (const auto& pcl_point : pcl_cloud) {
      RichPoint point(pcl_point.x, pcl_point.y, pcl_point.z);
      cloud.push_back(point);
    }
    return make_shared<Cloud>(cloud);
  }
#endif  // PCL_FOUND

 protected:
  RichPoint::AlignedVector _points;

  Pose _pose;
  Pose _sensor_pose;

  CloudProjection::Ptr _projection = nullptr;
};

}  // namespace depth_clustering

#endif  // SRC_UTILS_CLOUD_H_
