// Copyright (C) 2017  I. Bogoslavskyi, C. Stachniss, University of Bonn

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "utils/cloud.h"

namespace depth_clustering {

Cloud::Cloud(const Cloud& cloud)
    : _points{cloud.points()},
      _pose(cloud.pose()),
      _sensor_pose(cloud.sensor_pose()) {
  if (!cloud.projection_ptr()) {
    // no need to copy projection, there is none yet
    return;
  }
  // projection is a polymorphic type, we use clone therefore
  auto ptr = cloud.projection_ptr()->Clone();
  _projection = ptr;
}

std::list<const RichPoint*> Cloud::PointsProjectedToPixel(int row,
                                                          int col) const {
  std::list<const RichPoint*> point_list;
  if (!_projection) {
    return point_list;
  }
  for (const auto& index : _projection->at(row, col).points()) {
    point_list.push_back(&_points[index]);
  }
  return point_list;
}

void Cloud::TransformInPlace(const Pose& pose) {
  for (auto& point : _points) {
    point = pose * point.AsEigenVector();
  }
  // the projection makes no sense anymore after the coords of points changed.
  this->_projection.reset();
}

Cloud::Ptr Cloud::Transform(const Pose& pose) const {
  Cloud cloud_copy(*this);
  cloud_copy.TransformInPlace(pose);
  return make_shared<Cloud>(cloud_copy);
}

void Cloud::SetProjectionPtr(typename CloudProjection::Ptr proj_ptr) {
  _projection = proj_ptr;
}

void Cloud::InitProjection(const ProjectionParams& params) {
  if (_projection) {
    throw std::runtime_error("projection is already initialized");
  }
  _projection = CloudProjection::Ptr(new SphericalProjection(params));
  if (!_projection) {
    fprintf(stderr, "ERROR: failed to initalize projection.\n");
    return;
  }
  _projection = _projection->Clone();
  _projection->InitFromPoints(_points);
}

Cloud::Ptr Cloud::FromImage(const cv::Mat& image,
                            const ProjectionParams& params) {
  CloudProjection::Ptr proj = CloudProjection::Ptr(new RingProjection(params));
  proj->CheckImageAndStorage(image);
  proj->CloneDepthImage(image);
  Cloud cloud;
  for (int r = 0; r < image.rows; ++r) {
    for (int c = 0; c < image.cols; ++c) {
      if (image.at<float>(r, c) < 0.0001f) {
        continue;
      }
      RichPoint point = proj->UnprojectPoint(image, r, c);
      cloud.push_back(point);
      proj->at(r, c).points().push_back(cloud.points().size() - 1);
    }
  }
  cloud.SetProjectionPtr(proj);
  // we cannot share ownership of this cloud with others, so create a new one
  return boost::make_shared<Cloud>(cloud);
}

// this code will be only there if we use pcl
#ifdef PCL_FOUND

typename pcl::PointCloud<pcl::PointXYZL>::Ptr Cloud::ToPcl() const {
  using pcl::PointXYZL;
  using PclCloud = pcl::PointCloud<PointXYZL>;
  PclCloud pcl_cloud;
  for (const auto& point : _points) {
    PointXYZL pcl_point;
    pcl_point.x = point.x();
    pcl_point.y = point.y();
    pcl_point.z = point.z();
    pcl_point.label = point.ring();
    pcl_cloud.push_back(pcl_point);
  }
  return make_shared<PclCloud>(pcl_cloud);
}

template <>
Cloud::Ptr Cloud::FromPcl(const pcl::PointCloud<pcl::PointXYZL>& pcl_cloud) {
  Cloud cloud;
  for (const auto& pcl_point : pcl_cloud) {
    RichPoint point(pcl_point.x, pcl_point.y, pcl_point.z);
    point.ring() = pcl_point.label;
    cloud.push_back(point);
  }
  return make_shared<Cloud>(cloud);
}

#endif  // PCL_FOUND

}  // namespace depth_clustering
