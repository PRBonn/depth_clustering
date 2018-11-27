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

#include "projections/ring_projection.h"

#include <vector>

#include "utils/timer.h"

namespace depth_clustering {

void RingProjection::InitFromPoints(const RichPoint::AlignedVector& points) {
  fprintf(stderr, "Projecting cloud with %lu points\n", points.size());
  time_utils::Timer timer;
  this->CheckCloudAndStorage(points);
  // share ownership of input cloud
  for (size_t index = 0; index < points.size(); ++index) {
    const auto& point = points[index];
    float dist_to_sensor = point.DistToSensor2D();
    if (dist_to_sensor < 0.01f) {
      continue;
    }
    auto angle_cols = Radians::FromRadians(atan2(point.y(), point.x()));
    size_t bin_cols = _params.ColFromAngle(angle_cols);
    size_t bin_rows = point.ring();
    // adding point pointer
    this->_data[bin_cols][bin_rows].points().push_back(index);
    auto& current_written_depth =
        this->_depth_image.at<float>(bin_rows, bin_cols);
    if (current_written_depth < dist_to_sensor) {
      // write this point to the image only if it is closer
      current_written_depth = dist_to_sensor;
    }
  }
  fprintf(stderr, "Cloud projected in %lu us\n", timer.measure());
}

CloudProjection::Ptr RingProjection::Clone() const {
  return CloudProjection::Ptr(new RingProjection(*this));
}

RichPoint RingProjection::UnprojectPoint(const cv::Mat& image, const int row,
                                         const int col) const {
  RichPoint point = CloudProjection::UnprojectPoint(image, row, col);
  point.ring() = row;
  return point;
}

}  // namespace depth_clustering
