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
