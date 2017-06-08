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

#include <algorithm>
#include <math.h>
#include <vector>

#include "image_labelers/diff_helpers/normal_diff.h"

namespace depth_clustering {

NormalDiff::NormalDiff(const cv::Mat* source_image,
                       const ProjectionParams* params)
    : AbstractDiff{source_image}, _params{params} {
  if (source_image->type() != CV_32F) {
    throw std::runtime_error(" NormalComputer: depth_image should be float");
  }
  _angles_image = cv::Mat::zeros(source_image->size(), CV_32FC3);
  _point_image = cv::Mat::zeros(source_image->size(), CV_32FC3);
  for (int r = 0; r < source_image->rows; ++r) {
    for (int c = 0; c < source_image->cols; ++c) {
      float depth = source_image->at<float>(r, c);
      Radians angle_z = params->AngleFromRow(r);
      Radians angle_xy = params->AngleFromCol(c);
      cv::Vec3f point{depth * cosf(angle_z.val()) * cosf(angle_xy.val()),
                      depth * cosf(angle_z.val()) * sinf(angle_xy.val()),
                      depth * sinf(angle_z.val())};
      _point_image.at<cv::Vec3f>(r, c) = point;
    }
  }
  _normal_computer.initPointImage(_point_image);
  _normal_computer.compute();
  _normals = _normal_computer.normalImage();
  FillAngleImage();
}

float NormalDiff::DiffAt(const PixelCoord& from, const PixelCoord& to) const {
  return std::max(_angles_image.at<cv::Vec3f>(from.row, from.col)[0],
                  _angles_image.at<cv::Vec3f>(to.row, to.col)[0]);
}

void NormalDiff::FillAngleImage() {
  for (int r = 0; r < _point_image.rows; ++r) {
    for (int c = 0; c < _point_image.cols; ++c) {
      const auto& cv_point = _point_image.at<cv::Vec3f>(r, c);
      const auto& cv_normal = _normals.at<cv::Vec3f>(r, c);
      Radians angle =
          Radians::FromRadians(acos(cv::normalize(cv_point).dot(cv_normal)));
      if (angle.ToDegrees() > 90) { angle = 180_deg - angle; }
      _angles_image.at<cv::Vec3f>(r, c)[0] = angle.val();
    }
  }
}

}  // namespace depth_clustering
