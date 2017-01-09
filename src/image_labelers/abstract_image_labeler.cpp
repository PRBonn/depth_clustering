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

#include "./abstract_image_labeler.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <memory>
#include <queue>
#include <functional>
#include <algorithm>
#include <cmath>

#include "utils/radians.h"

namespace depth_clustering {

using std::array;
using cv::Mat;
using cv::Scalar;
using cv::Vec3b;

constexpr array<array<int, 3>, 200> AbstractImageLabeler::RANDOM_COLORS;

Mat AbstractImageLabeler::LabelsToColor(const Mat& label_image) {
  Mat color_image(label_image.size(), CV_8UC3, Scalar(0, 0, 0));
  for (int row = 0; row < label_image.rows; ++row) {
    for (int col = 0; col < label_image.cols; ++col) {
      auto label = label_image.at<uint16_t>(row, col);
      auto random_color = AbstractImageLabeler::RANDOM_COLORS
          [label % AbstractImageLabeler::RANDOM_COLORS.size()];
      Vec3b color = Vec3b(random_color[0], random_color[1], random_color[2]);
      color_image.at<Vec3b>(row, col) = color;
    }
  }
  return color_image;
}

}  // namespace depth_clustering
