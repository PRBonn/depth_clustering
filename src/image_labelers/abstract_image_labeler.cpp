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
