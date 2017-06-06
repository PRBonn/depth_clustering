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

#include "utils/normal_computer.h"

namespace depth_clustering {

void NormalComputer::compute() {
  computeSimpleNormals();
  blurNormals(1, 0);
  _normal_image = cv::Mat::zeros(_rows, _cols, CV_32FC3);
  // copy only the relevant part without borders into a new image
  _normal_image_smooth(cv::Rect(_col_gap, _row_gap, _cols, _rows))
      .copyTo(_normal_image);
}

void NormalComputer::computeSimpleNormals() {
  _normal_image_border = cv::Mat::zeros(_points_image_border.size(), CV_32FC3);
  float squared_max_distance = _max_distance * _max_distance;
  for (int r = _row_gap; r < _points_image_border.rows - _row_gap; ++r) {
    const cv::Vec3f* up_row_ptr =
        _points_image_border.ptr<const cv::Vec3f>(r - _row_gap);
    const cv::Vec3f* row_ptr = _points_image_border.ptr<const cv::Vec3f>(r);
    const cv::Vec3f* down_row_ptr =
        _points_image_border.ptr<const cv::Vec3f>(r + _row_gap);
    cv::Vec3f* dest_row_ptr = _normal_image_border.ptr<cv::Vec3f>(r);
    for (int c = _col_gap; c < _points_image_border.cols - _col_gap;
         ++c, ++up_row_ptr, ++row_ptr, ++down_row_ptr, ++dest_row_ptr) {
      const cv::Vec3f& p = _points_image_border.at<cv::Vec3f>(r, c);
      // if z is null, skip;
      if (p[2] == 0) continue;

      const cv::Vec3f& px00 = *(row_ptr - _col_gap);
      if (px00[2] == 0) continue;

      const cv::Vec3f& px01 = *(row_ptr + _col_gap);
      if (px01[2] == 0) continue;

      const cv::Vec3f& py00 = *up_row_ptr;
      if (py00[2] == 0) continue;

      const cv::Vec3f& py01 = *down_row_ptr;
      if (py01[2] == 0) continue;

      cv::Vec3f dx = px01 - px00;
      cv::Vec3f dy = py01 - py00;
      if (dx.dot(dx) > squared_max_distance ||
          dy.dot(dy) > squared_max_distance) {
        continue;
      }

      cv::Vec3f n = dy.cross(dx);
      n = normalize(n);
      if (n.dot(p) > 0) {
        n = -n;
      }
      *dest_row_ptr = n;
    }
  }
}

// from srrg_nicp
void NormalComputer::blurNormals(int window, int start) {
  _normal_image_smooth = cv::Mat::zeros(_normal_image_border.size(), CV_32FC3);
  int rows = _normal_image_smooth.rows;
  int cols = _normal_image_smooth.cols;
  cv::Mat normal_integral;
  cv::integral(_normal_image_border, normal_integral, CV_32F);
  for (int r = start + window; r < rows - start - window; ++r) {
    const cv::Vec3f* up_row_ptr =
        normal_integral.ptr<const cv::Vec3f>(r - window);
    const cv::Vec3f* down_row_ptr =
        normal_integral.ptr<const cv::Vec3f>(r + window);
    cv::Vec3f* dest_row_ptr = _normal_image_smooth.ptr<cv::Vec3f>(r);

    for (int c = start + window; c < cols - start - window;
         ++c, ++down_row_ptr, ++up_row_ptr, ++dest_row_ptr) {
      cv::Vec3f m11 = *(down_row_ptr + window);
      cv::Vec3f m00 = *(up_row_ptr - window);
      cv::Vec3f m01 = *(down_row_ptr - window);
      cv::Vec3f m10 = *(up_row_ptr + window);
      cv::Vec3f n_sum = m11 + m00 - m01 - m10;
      if (n_sum.dot(n_sum) > 0.2)
        *dest_row_ptr = normalize(n_sum);
      else
        *dest_row_ptr = cv::Vec3f(0, 0, 0);
    }
  }
}

NormalComputer::NormalComputer(int col_gap, int row_gap, int blur_cols,
                               int blur_rows, float max_distance) {
  _col_gap = col_gap;
  _row_gap = row_gap;
  _blur_cols = blur_cols;
  _blur_rows = blur_rows;
  _max_distance = max_distance;
}

void NormalComputer::initPointImage(const cv::Mat& points_image) {
  if (points_image.type() != CV_32FC3) {
    throw std::runtime_error(" NormalComputer: wrong point image type");
  }
  _rows = points_image.rows;
  _cols = points_image.cols;
  _points_image_border.create(points_image.rows + 2 * _row_gap,
                              points_image.cols + 2 * _col_gap, CV_32FC3);
  copyMakeBorder(points_image, _points_image_border, _row_gap, _row_gap,
                 _col_gap, _col_gap, cv::BORDER_REFLECT);
}

}  // namespace depth_clustering
