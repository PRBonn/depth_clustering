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

NormalComputer::NormalComputer(int col_gap,
                               int row_gap,
                               int blur_size,
                               float max_distance) {
  _col_gap = col_gap;
  _row_gap = row_gap;
  _blur_size = blur_size;
  _max_distance = max_distance;
}

void NormalComputer::compute() {
  computeNormals();
  blurNormals(_blur_size, 0);
  _normal_image = cv::Mat::zeros(_rows, _cols, CV_32FC3);
  // copy only the relevant part without borders into a new image
  _normal_image_smooth(cv::Rect(_col_gap, _row_gap, _cols, _rows))
      .copyTo(_normal_image);
}

void NormalComputer::computeNormals() {
  using cv::Vec3f;
  _normal_image_border = cv::Mat::zeros(_points_image_border.size(), CV_32FC3);
  float squared_max_distance = _max_distance * _max_distance;
  for (int r = _row_gap; r < _points_image_border.rows - _row_gap; ++r) {
    const auto& row_top = _points_image_border.row(r - _row_gap);
    const auto& row_current = _points_image_border.row(r);
    const auto& row_bottom = _points_image_border.row(r + _row_gap);
    for (int c = _col_gap; c < _points_image_border.cols - _col_gap; ++c) {
      const Vec3f& p = row_current.at<Vec3f>(c);
      if (p[2] == 0) { continue; }

      const cv::Vec3f& point_left = row_current.at<Vec3f>(c - _col_gap);
      if (point_left[2] == 0) { continue; }

      const cv::Vec3f& point_right = row_current.at<Vec3f>(c + _col_gap);
      if (point_right[2] == 0) { continue; }

      const cv::Vec3f& point_top = row_top.at<Vec3f>(c);
      if (point_top[2] == 0) { continue; }

      const cv::Vec3f& point_bottom = row_bottom.at<Vec3f>(c);
      if (point_bottom[2] == 0) { continue; }

      Vec3f dx = point_right - point_left;
      Vec3f dy = point_bottom - point_top;
      if (norm(dx, CV_L2) > squared_max_distance ||
          norm(dy, CV_L2) > squared_max_distance) {
        continue;
      }

      Vec3f n = normalize(dy.cross(dx));
      if (n.dot(p) > 0) { n = -n; }
      _normal_image_border.at<Vec3f>(r, c) = n;
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
void NormalComputer::initPointImage(const cv::Mat& points_image) {
  if (points_image.type() != CV_32FC3) {
    throw std::runtime_error(" NormalComputer: wrong point image type");
  }
  _rows = points_image.rows;
  _cols = points_image.cols;
  _points_image_border.create(points_image.rows + 2 * _row_gap,
                              points_image.cols + 2 * _col_gap, CV_32FC3);
  copyMakeBorder(points_image, _points_image_border, _row_gap, _row_gap,
                 _col_gap, _col_gap, cv::BORDER_REFLECT_101);
}

}  // namespace depth_clustering
