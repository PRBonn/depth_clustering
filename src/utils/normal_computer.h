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

#pragma once

#include <opencv2/opencv.hpp>

namespace depth_clustering {
/**
 * @brief      Class for computing normals. It extends the point image by
 *             reflecting its border and computes normals. It then blurs them
 *             and gives back only the relevant part of an image.
 */
class NormalComputer {
 public:
  NormalComputer(int col_gap, int row_gap, int blur_size, float max_distance);
  void initPointImage(const cv::Mat& points_image);

  void compute();

  inline const cv::Mat& normalImage() const { return _normal_image; }

 protected:
  void computeNormals();
  void blurNormals(int window, int start);
  cv::Mat _points_image_border;
  cv::Mat _normal_image_border;
  cv::Mat _normal_image_smooth;
  cv::Mat _normal_image;
  int _col_gap = 0;
  int _row_gap = 0;
  int _blur_size = 0;
  int _rows = 0;
  int _cols = 0;
  float _max_distance = 0.0f;
};

}  // namespace depth_clustering
