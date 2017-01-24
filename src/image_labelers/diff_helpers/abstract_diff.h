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

#ifndef SRC_IMAGE_LABELERS_DIFF_HELPERS_ABSTRACT_DIFF_H_
#define SRC_IMAGE_LABELERS_DIFF_HELPERS_ABSTRACT_DIFF_H_

#include <opencv2/opencv.hpp>

#include "image_labelers/pixel_coords.h"
#include "projections/projection_params.h"

namespace depth_clustering {

/**
 * @brief      Class for abstract difference.
 */
class AbstractDiff {
 public:
  /**
   * @brief      construct a class with a source image pointer
   *
   * @param[in]  source_image  The source image pointer (CV_32F type Mat)
   */
  explicit AbstractDiff(const cv::Mat* source_image)
      : _source_image{source_image} {}
  virtual ~AbstractDiff() {}

  /**
   * @brief      Gets diff between pixels from and to
   *
   * @param[in]  from  pixel from which we want difference
   * @param[in]  to    pixel to which we want difference
   *
   * @return     difference between the pixels
   */
  virtual float DiffAt(const PixelCoord& from, const PixelCoord& to) const = 0;

  /**
   * @brief      Does the difference satisfy a threshold?
   *
   * @param[in]  value      Query value
   * @param[in]  threshold  The threshold
   *
   * @return     true if satisfies, false, otherwise
   */
  virtual bool SatisfiesThreshold(float value, float threshold) const = 0;

  /**
   * @brief      Visualize the differences as a color image.
   *
   * @return     OpenCV Mat of type CV_8UC3 to visualize differences in color.
   */
  virtual cv::Mat Visualize() const { return cv::Mat(); }

 protected:
  const cv::Mat* _source_image = nullptr;
};
}  // namespace depth_clustering

#endif  // SRC_IMAGE_LABELERS_DIFF_HELPERS_ABSTRACT_DIFF_H_
