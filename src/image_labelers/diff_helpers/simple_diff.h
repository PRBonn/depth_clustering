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

#ifndef SRC_IMAGE_LABELERS_DIFF_HELPERS_SIMPLE_DIFF_H_
#define SRC_IMAGE_LABELERS_DIFF_HELPERS_SIMPLE_DIFF_H_

#include "image_labelers/diff_helpers/abstract_diff.h"

namespace depth_clustering {

/**
 * @brief      Class for simple difference. Just substract two values.
 */
class SimpleDiff : public AbstractDiff {
 public:
  explicit SimpleDiff(const cv::Mat* source_image)
      : AbstractDiff{source_image} {}

  /**
   * @brief      Overrides difference computation from parent
   */
  float DiffAt(const PixelCoord& from, const PixelCoord& to) const override {
    return fabs(_source_image->at<float>(from.row, from.col) -
                _source_image->at<float>(to.row, to.col));
  }
  /**
  * @brief      Checks if the value is below threshold
  */
  bool SatisfiesThreshold(float value, float threshold) const override {
    return value < threshold;
  }
};
}  // namespace depth_clustering

#endif  // SRC_IMAGE_LABELERS_DIFF_HELPERS_SIMPLE_DIFF_H_
