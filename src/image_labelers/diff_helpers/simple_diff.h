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
