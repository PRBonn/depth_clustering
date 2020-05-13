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
