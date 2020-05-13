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

#ifndef SRC_IMAGE_LABELERS_DIFF_HELPERS_ANGLE_DIFF_H_
#define SRC_IMAGE_LABELERS_DIFF_HELPERS_ANGLE_DIFF_H_

#include <math.h>
#include <algorithm>
#include <vector>

#include "image_labelers/diff_helpers/abstract_diff.h"

namespace depth_clustering {

/**
 * @brief      Class for angle difference.
 */
class AngleDiff : public AbstractDiff {
 public:
  /**
   * @brief      Precompute the angles to avoid losing time on that.
   *
   * @param[in]  source_image  The source image
   * @param[in]  params        The projection parameters
   */
  AngleDiff(const cv::Mat* source_image, const ProjectionParams* params);

  /**
   * @brief      Compute angle-based difference. See paper for details.
   *
   * @param[in]  from  Pixel from which to compute difference
   * @param[in]  to    Pixel to which to compute difference
   *
   * @return     Angle difference between the values
   */
  float DiffAt(const PixelCoord& from, const PixelCoord& to) const override;

  /**
   * @brief      Threshold is satisfied if angle is BIGGER than threshold
   */
  inline bool SatisfiesThreshold(float angle, float threshold) const override {
    return angle > threshold;
  }

  /**
   * @brief      Visualize \f$\beta\f$ angles as a `cv::Mat` color image.
   *
   * @return     `cv::Mat` color image with red channel showing \f$\beta\f$
   *              angles in row direction and green channel in col direction.
   */
  cv::Mat Visualize() const override { return cv::Mat(); }

 private:
  /**
   * @brief      Pre-compute values for angles for all cols and rows
   */
  void PreComputeAlphaVecs();

  /**
   * @brief      Calculates the alpha.
   *
   * @param[in]  current   The current
   * @param[in]  neighbor  The neighbor
   *
   * @return     The alpha.
   */
  float ComputeAlpha(const PixelCoord& current,
                     const PixelCoord& neighbor) const;

  const ProjectionParams* _params = nullptr;
  std::vector<float> _row_alphas;
  std::vector<float> _col_alphas;
};

/**
 * @brief      Class for angle difference.
 */
class AngleDiffPrecomputed : public AbstractDiff {
 public:
  /**
   * @brief      Precompute the angles to avoid losing time on that.
   *
   * @param[in]  source_image  The source image
   * @param[in]  params        The projection parameters
   */
  AngleDiffPrecomputed(const cv::Mat* source_image,
                       const ProjectionParams* params);

  /**
   * @brief      Compute angle-based difference. See paper for details. Only one
   *             of the following situations is possible:
   *             - from.row > to.row
   *             - from.row < to.row
   *             - from.col > to.col
   *             - from.col < to.col
   *
   * @param[in]  from  Pixel from which to compute difference
   * @param[in]  to    Pixel to which to compute difference
   *
   * @return     Angle difference between the values
   */
  float DiffAt(const PixelCoord& from, const PixelCoord& to) const override;
  /**
   * @brief      Threshold is satisfied if angle is BIGGER than threshold
   */

  inline bool SatisfiesThreshold(float angle, float threshold) const override {
    return angle > threshold;
  }

  /**
   * @brief      Visualize \f$\beta\f$ angles as a `cv::Mat` color image.
   *
   * @return     `cv::Mat` color image with red channel showing \f$\beta\f$
   *              angles in row direction and green channel in col direction.
   */
  cv::Mat Visualize() const;

 protected:
  /**
   * @brief      Pre-compute values for angles for all cols and rows
   */
  void PreComputeAlphaVecs();
  /**
   * @brief      Precompute all \f$\beta\f$ angles for the image. It generates
   *             two matrices for row-wise and col-wise angles. See picture for
   *             illustration. The squares store values of angles between pixels
   *             with computation direction shown by arrows. Note that the
   *             columns matrix wraps around, i.e. the last element stores the
   *             difference in angles between the last pixel of the original
   *             image and the first one.
   * @image html precompute_betas.png
   */
  void PreComputeBetaAngles();

  /**
   * @brief      Compute the angle \f$\beta\f$ of incline of the line spawned
   *             by two given beams.
   *
   * @param[in]  alpha           The angle \f$\alpha\f$ between the beams.
   * @param[in]  current_depth   The reading of the first beam in meters.
   * @param[in]  neighbor_depth  The reading of the second beam in meters.
   *
   * @return     The angle of incidence of the line spawned by endpoints of
   *             two given beams.
   */
  float GetBeta(float alpha, float current_depth, float neighbor_depth) const;

  const ProjectionParams* _params = nullptr;
  std::vector<float> _row_alphas;
  std::vector<float> _col_alphas;
  cv::Mat _beta_rows;
  cv::Mat _beta_cols;
};

}  // namespace depth_clustering

#endif  // SRC_IMAGE_LABELERS_DIFF_HELPERS_ANGLE_DIFF_H_
