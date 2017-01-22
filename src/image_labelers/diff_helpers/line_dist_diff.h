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

#ifndef SRC_IMAGE_LABELERS_DIFF_HELPERS_LINE_DIST_DIFF_H_
#define SRC_IMAGE_LABELERS_DIFF_HELPERS_LINE_DIST_DIFF_H_

#include <math.h>
#include <algorithm>
#include <vector>

#include "image_labelers/diff_helpers/abstract_diff.h"

namespace depth_clustering {

/**
 * @brief      Class for line-based difference. It is very alike to AngleDiff
 *             class, just that after we have computed the angle, we compute
 *             \f$d_1 sin(\beta)\f$ to get the distance to the line spawned
 *             with two measurements.
 */
class LineDistDiff : public AbstractDiff {
 public:
  /**
   * @brief      Precompute the line distances to avoid losing time on that.
   *
   * @param[in]  source_image  The source image
   * @param[in]  params        The projection parameters
   */
  LineDistDiff(const cv::Mat* source_image, const ProjectionParams* params);

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
class LineDistDiffPrecomputed : public AbstractDiff {
 public:
  /**
   * @brief      Precompute the angles to avoid losing time on that.
   *
   * @param[in]  source_image  The source image
   * @param[in]  params        The projection parameters
   */
  LineDistDiffPrecomputed(const cv::Mat* source_image,
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
  void PreComputeLineDists();

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
  float GetLineDist(float alpha, float current_depth,
                    float neighbor_depth) const;

  const ProjectionParams* _params = nullptr;
  std::vector<float> _row_alphas;
  std::vector<float> _col_alphas;
  cv::Mat _dists_row;
  cv::Mat _dists_col;
};

}  // namespace depth_clustering

#endif  // SRC_IMAGE_LABELERS_DIFF_HELPERS_LINE_DIST_DIFF_H_
