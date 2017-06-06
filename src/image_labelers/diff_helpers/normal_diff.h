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


#include <math.h>
#include <algorithm>
#include <vector>

#include "utils/normal_computer.h"
#include "image_labelers/diff_helpers/abstract_diff.h"

namespace depth_clustering {

/**
 * @brief      Class for normal-based difference computation.
 */
class NormalDiff : public AbstractDiff {
 public:
  /**
   * @brief      Initialize normals from a depth image.
   *
   * @param[in]  source_image  The source image
   * @param[in]  params        The projection parameters
   */
  NormalDiff(const cv::Mat* source_image, const ProjectionParams* params);

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
   * @brief      Threshold is satisfied if angle between the laser and the
   *             normal is SMALLER than threshold
   *
   * @param[in]  angle      The angle between the normal and the laser beam
   * @param[in]  threshold  The maximum angle in radians
   *
   * @return     true if threshold is satisfied
   */
  inline bool SatisfiesThreshold(float angle, float threshold) const override {
    return angle > threshold;
  }

  /**
   * @brief      Visualize normals as a `cv::Mat` color image.
   *
   * @return     `cv::Mat` color float image where each component of the normals
   *             is stored in a single channel of `cv::Mat` image pixel
   */
  cv::Mat Visualize() const override { return _normals; }

 private:
  const ProjectionParams* _params = nullptr;
  std::vector<float> _row_alphas;
  std::vector<float> _col_alphas;

  cv::Mat _normals;

  // TODO(igor): fix this ugly initialization here
  NormalComputer _normal_computer{5, 5, 5, 5, 5.0f};
};

}  // namespace depth_clustering

