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

#ifndef SRC_IMAGE_LABELERS_DIFF_HELPER_H_
#define SRC_IMAGE_LABELERS_DIFF_HELPER_H_

#include <opencv2/opencv.hpp>

#include <math.h>
#include <algorithm>
#include <vector>

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
  virtual double DiffAt(const PixelCoord& from, const PixelCoord& to) const = 0;

  /**
   * @brief      Does the difference satisfy a threshold?
   *
   * @param[in]  value      Query value
   * @param[in]  threshold  The threshold
   *
   * @return     true if satisfies, false, otherwise
   */
  virtual bool SatisfiesThreshold(float value, float threshold) const = 0;

 protected:
  const cv::Mat* _source_image = nullptr;
};

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
  double DiffAt(const PixelCoord& from, const PixelCoord& to) const override {
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
  AngleDiff(const cv::Mat* source_image, const ProjectionParams* params)
      : AbstractDiff{source_image}, _params{params} {
    PreComputeAlphaVecs();
  }

  /**
   * @brief      Compute angle-based difference. See paper for details.
   *
   * @param[in]  from  Pixel from which to compute difference
   * @param[in]  to    Pixel to which to compute difference
   *
   * @return     Angle difference between the values
   */
  double DiffAt(const PixelCoord& from, const PixelCoord& to) const override {
    const float& current_depth = _source_image->at<float>(from.row, from.col);
    const float& neighbor_depth = _source_image->at<float>(to.row, to.col);
    double alpha = ComputeAlpha(from, to);
    if (alpha > _params->h_span().val() - 0.05) {
      // we are over the border
      const double span = _params->h_span().val();
      if (alpha > span) {
        alpha -= span;
      } else {
        alpha = span - alpha;
      }
    }
    double d1 = std::max(current_depth, neighbor_depth);
    double d2 = std::min(current_depth, neighbor_depth);
    double beta = std::atan2((d2 * sin(alpha)), (d1 - d2 * cos(alpha)));
    return fabs(beta);
  }

  /**
   * @brief      Threshold is satisfied if angle is BIGGER than threshold
   */
  bool SatisfiesThreshold(float angle, float threshold) const override {
    return angle > threshold;
  }

 private:
  /**
   * @brief      Pre-compute values for angles for all cols and rows
   */
  void PreComputeAlphaVecs() {
    _row_alphas.reserve(_params->rows());
    for (size_t r = 0; r < _params->rows() - 1; ++r) {
      _row_alphas.push_back(fabs(
          (_params->AngleFromRow(r + 1) - _params->AngleFromRow(r)).val()));
    }
    // add last row alpha
    _row_alphas.push_back(0.0f);
    // now handle the cols
    _col_alphas.reserve(_params->cols());
    for (size_t c = 0; c < _params->cols() - 1; ++c) {
      _col_alphas.push_back(fabs(
          (_params->AngleFromCol(c + 1) - _params->AngleFromCol(c)).val()));
    }
    // handle last angle where we wrap columns
    float last_alpha = fabs((_params->AngleFromCol(0) -
                             _params->AngleFromCol(_params->cols() - 1)).val());
    last_alpha -= _params->h_span().val();
    _col_alphas.push_back(last_alpha);
  }

  /**
   * @brief      Calculates the alpha.
   *
   * @param[in]  current   The current
   * @param[in]  neighbor  The neighbor
   *
   * @return     The alpha.
   */
  double ComputeAlpha(const PixelCoord& current,
                      const PixelCoord& neighbor) const {
    if ((current.col == 0 &&
         neighbor.col == static_cast<int>(_params->cols() - 1)) ||
        (neighbor.col == 0 &&
         current.col == static_cast<int>(_params->cols() - 1))) {
      // this means we wrap around
      return _col_alphas.back();
    }
    if (current.row < neighbor.row) {
      return _row_alphas[current.row];
    } else if (current.row > neighbor.row) {
      return _row_alphas[neighbor.row];
    } else if (current.col < neighbor.col) {
      return _col_alphas[current.col];
    } else if (current.col > neighbor.col) {
      return _col_alphas[neighbor.col];
    }
    return 0;
  }

  const ProjectionParams* _params = nullptr;
  std::vector<float> _row_alphas;
  std::vector<float> _col_alphas;
};
}  // namespace depth_clustering

#endif  // SRC_IMAGE_LABELERS_DIFF_HELPER_H_
