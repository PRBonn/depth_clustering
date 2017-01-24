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

#include <math.h>
#include <algorithm>
#include <vector>

#include "image_labelers/diff_helpers/line_dist_diff.h"

namespace depth_clustering {

LineDistDiff::LineDistDiff(const cv::Mat* source_image,
                           const ProjectionParams* params)
    : AbstractDiff{source_image}, _params{params} {
  PreComputeAlphaVecs();
}

float LineDistDiff::DiffAt(const PixelCoord& from, const PixelCoord& to) const {
  const float& current_depth = _source_image->at<float>(from.row, from.col);
  const float& neighbor_depth = _source_image->at<float>(to.row, to.col);
  float alpha = ComputeAlpha(from, to);
  if (alpha > _params->h_span().val() - 0.05) {
    // we are over the border
    const float span = _params->h_span().val();
    if (alpha > span) {
      alpha -= span;
    } else {
      alpha = span - alpha;
    }
  }
  float d1 = std::max(current_depth, neighbor_depth);
  float d2 = std::min(current_depth, neighbor_depth);
  float beta = fabs(std::atan2((d2 * sin(alpha)), (d1 - d2 * cos(alpha))));
  return d1 * sin(beta);
}

void LineDistDiff::PreComputeAlphaVecs() {
  _row_alphas.reserve(_params->rows());
  for (size_t r = 0; r < _params->rows() - 1; ++r) {
    _row_alphas.push_back(
        fabs((_params->AngleFromRow(r + 1) - _params->AngleFromRow(r)).val()));
  }
  // add last row alpha
  _row_alphas.push_back(0.0f);
  // now handle the cols
  _col_alphas.reserve(_params->cols());
  for (size_t c = 0; c < _params->cols() - 1; ++c) {
    _col_alphas.push_back(
        fabs((_params->AngleFromCol(c + 1) - _params->AngleFromCol(c)).val()));
  }
  // handle last angle where we wrap columns
  float last_alpha = fabs(
      (_params->AngleFromCol(0) - _params->AngleFromCol(_params->cols() - 1))
          .val());
  last_alpha -= _params->h_span().val();
  _col_alphas.push_back(last_alpha);
}

float LineDistDiff::ComputeAlpha(const PixelCoord& current,
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

LineDistDiffPrecomputed::LineDistDiffPrecomputed(const cv::Mat* source_image,
                                                 const ProjectionParams* params)
    : AbstractDiff{source_image}, _params{params} {
  PreComputeAlphaVecs();
  PreComputeLineDists();
}

float LineDistDiffPrecomputed::DiffAt(const PixelCoord& from,
                                      const PixelCoord& to) const {
  int16_t row = 0;
  int16_t col = 0;
  /// If one of rows is biggest possible - use it. Otherwise use
  /// `std::min(from.row, to.row)`.
  int16_t last_row = static_cast<int16_t>(_params->rows()) - 1;
  bool row_crosses_border = (from.row == last_row && to.row == 0) ||
                            (from.row == 0 && to.row == last_row);
  if (row_crosses_border) {
    row = last_row;
  } else {
    row = std::min(from.row, to.row);
  }
  /// If one of cols is biggest possible - use it. Otherwise use
  /// `std::min(from.col, to.col)`.
  int16_t last_col = static_cast<int16_t>(_params->cols()) - 1;
  bool col_crosses_border = (from.col == last_col && to.col == 0) ||
                            (from.col == 0 && to.col == last_col);
  if (col_crosses_border) {
    col = last_col;
  } else {
    col = std::min(from.col, to.col);
  }
  // If row changed -> pick row-wise beta matrix
  if (from.row != to.row) {
    return _dists_row.at<float>(row, col);
  }
  // If col changed -> pick col-wise beta matrix
  if (from.col != to.col) {
    return _dists_col.at<float>(row, col);
  }
  throw std::runtime_error("Asking for difference of same pixels.");
}

cv::Mat LineDistDiffPrecomputed::Visualize() const {
  cv::Mat colors = cv::Mat::zeros(_dists_row.rows, _dists_row.cols, CV_8UC3);

  float max_dist = 20.0f;
  for (int r = 0; r < _dists_row.rows; ++r) {
    for (int c = 0; c < _dists_row.cols; ++c) {
      if (_source_image->at<float>(r, c) < 0.01f) {
        continue;
      }
      uint8_t row_color = 255 * (_dists_row.at<float>(r, c) / max_dist);
      uint8_t col_color = 255 * (_dists_col.at<float>(r, c) / max_dist);
      cv::Vec3b color(255 - row_color, 255 - col_color, 0);
      colors.at<cv::Vec3b>(r, c) = color;
    }
  }
  return colors;
}

void LineDistDiffPrecomputed::PreComputeAlphaVecs() {
  _row_alphas.reserve(_params->rows());
  for (size_t r = 0; r < _params->rows() - 1; ++r) {
    _row_alphas.push_back(
        fabs((_params->AngleFromRow(r + 1) - _params->AngleFromRow(r)).val()));
  }
  // add last row alpha
  _row_alphas.push_back(0.0f);
  // now handle the cols
  _col_alphas.reserve(_params->cols());
  for (size_t c = 0; c < _params->cols() - 1; ++c) {
    _col_alphas.push_back(
        fabs((_params->AngleFromCol(c + 1) - _params->AngleFromCol(c)).val()));
  }
  // handle last angle where we wrap columns
  float last_alpha = fabs(
      (_params->AngleFromCol(0) - _params->AngleFromCol(_params->cols() - 1))
          .val());
  last_alpha -= _params->h_span().val();
  _col_alphas.push_back(fabs(last_alpha));
}

void LineDistDiffPrecomputed::PreComputeLineDists() {
  _dists_row = cv::Mat::zeros(_params->rows(), _params->cols(), CV_32F);
  _dists_col = cv::Mat::zeros(_params->rows(), _params->cols(), CV_32F);
  for (size_t r = 0; r < _params->rows(); ++r) {
    float angle_rows = _row_alphas[r];
    for (size_t c = 0; c < _params->cols(); ++c) {
      if (_source_image->at<float>(r, c) < 0.001) {
        continue;
      }
      float angle_cols = _col_alphas[c];
      float curr = _source_image->at<float>(r, c);
      // Compute beta in horizontal (col) direction. Note wrapping around.
      size_t next_c = (c + 1) % _params->cols();
      _dists_col.at<float>(r, c) =
          GetLineDist(angle_cols, curr, _source_image->at<float>(r, next_c));
      // Compute beta in vertical (row) direction
      // Protect the last row. There are 0s stores there.
      size_t next_r = r + 1;
      if (next_r >= _params->rows()) {
        continue;
      }
      _dists_row.at<float>(r, c) =
          GetLineDist(angle_rows, curr, _source_image->at<float>(next_r, c));
    }
  }
}

float LineDistDiffPrecomputed::GetLineDist(float alpha, float current_depth,
                                           float neighbor_depth) const {
  float d1 = std::max(current_depth, neighbor_depth);
  float d2 = std::min(current_depth, neighbor_depth);
  float beta = fabs(std::atan2((d2 * sin(alpha)), (d1 - d2 * cos(alpha))));
  return d1 * sin(beta);
}
}  // namespace depth_clustering
