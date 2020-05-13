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

#ifndef SRC_IMAGE_LABELERS_LINEAR_IMAGE_LABELER_H_
#define SRC_IMAGE_LABELERS_LINEAR_IMAGE_LABELER_H_

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <queue>

#include "image_labelers/abstract_image_labeler.h"
#include "image_labelers/pixel_coords.h"

namespace depth_clustering {

/**
 * @brief      Class for linear image labeler.
 *
 * @tparam     STEP_ROW  step we do over image over rows
 * @tparam     STEP_COL  step we do over image over cols
 */
template <int16_t STEP_ROW = 1, int16_t STEP_COL = 1>
class LinearImageLabeler : public AbstractImageLabeler {
 public:
  static constexpr int16_t NEIGH_SIZE = 2 * STEP_ROW + 2 * STEP_COL;
  std::array<PixelCoord, NEIGH_SIZE> Neighborhood;

  /**
   * @brief      Initialize Linear image labeler
   *
   * @param[in]  depth_image      The depth image
   * @param[in]  params           The projection parameters
   * @param[in]  angle_threshold  The angle threshold to seaparate clusters
   */
  explicit LinearImageLabeler(const cv::Mat& depth_image,
                              const ProjectionParams& params,
                              const Radians& angle_threshold)
      : AbstractImageLabeler(depth_image, params, angle_threshold) {
    // this can probably be done at compile time
    int16_t counter = 0;
    for (int16_t r = STEP_ROW; r > 0; --r) {
      Neighborhood[counter++] = PixelCoord(-r, 0);
      Neighborhood[counter++] = PixelCoord(r, 0);
    }
    for (int16_t c = STEP_COL; c > 0; --c) {
      Neighborhood[counter++] = PixelCoord(0, -c);
      Neighborhood[counter++] = PixelCoord(0, c);
    }
  }
  virtual ~LinearImageLabeler() {}

  /**
   * @brief      Label a single connected component with BFS. Can be done faster
   *             if we augment the queue with a hash or smth.
   *
   * @param[in]  label        Label this component with this label
   * @param[in]  start        Start pixel
   * @param[in]  diff_helper  The difference helper
   */
  void LabelOneComponent(uint16_t label, const PixelCoord& start,
                         const AbstractDiff* diff_helper) {
    // breadth first search
    std::queue<PixelCoord> labeling_queue;
    labeling_queue.push(start);
    // while the queue is not empty continue removing front point adding its
    // neighbors back to the queue - breadth-first-search one component
    size_t max_queue_size = 0;
    while (!labeling_queue.empty()) {
      max_queue_size = std::max(labeling_queue.size(), max_queue_size);
      // copy the current coordinate
      const PixelCoord current = labeling_queue.front();
      labeling_queue.pop();
      uint16_t current_label = LabelAt(current);
      if (current_label > 0) {
        // we have already labeled this point. No need to add it.
        continue;
      }
      // set the label of this point to current label
      SetLabel(current, label);

      // check the depth
      auto current_depth = DepthAt(current);
      if (current_depth < 0.001f) {
        // depth of this point is wrong, so don't bother adding it to queue
        continue;
      }
      for (const auto& step : Neighborhood) {
        PixelCoord neighbor = current + step;
        if (neighbor.row < 0 || neighbor.row >= _label_image.rows) {
          // point doesn't fit
          continue;
        }
        // if we just went over the borders in horiz direction - wrap around
        neighbor.col = WrapCols(neighbor.col);
        uint16_t neigh_label = LabelAt(neighbor);
        if (neigh_label > 0) {
          // we have already labeled this one
          continue;
        }
        auto diff = diff_helper->DiffAt(current, neighbor);
        if (diff_helper->SatisfiesThreshold(diff, _radians_threshold)) {
          labeling_queue.push(neighbor);
        }
      }
    }
  }

  /**
   * @brief      Gets depth value at pixel
   *
   * @param[in]  coord  Pixel coordinate
   *
   * @return     depth at pixel
   */
  inline float DepthAt(const PixelCoord& coord) const {
    return _depth_image_ptr->at<float>(coord.row, coord.col);
  }

  /**
   * @brief      Gets label of a given pixel
   *
   * @param[in]  coord  Pixel coordinate
   *
   * @return     label for pixel
   */
  inline uint16_t LabelAt(const PixelCoord& coord) const {
    return _label_image.at<uint16_t>(coord.row, coord.col);
  }

  /**
   * @brief      Sets label of a given pixel
   *
   * @param[in]  coord  Pixel coordinate
   */
  inline void SetLabel(const PixelCoord& coord, uint16_t label) {
    _label_image.at<uint16_t>(coord.row, coord.col) = label;
  }

  /**
   * @brief      Wrap columns around image
   */
  int16_t WrapCols(int16_t col) const {
    // we allow our space to fold around cols
    if (col < 0) {
      return col + _label_image.cols;
    }
    if (col >= _label_image.cols) {
      return col - _label_image.cols;
    }
    return col;
  }

  /**
   * @brief      Calculates the labels running over the whole image.
   */
  void ComputeLabels(DiffFactory::DiffType diff_type) override {
    _label_image =
        cv::Mat::zeros(_depth_image_ptr->size(), cv::DataType<uint16_t>::type);
    auto diff_helper_ptr =
        DiffFactory::Build(diff_type, _depth_image_ptr, &_params);
    // initialize the label
    uint16_t label = 1;

    for (int row = 0; row < _label_image.rows; ++row) {
      for (int col = 0; col < _label_image.cols; ++col) {
        if (_label_image.at<uint16_t>(row, col) > 0) {
          // we have already labeled this point
          continue;
        }
        if (_depth_image_ptr->at<float>(row, col) < 0.001f) {
          // depth is zero, not interested
          continue;
        }
        LabelOneComponent(label, PixelCoord(row, col), diff_helper_ptr.get());
        // we have finished labeling this connected component. We now need to
        // label the next one, so we increment the label
        label++;
      }
    }
  }
};

}  // namespace depth_clustering

#endif  // SRC_IMAGE_LABELERS_LINEAR_IMAGE_LABELER_H_
