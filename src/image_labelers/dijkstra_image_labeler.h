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

#ifndef SRC_IMAGE_LABELERS_DIJKSTRA_IMAGE_LABELER_H_
#define SRC_IMAGE_LABELERS_DIJKSTRA_IMAGE_LABELER_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <string>
#include <queue>
#include <algorithm>

#include "image_labelers/abstract_image_labeler.h"
#include "projections/projection_params.h"

namespace depth_clustering {

/**
 * @brief      Label image with Dijkstra. Slower, then linear.
 */
template <int16_t STEP_ROW = 1, int16_t STEP_COL = 1>
class DijkstraImageLabeler : public AbstractImageLabeler {
 public:
  struct PixelCoord {
    PixelCoord() : row(0), col(0) {}
    PixelCoord(int16_t row_, int16_t col_) : row(row_), col(col_) {}

    int16_t row;
    int16_t col;
  };

  struct WeightedPixelCoord : public PixelCoord {
    WeightedPixelCoord() : PixelCoord(), keep_label(true) {}
    WeightedPixelCoord(int16_t row_, int16_t col_, bool keep_label_)
        : PixelCoord(row_, col_), keep_label(keep_label_) {}

    bool operator<(const WeightedPixelCoord& other) const {
      return keep_label < other.keep_label;
    }

    bool keep_label;
  };

  static constexpr int16_t NEIGH_SIZE = 2 * STEP_ROW + 2 * STEP_COL;
  std::array<PixelCoord, NEIGH_SIZE> Neighborhood;

  explicit DijkstraImageLabeler(const cv::Mat& depth_image,
                                const ProjectionParams& params,
                                const Radians& angle_threshold)
      : AbstractImageLabeler(depth_image, params, angle_threshold) {
    // this can probably be done in compile time
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

  virtual ~DijkstraImageLabeler() {}

  void ComputeLabels(DiffFactory::DiffType diff_type) override {
    _label_image =
        cv::Mat::zeros(_depth_image_ptr->size(), cv::DataType<uint16_t>::type);
    // initialize the label
    uint16_t label = 1;

    std::priority_queue<WeightedPixelCoord> labeling_queue;
    labeling_queue.push(WeightedPixelCoord());
    // while the queue is not empty continue removing front point adding its
    // neighbors back to the queue - breadth-first-search one component
    while (!labeling_queue.empty()) {
      // copy the current coordinate
      const WeightedPixelCoord current = labeling_queue.top();
      labeling_queue.pop();

      uint16_t& current_label =
          _label_image.at<uint16_t>(current.row, current.col);
      if (current_label > 0) {
        // we have already labeled this point. No need to add it.
        continue;
      }

      // this point hints it is time to change the label
      if (!current.keep_label) {
        label++;
      }

      // set the label of this point to current label
      current_label = label;
      auto current_depth =
          _depth_image_ptr->at<float>(current.row, current.col);
      if (current_depth < 0.001f) {
        // we have already labeled this point. No need to add it.
        continue;
      }
      for (const auto& step : Neighborhood) {
        int16_t neighbor_row = current.row + step.row;
        if (neighbor_row < 0 || neighbor_row >= _label_image.rows) {
          // point doesn't fit
          continue;
        }
        int16_t neighbor_col = current.col + step.col;
        if (neighbor_col < 0) {
          neighbor_col += _label_image.cols;
        }
        if (neighbor_col >= _label_image.cols) {
          neighbor_col -= _label_image.cols;
        }
        if (_label_image.at<uint16_t>(neighbor_row, neighbor_col) > 0) {
          continue;
        }
        float alpha;
        if (step.col == 0) {
          // means that there is a row step
          alpha = (_params.AngleFromRow(neighbor_row) -
                   _params.AngleFromRow(current.row)).val();
        } else {
          alpha = (_params.AngleFromCol(neighbor_col) -
                   _params.AngleFromCol(current.col)).val();
        }
        const float& neighbor_depth =
            _depth_image_ptr->at<float>(neighbor_row, neighbor_col);
        labeling_queue.push(WeightedPixelCoord(
            neighbor_row, neighbor_col,
            BelongToOneCluster(current_depth, neighbor_depth, alpha)));
      }
    }
  }
};

}  // namespace depth_clustering

#endif  // SRC_IMAGE_LABELERS_DIJKSTRA_IMAGE_LABELER_H_
