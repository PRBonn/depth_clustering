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

#include "./depth_ground_remover.h"

#include <opencv2/highgui/highgui.hpp>

#include <algorithm>

#include "utils/velodyne_utils.h"
#include "image_labelers/linear_image_labeler.h"
#include "image_labelers/diff_helpers/angle_diff.h"
#include "image_labelers/diff_helpers/simple_diff.h"
#include "utils/timer.h"

namespace depth_clustering {

using cv::Mat;
using cv::DataType;
using std::to_string;
using time_utils::Timer;

const cv::Point ANCHOR_CENTER = cv::Point(-1, -1);
const int SAME_OUTPUT_TYPE = -1;

void DepthGroundRemover::OnNewObjectReceived(const Cloud& cloud,
                                             const int sender_id) {
  // this can be done even faster if we switch to column-major implementation
  // thus allowing us to load whole row in L1 cache
  if (!cloud.projection_ptr()) {
    fprintf(stderr, "No projection in cloud. Skipping ground removal.\n");
    return;
  }
  Cloud cloud_copy(cloud);
  const cv::Mat& depth_image =
      RepairDepth(cloud.projection_ptr()->depth_image(), 5, 1.0f);
  Timer total_timer;
  auto angle_image = CreateAngleImage(depth_image);
  auto smoothed_image = ApplySavitskyGolaySmoothing(angle_image, _window_size);
  auto no_ground_image = ZeroOutGroundBFS(depth_image, smoothed_image,
                                          _ground_remove_angle, _window_size);
  fprintf(stderr, "INFO: Ground removed in %lu us\n", total_timer.measure());
  cloud_copy.projection_ptr()->depth_image() = no_ground_image;
  this->ShareDataWithAllClients(cloud_copy);
  _counter++;
}

Mat DepthGroundRemover::ZeroOutGround(const cv::Mat& image,
                                      const cv::Mat& angle_image,
                                      const Radians& threshold) const {
  // TODO(igor): test if its enough to remove only values starting from the
  // botom pixel. I don't like removing all values based on a threshold.
  // But that's a start, so let's stick with it for now.
  Mat res = cv::Mat::zeros(image.size(), CV_32F);
  for (int r = 0; r < image.rows; ++r) {
    for (int c = 0; c < image.cols; ++c) {
      if (angle_image.at<float>(r, c) > threshold.val()) {
        res.at<float>(r, c) = image.at<float>(r, c);
      }
    }
  }
  return res;
}

Mat DepthGroundRemover::ZeroOutGroundBFS(const cv::Mat& image,
                                         const cv::Mat& angle_image,
                                         const Radians& threshold,
                                         int kernel_size) const {
  Mat res = cv::Mat::zeros(image.size(), CV_32F);
  LinearImageLabeler<> image_labeler(image, _params, threshold);
  SimpleDiff simple_diff_helper(&angle_image);
  Radians start_thresh = 30_deg;
  for (int c = 0; c < image.cols; ++c) {
    // start at bottom pixels and do bfs
    int r = image.rows - 1;
    while (r > 0 && image.at<float>(r, c) < 0.001f) {
      --r;
    }
    auto current_coord = PixelCoord(r, c);
    uint16_t current_label = image_labeler.LabelAt(current_coord);
    if (current_label > 0) {
      // this coord was already labeled, skip
      continue;
    }
    // TODO(igor): this is a test. Maybe switch it on, maybe off.
    if (angle_image.at<float>(r, c) > start_thresh.val()) {
      continue;
    }
    image_labeler.LabelOneComponent(1, current_coord, &simple_diff_helper);
  }
  auto label_image_ptr = image_labeler.GetLabelImage();
  if (label_image_ptr->rows != res.rows || label_image_ptr->cols != res.cols) {
    fprintf(stderr, "ERROR: label image and res do not correspond.\n");
    return res;
  }
  kernel_size = std::max(kernel_size - 2, 3);
  Mat kernel = GetUniformKernel(kernel_size, CV_8U);
  Mat dilated = Mat::zeros(label_image_ptr->size(), label_image_ptr->type());
  cv::dilate(*label_image_ptr, dilated, kernel);
  for (int r = 0; r < dilated.rows; ++r) {
    for (int c = 0; c < dilated.cols; ++c) {
      if (dilated.at<uint16_t>(r, c) == 0) {
        // all unlabeled points are non-ground
        res.at<float>(r, c) = image.at<float>(r, c);
      }
    }
  }
  return res;
}

Mat DepthGroundRemover::RepairDepth(const Mat& no_ground_image, int step,
                                    float depth_threshold) {
  Mat inpainted_depth = no_ground_image.clone();
  for (int c = 0; c < inpainted_depth.cols; ++c) {
    for (int r = 0; r < inpainted_depth.rows; ++r) {
      float& curr_depth = inpainted_depth.at<float>(r, c);
      if (curr_depth < 0.001f) {
        int counter = 0;
        float sum = 0.0f;
        for (int i = 1; i < step; ++i) {
          if (r - i < 0) {
            continue;
          }
          for (int j = 1; j < step; ++j) {
            if (r + j > inpainted_depth.rows - 1) {
              continue;
            }
            const float& prev = inpainted_depth.at<float>(r - i, c);
            const float& next = inpainted_depth.at<float>(r + j, c);
            if (prev > 0.001f && next > 0.001f &&
                fabs(prev - next) < depth_threshold) {
              sum += prev + next;
              counter += 2;
            }
          }
        }
        if (counter > 0) {
          curr_depth = sum / counter;
        }
      }
    }
  }
  return inpainted_depth;
}

Mat DepthGroundRemover::RepairDepth(const Mat& depth_image) {
  Mat kernel = GetUniformKernel(5);
  Mat inpainted_depth;  // init an empty smoothed image
  cv::filter2D(depth_image, inpainted_depth, SAME_OUTPUT_TYPE, kernel,
               ANCHOR_CENTER, 0, cv::BORDER_REFLECT101);
  Mat mask = depth_image > 0;
  depth_image.copyTo(inpainted_depth, mask);
  return inpainted_depth;
}

Mat DepthGroundRemover::CreateAngleImage(const Mat& depth_image) {
  Mat angle_image = Mat::zeros(depth_image.size(), DataType<float>::type);
  Mat x_mat = Mat::zeros(depth_image.size(), DataType<float>::type);
  Mat y_mat = Mat::zeros(depth_image.size(), DataType<float>::type);
  const auto& sines_vec = _params.RowAngleSines();
  const auto& cosines_vec = _params.RowAngleCosines();
  float dx, dy;
  x_mat.row(0) = depth_image.row(0) * cosines_vec[0];
  y_mat.row(0) = depth_image.row(0) * sines_vec[0];
  for (int r = 1; r < angle_image.rows; ++r) {
    x_mat.row(r) = depth_image.row(r) * cosines_vec[r];
    y_mat.row(r) = depth_image.row(r) * sines_vec[r];
    for (int c = 0; c < angle_image.cols; ++c) {
      dx = fabs(x_mat.at<float>(r, c) - x_mat.at<float>(r - 1, c));
      dy = fabs(y_mat.at<float>(r, c) - y_mat.at<float>(r - 1, c));
      angle_image.at<float>(r, c) = atan2(dy, dx);
    }
  }
  return angle_image;
}

Mat DepthGroundRemover::GetSavitskyGolayKernel(int window_size) const {
  if (window_size % 2 == 0) {
    throw std::logic_error("only odd window size allowed");
  }
  bool window_size_ok = window_size == 5 || window_size == 7 ||
                        window_size == 9 || window_size == 11;
  if (!window_size_ok) {
    throw std::logic_error("bad window size");
  }
  // below are no magic constants. See Savitsky-golay filter.
  Mat kernel;
  switch (window_size) {
    case 5:
      kernel = Mat::zeros(window_size, 1, CV_32F);
      kernel.at<float>(0, 0) = -3.0f;
      kernel.at<float>(0, 1) = 12.0f;
      kernel.at<float>(0, 2) = 17.0f;
      kernel.at<float>(0, 3) = 12.0f;
      kernel.at<float>(0, 4) = -3.0f;
      kernel /= 35.0f;
      return kernel;
    case 7:
      kernel = Mat::zeros(window_size, 1, CV_32F);
      kernel.at<float>(0, 0) = -2.0f;
      kernel.at<float>(0, 1) = 3.0f;
      kernel.at<float>(0, 2) = 6.0f;
      kernel.at<float>(0, 3) = 7.0f;
      kernel.at<float>(0, 4) = 6.0f;
      kernel.at<float>(0, 5) = 3.0f;
      kernel.at<float>(0, 6) = -2.0f;
      kernel /= 21.0f;
      return kernel;
    case 9:
      kernel = Mat::zeros(window_size, 1, CV_32F);
      kernel.at<float>(0, 0) = -21.0f;
      kernel.at<float>(0, 1) = 14.0f;
      kernel.at<float>(0, 2) = 39.0f;
      kernel.at<float>(0, 3) = 54.0f;
      kernel.at<float>(0, 4) = 59.0f;
      kernel.at<float>(0, 5) = 54.0f;
      kernel.at<float>(0, 6) = 39.0f;
      kernel.at<float>(0, 7) = 14.0f;
      kernel.at<float>(0, 8) = -21.0f;
      kernel /= 231.0f;
      return kernel;
    case 11:
      kernel = Mat::zeros(window_size, 1, CV_32F);
      kernel.at<float>(0, 0) = -36.0f;
      kernel.at<float>(0, 1) = 9.0f;
      kernel.at<float>(0, 2) = 44.0f;
      kernel.at<float>(0, 3) = 69.0f;
      kernel.at<float>(0, 4) = 84.0f;
      kernel.at<float>(0, 5) = 89.0f;
      kernel.at<float>(0, 6) = 84.0f;
      kernel.at<float>(0, 7) = 69.0f;
      kernel.at<float>(0, 8) = 44.0f;
      kernel.at<float>(0, 9) = 9.0f;
      kernel.at<float>(0, 10) = -36.0f;
      kernel /= 429.0f;
      return kernel;
  }
  return kernel;
}

Mat DepthGroundRemover::GetUniformKernel(int window_size, int type) const {
  if (window_size % 2 == 0) {
    throw std::logic_error("only odd window size allowed");
  }
  Mat kernel = Mat::zeros(window_size, 1, type);
  kernel.at<float>(0, 0) = 1;
  kernel.at<float>(window_size - 1, 0) = 1;
  kernel /= 2;
  return kernel;
}

Mat DepthGroundRemover::ApplySavitskyGolaySmoothing(const Mat& image,
                                                    int window_size) {
  Mat kernel = GetSavitskyGolayKernel(window_size);

  Mat smoothed_image;  // init an empty smoothed image
  cv::filter2D(image, smoothed_image, SAME_OUTPUT_TYPE, kernel, ANCHOR_CENTER,
               0, cv::BORDER_REFLECT101);
  return smoothed_image;
}

Radians DepthGroundRemover::GetLineAngle(const Mat& depth_image, int col,
                                         int row_curr, int row_neigh) {
  // compute inclination angle of the line given the depth of two pixels and
  // their position in the image. We use config to determine the needed angles
  // All following angles are in degrees
  Radians current_angle;
  Radians neighbor_angle;
  current_angle = _params.AngleFromRow(row_curr);
  neighbor_angle = _params.AngleFromRow(row_neigh);
  // for easiness copy references to depth of current and neighbor positions
  const float& depth_current = depth_image.at<float>(row_curr, col);
  const float& depth_neighbor = depth_image.at<float>(row_neigh, col);
  if (depth_current < _eps || depth_neighbor < _eps) {
    // if either of these depth vales is close to zero this depth is not
    // reliable, so we will just report a 0 instead.
    return 0_deg;
  }
  auto x_current = depth_current * cos(current_angle.val());
  auto y_current = depth_current * sin(current_angle.val());
  auto x_neighbor = depth_neighbor * cos(neighbor_angle.val());
  auto y_neighbor = depth_neighbor * sin(neighbor_angle.val());
  auto dx = fabs(x_current - x_neighbor);
  auto dy = fabs(y_current - y_neighbor);
  auto angle = Radians::FromRadians(std::atan2(dy, dx));
  return angle;
}

}  // namespace depth_clustering
