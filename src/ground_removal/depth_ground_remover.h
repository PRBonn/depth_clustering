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

#ifndef SRC_GROUND_REMOVAL_DEPTH_GROUND_REMOVER_H_
#define SRC_GROUND_REMOVAL_DEPTH_GROUND_REMOVER_H_

#include <opencv2/opencv.hpp>

#include <algorithm>

#include "communication/abstract_client.h"
#include "communication/abstract_sender.h"
#include "projections/projection_params.h"
#include "utils/radians.h"
#include "utils/cloud.h"

namespace depth_clustering {

/**
 * @brief      A class to remove ground based upon depth image
 * @details    Given a depth image and image config this class should remove the
 *             ground and send the new depth image with no ground further
 *             through the pipeline to its clients.
 *
 * @param      params  projection params
 */
class DepthGroundRemover : public AbstractClient<Cloud>,
                           public AbstractSender<Cloud> {
  using ClientT = AbstractClient<Cloud>;
  using SenderT = AbstractSender<Cloud>;

 public:
  explicit DepthGroundRemover(const ProjectionParams& params,
                              const Radians& ground_remove_angle,
                              int window_size = 5)
      : ClientT{},
        SenderT{SenderType::STREAMER},
        _params{params},
        _window_size{window_size},
        _ground_remove_angle{ground_remove_angle} {}
  virtual ~DepthGroundRemover() {}

  /**
   * @brief      when someone sends us an object we process it
   * @details    receiving a depth image we remove ground from it and send to
   *             next recepient
   *
   * @param      depth_image  32 bit depth image
   * @param      sender_id    id of the sender
   */
  void OnNewObjectReceived(const Cloud& cloud, const int sender_id) override;

 protected:
  /**
   * @brief      Zero out all pixels that belong to ground
   *
   * @param[in]  image        Input depth image
   * @param[in]  angle_image  The angle image
   * @param[in]  threshold    angle threshold
   *
   * @return     depth image with 0 instead of ground pixels
   */
  cv::Mat ZeroOutGround(const cv::Mat& image, const cv::Mat& angle_image,
                        const Radians& threshold) const;

  cv::Mat ZeroOutGroundBFS(const cv::Mat& image, const cv::Mat& angle_image,
                           const Radians& threshold, int kernel_size) const;

  /**
   * @brief      create a help image with angle in radians written for each
   *             pixel
   *
   * @param      depth_image  [input depth image]
   * @return     [32 bit float image with angle in radians written in every
   *             pixel]
   */
  cv::Mat CreateAngleImage(const cv::Mat& depth_image);

  /**
   * @brief      Get kernel for Savitsky-Golay filter
   * @details    Get a column filter to process an image filled with data with
   *             Savitsky-Golay filter
   *
   * @param      window_size  size of the kernel
   * @return     column Mat kernel
   */
  cv::Mat GetSavitskyGolayKernel(int window_size) const;
  cv::Mat GetUniformKernel(int window_size, int type = CV_32F) const;

  /**
   * @brief      Apply Savitsky-Golay smoothing to a column
   * @param      column  [A column of an angle image]
   * @return     [a smoothed column]
   */

  cv::Mat ApplySavitskyGolaySmoothing(const cv::Mat& column, int window_size);
  /**
   * @brief      Get line angle
   * @details    Given two depth values and their angles compute the angle of
   *             the line that they spawn in the sensor frame.
   *
   * @param      depth_image  [32 bit float image]
   * @param      col          [current column]
   * @param      row_curr     [current row]
   * @param      row_neigh    [neighbor row]
   * @return     [angle of the line]
   */
  Radians GetLineAngle(const cv::Mat& depth_image, int col, int row_curr,
                       int row_neigh);

  /**
   * @brief      Repair zeros in the depth image
   *
   * @param[in]  depth_image  The depth image
   *
   * @return     depth image with repaired values
   */
  cv::Mat RepairDepth(const cv::Mat& no_ground_image, int step,
                      float depth_threshold);

  cv::Mat RepairDepth(const cv::Mat& depth_image);

  ProjectionParams _params;
  int _window_size = 5;
  Radians _ground_remove_angle = 5_deg;
  float _eps = 0.001f;

  mutable int _counter = 0;
};

}  // namespace depth_clustering

#endif  // SRC_GROUND_REMOVAL_DEPTH_GROUND_REMOVER_H_
