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

#ifndef SRC_CLUSTERERS_IMAGE_BASED_CLUSTERER_H_
#define SRC_CLUSTERERS_IMAGE_BASED_CLUSTERER_H_

#include <chrono>
#include <ctime>
#include <map>
#include <opencv/cv.h>
#include <string>
#include <unordered_map>
#include <vector>

#include "communication/abstract_client.h"
#include "communication/abstract_sender.h"
#include "utils/cloud.h"
#include "utils/radians.h"
#include "utils/timer.h"
#include "utils/useful_typedefs.h"

#include "clusterers/abstract_clusterer.h"
#include "image_labelers/diff_helpers/diff_factory.h"
#include "image_labelers/linear_image_labeler.h"
#include "projections/cloud_projection.h"

namespace depth_clustering {

/**
 * @brief      Class for image based clusterer.
 *
 * @tparam     LabelerT  A Labeler class to be used for labeling.
 */
template <typename LabelerT>
class ImageBasedClusterer : public AbstractClusterer {
 public:
  using Receiver = AbstractClient<Cloud>;
  using Sender = AbstractSender<std::unordered_map<uint16_t, Cloud>>;

  /**
   * @brief      Construct an image-based clusterer.
   *
   * @param[in]  angle_tollerance  The angle tollerance to separate objects
   * @param[in]  min_cluster_size  The minimum cluster size to send
   * @param[in]  max_cluster_size  The maximum cluster size to send
   */
  explicit ImageBasedClusterer(Radians angle_tollerance = 8_deg,
                               uint16_t min_cluster_size = 100,
                               uint16_t max_cluster_size = 25000)
      : AbstractClusterer(0.0, min_cluster_size, max_cluster_size),
        _counter(0),
        _angle_tollerance(angle_tollerance),
        _label_client{nullptr} {}

  virtual ~ImageBasedClusterer() {}

  /**
   * @brief      Sets the difference type.
   *
   * @param[in]  diff_type  The difference type
   */
  void SetDiffType(DiffFactory::DiffType diff_type) { _diff_type = diff_type; }

  /**
   * @brief      Sets the label image client.
   *
   * @param      client  The client to receive color images with labels
   */
  void SetLabelImageClient(AbstractClient<cv::Mat>* client) {
    this->_label_client = client;
  }

  /**
   * @brief      Gets called when clusterer receives a cloud to cluster
   *
   * @param[in]  cloud      The cloud to cluster
   * @param[in]  sender_id  The sender identifier
   */
  void OnNewObjectReceived(const Cloud& cloud, int) override {
    // generate a projection from a point cloud
    if (!cloud.projection_ptr()) {
      fprintf(stderr, "ERROR: projection not initialized in cloud.\n");
      fprintf(stderr, "INFO: cannot label this cloud.\n");
      return;
    }
    time_utils::Timer timer;
    LabelerT image_labeler(cloud.projection_ptr()->depth_image(),
                           cloud.projection_ptr()->params(), _angle_tollerance);
    image_labeler.ComputeLabels(_diff_type);
    const cv::Mat* labels_ptr = image_labeler.GetLabelImage();
    fprintf(stderr, "INFO: image based labeling took: %lu us\n",
            timer.measure());

    // send image to whoever wants to get it
    if (_label_client) {
      _label_client->OnNewObjectReceived(*labels_ptr, this->id());
    }

    fprintf(stderr, "INFO: labels image sent to clients in: %lu us\n",
            timer.measure());

    // create 3d clusters from image labels
    std::unordered_map<uint16_t, Cloud> clusters;
    for (int row = 0; row < labels_ptr->rows; ++row) {
      for (int col = 0; col < labels_ptr->cols; ++col) {
        const auto& point_container = cloud.projection_ptr()->at(row, col);
        if (point_container.IsEmpty()) {
          // this is ok, just continue, nothing interesting here, no points.
          continue;
        }
        uint16_t label = labels_ptr->at<uint16_t>(row, col);
        if (label < 1) {
          // this is a default label, skip
          continue;
        }
        for (const auto& point_idx : point_container.points()) {
          const auto& point = cloud.points()[point_idx];
          clusters[label].push_back(point);
        }
      }
    }

    // filter out unfitting clusters
    std::vector<uint16_t> labels_to_erase;
    for (const auto& kv : clusters) {
      const auto& cluster = kv.second;
      if (cluster.size() < this->_min_cluster_size ||
          cluster.size() > this->_max_cluster_size) {
        labels_to_erase.push_back(kv.first);
      }
    }
    for (auto label : labels_to_erase) {
      clusters.erase(label);
    }

    fprintf(stderr, "INFO: prepared clusters in: %lu us\n", timer.measure());

    this->ShareDataWithAllClients(clusters);
    fprintf(stderr, "INFO: clusters shared: %lu us\n", timer.measure());
  }

 private:
  int _counter;
  Radians _angle_tollerance;

  AbstractClient<cv::Mat>* _label_client;

  DiffFactory::DiffType _diff_type = DiffFactory::DiffType::NONE;
};

}  // namespace depth_clustering

#endif  // SRC_CLUSTERERS_IMAGE_BASED_CLUSTERER_H_
