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

#ifndef SRC_CLUSTERERS_EUCLIDEAN_CLUSTERER_H_
#define SRC_CLUSTERERS_EUCLIDEAN_CLUSTERER_H_

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/time.h>

#include <vector>
#include <ctime>
#include <chrono>

#include "communication/abstract_client.h"
#include "communication/abstract_sender.h"
#include "utils/useful_typedefs.h"
#include "utils/cloud.h"

#include "clusterers/abstract_clusterer.h"

namespace depth_clustering {

using std::chrono::microseconds;
using std::chrono::milliseconds;
using std::chrono::high_resolution_clock;

/**
 * @brief      Class for euclidean clustering.
 */
class EuclideanClusterer : public AbstractClusterer {
 public:
  using PointT = pcl::PointXYZL;
  explicit EuclideanClusterer(double cluster_tollerance = 0.2,
                              uint16_t min_cluster_size = 100,
                              uint16_t max_cluster_size = 25000,
                              uint16_t skip = 10)
      : AbstractClusterer(cluster_tollerance, min_cluster_size,
                          max_cluster_size, skip) {}
  virtual ~EuclideanClusterer() {}

  /**
   * @brief      Gets called when somebody sends this client an object.
   *
   * @param[in]  cloud      The cloud to cluster
   * @param[in]  sender_id  The sender identifier
   */
  void OnNewObjectReceived(const Cloud& cloud, const int sender_id) override {
    auto pcl_cloud_ptr = cloud.ToPcl();
    std::unordered_map<uint16_t, Cloud> clusters;
    if (this->_counter++ % this->_skip != 0) {
      // share empty clusters
      this->ShareDataWithAllClients(clusters);
      return;
    }
    auto start = high_resolution_clock::now();
    typename pcl::search::KdTree<PointT>::Ptr tree(
        new pcl::search::KdTree<PointT>);
    tree->setInputCloud(pcl_cloud_ptr);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> clusterer;
    clusterer.setClusterTolerance(this->_cluster_tollerance);
    clusterer.setMinClusterSize(this->_min_cluster_size);
    clusterer.setMaxClusterSize(this->_max_cluster_size);
    clusterer.setSearchMethod(tree);
    clusterer.setInputCloud(pcl_cloud_ptr);
    clusterer.extract(cluster_indices);

    auto end = high_resolution_clock::now();
    fprintf(stderr, "euclidian based labeling took: %lu us\n",
            std::chrono::duration_cast<microseconds>(end - start).count());
    for (auto cluster_iter = cluster_indices.begin();
         cluster_iter != cluster_indices.end(); ++cluster_iter) {
      int idx = std::distance(cluster_indices.begin(), cluster_iter);
      Cloud cloud_cluster(cloud.pose());
      cloud_cluster.reserve(cluster_iter->indices.size());
      for (auto point_iter = cluster_iter->indices.begin();
           point_iter != cluster_iter->indices.end(); ++point_iter) {
        cloud_cluster.push_back(cloud[*point_iter]);
      }
      clusters[idx] = cloud_cluster;
    }
    this->ShareDataWithAllClients(clusters);
  }
};

}  // namespace depth_clustering

#endif  // SRC_CLUSTERERS_EUCLIDEAN_CLUSTERER_H_
