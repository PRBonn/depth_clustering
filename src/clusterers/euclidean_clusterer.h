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
