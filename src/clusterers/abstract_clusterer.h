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

#ifndef SRC_CLUSTERERS_ABSTRACT_CLUSTERER_H_
#define SRC_CLUSTERERS_ABSTRACT_CLUSTERER_H_

#include <unordered_map>
#include <vector>

#include "communication/abstract_client.h"
#include "communication/abstract_sender.h"
#include "utils/cloud.h"

namespace depth_clustering {

/**
 * @brief      Class for abstract clusterer.
 */
class AbstractClusterer
    : public AbstractClient<Cloud>,
      public AbstractSender<std::unordered_map<uint16_t, Cloud>> {
 public:
  using Receiver = AbstractClient<Cloud>;
  using Sender = AbstractSender<std::unordered_map<uint16_t, Cloud>>;

  /**
   * @brief      Construct a clusterer.
   *
   * @param[in]  cluster_tollerance  The cluster tollerance
   * @param[in]  min_cluster_size    The minimum cluster size
   * @param[in]  max_cluster_size    The maximum cluster size
   * @param[in]  skip                Only cluster every skip cloud
   */
  explicit AbstractClusterer(double cluster_tollerance = 0.2,
                             uint16_t min_cluster_size = 100,
                             uint16_t max_cluster_size = 25000,
                             uint16_t skip = 10)
      : Receiver(),
        Sender(SenderType::STREAMER),
        _cluster_tollerance(cluster_tollerance),
        _min_cluster_size(min_cluster_size),
        _max_cluster_size(max_cluster_size),
        _skip(skip),
        _counter(0) {}
  virtual ~AbstractClusterer() {}

 protected:
  double _cluster_tollerance;
  uint16_t _min_cluster_size;
  uint16_t _max_cluster_size;
  uint16_t _skip;
  uint32_t _counter;
};

}  // namespace depth_clustering

#endif  // SRC_CLUSTERERS_ABSTRACT_CLUSTERER_H_
