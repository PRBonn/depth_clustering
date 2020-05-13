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
