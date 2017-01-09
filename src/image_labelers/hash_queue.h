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

#ifndef SRC_IMAGE_LABELERS_HASH_QUEUE_H_
#define SRC_IMAGE_LABELERS_HASH_QUEUE_H_

#include <queue>
#include <unordered_map>

#include "image_labelers/pixel_coords.h"

namespace depth_clustering {

class HashQueue {
 public:
  explicit HashQueue(int estimated_size = 0) {
    _hash_map.reserve(estimated_size);
  }
  ~HashQueue() {}

  void push(const PixelCoord& coord) {
    _hash_map[coord.row][coord.col] = true;
    _queue.push(coord);
  }

  void pop() { _queue.pop(); }

  bool empty() const { return _queue.empty(); }

  const PixelCoord& front() const { return _queue.front(); }

  bool contains(const PixelCoord& coord) const {
    const auto& col_map_iter = _hash_map.find(coord.row);
    if (col_map_iter == _hash_map.end()) {
      return false;
    }
    const auto& col_iter = col_map_iter->second.find(coord.col);
    if (col_iter == col_map_iter->second.end()) {
      return false;
    }
    return true;
  }

 private:
  std::queue<PixelCoord> _queue;
  std::unordered_map<int16_t, std::unordered_map<int16_t, bool>> _hash_map;
};

}  // namespace depth_clustering

#endif  // SRC_IMAGE_LABELERS_HASH_QUEUE_H_
