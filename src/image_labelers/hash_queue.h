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
