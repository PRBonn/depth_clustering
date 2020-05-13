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

#include "utils/bbox.h"

#include <algorithm>

Bbox::Bbox(const depth_clustering::Cloud& cloud) {
  for (const auto& point : cloud.points()) {
    _min_point << std::min(_min_point.x(), point.x()),
        std::min(_min_point.y(), point.y()),
        std::min(_min_point.z(), point.z());
    _max_point << std::max(_max_point.x(), point.x()),
        std::max(_max_point.y(), point.y()),
        std::max(_max_point.z(), point.z());
  }
  UpdateScaleAndCenter();
}

Bbox::Bbox(const Eigen::Vector3f& min_point, const Eigen::Vector3f& max_point) {
  _min_point = min_point;
  _max_point = max_point;
  UpdateScaleAndCenter();
}

Bbox Bbox::Intersect(const Bbox& other) const {
  Eigen::Vector3f min_point(std::max(_min_point.x(), other.min_point().x()),
                            std::max(_min_point.y(), other.min_point().y()),
                            std::max(_min_point.z(), other.min_point().z()));
  Eigen::Vector3f max_point(std::min(_max_point.x(), other.max_point().x()),
                            std::min(_max_point.y(), other.max_point().y()),
                            std::min(_max_point.z(), other.max_point().z()));
  Bbox intersection(min_point, max_point);
  return intersection;
}

bool Bbox::Intersects(const Bbox& other) const {
  auto intersection = this->Intersect(other);
  return intersection.volume() > 0.0f;
}

void Bbox::UpdateScaleAndCenter() {
  _scale = _max_point - _min_point;
  if ((_scale.array() <= 0.0f).any()) {
    _scale = Eigen::Vector3f::Zero();
    _center = Eigen::Vector3f::Zero();
    _volume = Bbox::WRONG_VOLUME;
    return;
  }
  _center = 0.5 * (_min_point + _max_point);
  _volume = _scale.prod();
}
