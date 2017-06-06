// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

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
